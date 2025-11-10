# -*- coding: utf-8 -*-
import math, struct, random

# -------- arena & obstacle --------
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35
FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
OBST_CX, OBST_CY = (-0.1, 0.475)

# -------- drive & loop --------
MAX_WHEEL = 35
TURN_K    = 3.0
FWD_FAST  = 0.95
FWD_SLOW  = 0.30
FWD_MIN   = 0.45
CMD_SMOOTH= 0.18
LOOP_DT_MS= 40

# -------- waves --------
WAVE_PERIOD    = 4.0   # leader emits pulse every 4s
DASH_TIME      = 1.2   # duration of the slash dash
AIM_TIME       = 0.35  # aim turn time before dash
RIGHT_HEADING  = 0.0   # +x̂
ANGLE_JITTER   = 0.22  # small diagonal variety
COOLDOWN       = 2.0

# -------- boundary softness --------
SOFT_MARGIN     = 0.08
CRIT_MARGIN     = 0.02
SOFT_MAX_FORCE  = 0.35

# -------- messaging --------
TYPE_PULSE = 2
P_FMT = 'iiii'  # (type, leader_vid, seq, sentinel) — int-only, tiny
P_BYTES = struct.calcsize(P_FMT)

# -------- small helpers --------
def clamp(v, lo, hi): return lo if v < lo else (hi if v > hi else v)
def wrap_angle(a):
    while a >  math.pi: a -= 2.0*math.pi
    while a <= -math.pi: a += 2.0*math.pi
    return a

def soft_boundary_force(x, y):
    fx = fy = 0.0
    if x < X_MIN + SOFT_MARGIN:
        fx += SOFT_MAX_FORCE * (1.0 - (x - X_MIN)/SOFT_MARGIN)
    elif x > X_MAX - SOFT_MARGIN:
        fx -= SOFT_MAX_FORCE * (1.0 - (X_MAX - x)/SOFT_MARGIN)
    if y < Y_MIN + SOFT_MARGIN:
        fy += SOFT_MAX_FORCE * (1.0 - (y - Y_MIN)/SOFT_MARGIN)
    elif y > Y_MAX - SOFT_MARGIN:
        fy -= SOFT_MAX_FORCE * (1.0 - (Y_MAX - y)/SOFT_MARGIN)
    return fx, fy

def soft_obstacle_force(x, y, max_force=0.6, buffer_width=0.10):
    dx = x - OBST_CX; dy = y - OBST_CY; r = math.hypot(dx, dy)
    if r < SAFE_BUBBLE + buffer_width:
        if r < 1e-6: return max_force, 0.0
        s = max(0.0, (SAFE_BUBBLE + buffer_width - r) / buffer_width) * max_force
        return s * (dx / r), s * (dy / r)
    return 0.0, 0.0

def soft_boundary_check(x, y):
    if (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
        y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN): return 2
    if (x < X_MIN + SOFT_MARGIN or x > X_MAX - SOFT_MARGIN or
        y < Y_MIN + SOFT_MARGIN or y > Y_MAX - SOFT_MARGIN): return 1
    return 0

def safe_pose(robot):
    p = robot.get_pose()
    if p and len(p) >= 3: return float(p[0]), float(p[1]), float(p[2])
    return None

# --- sim shims (easy port back to hardware) ---
def get_id(robot):
    # SIM:
    try: return int(robot.id())
    except: return 0
    # HW: change to -> return int(robot.virtual_id())

def logw(msg):
    # SIM:
    print(msg)
    # HW: change to -> log.write(msg + "\n")

def usr(robot):
    robot.delay(400)  # sim settle
    vid = get_id(robot)
    random.seed((vid if vid is not None else 0)*1103515245 & 0xFFFFFFFF)

    # leader election: lowest vid observed
    my_vid = int(vid)
    lowest_seen = my_vid
    last_pulse_emit = -1e9
    seq = 0

    seen_seqs = set()
    dash_active = False
    dash_until  = 0.0
    aim_until   = 0.0
    aim_heading = RIGHT_HEADING

    last_left = last_right = 0
    last_print = 0.0
    cool_until = 0.0

    # nudge localization
    robot.set_vel(20, 20); robot.delay(120)

    try:
        while True:
            pose = safe_pose(robot)
            if pose is None:
                robot.set_vel(0,0); robot.delay(LOOP_DT_MS); continue
            x, y, th = pose
            now = robot.get_clock()

            # boundary / safety LEDs (0–100 scale)
            bstat = soft_boundary_check(x, y)
            if bstat == 2:
                robot.set_led(100,0,0); robot.set_vel(0,0)
                logw(f"[SIM slash_p2p] CRITICAL id={vid} boundary [{x:.3f},{y:.3f}]")
                robot.delay(LOOP_DT_MS); continue
            elif bstat == 1: robot.set_led(100,59,0)   # amber-ish
            else:            robot.set_led(0,70,70)    # teal

            # listen for pulses; learn population
            msgs = robot.recv_msg() or []
            for m in msgs:
                try:
                    tpe, lvid, s, sent = struct.unpack(P_FMT, m[:P_BYTES])
                    if tpe == TYPE_PULSE:
                        lowest_seen = min(lowest_seen, int(lvid))
                        if s not in seen_seqs:
                            seen_seqs.add(s)
                            # relay to help flooding
                            robot.send_msg(struct.pack(P_FMT, TYPE_PULSE, int(lvid), int(s), 0))
                            # trigger own dash if cooled
                            if now >= cool_until:
                                aim_heading = RIGHT_HEADING + random.uniform(-ANGLE_JITTER, ANGLE_JITTER)
                                aim_until = now + AIM_TIME
                                dash_until = aim_until + DASH_TIME
                                dash_active = True
                                cool_until = dash_until + COOLDOWN
                                robot.set_led(0,100,47)  # mint

                except Exception:
                    pass

            # if I am leader, emit pulses every WAVE_PERIOD
            if my_vid == lowest_seen and (now - last_pulse_emit) >= WAVE_PERIOD:
                seq += 1
                pkt = struct.pack(P_FMT, TYPE_PULSE, my_vid, seq, 0)
                robot.send_msg(pkt)
                last_pulse_emit = now
                seen_seqs.add(seq)
                if now >= cool_until:
                    aim_heading = RIGHT_HEADING + random.uniform(-ANGLE_JITTER, ANGLE_JITTER)
                    aim_until = now + AIM_TIME
                    dash_until = aim_until + DASH_TIME
                    dash_active = True
                    cool_until = dash_until + COOLDOWN
                    robot.set_led(0,100,47)  # mint

            # base gentle drift + safety fields
            vx = vy = 0.0
            bx, by = soft_boundary_force(x, y); vx += bx; vy += by
            ox, oy = soft_obstacle_force(x, y);  vx += ox; vy += oy

            # if dashing: steer to heading then surge forward
            if dash_active:
                if now < aim_until:
                    target = aim_heading
                    err = wrap_angle(target - th)
                    fwd = FWD_SLOW * 0.6
                else:
                    if now >= dash_until:
                        dash_active = False
                        robot.set_led(0,70,70)  # teal idle
                        fwd = FWD_SLOW
                        err = wrap_angle(aim_heading - th)
                    else:
                        target = aim_heading
                        err = wrap_angle(target - th)
                        fwd = FWD_FAST
                turn = clamp(TURN_K * err, -1.5, 1.5)
                lcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL,  MAX_WHEEL)
                rcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL,  MAX_WHEEL)
            else:
                # gentle idle drift to keep composition alive
                vx += 0.05
                hdg = math.atan2(vy, vx)
                err = wrap_angle(hdg - th)
                ae = abs(err)
                if ae < 0.5:   fwd = FWD_SLOW
                elif ae < 1.2: fwd = FWD_SLOW * 0.8
                else:          fwd = FWD_SLOW * 0.6
                if bstat == 1: fwd *= 0.7
                if fwd < FWD_MIN: fwd = FWD_MIN
                turn = clamp(TURN_K * err, -1.5, 1.5)
                lcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL,  MAX_WHEEL)
                rcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL,  MAX_WHEEL)

            left  = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * last_left)
            right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * last_right)
            last_left, last_right = left, right
            robot.set_vel(left, right)

            # light logging
            if now - last_print > 2.0:
                state = "dash" if dash_active else "idle"
                logw(f"[SIM slash_p2p] id={vid} leader={lowest_seen} state={state}")
                last_print = now

            robot.delay(LOOP_DT_MS)

    except Exception:
        try: robot.set_vel(0,0); robot.set_led(100,0,0)
        except: pass
        raise
    finally:
        try: robot.set_vel(0,0)
        except: pass
