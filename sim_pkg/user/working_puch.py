#  -*- coding: utf-8 -*-
# SLASH (P2P waves) - COACHBOT VERSION
# Coordinated wave-like dashes triggered by leader pulses
# Python 2.7 compatible

from __future__ import division
import math
import struct
import random

# -------- arena & obstacle (meters) --------
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
FWD_SLOW  = 0.10
FWD_MIN   = 0.45
CMD_SMOOTH= 0.18
LOOP_DT_MS= 40

# -------- waves --------
WAVE_PERIOD    = 4.0   # leader emits pulse every 4s
DASH_TIME      = 1.2   # duration of the slash dash
AIM_TIME       = 0.35  # aim turn time before dash
RIGHT_HEADING  = 0.0   # +x direction
ANGLE_JITTER   = 0.22  # small diagonal variety
COOLDOWN       = 5.0   # rest af.ter a dash

# -------- boundary softness --------
SOFT_MARGIN     = 0.08
CRIT_MARGIN     = 0.02
SOFT_MAX_FORCE  = 0.35

# -------- messaging --------
TYPE_PULSE = 2
P_FMT = 'iiii'  # (type, leader_vid, seq, sentinel)
P_BYTES = struct.calcsize(P_FMT)

# -------- utils --------
def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def wrap_angle(a):
    while a >  math.pi:
        a -= 2.0*math.pi
    while a <= -math.pi:
        a += 2.0*math.pi
    return a

def soft_boundary_force(x, y):
    fx = 0.0
    fy = 0.0
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
    dx = x - OBST_CX
    dy = y - OBST_CY
    r = math.sqrt(dx*dx + dy*dy)  # FIXED: no math.hypot
    if r < SAFE_BUBBLE + buffer_width:
        if r < 1e-6:
            return max_force, 0.0
        s = max(0.0, (SAFE_BUBBLE + buffer_width - r) / buffer_width) * max_force
        return s * (dx / r), s * (dy / r)
    return 0.0, 0.0

def soft_boundary_check(x, y):
    if (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
        y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN):
        return 2
    if (x < X_MIN + SOFT_MARGIN or x > X_MAX - SOFT_MARGIN or
        y < Y_MIN + SOFT_MARGIN or y > Y_MAX - SOFT_MARGIN):
        return 1
    return 0

def safe_pose(robot):
    p = robot.get_pose()
    if p and len(p) >= 3:
        return float(p[0]), float(p[1]), float(p[2])
    return None

def usr(robot):
    # --- identity (HARDWARE) ---
    try:
        vid = int(robot.virtual_id())
    except:
        vid = -1

    # seed per-robot
    random.seed((vid if vid is not None else 0)*1103515245 & 0xFFFFFFFF)
   
    print('Robot %d starting SLASH choreography' % vid)

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

    last_left = 0
    last_right = 0
    last_print = 0.0
    cool_until = 0.0

    # brief nudge to wake localization
    robot.set_vel(20, 20)
    robot.delay(120)

    try:
        while True:
            pose = safe_pose(robot)
            if pose is None:
                robot.set_vel(0,0)
                robot.delay(LOOP_DT_MS)
                continue
           
            x = pose[0]
            y = pose[1]
            th = pose[2]
            now = robot.get_clock()

            # boundary / safety LEDs (0-100 scale)
            bstat = soft_boundary_check(x, y)
            if bstat == 2:
                robot.set_led(100,0,0)
                robot.set_vel(0,0)
                print('[slash_p2p] CRITICAL id=%d boundary [%.3f,%.3f]' % (vid, x, y))
                robot.delay(LOOP_DT_MS)
                continue
            elif bstat == 1:
                robot.set_led(100,59,0)   # amber-ish
            else:
                robot.set_led(0,70,70)    # teal

            # listen for pulses; learn population
            msgs = robot.recv_msg()
            if msgs:
                for m in msgs:
                    try:
                        tpe, lvid, s, _sent = struct.unpack(P_FMT, m[:P_BYTES])
                        if tpe == TYPE_PULSE:
                            lowest_seen = min(lowest_seen, int(lvid))
                            if s not in seen_seqs:
                                seen_seqs.add(s)
                                # relay once to help flooding (50ms gap before another recv)
                                robot.send_msg(struct.pack(P_FMT, TYPE_PULSE, int(lvid), int(s), 0))
                                robot.delay(50)
                                # trigger own dash if cooled
                                if now >= cool_until:
                                    aim_heading = RIGHT_HEADING + random.uniform(-ANGLE_JITTER, ANGLE_JITTER)
                                    aim_until   = now + AIM_TIME
                                    dash_until  = aim_until + DASH_TIME
                                    dash_active = True
                                    cool_until  = dash_until + COOLDOWN
                                    robot.set_led(0,100,47)  # mint
                                    print('[slash_p2p] id=%d DASH start (pulse %d)' % (vid, s))
                    except:
                        # ignore malformed messages
                        pass

            # if I am leader, emit pulses every WAVE_PERIOD
            if my_vid == lowest_seen and (now - last_pulse_emit) >= WAVE_PERIOD:
                seq += 1
                pkt = struct.pack(P_FMT, TYPE_PULSE, my_vid, seq, 0)
                robot.send_msg(pkt)
                last_pulse_emit = now
                seen_seqs.add(seq)
                print('[slash_p2p] leader=%d pulse seq=%d' % (my_vid, seq))
                if now >= cool_until:
                    aim_heading = RIGHT_HEADING + random.uniform(-ANGLE_JITTER, ANGLE_JITTER)
                    aim_until   = now + AIM_TIME
                    dash_until  = aim_until + DASH_TIME
                    dash_active = True
                    cool_until  = dash_until + COOLDOWN
                    robot.set_led(0,100,47)

            # base gentle drift + safety fields
            vx = 0.0
            vy = 0.0
            bx, by = soft_boundary_force(x, y)
            vx += bx
            vy += by
            ox, oy = soft_obstacle_force(x, y)
            vx += ox
            vy += oy

            # slash behavior
            if dash_active:
                if now < aim_until:
                    # short aim/turn phase
                    target = aim_heading
                    err = wrap_angle(target - th)
                    fwd = FWD_SLOW * 0.6
                else:
                    if now >= dash_until:
                        dash_active = False
                        robot.set_led(0,70,70)  # teal idle
                        fwd = FWD_SLOW
                        err = wrap_angle(aim_heading - th)
                        print('[slash_p2p] id=%d DASH end' % vid)
                    else:
                        # surge
                        target = aim_heading
                        err = wrap_angle(target - th)
                        fwd = FWD_FAST
                turn = clamp(TURN_K * err, -1.5, 1.5)
                lcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL,  MAX_WHEEL)
                rcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL,  MAX_WHEEL)
            else:
                # gentle idle drift (keeps composition alive)
                vx += 0.05
                hdg = math.atan2(vy, vx)
                err = wrap_angle(hdg - th)
                ae = abs(err)
                if ae < 0.5:
                    fwd = FWD_SLOW
                elif ae < 1.2:
                    fwd = FWD_SLOW * 0.8
                else:
                    fwd = FWD_SLOW * 0.6
                if bstat == 1:
                    fwd = fwd * 0.7
                if fwd < FWD_MIN:
                    fwd = FWD_MIN
                turn = clamp(TURN_K * err, -1.5, 1.5)
                lcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL,  MAX_WHEEL)
                rcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL,  MAX_WHEEL)

            # EMA smoothing to reduce jerk on hardware
            left  = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * last_left)
            right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * last_right)
            last_left = left
            last_right = right
            robot.set_vel(left, right)

            # sparse status
            if now - last_print > 2.0:
                state = "dash" if dash_active else "idle"
                print('[slash_p2p] id=%d leader=%d state=%s pos[%.3f,%.3f]' % (vid, lowest_seen, state, x, y))
                last_print = now

            robot.delay(LOOP_DT_MS)

    except Exception as e:
        # stop + red LED on error
        try:
            robot.set_vel(0,0)
            robot.set_led(100,0,0)
        except:
            pass
        print('[slash_p2p] ERROR id=%d: %s' % (vid, repr(e)))
        raise
    finally:
        try:
            robot.set_vel(0,0)
        except:
            pass