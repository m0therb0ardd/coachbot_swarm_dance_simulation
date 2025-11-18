# -*- coding: utf-8 -*-
# PUNCH (CoachBot): sharp downward zig-zag bursts
# - Overall motion: downward
# - Each pulse: turn to a diagonal, then dash along it
# - Diagonals alternate: down-right, down-left, down-right, ...
# - Uses leader pulses (decentralized, P2P)
# - No dancer obstacle; walls only
from __future__ import division
import math
import struct
import random

# -------- arena bounds (meters) --------
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

# -------- drive & loop --------
MAX_WHEEL       = 35
FWD_FAST        = 0.95     # dash speed
FWD_SLOW        = 0.10     # idle creep
FWD_MIN         = 0.45
TURN_K_DASH     = 6.0      # strong turning in dash/aim
TURN_K_IDLE     = 3.0      # gentler when idle
CMD_SMOOTH_DASH = 0.08     # less smoothing during dash (sharper)
CMD_SMOOTH_IDLE = 0.18     # more smoothing in idle
LOOP_DT_MS      = 40

# -------- waves / dash timing --------
WAVE_PERIOD = 3.0    # leader emits pulse every 3s (feel free to tune)
AIM_TIME    = 0.35   # aim turn time before dash
DASH_TIME   = 1.3    # duration of the punch dash
COOLDOWN    = 3.0    # rest after a dash

# -------- zig-zag geometry --------
# Base DOWN heading: -pi/2 (negative y)
PUNCH_HEADING = -0.5 * math.pi

# Diagonal offset around DOWN
ZIG_ANGLE = 0.35 * math.pi   # ~63° off down (very diagonal)

# -------- boundary softness (walls only) --------
SOFT_MARGIN    = 0.08
CRIT_MARGIN    = 0.02
SOFT_MAX_FORCE = 0.35

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

def soft_boundary_check(x, y):
    if (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
        y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN):
        return 2
    if (x < X_MIN + SOFT_MARGIN or x > X_MAX - SOFT_MARGIN or
        y < Y_MIN + SOFT_MARGIN or y > Y_MAX - SOFT_MARGIN):
        return 1
    return 0

# -------- messaging --------
TYPE_PULSE = 2
P_FMT      = 'iiii'  # (type, leader_vid, seq, sentinel)
P_BYTES    = struct.calcsize(P_FMT)

def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def wrap_angle(a):
    while a >  math.pi:
        a -= 2.0*math.pi
    while a <= -math.pi:
        a += 2.0*math.pi
    return a

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

    random.seed((vid if vid is not None else 0)*1103515245 & 0xFFFFFFFF)
    print('Robot %d starting PUNCH zig-leg choreography' % vid)

    my_vid       = int(vid)
    lowest_seen  = my_vid
    last_pulse_emit = -1e9
    seq          = 0

    seen_seqs      = set()
    dash_active    = False
    dash_until     = 0.0
    aim_until      = 0.0
    dash_start_time= 0.0

    # zig sign: +1 = down-right, -1 = down-left
    # we’ll flip this each time we start a dash
    # seed based on vid so half start right, half left
    zig_sign = 1 if (my_vid % 2 == 0) else -1
    zig_heading = PUNCH_HEADING + zig_sign * ZIG_ANGLE

    last_left  = 0
    last_right = 0
    last_print = 0.0
    cool_until = 0.0

    # wake localization
    robot.set_vel(20, 20)
    robot.delay(120)

    try:
        while True:
            pose = safe_pose(robot)
            if pose is None:
                robot.set_vel(0,0)
                robot.delay(LOOP_DT_MS)
                continue
           
            x, y, th = pose
            now = robot.get_clock()

            # boundary / LEDs
            bstat = soft_boundary_check(x, y)
            if bstat == 2:
                robot.set_led(100,0,0)
                robot.set_vel(0,0)
                print('[punch_zigleg] CRITICAL id=%d boundary [%.3f,%.3f]' % (vid, x, y))
                robot.delay(LOOP_DT_MS)
                continue
            elif bstat == 1:
                robot.set_led(100,59,0)   # amber-ish
            else:
                robot.set_led(0,70,70)    # teal

            # --- receive pulses ---
            msgs = robot.recv_msg()
            if msgs:
                for m in msgs:
                    try:
                        tpe, lvid, s, _sent = struct.unpack(P_FMT, m[:P_BYTES])
                        if tpe == TYPE_PULSE:
                            lowest_seen = min(lowest_seen, int(lvid))
                            if s not in seen_seqs:
                                seen_seqs.add(s)
                                # relay once
                                robot.send_msg(struct.pack(P_FMT,
                                                           TYPE_PULSE,
                                                           int(lvid),
                                                           int(s),
                                                           0))
                                robot.delay(50)
                                # trigger own leg if cooled
                                if now >= cool_until:
                                    # flip zig side
                                    zig_sign   *= -1
                                    zig_heading = PUNCH_HEADING + zig_sign * ZIG_ANGLE

                                    aim_until       = now + AIM_TIME
                                    dash_until      = aim_until + DASH_TIME
                                    dash_start_time = aim_until
                                    dash_active     = True
                                    cool_until      = dash_until + COOLDOWN
                                    robot.set_led(0,100,47)  # mint
                                    print('[punch_zigleg] id=%d DASH start (pulse %d, sign=%+d)' %
                                          (vid, s, zig_sign))
                    except:
                        pass

            # --- leader pulses ---
            if my_vid == lowest_seen and (now - last_pulse_emit) >= WAVE_PERIOD:
                seq += 1
                pkt = struct.pack(P_FMT, TYPE_PULSE, my_vid, seq, 0)
                robot.send_msg(pkt)
                last_pulse_emit = now
                seen_seqs.add(seq)
                print('[punch_zigleg] leader=%d pulse seq=%d' % (my_vid, seq))
                if now >= cool_until:
                    zig_sign   *= -1
                    zig_heading = PUNCH_HEADING + zig_sign * ZIG_ANGLE

                    aim_until       = now + AIM_TIME
                    dash_until      = aim_until + DASH_TIME
                    dash_start_time = aim_until
                    dash_active     = True
                    cool_until      = dash_until + COOLDOWN
                    robot.set_led(0,100,47)

            # --- base wall force (used in both modes) ---
            bx, by = soft_boundary_force(x, y)

            if dash_active:
                # ----- AIM + DASH -----
                if now < aim_until:
                    # Turn toward fixed zig_heading BEFORE dash
                    target_h = zig_heading
                    err = wrap_angle(target_h - th)
                    fwd = FWD_SLOW * 0.6
                    K_turn = TURN_K_DASH
                    smooth = CMD_SMOOTH_DASH
                else:
                    if now >= dash_until:
                        # dash finished
                        dash_active = False
                        robot.set_led(0,70,70)  # teal idle
                        fwd = FWD_SLOW
                        err = wrap_angle(PUNCH_HEADING - th)
                        K_turn = TURN_K_IDLE
                        smooth = CMD_SMOOTH_IDLE
                        print('[punch_zigleg] id=%d DASH end' % vid)
                    else:
                        # DASH: go fast along zig_heading (fixed)
                        target_h = zig_heading
                        err = wrap_angle(target_h - th)
                        fwd = FWD_FAST
                        K_turn = TURN_K_DASH
                        smooth = CMD_SMOOTH_DASH

                turn = clamp(K_turn * err, -2.6, 2.6)
                lcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)),
                             -MAX_WHEEL, MAX_WHEEL)
                rcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)),
                             -MAX_WHEEL, MAX_WHEEL)

                left  = int((1.0 - smooth) * lcmd + smooth * last_left)
                right = int((1.0 - smooth) * rcmd + smooth * last_right)

            else:
                # ----- IDLE drift: small downward + walls -----
                vx_idle = bx
                vy_idle = by - 0.06  # small downward bias

                if abs(vx_idle) < 1e-6 and abs(vy_idle) < 1e-6:
                    vy_idle = -1e-3

                hdg_idle = math.atan2(vy_idle, vx_idle)
                err = wrap_angle(hdg_idle - th)
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

                turn = clamp(TURN_K_IDLE * err, -2.0, 2.0)
                lcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)),
                             -MAX_WHEEL, MAX_WHEEL)
                rcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)),
                             -MAX_WHEEL, MAX_WHEEL)

                left  = int((1.0 - CMD_SMOOTH_IDLE) * lcmd + CMD_SMOOTH_IDLE * last_left)
                right = int((1.0 - CMD_SMOOTH_IDLE) * rcmd + CMD_SMOOTH_IDLE * last_right)

            last_left  = left
            last_right = right
            robot.set_vel(left, right)

            if now - last_print > 2.0:
                state = "dash" if dash_active else "idle"
                print('[punch_zigleg] id=%d leader=%d state=%s pos[%.3f,%.3f]' %
                      (vid, lowest_seen, state, x, y))
                last_print = now

            robot.delay(LOOP_DT_MS)

    except Exception as e:
        try:
            robot.set_vel(0,0)
            robot.set_led(100,0,0)
        except:
            pass
        print('[punch_zigleg] ERROR id=%d: %s' % (vid, repr(e)))
        raise
    finally:
        try:
            robot.set_vel(0,0)
        except:
            pass
