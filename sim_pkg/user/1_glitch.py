

# -*- coding: utf-8 -*-
# GLITCH 3.0 chaotic bursts + flicker + twitch
# FINAL STATE = perfectly expanded ring (same outward shift for all)
# Python 2.7 safe

import math
import random

CX, CY = (-0.1, 0.475)

RING_EXPANSION = 0.13    

SPEED = 0.88
TURN_K = 3.0
MAX_WHEEL = 35

X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35
CRIT_MARGIN = 0.02

def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def wrap(a):
    while a > math.pi:  a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a

def safe_pose(r):
    p = r.get_pose()
    if p and len(p)>=3:
        return float(p[0]), float(p[1]), float(p[2])
    return None

def is_critical(x,y):
    return (x < X_MIN+CRIT_MARGIN or x > X_MAX-CRIT_MARGIN or
            y < Y_MIN+CRIT_MARGIN or y > Y_MAX-CRIT_MARGIN)


def usr(robot):
    robot.delay(1500)

    try: vid = int(robot.id)
    except: vid = random.randint(0,99)

    random.seed((vid * 1103515245) & 0xFFFFFFFF)

    # initialize LEDs
    robot.set_led(255, 40, 40)  # pink-red glitch idle

    # ----- Phase 1: measure initial radius -----
    pose = safe_pose(robot)
    if not pose:
        robot.set_vel(0,0); robot.delay(40)
        pose = safe_pose(robot)
    if not pose:
        robot.set_vel(0,0)
        return

    x0,y0,_ = pose
    initial_r = math.hypot(x0-CX, y0-CY)
    final_r = initial_r + RING_EXPANSION

    # Random number of bursts
    num_segments = random.randint(3, 6)

    # Divide final expansion into random burst sizes
    # that sum EXACTLY to RING_EXPANSION
    raw = [random.uniform(0.8, 1.2) for _ in range(num_segments)]
    s = sum(raw)
    segments = [(RING_EXPANSION * r/s) for r in raw]

    current_segment = 0
    seg_target_r = initial_r + segments[0]

    start_time = robot.get_clock()
    next_delay = random.uniform(1.0, 5.0)

    last_flicker = 0.0
    in_burst = False

    while True:
        pose = safe_pose(robot)
        if not pose:
            robot.set_vel(0,0); robot.delay(40); continue

        x,y,th = pose

        # >>> NEW: print live bot position to terminal <<<
        try:
            print("Bot {} @ x={:.3f}, y={:.3f}, th={:.3f}".format(
                robot.id, x, y, th))
        except:
            print("Bot", robot.id, "@", x, y, th)
        # >>> END NEW <<<
        now = robot.get_clock()

        # ---------------- boundary kill ----------------
        if is_critical(x,y):
            robot.set_vel(0,0)
            robot.set_led(255,0,0)
            robot.delay(40)
            return

        # ---------------- LED flicker chaos ----------------
        if now - last_flicker > random.uniform(0.18, 0.45):
            r = random.random()
            if r < 0.3: robot.set_led(255,0,0)
            elif r < 0.6: robot.set_led(255,255,255)
            else: robot.set_led(150,20,30)
            last_flicker = now

        # ---------------- movement done? ----------------
        current_r = math.hypot(x-CX, y-CY)
        if current_r >= final_r - 0.005:
            # micro alignment correction so ring looks perfect
            dx, dy = x-CX, y-CY
            target_hdg = math.atan2(dy, dx)
            err = wrap(target_hdg - th)
            turn = clamp(TURN_K*err, -1.5,1.5)
            robot.set_vel(int(-turn*10), int(turn*10))
            robot.delay(80)

            robot.set_vel(0,0)
            robot.set_led(80,255,120)   # mint green resolved
            robot.delay(40)
            continue

        # ---------------- start next burst ----------------
        if not in_burst and (now - start_time) >= next_delay:
            in_burst = True
            robot.set_led(255,180,0)

        # ---------------- bursting ----------------
        if in_burst:
            if current_r >= seg_target_r:
                # finish segment
                in_burst = False
                current_segment += 1
                robot.set_led(100,255,100)

                if current_segment >= len(segments):
                    # final ring alignment
                    continue

                # next segment target
                seg_target_r = initial_r + sum(segments[:current_segment+1])

                # random delay until next burst
                start_time = now
                next_delay = random.uniform(0.8, 5.0)
                robot.delay(random.randint(70,160))
                robot.set_vel(0,0)
                continue

            # move outward radially
            dx = x - CX
            dy = y - CY
            hdg = math.atan2(dy, dx)
            err = wrap(hdg - th)
            turn = clamp(TURN_K*err, -1.5,1.5)

            left  = clamp(int(MAX_WHEEL * (SPEED - turn)), -MAX_WHEEL, MAX_WHEEL)
            right = clamp(int(MAX_WHEEL * (SPEED + turn)), -MAX_WHEEL, MAX_WHEEL)
            robot.set_vel(left, right)

        else:
            # twitch in idle
            if random.random() < 0.06:
                tw = random.choice([-18,18])
                robot.set_vel(tw, -tw)
                robot.delay(60)
            robot.set_vel(0,0)

        robot.delay(40)