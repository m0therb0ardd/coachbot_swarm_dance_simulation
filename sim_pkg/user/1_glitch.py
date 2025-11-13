
# -*- coding: utf-8 -*-
import math
import random

# --- field bounds (meters) ---
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

# --- drive / control ---
MAX_WHEEL = 35
TURN_K    = 3.0
FWD_FAST  = 0.8
FWD_SLOW  = 0.30
EPS       = 1e-3

# --- boundary softness ---
SOFT_MARGIN     = 0.08
CRIT_MARGIN     = 0.02
SOFT_MAX_FORCE  = 0.35

# --- jittery repulsion params ---
REPULSE_RADIUS  = 0.75 # was .35
REPULSE_GAIN    = 0.12 
NOISE_GAIN      = 0.12 #was .25
FWD_GAIN        = 0.95
LEFT_BIAS_VX    = 0.00  # set to -0.06 for slow drift left

# dancer no-go circle (meters)
FEET = 0.6048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET   # ~0.1524
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
OBST_CX, OBST_CY = (-0.1, 0.475)

# ----------------- helpers -----------------

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def wrap_angle(a):
    while a >  math.pi: a -= 2.0*math.pi
    while a <= -math.pi: a += 2.0*math.pi
    return a

def safe_pose(robot):
    p = robot.get_pose()
    if p and len(p) >= 3:
        return float(p[0]), float(p[1]), float(p[2])
    return None

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

def is_critical_boundary(x, y):
    return (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
            y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN)

def try_get_swarm_poses(robot):
    names = ('get_swarm_poses', 'get_all_poses', 'get_poses', 'swarm_poses')
    for nm in names:
        fn = getattr(robot, nm, None)
        if callable(fn):
            try:
                poses = fn()
                if poses:
                    return poses
            except:
                pass
    return []

def get_id(robot):
    vid_attr = getattr(robot, "virtual_id", None)
    try:
        return vid_attr() if callable(vid_attr) else int(vid_attr or 0)
    except:
        return -1

def soft_obstacle_force(x, y, max_force=0.6, buffer_width=0.10):
    dx, dy = x - OBST_CX, y - OBST_CY
    r = math.hypot(dx, dy)
    if r < SAFE_BUBBLE + buffer_width:
        if r < 1e-6:
            return max_force, 0.0
        strength = max(0.0, (SAFE_BUBBLE + buffer_width - r) / buffer_width)
        s = max_force * strength
        return s * (dx / r), s * (dy / r)
    return 0.0, 0.0

def is_critical_obstacle(x, y, critical_margin=0.0):
    dx, dy = x - OBST_CX, y - OBST_CY
    r = math.hypot(dx, dy)
    return r < (OBST_RADIUS + critical_margin)

# ----------------- main -----------------

def usr(robot):
    robot.delay(3000)
    my_id = get_id(robot)
    robot.set_led(0, 180, 180)
    print("Robot %s starting JITTERY REPULSION" % str(my_id))

    start_time = robot.get_clock()
    last_print = start_time
    told_no_swarm_api = False

    while True:
        pose = safe_pose(robot)
        if pose is None:
            robot.set_vel(0, 0)
            robot.delay(20)
            continue
        x, y, th = pose

        # 1) emergency stop if inside the dancer disk
        if is_critical_obstacle(x, y, 0.0):
            robot.set_vel(0, 0)
            robot.set_led(255, 50, 0)
            print("ROBOT %s OBSTACLE STOP at [%.3f, %.3f]" % (str(my_id), x, y))
            robot.delay(40)
            continue

        # 2) emergency boundary hold
        if is_critical_boundary(x, y):
            robot.set_vel(0, 0)
            robot.set_led(255, 50, 0)
            print("ROBOT %s CRITICAL STOP at [%.3f, %.3f]" % (str(my_id), x, y))
            robot.delay(40)
            continue

        # 3) base velocity from soft boundary + optional left bias
        bfx, bfy = soft_boundary_force(x, y)
        boundary_warn = (abs(bfx) + abs(bfy)) > 1e-6
        if boundary_warn: robot.set_led(255, 150, 0)
        else:             robot.set_led(0, 180, 180)

        vx = bfx + LEFT_BIAS_VX
        vy = bfy

        # 4) add soft obstacle repulsion
        obx, oby = soft_obstacle_force(x, y)
        vx += obx
        vy += oby

        # 5) add neighbor repulsion if API exists
        neighbors = try_get_swarm_poses(robot)
        if neighbors:
            for item in neighbors:
                if isinstance(item, (list, tuple)) and len(item) >= 3:
                    if len(item) == 4:
                        nid, nx, ny, nth = item
                    else:
                        nx, ny, nth = item[0], item[1], item[2]
                        nid = None
                    if (nid is not None) and (str(nid) == str(my_id)):
                        continue
                    dx, dy = x - nx, y - ny
                    r2 = dx*dx + dy*dy
                    if 1e-5 < r2 < (REPULSE_RADIUS*REPULSE_RADIUS):
                        inv = 1.0 / r2
                        s = REPULSE_GAIN * inv
                        vx += s * dx
                        vy += s * dy
        else:
            if not told_no_swarm_api:
                print("Robot %s: no swarm pose API; using jitter fallback" % str(my_id))
                told_no_swarm_api = True

        # 6) always add jitter to feel glitchy
        ang1 = random.uniform(-math.pi, math.pi)
        ang2 = random.uniform(-math.pi, math.pi)
        vx += NOISE_GAIN * math.cos(ang1) + 0.6 * NOISE_GAIN * math.cos(ang2)
        vy += NOISE_GAIN * math.sin(ang1) + 0.6 * NOISE_GAIN * math.sin(ang2)

        # 7) map (vx,vy) -> differential drive
        if abs(vx) + abs(vy) < EPS:
            robot.set_vel(0, 0)
        else:
            hdg = math.atan2(vy, vx)
            err = wrap_angle(hdg - th)

            ae = abs(err)
            if ae < 0.5:    fwd = FWD_FAST * FWD_GAIN
            elif ae < 1.2:  fwd = FWD_FAST * 0.7 * FWD_GAIN
            else:           fwd = FWD_SLOW * 0.6 * FWD_GAIN

            if boundary_warn:
                fwd *= 0.7

            turn = clamp(TURN_K * err, -1.5, 1.5)
            left  = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
            right = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
            robot.set_vel(left, right)

        # 8) light console output without spamming
        now = robot.get_clock()
        if now - last_print > 2.0:
            print("Robot %s pos [%.3f, %.3f]" % (str(my_id), x, y))
            last_print = now

        robot.delay(40)
