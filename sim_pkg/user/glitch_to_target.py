# -*- coding: utf-8 -*-
import math, random, csv, os

# ===== arena =====
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

# ===== drive / control =====
MAX_WHEEL = 35
TURN_K    = 3.0
FWD_FAST  = 0.8
FWD_SLOW  = 0.30
EPS       = 1e-3

# ===== boundary softness =====
SOFT_MARGIN     = 0.08
CRIT_MARGIN     = 0.02
SOFT_MAX_FORCE  = 0.35

# ===== dancer obstacle =====
FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET   # ≈0.1524
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
OBST_CX, OBST_CY = (-0.1, 0.475)

# ===== neighbor + noise (same "glitch" feel) =====
REPULSE_RADIUS  = 0.75
REPULSE_GAIN    = 0.12
NOISE_GAIN      = 0.12
FWD_GAIN        = 0.95
LEFT_BIAS_VX    = 0.00   # optional slow drift

# ===== targets (id, x, y, theta) =====
TARGETS = [
    (0, 0.5078551660273687, -1.0812290217145453, 0.9990202882509482),
    (1, 0.13482491407732633, 0.6043453180590512, 1.7637217158701084),
    (2, 0.19445884973244154, 1.9098751381193253, -1.5914721270335777),
    (3, -0.49845699065487636, 0.651699144665435, -1.569300771936653),
    (4, -0.6464343869375285, -0.5612916266020493, 0.8665055995553601),
    (5, -0.3623178566471784, -0.17672454197079146, 1.370279307855518),
    (6, 0.8105872562011283, -1.120767448066587, 2.3202675408100557),
    (7, -0.8691824340609033, 2.008201591431108, 3.009576639866528),
    (8, 0.36633176585834737, -1.0087409862924022, 1.2428304282449938),
    (9, 0.09688731022063135, 2.16907250173348, -0.4933500588987916),
    (10, 0.45096860980149067, 1.9716893256443195, 0.978117584544111),
]

# set to True to load a CSV with rows: id,x,y,theta
LOAD_CSV = False
CSV_PATH = "init_pose_near_dancer.csv"

# ===== target attraction (new) =====
# gentle spring toward (xt,yt), saturate so it never kills the "glitch" motion
K_TARGET     = 0.35       # attraction gain (m/s per m of error)
MAX_SEEK     = 0.25       # cap for |target velocity| contribution
ARRIVE_RAD   = 0.10       # consider “arrived” inside this radius
HOLD_SPRING  = 0.10       # small spring after arrival to hold position while still glitching
ALIGN_AT_END = True       # face target theta once arrived (optional)

PRINT_PERIOD = 2.0

# ----------------- helpers -----------------
def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

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

def try_get_swarm_poses(robot):
    for nm in ('get_swarm_poses', 'get_all_poses', 'get_poses', 'swarm_poses'):
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

def load_targets():
    if LOAD_CSV and os.path.exists(CSV_PATH):
        tgt = []
        with open(CSV_PATH, "r") as f:
            for row in csv.reader(f):
                if not row: continue
                try:
                    rid = int(row[0]); x = float(row[1]); y = float(row[2])
                    th = float(row[3]) if len(row) > 3 else 0.0
                    tgt.append((rid, x, y, th))
                except:
                    pass
        if tgt: return tgt
    return TARGETS

# ----------------- main -----------------
def usr(robot):
    robot.delay(2000)
    my_id = get_id(robot)
    robot.set_led(0, 180, 180)
    print(f"Robot {my_id} starting GLITCH → TARGETS")

    # build target map: id -> (xt, yt, ttheta)
    tgt_map = {rid: (x, y, th) for (rid, x, y, th) in load_targets()}
    if my_id not in tgt_map:
        print(f"Robot {my_id}: no target specified; free-glitching")
    xt, yt, ttheta = tgt_map.get(my_id, (None, None, None))

    start_time = robot.get_clock()
    last_print = start_time
    told_no_swarm_api = False
    arrived = False

    while True:
        pose = safe_pose(robot)
        if pose is None:
            robot.set_vel(0, 0)
            robot.delay(20)
            continue
        x, y, th = pose

        # 1) emergency stops
        if is_critical_obstacle(x, y, 0.0) or is_critical_boundary(x, y):
            robot.set_vel(0, 0)
            robot.set_led(255, 50, 0)
            robot.delay(40)
            continue

        # 2) base forces (as in glitch)
        bfx, bfy = soft_boundary_force(x, y)
        boundary_warn = (abs(bfx) + abs(bfy)) > 1e-6
        if boundary_warn: robot.set_led(255, 150, 0)
        else:             robot.set_led(0, 180, 180)

        vx = bfx + LEFT_BIAS_VX
        vy = bfy

        obx, oby = soft_obstacle_force(x, y)
        vx += obx; vy += oby

        neighbors = try_get_swarm_poses(robot)
        if neighbors:
            for item in neighbors:
                if isinstance(item, (list, tuple)) and len(item) >= 3:
                    if len(item) == 4:
                        nid, nx, ny, nth = item
                    else:
                        nx, ny, nth = item[0], item[1], item[2]
                        nid = None
                    if (nid is not None) and (str(nid) == str(my_id)): continue
                    dx, dy = x - nx, y - ny
                    r2 = dx*dx + dy*dy
                    if 1e-5 < r2 < (REPULSE_RADIUS*REPULSE_RADIUS):
                        s = REPULSE_GAIN / r2
                        vx += s * dx; vy += s * dy
        else:
            if not told_no_swarm_api:
                print(f"Robot {my_id}: no swarm pose API; using jitter only")
                told_no_swarm_api = True

        # 3) add jitter (the 'glitch' look)
        ang1 = random.uniform(-math.pi, math.pi)
        ang2 = random.uniform(-math.pi, math.pi)
        vx += NOISE_GAIN * math.cos(ang1) + 0.6 * NOISE_GAIN * math.cos(ang2)
        vy += NOISE_GAIN * math.sin(ang1) + 0.6 * NOISE_GAIN * math.sin(ang2)

        # 4) goal attraction (new)
        if xt is not None:
            tx, ty = xt, yt
            ex, ey = tx - x, ty - y
            dist = math.hypot(ex, ey)

            if not arrived:
                if dist < ARRIVE_RAD:
                    arrived = True
                    robot.set_led(0, 255, 0)  # green when reached
                # spring toward target, but saturate so it doesn't dominate
                gx = K_TARGET * ex
                gy = K_TARGET * ey
                gnorm = math.hypot(gx, gy)
                if gnorm > MAX_SEEK:
                    scale = MAX_SEEK / max(gnorm, 1e-6)
                    gx *= scale; gy *= scale
                vx += gx; vy += gy
            else:
                # arrived: keep tiny spring so they hover around the point
                vx += HOLD_SPRING * ex
                vy += HOLD_SPRING * ey
        else:
            dist = float('inf')

        # 5) map (vx,vy) -> wheels
        if abs(vx) + abs(vy) < EPS:
            robot.set_vel(0, 0)
        else:
            hdg = math.atan2(vy, vx)
            err = wrap_angle(hdg - th)

            ae = abs(err)
            if ae < 0.5:    fwd = FWD_FAST * FWD_GAIN
            elif ae < 1.2:  fwd = FWD_FAST * 0.7 * FWD_GAIN
            else:           fwd = FWD_SLOW * 0.6 * FWD_GAIN

            if boundary_warn: fwd *= 0.7

            # if aligned-at-end requested, gently rotate toward final theta when arrived
            if ALIGN_AT_END and arrived and (ttheta is not None):
                # blend heading control a bit more when close
                err_hold = wrap_angle(ttheta - th)
                err = 0.5*err + 0.5*err_hold

            turn = clamp(TURN_K * err, -1.5, 1.5)
            left  = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
            right = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
            robot.set_vel(left, right)

        # 6) light console output
        now = robot.get_clock()
        if now - last_print > PRINT_PERIOD:
            s = f"Robot {my_id} pos[{x:.3f},{y:.3f}]"
            if xt is not None:
                s += f" → tgt[{xt:.3f},{yt:.3f}] d={dist:.3f} {'ARRIVED' if arrived else ''}"
            print(s)
            last_print = now

        robot.delay(40)
