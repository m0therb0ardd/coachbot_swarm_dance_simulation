# -*- coding: utf-8 -*-
# Two-Ring Encircle: Adaptive assignment based on initial positions
# Modified to write a CSV log of poses: vid, t, x, y, th

import math, struct, random, os, csv

# ----------------- Logging & CSV ​----------------
log     = None
log_out = None
csv_file = None
csv_writer = None

def logw(msg):
    global log, log_out
    try:
        s = str(msg)
    except:
        s = repr(msg)
    if not s.endswith("\n"):
        s = s + "\n"
    if log is None:
        try:
            log = open("experiment_log.txt", "a", 0)
        except:
            log = None
    if log is not None:
        try:
            log.write(s)
            log.flush()
        except:
            pass
    if log_out is None:
        try:
            log_out = open("/home/pi/experiment_output", "a", 0)
        except:
            log_out = None
    if log_out is not None:
        try:
            log_out.write(s)
            log_out.flush()
        except:
            pass
    try:
        print(s.rstrip("\n"))
    except:
        pass

def init_csv():
    global csv_file, csv_writer
    try:
        csv_file = open("poses_log.csv", "w", newline='')
        csv_writer = csv.writer(csv_file)
        # write header
        csv_writer.writerow(["vid", "t", "x", "y", "th"])
    except Exception as e:
        logw("ERROR opening CSV for writing: %s" % repr(e))
        csv_file = None
        csv_writer = None

def write_pose_csv(vid, t, x, y, th):
    global csv_writer
    if csv_writer is not None:
        csv_writer.writerow([vid, "%.6f" % t, "%.6f" % x, "%.6f" % y, "%.6f" % th])

# ----------------- Arena & params ----------------
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
CX, CY      = (-0.1, 0.475)

R_INNER = SAFE_BUBBLE + 0.22
R_OUTER = SAFE_BUBBLE + 0.40
R_SPLIT_THRESHOLD = None
R_RING_TOL = 0.10

V_TANGENT_BASE = 0.26
K_R, RADIAL_CLAMP = 1.3, 0.12
MIN_LINEAR_SEP = 0.18

SOFT_MARGIN    = 0.08
CRIT_MARGIN    = 0.05
SOFT_MAX_FORCE = 0.35

MAX_WHEEL  = 35
TURN_K     = 3.0
FWD_FAST   = 0.80
FWD_SLOW   = 0.30
FWD_MIN    = 0.40
CMD_SMOOTH = 0.20
EPS        = 1e-3
DT_MS      = 40
logw_PERIOD= 2.0

HB_FMT   = 'fffi'
HB_BYTES = struct.calcsize(HB_FMT)
HB_DT    = 0.12
STALE_S   = 0.8

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def wrap(a):
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

def is_critical_boundary(x, y):
    return (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
            y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN)

def try_get_swarm_poses(robot):
    for nm in ('get_swarm_poses','get_all_poses','get_poses','swarm_poses'):
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
    if hasattr(robot, "id"):
        try:
            return int(robot.id)
        except:
            pass
    return -1

def find_natural_split(distances):
    if len(distances) < 2:
        return 0.7
    sorted_dists = sorted(distances)
    max_gap = 0
    split_idx = 0
    for i in range(1, len(sorted_dists)):
        gap = sorted_dists[i] - sorted_dists[i-1]
        if gap > max_gap:
            max_gap = gap
            split_idx = i
    if max_gap > 0.1:
        threshold = (sorted_dists[split_idx-1] + sorted_dists[split_idx]) / 2.0
        logw("Found natural gap threshold=%.3f" % threshold)
        return threshold
    median = sorted_dists[len(sorted_dists)//2]
    logw("No clear gap, using median=%.3f" % median)
    return median

# ----------------- Main behavior ----------------
def usr(robot):
    global log, log_out
    init_csv()
    try:
        log = open("experiment_log.txt", "a", 0)
    except:
        log = None

    robot.delay(800)
    rid = get_id(robot)
    logw("Bot starting ID: %d" % rid)
    random.seed((rid * 1103515245) & 0xFFFFFFFF)

    # Discovery phase etc …
    # [Insert your existing ring assignment code unchanged here]
    # Make sure you keep the discovery, assignments, etc.

    # After discovery, move into main loop:
    while True:
        pose = safe_pose(robot)
        if not pose:
            robot.set_vel(0, 0)
            robot.delay(DT_MS)
            continue

        x, y, th = pose
        now     = robot.get_clock()

        # --- write CSV row ---
        write_pose_csv(rid, now, x, y, th)

        # --- existing logic continues ---
        # boundary checks, ring motion calculation, wheel commands, LED updates …

        robot.delay(DT_MS)

# End of file
