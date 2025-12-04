# -*- coding: utf-8 -*-
# Double-ring restore script for Coachbots (Python 2.7)
# Hard-coded inner radius 0.53 and outer radius 0.83
# Dancer center (-0.1, 0.475)
# Collision-safe ring restoration using flocking + per-ring equal spacing.

from __future__ import division
import math, struct, os, random

# --------------------------------------------
# Logging (lab-safe)
# --------------------------------------------
log = None
log_out = None

def logw(msg):
    global log, log_out
    s = str(msg)
    if not s.endswith("\n"):
        s += "\n"
    if log:
        try:
            log.write(s)
            log.flush()
            os.fsync(log.fileno())
        except:
            pass
    if log_out:
        try:
            log_out.write(s)
            log_out.flush()
            os.fsync(log_out.fileno())
        except:
            pass

# --------------------------------------------
# Field
# --------------------------------------------
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

CENTER_X = -0.1
CENTER_Y = 0.475

R_INNER = 0.53
R_OUTER = 0.83

# --------------------------------------------
# Control
# --------------------------------------------
MAX_WHEEL = 35
TURN_K = 3.0
DT_MS = 40

FWD = 0.55
FWD_MIN = 0.08

K_POS = 2.1
K_RING = 0.9
K_SEP = 0.28

SEP_RADIUS = 0.25
NEIGH_RADIUS = 0.75

SOFT_MARGIN = 0.08
CRIT_MARGIN = 0.02
SOFT_MAX_F = 0.35

# --------------------------------------------
# Heartbeat
# --------------------------------------------
HB_FMT = "fffffi"
HB_BYTES = struct.calcsize(HB_FMT)
HB_DT = 0.12
STALE_S = 1.0

# --------------------------------------------
# Helpers
# --------------------------------------------
def clamp(v, lo, hi):
    if v < lo: return lo
    if v > hi: return hi
    return v

def wrap(a):
    while a > math.pi: a -= 2.0*math.pi
    while a <= -math.pi: a += 2.0*math.pi
    return a

def safe_pose(robot):
    p = robot.get_pose()
    if p and len(p) >= 3:
        return float(p[0]), float(p[1]), float(p[2])
    return None

def boundary_state(x, y):
    if (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
        y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN):
        return 2
    if (x < X_MIN + SOFT_MARGIN or x > X_MAX - SOFT_MARGIN or
        y < Y_MIN + SOFT_MARGIN or y > Y_MAX - SOFT_MARGIN):
        return 1
    return 0

def soft_boundary_force(x, y):
    fx = fy = 0.0
    if x < X_MIN + SOFT_MARGIN:
        fx += SOFT_MAX_F * (1 - (x - X_MIN) / SOFT_MARGIN)
    elif x > X_MAX - SOFT_MARGIN:
        fx -= SOFT_MAX_F * (1 - (X_MAX - x) / SOFT_MARGIN)

    if y < Y_MIN + SOFT_MARGIN:
        fy += SOFT_MAX_F * (1 - (y - Y_MIN) / SOFT_MARGIN)
    elif y > Y_MAX - SOFT_MARGIN:
        fy -= SOFT_MAX_F * (1 - (Y_MAX - y) / SOFT_MARGIN)

    return fx, fy

# --------------------------------------------
# Main
# --------------------------------------------
def usr(robot):
    global log, log_out

    try:
        log = open("experiment_log.txt", "a", 0)
    except:
        log = None
    try:
        log_out = open("/home/pi/experiment_output", "a", 0)
    except:
        log_out = None

    robot.delay(300)

    try:
        vid = int(robot.id)
    except:
        vid = 0

    neighbors = {}
    last_seen = {}
    last_hb = -1e9
    lastL = lastR = 0

    # wake localization
    robot.set_vel(20,20)
    robot.delay(150)
    robot.set_vel(0,0)
    robot.delay(150)

    logw("RESTORE_RINGS start vid=%d" % vid)

    # MAIN LOOP
    while True:
        pose = safe_pose(robot)
        if not pose:
            robot.set_vel(0,0)
            robot.delay(DT_MS)
            continue

        x, y, th = pose
        now = robot.get_clock()

        # boundary safety
        b = boundary_state(x,y)
        if b == 2:
            robot.set_led(255,0,0)
            robot.set_vel(0,0)
            robot.delay(DT_MS)
            continue
        elif b == 1:
            robot.set_led(255,150,0)
        else:
            robot.set_led(0,150,255)

        # heartbeats
        if now - last_hb >= HB_DT:
            robot.send_msg(struct.pack(HB_FMT,
                                       float(x), float(y), float(th),
                                       0.0, 0.0, vid))
            last_hb = now

        msgs = robot.recv_msg()
        if msgs:
            for m in msgs:
                if len(m) >= HB_BYTES:
                    nx,ny,nth,nvx,nvy,nid = struct.unpack(HB_FMT, m[:HB_BYTES])
                    nid = int(nid)
                    if nid != vid:
                        neighbors[nid] = (nx,ny,nth,nvx,nvy)
                        last_seen[nid] = now

        # prune stale
        cutoff = now - STALE_S
        for nid in list(neighbors.keys()):
            if last_seen.get(nid,0) < cutoff:
                neighbors.pop(nid, None)
                last_seen.pop(nid, None)

        # -------------------------------------------------
        # Compute which bots belong to inner vs outer ring
        # -------------------------------------------------
        all_ids = neighbors.keys() + [vid]
        dist_list = []
        for nid in all_ids:
            if nid == vid:
                dx = x - CENTER_X
                dy = y - CENTER_Y
            else:
                nx,ny,_,_,_ = neighbors[nid]
                dx = nx - CENTER_X
                dy = ny - CENTER_Y
            dist_list.append((nid, math.hypot(dx,dy)))

        dist_list.sort(key=lambda t: t[1])
        half = len(dist_list)//2

        inner_ids = [d[0] for d in dist_list[:half]]
        outer_ids = [d[0] for d in dist_list[half:]]

        # -------------------------------------------------
        # Compute target angle ordering for equal spacing
        # -------------------------------------------------
        def compute_angle_of(nid):
            if nid == vid:
                ax = x - CENTER_X
                ay = y - CENTER_Y
            else:
                nx,ny,_,_,_ = neighbors[nid]
                ax = nx - CENTER_X
                ay = ny - CENTER_Y
            return math.atan2(ay, ax)

        inner_sorted = sorted(inner_ids, key=compute_angle_of)
        outer_sorted = sorted(outer_ids, key=compute_angle_of)

        # pick this robot's ring and index
        if vid in inner_sorted:
            ring = "inner"
            ids = inner_sorted
            R = R_INNER
        else:
            ring = "outer"
            ids = outer_sorted
            R = R_OUTER

        idx = ids.index(vid)
        angle = 2.0*math.pi * idx / max(1, len(ids))

        # target
        tx = CENTER_X + R * math.cos(angle)
        ty = CENTER_Y + R * math.sin(angle)

        # desired tangent heading
        th_goal = angle + math.pi/2.0

        # -------------------------------------------------
        # Flocking + ring correction
        # -------------------------------------------------
        bfx, bfy = soft_boundary_force(x, y)

        # spacing correction
        vx = bfx + K_POS*(tx - x)
        vy = bfy + K_POS*(ty - y)

        # simple repulsion
        repx=repy=0.0
        for nid,(nx,ny,nth,nvx,nvy) in neighbors.items():
            dxn = x - nx
            dyn = y - ny
            d2 = dxn*dxn + dyn*dyn
            if d2>1e-12:
                d = math.sqrt(d2)
                if d < SEP_RADIUS:
                    s = K_SEP*(SEP_RADIUS-d)/SEP_RADIUS
                    repx += s*(dxn/d)
                    repy += s*(dyn/d)

        vx += repx
        vy += repy

        # -------------------------------------------------
        # Wheel control
        # -------------------------------------------------
        hdg = math.atan2(vy, vx)
        err = wrap(hdg - th)

        fwd = FWD
        if b==1:
            fwd *= 0.7
        if fwd < FWD_MIN:
            fwd = FWD_MIN

        turn = clamp(TURN_K * err, -1.5, 1.5)

        L = int(MAX_WHEEL*(fwd - 0.75*turn))
        Rw = int(MAX_WHEEL*(fwd + 0.75*turn))

        robot.set_vel(L, Rw)
        robot.delay(DT_MS)


