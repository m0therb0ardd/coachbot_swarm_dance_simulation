# # -*- coding: utf-8 -*-
# import math

# # ===== Arena =====
# X_MIN, X_MAX = -1.2, 1.0
# Y_MIN, Y_MAX = -1.4, 2.35

# # ===== Dancer no-go circle =====
# FEET = 0.3048
# OBST_DIAM_FT = 1.0
# OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
# OBST_MARGIN  = 0.03
# SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
# CX, CY = (-0.1, 0.475)

# # ===== Dual-ring parameters =====
# # Inner ring: tighter & CCW; Outer ring: larger & CW
# R_INNER    = SAFE_BUBBLE + 0.24
# R_OUTER    = SAFE_BUBBLE + 0.42
# DIR_INNER  = +1     # +1 = CCW
# DIR_OUTER  = -1     # -1 = CW

# # Split policy: "by_id" (even/odd virtual_id) or "by_initial_radius"
# ASSIGN_MODE = "by_id"

# # ===== Speeds & gains (shared) =====
# V_TANGENT_BASE = 0.26
# K_R            = 1.2
# RADIAL_CLAMP   = 0.10

# # ===== Angular spacing (within each ring) =====
# ANG_REP_GAIN   = 0.24
# ANG_REP_POW    = 1.2
# ANG_REP_CUTOFF = 1.2
# MIN_LINEAR_SEP = 0.18

# # ===== Boundary cushion =====
# SOFT_MARGIN     = 0.08
# CRIT_MARGIN     = 0.02
# SOFT_MAX_FORCE  = 0.35

# # ===== Drive model =====
# MAX_WHEEL = 35
# TURN_K    = 3.0
# FWD_FAST  = 0.8
# FWD_SLOW  = 0.30
# FWD_MIN   = 0.40
# EPS       = 1e-3
# PRINT_PERIOD = 2.0

# # ---------- helpers ----------
# def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

# def wrap_angle(a):
#     while a >  math.pi: a -= 2.0*math.pi
#     while a <= -math.pi: a += 2.0*math.pi
#     return a

# def safe_pose(robot):
#     p = robot.get_pose()
#     if p and len(p) >= 3:
#         return float(p[0]), float(p[1]), float(p[2])
#     return None

# def soft_boundary_force(x, y):
#     fx = fy = 0.0
#     if x < X_MIN + SOFT_MARGIN:
#         fx += SOFT_MAX_FORCE * (1.0 - (x - X_MIN)/SOFT_MARGIN)
#     elif x > X_MAX - SOFT_MARGIN:
#         fx -= SOFT_MAX_FORCE * (1.0 - (X_MAX - x)/SOFT_MARGIN)
#     if y < Y_MIN + SOFT_MARGIN:
#         fy += SOFT_MAX_FORCE * (1.0 - (y - Y_MIN)/SOFT_MARGIN)
#     elif y > Y_MAX - SOFT_MARGIN:
#         fy -= SOFT_MAX_FORCE * (1.0 - (Y_MAX - y)/SOFT_MARGIN)
#     return fx, fy

# def is_critical_boundary(x, y):
#     return (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
#             y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN)

# def try_get_swarm_poses(robot):
#     for nm in ('get_swarm_poses','get_all_poses','get_poses','swarm_poses'):
#         fn = getattr(robot, nm, None)
#         if callable(fn):
#             try:
#                 poses = fn()
#                 if poses: return poses
#             except: pass
#     return []

# def get_vid(robot):
#     try: return robot.virtual_id()
#     except: return -1

# # which ring does (x,y) belong to if using radius-based assignment?
# def nearest_ring_radius(r):
#     return R_INNER if abs(r - R_INNER) <= abs(r - R_OUTER) else R_OUTER

# def ring_dir(ring_R):
#     return DIR_INNER if abs(ring_R - R_INNER) < abs(ring_R - R_OUTER) else DIR_OUTER

# def usr(robot):
#     robot.delay(2000)
#     rid = get_vid(robot)
#     robot.set_led(0, 180, 180)

#     # ring assignment
#     assigned_R = None
#     assigned_dir = None
#     init_done = False

#     last_print = robot.get_clock()
#     told_no_api = False

#     while True:
#         pose = safe_pose(robot)
#         if pose is None:
#             robot.set_vel(0,0); robot.delay(20); continue
#         x, y, th = pose

#         # safety
#         rdx, rdy = x - CX, y - CY
#         r = math.hypot(rdx, rdy)
#         if r < OBST_RADIUS or is_critical_boundary(x, y):
#             robot.set_vel(0,0); robot.set_led(255,50,0); robot.delay(40); continue

#         # one-time assignment
#         if not init_done:
#             if ASSIGN_MODE == "by_id":
#                 if rid % 2 == 0:
#                     assigned_R = R_INNER; assigned_dir = DIR_INNER
#                 else:
#                     assigned_R = R_OUTER; assigned_dir = DIR_OUTER
#             else:  # "by_initial_radius"
#                 assigned_R = nearest_ring_radius(r)
#                 assigned_dir = ring_dir(assigned_R)
#             init_done = True
#             print(f"Robot {rid} → ring R={assigned_R:.2f} dir={'CCW' if assigned_dir>0 else 'CW'}")

#         # polar basis
#         if r < 1e-6:
#             urx, ury = 1.0, 0.0
#         else:
#             urx, ury = rdx/r, rdy/r
#         utx, uty = -ury, urx
#         if assigned_dir < 0: utx, uty = -utx, -uty

#         # base tangent (never zero)
#         vx = V_TANGENT_BASE * utx
#         vy = V_TANGENT_BASE * uty

#         # small radial correction to hold assigned ring
#         radial = clamp(K_R * (assigned_R - r), -RADIAL_CLAMP, RADIAL_CLAMP)
#         vx += radial * urx
#         vy += radial * ury

#         # ANGULAR SPACING *within your ring only*
#         neighbors = try_get_swarm_poses(robot)
#         if neighbors:
#             theta = math.atan2(rdy, rdx)
#             for item in neighbors:
#                 if isinstance(item,(list,tuple)) and len(item)>=3:
#                     if len(item)==4: nid, nx, ny, nth = item
#                     else: nx, ny, nth = item[0], item[1], item[2]

#                     # neighbor's ring choice (mirror our policy)
#                     nr = math.hypot(nx - CX, ny - CY)
#                     nR = None
#                     if ASSIGN_MODE == "by_id" and len(item)==4:
#                         nR = R_INNER if (int(nid) % 2 == 0) else R_OUTER
#                     else:
#                         nR = nearest_ring_radius(nr)

#                     # only repel tangentially from neighbors on same ring
#                     if abs(nR - assigned_R) < abs(R_OUTER - R_INNER)/2.0:
#                         # linear min-sep (small radial buffer)
#                         ddx, ddy = x - nx, y - ny
#                         d2 = ddx*ddx + ddy*ddy
#                         if d2 < MIN_LINEAR_SEP*MIN_LINEAR_SEP:
#                             s = (MIN_LINEAR_SEP*MIN_LINEAR_SEP - d2) / (MIN_LINEAR_SEP*MIN_LINEAR_SEP)
#                             vx += s * urx; vy += s * ury

#                         ntheta = math.atan2(ny - CY, nx - CX)
#                         dtheta = wrap_angle(theta - ntheta)
#                         ad = abs(dtheta)
#                         if 1e-3 < ad <= ANG_REP_CUTOFF:
#                             strength = ANG_REP_GAIN / (ad ** ANG_REP_POW)
#                             tang_push = strength * (1.0 if dtheta>0 else -1.0)
#                             vx += tang_push * utx
#                             vy += tang_push * uty
#         else:
#             if not told_no_api:
#                 print(f"Robot {rid}: no swarm pose API; angular spacing limited")
#                 told_no_api = True

#         # boundary cushion projected radially
#         bfx, bfy = soft_boundary_force(x, y)
#         b_norm = bfx*urx + bfy*ury
#         vx += b_norm * urx
#         vy += b_norm * ury

#         # LEDs
#         near_soft = abs(b_norm) > 1e-6
#         on_ring = abs(assigned_R - r) < 0.04
#         if near_soft:          robot.set_led(255,150,0)
#         elif on_ring:          robot.set_led(0,255,0)
#         else:                  robot.set_led(0,180,180)

#         # map (vx,vy) -> wheels
#         spd = math.hypot(vx, vy)
#         if spd < EPS:
#             vx += 0.08 * utx; vy += 0.08 * uty; spd = math.hypot(vx, vy)

#         hdg = math.atan2(vy, vx)
#         err = wrap_angle(hdg - th)

#         ae = abs(err)
#         if ae < 0.5:    fwd = FWD_FAST
#         elif ae < 1.2:  fwd = FWD_FAST * 0.7
#         else:           fwd = FWD_SLOW
#         fwd = max(fwd, FWD_MIN)
#         if near_soft: fwd *= 0.85

#         turn = clamp(TURN_K * err, -1.5, 1.5)
#         left  = clamp(int(MAX_WHEEL*0.9*(fwd - 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
#         right = clamp(int(MAX_WHEEL*0.9*(fwd + 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
#         robot.set_vel(left, right)

#         now = robot.get_clock()
#         if now - last_print > PRINT_PERIOD:
#             print(f"Robot {rid} R*={assigned_R:.2f} r={r:.3f} pos[{x:.3f},{y:.3f}] spd={spd:.3f}")
#             last_print = now

#         robot.delay(40)

# -*- coding: utf-8 -*-
# Dual-Ring Encircle (SIM, P2P): split swarm into two counter-rotating rings (inner=CCW, outer=CW)
# Even angular spacing on each ring via P2P heartbeats (no global coordinator).
# SIM: uses robot.id() and print(). To port to hardware, swap id()->virtual_id(), print->log.write.

# -*- coding: utf-8 -*-
# Dual-Ring Encircle (SIM robust): inner=CCW, outer=CW, even spacing.
# Uses P2P heartbeats if available; otherwise auto-falls back to simulator swarm-poses.
# Also ignores robot.id() if IDs are not unique and splits by angle instead.

import math, struct, random

# ===== Arena =====
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

# ===== Dancer no-go circle =====
FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
CX, CY = (-0.1, 0.475)

# ===== Two rings =====
R_INNER, DIR_INNER = SAFE_BUBBLE + 0.24, +1   # CCW
R_OUTER, DIR_OUTER = SAFE_BUBBLE + 0.42, -1   # CW
SAME_RING_TOL = 0.12

# ===== Motion / spacing =====
V_TANGENT_BASE = 0.26
K_R, RADIAL_CLAMP = 1.3, 0.12
ANG_REP_GAIN, ANG_REP_POW, ANG_REP_CUTOFF = 0.30, 1.2, 1.4
MIN_LINEAR_SEP = 0.18

# ===== Boundary cushion =====
SOFT_MARGIN, CRIT_MARGIN, SOFT_MAX_FORCE = 0.08, 0.02, 0.35

# ===== Drive model =====
MAX_WHEEL, TURN_K = 35, 3.0
FWD_FAST, FWD_SLOW, FWD_MIN = 0.80, 0.30, 0.40
CMD_SMOOTH, EPS, DT_MS = 0.20, 1e-3, 40
PRINT_PERIOD = 2.0

# ===== Heartbeats (pose+ring) =====
HB_FMT = 'ffffii'  # x,y,theta, ring_R, ring_dir, vid
HB_BYTES = struct.calcsize(HB_FMT)
HB_DT, STALE_S = 0.12, 0.8

def clamp(v, lo, hi): return lo if v < lo else (hi if v > hi else v)
def wrap(a):
    while a >  math.pi: a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a

def safe_pose(robot):
    p = robot.get_pose()
    if p and len(p) >= 3: return float(p[0]), float(p[1]), float(p[2])
    return None

def soft_boundary_force(x, y):
    fx = fy = 0.0
    if x < X_MIN + SOFT_MARGIN:        fx += SOFT_MAX_FORCE * (1 - (x - X_MIN)/SOFT_MARGIN)
    elif x > X_MAX - SOFT_MARGIN:      fx -= SOFT_MAX_FORCE * (1 - (X_MAX - x)/SOFT_MARGIN)
    if y < Y_MIN + SOFT_MARGIN:        fy += SOFT_MAX_FORCE * (1 - (y - Y_MIN)/SOFT_MARGIN)
    elif y > Y_MAX - SOFT_MARGIN:      fy -= SOFT_MAX_FORCE * (1 - (Y_MAX - y)/SOFT_MARGIN)
    return fx, fy

def is_critical_boundary(x, y):
    return (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
            y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN)

def try_get_swarm_poses(robot):
    for nm in ('get_swarm_poses','get_all_poses','get_poses','swarm_poses'):
        fn = getattr(robot, nm, None)
        if callable(fn):
            try:
                poses = fn()
                if poses: return poses
            except: pass
    return []

# --- sim shims ---
def get_vid(robot):
    try: return int(robot.id())
    except: return 0
def logw(msg): print(msg)

def angle_split(theta):
    """Deterministic, ID-free split: upper half-plane -> inner/CCW, lower -> outer/CW."""
    if theta >= 0.0:
        return R_INNER, DIR_INNER
    else:
        return R_OUTER, DIR_OUTER

def usr(robot):
    robot.delay(800)
    rid = get_vid(robot)
    random.seed((rid if rid is not None else 0)*1103515245 & 0xFFFFFFFF)

    # assignment
    assigned_R = None
    assigned_dir = None
    init_done = False

    # neighbors: id -> (x,y,theta,R,dir,t_last)
    nbrs = {}
    last_hb = -1e9
    last_print = 0.0
    lastL = lastR = 0

    # detection flags
    p2p_ever_seen = False         # did we ever receive any heartbeat?
    ids_seen = set()

    # nudge localization
    robot.set_vel(20,20); robot.delay(150)

    while True:
        pose = safe_pose(robot)
        if not pose:
            robot.set_vel(0,0); robot.delay(DT_MS); continue
        x,y,th = pose
        now = robot.get_clock()

        # safety
        rdx, rdy = x - CX, y - CY
        r = math.hypot(rdx, rdy)
        if r < OBST_RADIUS or is_critical_boundary(x, y):
            robot.set_led(100,0,0); robot.set_vel(0,0); robot.delay(DT_MS); continue

        theta = math.atan2(rdy, rdx)

        # one-time assignment (robust):
        # 1) try ID split if IDs look unique; 2) else use angle split
        if not init_done:
            # try to guess uniqueness: in many sims all ids==0; we guard for that
            unique_id = (rid is not None and rid != 0)
            if unique_id:
                if rid % 2 == 0: assigned_R, assigned_dir = R_INNER, DIR_INNER
                else:            assigned_R, assigned_dir = R_OUTER, DIR_OUTER
            else:
                assigned_R, assigned_dir = angle_split(theta)
            init_done = True
            logw(f"[dual_ring_robust] id={rid} -> R={assigned_R:.2f} dir={'CCW' if assigned_dir>0 else 'CW'} (split={'id' if unique_id else 'angle'})")

        # unit frames
        if r < 1e-6: urx, ury = 1.0, 0.0
        else:        urx, ury = rdx/r, rdy/r
        utx, uty = -ury, urx
        if assigned_dir < 0: utx, uty = -utx, -uty

        # --- send heartbeat (pose+ring) ---
        if now - last_hb >= HB_DT:
            try:
                robot.send_msg(struct.pack(HB_FMT, float(x), float(y), float(th),
                                           float(assigned_R), int(assigned_dir), int(rid)))
            except: pass
            last_hb = now

        # --- receive heartbeats (if sim wires them up) ---
        msgs = robot.recv_msg() or []
        for m in msgs:
            try:
                nx, ny, nth, nR, ndir, nid = struct.unpack(HB_FMT, m[:HB_BYTES])
                if int(nid) != rid:
                    nbrs[int(nid)] = (float(nx), float(ny), float(nth), float(nR), int(ndir), now)
                    ids_seen.add(int(nid))
                    p2p_ever_seen = True
            except: pass

        # if P2P not working, fall back to sim API for neighbors (pose only).
        # we synthesize ring_R,dir for them using angle split.
        if not p2p_ever_seen:
            sim_poses = try_get_swarm_poses(robot)
            if sim_poses:
                nbrs.clear()
                for item in sim_poses:
                    # accepted shapes: (id,x,y,theta) or (x,y,theta)
                    if isinstance(item,(list,tuple)) and len(item)>=3:
                        if len(item)>=4:
                            nid, nx, ny, nth = int(item[0]), float(item[1]), float(item[2]), float(item[3])
                        else:
                            nx, ny, nth = float(item[0]), float(item[1]), float(item[2])
                            nid = 10_000 + int(1000*nx) ^ int(1000*ny)  # synthesize id
                        if nid == rid: continue
                        ntheta = math.atan2(ny - CY, nx - CX)
                        nR, ndir = angle_split(ntheta)
                        nbrs[int(nid)] = (nx, ny, nth, nR, ndir, now)

        # prune stale
        cutoff = now - STALE_S
        for nid in list(nbrs.keys()):
            if nbrs[nid][5] < cutoff:
                nbrs.pop(nid, None)

        # ---- base tangent + radial hold ----
        vx = V_TANGENT_BASE * utx
        vy = V_TANGENT_BASE * uty
        radial = clamp(K_R * (assigned_R - r), -RADIAL_CLAMP, RADIAL_CLAMP)
        vx += radial * urx; vy += radial * ury

        # ---- ring-local angular spacing ----
        for _, (nx, ny, nth, nR, ndir, tlast) in nbrs.items():
            if abs(nR - assigned_R) <= SAME_RING_TOL and ndir == assigned_dir:
                ddx, ddy = x - nx, y - ny
                d2 = ddx*ddx + ddy*ddy
                if d2 < MIN_LINEAR_SEP*MIN_LINEAR_SEP:
                    s = (MIN_LINEAR_SEP*MIN_LINEAR_SEP - d2) / (MIN_LINEAR_SEP*MIN_LINEAR_SEP)
                    vx += s * urx; vy += s * ury
                ntheta = math.atan2(ny - CY, nx - CX)
                dtheta = wrap(theta - ntheta)
                ad = abs(dtheta)
                if 1e-3 < ad <= ANG_REP_CUTOFF:
                    strength = ANG_REP_GAIN / (ad ** ANG_REP_POW)
                    push = strength * (1.0 if dtheta > 0 else -1.0)
                    vx += push * utx; vy += push * uty

        # boundary cushion (radial projection)
        bfx, bfy = soft_boundary_force(x, y)
        b_norm = bfx*urx + bfy*ury
        vx += b_norm * urx; vy += b_norm * ury

        # LEDs
        near_soft = abs(b_norm) > 1e-6
        on_ring   = abs(assigned_R - r) < 0.04
        if near_soft:          robot.set_led(100,60,0)
        elif on_ring and assigned_dir>0: robot.set_led(0,100,0)   # inner/CCW green
        elif on_ring and assigned_dir<0: robot.set_led(0,0,100)   # outer/CW blue
        else:                  robot.set_led(0,70,80)

        # map to wheels
        spd = math.hypot(vx, vy)
        if spd < EPS:
            vx += 0.08 * utx; vy += 0.08 * uty; spd = math.hypot(vx, vy)
        hdg = math.atan2(vy, vx)
        err = wrap(hdg - th)

        ae = abs(err)
        if ae < 0.5:    fwd = FWD_FAST
        elif ae < 1.2:  fwd = FWD_FAST * 0.7
        else:           fwd = FWD_SLOW
        fwd = max(fwd, FWD_MIN)
        if near_soft: fwd *= 0.85

        turn = clamp(TURN_K * err, -1.5, 1.5)
        lcmd = clamp(int(MAX_WHEEL*0.9*(fwd - 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
        rcmd = clamp(int(MAX_WHEEL*0.9*(fwd + 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)

        left  = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
        right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
        lastL, lastR = left, right
        robot.set_vel(left, right)

        # occasional print
        if now - last_print > PRINT_PERIOD:
            n_inner = sum(1 for v in nbrs.values() if abs(v[3]-R_INNER)<=SAME_RING_TOL and v[4]==DIR_INNER)
            n_outer = sum(1 for v in nbrs.values() if abs(v[3]-R_OUTER)<=SAME_RING_TOL and v[4]==DIR_OUTER)
            logw(f"[dual_ring_robust] id={rid} inner={n_inner + (1 if (abs(assigned_R-R_INNER)<=SAME_RING_TOL and assigned_dir==DIR_INNER) else 0)} "
                 f"outer={n_outer + (1 if (abs(assigned_R-R_OUTER)<=SAME_RING_TOL and assigned_dir==DIR_OUTER) else 0)} "
                 f"split={'P2P' if p2p_ever_seen else 'SIM-API/angle'}")
            last_print = now

        robot.delay(DT_MS)
