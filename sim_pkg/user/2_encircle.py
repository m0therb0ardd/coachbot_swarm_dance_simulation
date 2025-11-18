# -*- coding: utf-8 -*-
# Single-Ring Encircle: Velocity-based spacing using P2P
# Bots adjust speed based on distance to neighbors to achieve equal spacing
# Python 2.7 + Coachbot messaging + lab logging pattern

import math, struct, random, os

# ----------------- Logging -----------------
log = None       # /home/pi/control/experiment_log
log_out = None   # /home/pi/experiment_output

def logw(msg):
    """Write to experiment_log (lab spec) AND /home/pi/experiment_output for fetch_logs."""
    global log, log_out
    try:
        s = str(msg)
    except:
        s = repr(msg)
    if not s.endswith("\n"):
        s = s + "\n"

    # 1) standard experiment_log in current dir
    if log is None:
        try:
            log = open("experiment_log", "wb")  # lab requirement
        except:
            log = None
    if log is not None:
        try:
            log.write(s)
            log.flush()
        except:
            pass

    # 2) extra file where cctl.fetch_output_handler expects things
    if log_out is None:
        try:
            # absolute path to match scp pi@ip:/home/pi/experiment_output ...
            log_out = open("/home/pi/experiment_output", "wb")
        except:
            log_out = None
    if log_out is not None:
        try:
            log_out.write(s)
            log_out.flush()
        except:
            pass

    # Optional: also print to console (sim)
    try:
        print(s.rstrip("\n"))
    except:
        pass

# ----------------- Arena -----------------
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

# ----------------- Dancer no-go circle -----------------
FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
CX, CY = (-0.1, 0.475)

# ----------------- Single ring parameters -----------------
R_TARGET  = SAFE_BUBBLE + 0.33  # Single target radius
DIRECTION = +1                  # CCW rotation (use -1 for CW)

# ----------------- Motion / spacing -----------------
V_TANGENT_BASE = 0.26
K_R, RADIAL_CLAMP = 1.3, 0.12
MIN_LINEAR_SEP = 0.18
IDEAL_ANGULAR_SEP = 2 * math.pi / 8.0  # reference; we estimate from neighbors at runtime

# ----------------- Velocity adjustment gains -----------------
ANGULAR_VELOCITY_GAIN   = 0.8   # (kept for reference)
MAX_VELOCITY_ADJUSTMENT = 0.15  # (kept for reference)

# ----------------- Boundary cushion -----------------
SOFT_MARGIN    = 0.08
CRIT_MARGIN    = 0.05   # Increased crit margin
SOFT_MAX_FORCE = 0.35

# ----------------- Drive model -----------------
MAX_WHEEL  = 35
TURN_K     = 3.0
FWD_FAST   = 0.80
FWD_SLOW   = 0.30
FWD_MIN    = 0.40
CMD_SMOOTH = 0.20
EPS        = 1e-3
DT_MS      = 40
logw_PERIOD = 2.0

# ----------------- Heartbeats (pose) -----------------
HB_FMT   = 'fffi'  # x,y,theta, vid
HB_BYTES = struct.calcsize(HB_FMT)
HB_DT    = 0.12
STALE_S  = 0.8

# ----------------- Helpers -----------------
def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def wrap(a):
    while a >  math.pi:
        a -= 2*math.pi
    while a <= -math.pi:
        a += 2*math.pi
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
    """
    Prefer hardware virtual_id(), fall back to sim .id.
    (virtual_id block currently commented out to force robot.id, matching your model script)
    """
    # # Real hardware ID
    # if hasattr(robot, "virtual_id") and callable(robot.virtual_id):
    #     try:
    #         return int(robot.virtual_id())
    #     except:
    #         pass

    # Simulation ID
    if hasattr(robot, "id"):
        try:
            return int(robot.id)
        except:
            pass
    return -1

# ----------------- Main -----------------
def usr(robot):
    global log, log_out
    # ---- open log exactly as lab wants ----
    try:
        log = open("experiment_log", "wb")
    except:
        log = None

    robot.delay(800)

    rid = get_id(robot)
    logw("Bot starting with ID: %d" % rid)

    random.seed((rid * 1103515245) & 0xFFFFFFFF)

    # neighbors: id -> (x,y,theta,t_last)
    nbrs = {}
    last_hb   = -1e9
    last_logw = 0.0
    lastL = lastR = 0

    # detection flags
    p2p_ever_seen = False

    # nudge localization
    robot.set_vel(20, 20)
    robot.delay(150)

    while True:
        pose = safe_pose(robot)
        if not pose:
            robot.set_vel(0, 0)
            robot.delay(DT_MS)
            continue

        x, y, th = pose
        now = robot.get_clock()

        # ----------------- SAFETY -----------------
        rdx, rdy = x - CX, y - CY
        r = math.hypot(rdx, rdy)
        critical_boundary = is_critical_boundary(x, y)

        if r < OBST_RADIUS or critical_boundary:
            # Recovery behavior: back away from obstacle/wall
            back_force = 0.4
            if r < OBST_RADIUS:
                # Back away from center
                if r > 1e-6:
                    vx = -back_force * (rdx / r)
                    vy = -back_force * (rdy / r)
                else:
                    vx = -back_force
                    vy = 0.0
            else:
                # Back away from boundary
                bfx = 0.0
                bfy = 0.0
                if x < X_MIN + CRIT_MARGIN:
                    bfx = 1.0
                elif x > X_MAX - CRIT_MARGIN:
                    bfx = -1.0
                if y < Y_MIN + CRIT_MARGIN:
                    bfy = 1.0
                elif y > Y_MAX - CRIT_MARGIN:
                    bfy = -1.0
                norm = math.hypot(bfx, bfy)
                if norm > 1e-6:
                    vx = back_force * bfx / norm
                    vy = back_force * bfy / norm
                else:
                    vx = vy = 0.0

            robot.set_led(100, 0, 0)  # Red for recovery

            # Convert to wheel commands and continue
            if abs(vx) + abs(vy) > EPS:
                hdg = math.atan2(vy, vx)
            else:
                hdg = th
            err = wrap(hdg - th)
            turn = clamp(TURN_K * err, -1.5, 1.5)
            lcmd = clamp(int(MAX_WHEEL * 0.9 * (FWD_SLOW - 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
            rcmd = clamp(int(MAX_WHEEL * 0.9 * (FWD_SLOW + 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
            left  = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
            right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
            lastL, lastR = left, right
            robot.set_vel(left, right)
            robot.delay(DT_MS)
            continue

        theta = math.atan2(rdy, rdx)

        # unit frames
        if r < 1e-6:
            urx, ury = 1.0, 0.0
        else:
            urx, ury = rdx / r, rdy / r
        utx, uty = -ury, urx
        if DIRECTION < 0:
            utx, uty = -utx, -uty

        # ----------------- HEARTBEAT: SEND -----------------
        if now - last_hb >= HB_DT:
            try:
                msg = struct.pack(HB_FMT, float(x), float(y), float(th), int(rid))
                robot.send_msg(msg)
            except:
                pass
            last_hb = now

        # ----------------- HEARTBEAT: RECV -----------------
        msgs = robot.recv_msg(clear=True)
        if msgs is None:
            msgs = []
        elif not isinstance(msgs, list):
            msgs = [msgs]

        for m in msgs:
            try:
                if len(m) >= HB_BYTES:
                    nx, ny, nth, nid = struct.unpack(HB_FMT, m[:HB_BYTES])
                    nid = int(nid)
                    if nid != rid:
                        nbrs[nid] = (float(nx), float(ny), float(nth), now)
                        p2p_ever_seen = True
            except:
                pass

        # Fallback to sim API if no P2P
        if not p2p_ever_seen:
            sim_poses = try_get_swarm_poses(robot)
            if sim_poses:
                nbrs.clear()
                for item in sim_poses:
                    if isinstance(item, (list, tuple)) and len(item) >= 3:
                        if len(item) >= 4:
                            nid, nx, ny, nth = int(item[0]), float(item[1]), float(item[2]), float(item[3])
                        else:
                            nx, ny, nth = float(item[0]), float(item[1]), float(item[2])
                            nid = 10000 + int(1000 * abs(nx)) + int(1000 * abs(ny))
                        if nid == rid:
                            continue
                        nbrs[int(nid)] = (nx, ny, nth, now)

        # prune stale
        cutoff = now - STALE_S
        for nid in list(nbrs.keys()):
            if nbrs[nid][3] < cutoff:
                nbrs.pop(nid, None)

        # ----------------- BASE tangent + radial hold -----------------
        base_v_tangent = V_TANGENT_BASE
        vx = base_v_tangent * utx
        vy = base_v_tangent * uty
        radial = clamp(K_R * (R_TARGET - r), -RADIAL_CLAMP, RADIAL_CLAMP)
        vx += radial * urx
        vy += radial * ury

        # ----------------- VELOCITY-BASED SPACING -----------------
        if len(nbrs) > 0:
            neighbor_angles = []
            for _, (nx, ny, nth, tlast) in nbrs.items():
                ntheta = math.atan2(ny - CY, nx - CX)
                neighbor_angles.append(ntheta)

            gaps = []
            for ntheta in neighbor_angles:
                gap = wrap(ntheta - theta)
                gaps.append(gap)

            positive_gaps = [g for g in gaps if g > 0.0]
            negative_gaps = [g for g in gaps if g < 0.0]

            front_gap  = min(positive_gaps) if positive_gaps else math.pi
            behind_gap = -max(negative_gaps) if negative_gaps else math.pi

            estimated_total = len(nbrs) + 1  # neighbors + self
            ideal_gap = 2.0 * math.pi / float(estimated_total)

            current_min_gap = min(front_gap, behind_gap)

            if current_min_gap < ideal_gap * 0.6:
                # Way too close - slow down significantly
                vx *= 0.4
                vy *= 0.4
            elif current_min_gap < ideal_gap * 0.8:
                # Too close - moderate slowdown
                vx *= 0.7
                vy *= 0.7
            elif current_min_gap > ideal_gap * 1.5:
                # Too far - speed up
                vx *= 1.3
                vy *= 1.3
            elif current_min_gap > ideal_gap * 2.0:
                # Way too far - significant speedup
                vx *= 1.6
                vy *= 1.6

        # ----------------- COLLISION AVOIDANCE -----------------
        collision_detected = False
        for _, (nx, ny, nth, tlast) in nbrs.items():
            ddx = x - nx
            ddy = y - ny
            d_linear = math.hypot(ddx, ddy)

            if d_linear < MIN_LINEAR_SEP:
                collision_detected = True
                repulsion_strength = 0.6 * (MIN_LINEAR_SEP - d_linear) / MIN_LINEAR_SEP
                if d_linear > 1e-6:
                    repulse_x = ddx / d_linear
                    repulse_y = ddy / d_linear
                    vx += repulsion_strength * repulse_x
                    vy += repulsion_strength * repulse_y

        # ----------------- Boundary cushion -----------------
        bfx, bfy = soft_boundary_force(x, y)
        b_norm = bfx*urx + bfy*ury
        vx += b_norm * urx
        vy += b_norm * ury

        # ----------------- LEDs -----------------
        near_soft = abs(b_norm) > 1e-6
        on_ring   = abs(R_TARGET - r) < 0.04

        if collision_detected:
            robot.set_led(100, 50, 0)   # Orange for collision avoidance
        elif near_soft:
            robot.set_led(100, 60, 0)   # Yellow for near boundary
        elif on_ring:
            robot.set_led(0, 100, 0)    # Green when on target ring
        else:
            robot.set_led(0, 70, 80)    # Cyan when moving to position

        # ----------------- Map (vx,vy) -> wheels -----------------
        spd = math.hypot(vx, vy)
        if spd < EPS:
            vx += 0.08 * utx
            vy += 0.08 * uty
            spd = math.hypot(vx, vy)

        hdg = math.atan2(vy, vx)
        err = wrap(hdg - th)

        ae = abs(err)
        if ae < 0.5:
            fwd = FWD_FAST
        elif ae < 1.2:
            fwd = FWD_FAST * 0.7
        else:
            fwd = FWD_SLOW
        if fwd < FWD_MIN:
            fwd = FWD_MIN
        if near_soft or collision_detected:
            fwd *= 0.8

        turn = clamp(TURN_K * err, -1.5, 1.5)
        lcmd = clamp(int(MAX_WHEEL*0.9*(fwd - 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
        rcmd = clamp(int(MAX_WHEEL*0.9*(fwd + 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)

        left  = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
        right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
        lastL, lastR = left, right
        robot.set_vel(left, right)

        # ----------------- Occasional log -----------------
        if now - last_logw > logw_PERIOD:
            re = R_TARGET - r
            logw("[velocity_spacing] id=%d neighbors=%d radius_error=%.3f"
                 % (rid, len(nbrs), re))
            last_logw = now

        robot.delay(DT_MS)
