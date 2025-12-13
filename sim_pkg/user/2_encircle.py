
# -*- coding: utf-8 -*-
# Two-Ring Encircle: Adaptive assignment based on initial positions
# Inner ring CCW, outer ring CW. Bots are assigned to rings based on initial distance bands.
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
            log_out = open("/home/pi/experiment_output", "wb")
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

# ----------------- Two ring parameters -----------------
R_INNER = SAFE_BUBBLE + 0.22
R_OUTER = SAFE_BUBBLE + 0.40

# Adaptive threshold  will be calculated based on discovered positions
R_SPLIT_THRESHOLD = None

# Tolerance to treat a neighbor as on my rin for spacing
R_RING_TOL = 0.10

# ----------------- Motion / spacing -----------------
V_TANGENT_BASE = 0.26
K_R, RADIAL_CLAMP = 1.3, 0.12
MIN_LINEAR_SEP = 0.18

# ----------------- Boundary cushion -----------------
SOFT_MARGIN    = 0.08
CRIT_MARGIN    = 0.05
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

# ----------------- Heartbeats (pose + ring assignment) -----------------
HB_FMT   = 'fffi'  # x,y,theta, vid  
RING_FMT = 'ifi'   # ring_assigned, split_threshold, vid
HB_BYTES = struct.calcsize(HB_FMT)
RING_BYTES = struct.calcsize(RING_FMT)
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
    if hasattr(robot, "id"):
        try:
            return int(robot.id)
        except:
            pass
    return -1

def find_natural_split(distances):
    """Find natural gap between inner and outer rings in distances"""
    if len(distances) < 2:
        return 0.7  # Default fallback
    
    sorted_dists = sorted(distances)
    
    # Find the largest gap between consecutive distances
    max_gap = 0
    split_index = 0
    
    for i in range(1, len(sorted_dists)):
        gap = sorted_dists[i] - sorted_dists[i-1]
        if gap > max_gap:
            max_gap = gap
            split_index = i
    
    # Use midpoint of the largest gap as threshold
    if max_gap > 0.1:  # Significant gap found
        threshold = (sorted_dists[split_index-1] + sorted_dists[split_index]) / 2.0
        logw("Found natural gap at index %d: %.3f -> %.3f (threshold=%.3f)" % 
             (split_index, sorted_dists[split_index-1], sorted_dists[split_index], threshold))
        return threshold
    else:
        # No clear gap, use median
        median = sorted_dists[len(sorted_dists)//2]
        logw("No clear gap, using median: %.3f" % median)
        return median

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
    logw("Bot starting with ID: %d (adaptive two-ring encircle)" % rid)

    random.seed((rid * 1103515245) & 0xFFFFFFFF)

    # neighbors: id  (x,y,theta,t_last, ring_assigned)
    nbrs = {}
    last_hb   = -1e9
    last_logw = 0.0
    lastL = lastR = 0

    # Ring assignment state
    R_TARGET_LOCAL = None
    DIRECTION_LOCAL = None
    ring_assignment_complete = False
    my_ring_assigned = None
    global R_SPLIT_THRESHOLD
    
    # detection flags
    p2p_ever_seen = False

    # ----------------- ADAPTIVE DISCOVERY & RING ASSIGNMENT -----------------
    logw("Bot %d: Starting adaptive discovery phase..." % rid)
    discovery_start = robot.get_clock()
    discovery_duration = 4.0  # Extended discovery
    
    initial_positions = {}  # id  (x, y)
    
    while robot.get_clock() - discovery_start < discovery_duration:
        pose = safe_pose(robot)
        if pose:
            x, y, th = pose
            initial_positions[rid] = (x, y)
            
            # Broadcast my pose
            try:
                msg = struct.pack(HB_FMT, float(x), float(y), float(th), int(rid))
                robot.send_msg(msg)
            except:
                pass

        # Receive others poses
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
                        initial_positions[nid] = (float(nx), float(ny))
                        p2p_ever_seen = True
            except:
                pass

        robot.delay(50)

    # Fallback to sim API if no P2P
    if not p2p_ever_seen:
        sim_poses = try_get_swarm_poses(robot)
        if sim_poses:
            for item in sim_poses:
                if isinstance(item, (list, tuple)) and len(item) >= 3:
                    if len(item) >= 4:
                        nid, nx, ny, nth = int(item[0]), float(item[1]), float(item[2]), float(item[3])
                    else:
                        nx, ny, nth = float(item[0]), float(item[1]), float(item[2])
                        nid = 10000 + int(1000 * abs(nx)) + int(1000 * abs(ny))
                    if nid != rid:
                        initial_positions[nid] = (nx, ny)

    # ----------------- ADAPTIVE RING ASSIGNMENT -----------------
    if len(initial_positions) > 0:
        # Calculate initial distances from center for all discovered robots
        distances = []
        for bot_id, (bx, by) in initial_positions.items():
            dist = math.hypot(bx - CX, by - CY)
            distances.append(dist)
        
        # Find natural split between inner and outer rings
        R_SPLIT_THRESHOLD = find_natural_split(distances)
        
        # Calculate my initial distance
        my_initial_x, my_initial_y = initial_positions[rid]
        my_initial_dist = math.hypot(my_initial_x - CX, my_initial_y - CY)
        
        # Assign ring based on adaptive threshold
        if my_initial_dist <= R_SPLIT_THRESHOLD:
            # Closer bots  inner ring  CCW
            R_TARGET_LOCAL = R_INNER
            DIRECTION_LOCAL = +1
            my_ring_assigned = "INNER (CCW)"
        else:
            # Further bots  outer ring  CW  
            R_TARGET_LOCAL = R_OUTER
            DIRECTION_LOCAL = -1
            my_ring_assigned = "OUTER (CW)"
        
        ring_assignment_complete = True
        
        logw("Bot %d: Discovered %d bots. Split threshold: %.3f" % (rid, len(initial_positions), R_SPLIT_THRESHOLD))
        logw("Bot %d: Initial distance: %.3f -> Assigned to %s" % (rid, my_initial_dist, my_ring_assigned))
        
        # Log the full assignment for debugging
        logw("Full adaptive ring assignment:")
        for bot_id, (bx, by) in initial_positions.items():
            dist = math.hypot(bx - CX, by - CY)
            ring = "INNER" if dist <= R_SPLIT_THRESHOLD else "OUTER"
            mark = " <== ME" if bot_id == rid else ""
            logw("  Bot %d: dist=%.3f -> %s%s" % (bot_id, dist, ring, mark))
            
    else:
        # Fallback: if no discovery, use conservative assignment
        pose = safe_pose(robot)
        if pose:
            x, y, th = pose
            my_initial_dist = math.hypot(x - CX, y - CY)
            # Conservative: if no info, assume inner ring
            R_TARGET_LOCAL = R_INNER
            DIRECTION_LOCAL = +1
            my_ring_assigned = "INNER (CCW) [FALLBACK]"
            logw("Bot %d: No bots discovered, using fallback. Distance: %.3f -> %s" % 
                 (rid, my_initial_dist, my_ring_assigned))
        else:
            R_TARGET_LOCAL = R_INNER
            DIRECTION_LOCAL = +1
            my_ring_assigned = "INNER (CCW) [NO POSE]"
            logw("Bot %d: No pose available, defaulting to %s" % (rid, my_ring_assigned))

    # nudge localization
    robot.set_vel(20, 20)
    robot.delay(150)

    # Main control loop
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

        # Use the FIXED ring assignment from discovery phase
        # This doesnt change during the experimen

        if r < OBST_RADIUS or critical_boundary:
            # Recovery behavior: back away from obstacle/wall
            back_force = 0.4
            if r < OBST_RADIUS:
                if r > 1e-6:
                    vx = -back_force * (rdx / r)
                    vy = -back_force * (rdy / r)
                else:
                    vx = -back_force
                    vy = 0.0
            else:
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
        if DIRECTION_LOCAL < 0:
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
        radial = clamp(K_R * (R_TARGET_LOCAL - r), -RADIAL_CLAMP, RADIAL_CLAMP)
        vx += radial * urx
        vy += radial * ury

        # ----------------- VELOCITY-BASED SPACING (per ring) -----------------
        if len(nbrs) > 0:
            neighbor_angles_same_ring = []

            for _, (nx, ny, nth, tlast) in nbrs.items():
                ndx = nx - CX
                ndy = ny - CY
                nr  = math.hypot(ndx, ndy)

                # only treat neighbors near my ring radius as same ring
                if abs(nr - R_TARGET_LOCAL) <= R_RING_TOL:
                    ntheta = math.atan2(ndy, ndx)
                    neighbor_angles_same_ring.append(ntheta)

            if neighbor_angles_same_ring:
                gaps = []
                for ntheta in neighbor_angles_same_ring:
                    gap = wrap(ntheta - theta)
                    gaps.append(gap)

                positive_gaps = [g for g in gaps if g > 0.0]
                negative_gaps = [g for g in gaps if g < 0.0]

                front_gap  = min(positive_gaps) if positive_gaps else math.pi
                behind_gap = -max(negative_gaps) if negative_gaps else math.pi

                estimated_total = len(neighbor_angles_same_ring) + 1  # same-ring neighbors + self
                ideal_gap = 2.0 * math.pi / float(estimated_total)

                current_min_gap = min(front_gap, behind_gap)

                if current_min_gap < ideal_gap * 0.6:
                    vx *= 0.4
                    vy *= 0.4
                elif current_min_gap < ideal_gap * 0.8:
                    vx *= 0.7
                    vy *= 0.7
                elif current_min_gap > ideal_gap * 1.5:
                    vx *= 1.3
                    vy *= 1.3
                elif current_min_gap > ideal_gap * 2.0:
                    vx *= 1.6
                    vy *= 1.6

        # ----------------- COLLISION AVOIDANCE (all neighbors) -----------------
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
        on_ring   = abs(R_TARGET_LOCAL - r) < 0.04

        if collision_detected:
            robot.set_led(100, 50, 0)   # Orange for collision avoidance
        elif near_soft:
            robot.set_led(100, 60, 0)   # Yellow for near boundary
        elif on_ring:
            if DIRECTION_LOCAL > 0:  # Inner ring (CCW)
                robot.set_led(0, 100, 0)    # Green for inner ring
            else:  # Outer ring (CW)
                robot.set_led(0, 0, 100)    # Blue for outer ring
        else:
            if DIRECTION_LOCAL > 0:  # Inner ring (CCW)
                robot.set_led(0, 70, 80)    # Cyan for inner ring
            else:  # Outer ring (CW)
                robot.set_led(80, 0, 80)    # Purple for outer ring

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
            re = R_TARGET_LOCAL - r
            ring_name = "INNER-CCW" if DIRECTION_LOCAL > 0 else "OUTER-CW"
            logw("[%s] id=%d neighbors=%d radius_error=%.3f r=%.3f ring_target=%.3f"
                 % (ring_name, rid, len(nbrs), re, r, R_TARGET_LOCAL))
            last_logw = now

        robot.delay(DT_MS)