# -*- coding: utf-8 -*-
# Single-Ring Encircle: Velocity-based spacing using P2P
# Bots adjust speed based on distance to neighbors to achieve equal spacing

import math, struct, random, os

# ----------------- Logging (sim + hardware) -----------------
LOG = None   # global log handle

def init_log():
    """
    Try to open experiment_log.txt in a hardware-like way.
    If it fails (e.g., sim with no FS), we just fall back to logw only.
    """
    global LOG
    if LOG is not None:
        return
    try:
        # line-buffered like your FLOAT HW script
        LOG = open("experiment_log.txt", "a", 1)
    except Exception:
        LOG = None

def logw(msg):
    """
    Write to log file (if available) AND logw to stdout.
    Safe in both sim and hardware.
    """
    if not isinstance(msg, str):
        msg = str(msg)
    line = msg if msg.endswith("\n") else msg + "\n"

    # Log file (hardware) if available
    if LOG is not None:
        try:
            LOG.write(line)
            LOG.flush()
            os.fsync(LOG.fileno())
        except Exception:
            pass

    # Always also logw (sim / console)
    print(line.rstrip("\n"))

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
R_TARGET = SAFE_BUBBLE + 0.33  # Single target radius
DIRECTION = +1  # CCW rotation (use -1 for CW)

# ----------------- Motion / spacing -----------------
V_TANGENT_BASE = 0.26
K_R, RADIAL_CLAMP = 1.3, 0.12
MIN_LINEAR_SEP = 0.18
IDEAL_ANGULAR_SEP = 2 * math.pi / 8  # Assuming ~8 bots, adjust as needed

# ----------------- Velocity adjustment gains -----------------
ANGULAR_VELOCITY_GAIN = 0.8  # How aggressively to adjust speed based on spacing
MAX_VELOCITY_ADJUSTMENT = 0.15  # Max speed change

# ----------------- Boundary cushion -----------------
SOFT_MARGIN, CRIT_MARGIN, SOFT_MAX_FORCE = 0.08, 0.05, 0.35  # Increased crit margin

# ----------------- Drive model -----------------
MAX_WHEEL, TURN_K = 35, 3.0
FWD_FAST, FWD_SLOW, FWD_MIN = 0.80, 0.30, 0.40
CMD_SMOOTH, EPS, DT_MS = 0.20, 1e-3, 40
logw_PERIOD = 2.0

# ----------------- Heartbeats (pose) -----------------
HB_FMT = 'fffi'  # x,y,theta, vid
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

def usr(robot):
    init_log()
    robot.delay(800)
    
    # SIMULATION: Use robot.id (without parentheses)
    rid = robot.id
    logw(f"Bot starting with ID: {rid}")
    
    random.seed(rid * 1103515245 & 0xFFFFFFFF)

    # neighbors: id -> (x,y,theta,t_last)
    nbrs = {}
    last_hb = -1e9
    last_logw = 0.0
    lastL = lastR = 0

    # detection flags
    p2p_ever_seen = False
    ids_seen = set()

    # nudge localization
    robot.set_vel(20,20); robot.delay(150)

    while True:
        pose = safe_pose(robot)
        if not pose:
            robot.set_vel(0,0); robot.delay(DT_MS); continue
        x,y,th = pose
        now = robot.get_clock()

        # safety - with recovery behavior instead of hard stop
        rdx, rdy = x - CX, y - CY
        r = math.hypot(rdx, rdy)
        critical_boundary = is_critical_boundary(x, y)
        
        if r < OBST_RADIUS or critical_boundary:
            # Recovery behavior: back away from obstacle/wall
            back_force = 0.4
            if r < OBST_RADIUS:
                # Back away from center
                vx = -back_force * rdx/r if r > 1e-6 else -back_force
                vy = -back_force * rdy/r if r > 1e-6 else -back_force
            else:
                # Back away from boundary
                bfx, bfy = 0, 0
                if x < X_MIN + CRIT_MARGIN: bfx = 1
                elif x > X_MAX - CRIT_MARGIN: bfx = -1
                if y < Y_MIN + CRIT_MARGIN: bfy = 1
                elif y > Y_MAX - CRIT_MARGIN: bfy = -1
                norm = math.hypot(bfx, bfy)
                vx = back_force * bfx/norm if norm > 1e-6 else 0
                vy = back_force * bfy/norm if norm > 1e-6 else 0
            
            robot.set_led(100,0,0)  # Red for recovery
            # Convert to wheel commands and continue
            hdg = math.atan2(vy, vx)
            err = wrap(hdg - th)
            turn = clamp(TURN_K * err, -1.5, 1.5)
            lcmd = clamp(int(MAX_WHEEL*0.9*(FWD_SLOW - 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
            rcmd = clamp(int(MAX_WHEEL*0.9*(FWD_SLOW + 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
            left = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
            right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
            lastL, lastR = left, right
            robot.set_vel(left, right)
            robot.delay(DT_MS)
            continue

        theta = math.atan2(rdy, rdx)

        # unit frames
        if r < 1e-6: urx, ury = 1.0, 0.0
        else:        urx, ury = rdx/r, rdy/r
        utx, uty = -ury, urx
        if DIRECTION < 0: utx, uty = -utx, -uty

        # --- send heartbeat (pose only) ---
        if now - last_hb >= HB_DT:
            try:
                robot.send_msg(struct.pack(HB_FMT, float(x), float(y), float(th), int(rid)))
            except: pass
            last_hb = now

        # --- receive heartbeats ---
        msgs = robot.recv_msg(clear=True)
        if msgs is None:
            msgs = []
        elif not isinstance(msgs, list):
            msgs = [msgs]
            
        for m in msgs:
            try:
                if len(m) >= HB_BYTES:
                    nx, ny, nth, nid = struct.unpack(HB_FMT, m[:HB_BYTES])
                    if int(nid) != rid:
                        nbrs[int(nid)] = (float(nx), float(ny), float(nth), now)
                        ids_seen.add(int(nid))
                        p2p_ever_seen = True
            except: pass

        # Fallback to sim API if no P2P
        if not p2p_ever_seen:
            sim_poses = try_get_swarm_poses(robot)
            if sim_poses:
                nbrs.clear()
                for item in sim_poses:
                    if isinstance(item,(list,tuple)) and len(item)>=3:
                        if len(item)>=4:
                            nid, nx, ny, nth = int(item[0]), float(item[1]), float(item[2]), float(item[3])
                        else:
                            nx, ny, nth = float(item[0]), float(item[1]), float(item[2])
                            nid = 10000 + int(1000 * abs(nx)) + int(1000 * abs(ny))
                        if nid == rid: continue
                        nbrs[int(nid)] = (nx, ny, nth, now)

        # prune stale
        cutoff = now - STALE_S
        for nid in list(nbrs.keys()):
            if nbrs[nid][3] < cutoff:
                nbrs.pop(nid, None)

        # ---- base tangent + radial hold ----
        base_v_tangent = V_TANGENT_BASE
        vx = base_v_tangent * utx
        vy = base_v_tangent * uty
        radial = clamp(K_R * (R_TARGET - r), -RADIAL_CLAMP, RADIAL_CLAMP)
        vx += radial * urx; vy += radial * ury

        # # ---- VELOCITY-BASED SPACING ----
        # if len(nbrs) > 0:
        #     # Calculate angular positions of all neighbors
        #     neighbor_angles = []
        #     for _, (nx, ny, nth, tlast) in nbrs.items():
        #         ntheta = math.atan2(ny - CY, nx - CX)
        #         neighbor_angles.append(ntheta)
            
        #     # Find closest neighbors in front and behind
        #     front_gap = float('inf')
        #     behind_gap = float('inf')
            
        #     for ntheta in neighbor_angles:
        #         angular_diff = wrap(ntheta - theta)
                
        #         # For CCW direction, positive diff is "in front", negative is "behind"
        #         if DIRECTION > 0:  # CCW
        #             if angular_diff > 0 and angular_diff < front_gap:
        #                 front_gap = angular_diff
        #             elif angular_diff < 0 and -angular_diff < behind_gap:
        #                 behind_gap = -angular_diff
        #         else:  # CW (reverse the logic)
        #             if angular_diff < 0 and -angular_diff < front_gap:
        #                 front_gap = -angular_diff
        #             elif angular_diff > 0 and angular_diff < behind_gap:
        #                 behind_gap = angular_diff
            
        #     # If we found gaps, adjust velocity
        #     if front_gap < float('inf') and behind_gap < float('inf'):
        #         # Calculate velocity adjustment
        #         total_gap = front_gap + behind_gap
        #         if total_gap > 0:
        #             # Normalize by ideal spacing
        #             front_ratio = front_gap / IDEAL_ANGULAR_SEP
                    
        #             # Speed up if front gap is too large, slow down if too small
        #             velocity_adjustment = ANGULAR_VELOCITY_GAIN * (front_ratio - 1.0)
        #             velocity_adjustment = clamp(velocity_adjustment, -MAX_VELOCITY_ADJUSTMENT, MAX_VELOCITY_ADJUSTMENT)
                    
        #             # Apply adjustment to tangential velocity
        #             adjusted_v_tangent = base_v_tangent + velocity_adjustment
        #             vx = adjusted_v_tangent * utx
        #             vy = adjusted_v_tangent * uty
                    
        #             # Re-apply radial component
        #             vx += radial * urx
        #             vy += radial * ury
                    
        #             # Debug logw
        #             if now - last_logw > logw_PERIOD:
        #                 logw(f"Bot {rid}: front_gap={front_gap:.3f}, behind_gap={behind_gap:.3f}, adj={velocity_adjustment:.3f}")

        # Replace the current velocity-based spacing section with:
        if len(nbrs) > 0:
            neighbor_angles = []
            for _, (nx, ny, nth, tlast) in nbrs.items():
                ntheta = math.atan2(ny - CY, nx - CX)
                neighbor_angles.append(ntheta)
            
            # Find gaps to closest neighbors on both sides
            gaps = []
            for ntheta in neighbor_angles:
                gap = wrap(ntheta - theta)
                gaps.append(gap)
            
            # Sort gaps to find immediate neighbors
            positive_gaps = [g for g in gaps if g > 0]
            negative_gaps = [g for g in gaps if g < 0]
            
            front_gap = min(positive_gaps) if positive_gaps else math.pi
            behind_gap = -max(negative_gaps) if negative_gaps else math.pi
            
            # Calculate ideal gap based on number of neighbors
            estimated_total = len(nbrs) + 1  # neighbors + self
            ideal_gap = 2 * math.pi / estimated_total
            
            # STRONG enforcement: adjust speed to match ideal spacing
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

        # ---- COLLISION AVOIDANCE ----
        collision_detected = False
        for _, (nx, ny, nth, tlast) in nbrs.items():
            ddx, ddy = x - nx, y - ny
            d_linear = math.hypot(ddx, ddy)
            
            if d_linear < MIN_LINEAR_SEP:
                collision_detected = True
                repulsion_strength = 0.6 * (MIN_LINEAR_SEP - d_linear) / MIN_LINEAR_SEP
                if d_linear > 1e-6:
                    repulse_x = ddx / d_linear
                    repulse_y = ddy / d_linear
                    vx += repulsion_strength * repulse_x
                    vy += repulsion_strength * repulse_y

        # boundary cushion
        bfx, bfy = soft_boundary_force(x, y)
        b_norm = bfx*urx + bfy*ury
        vx += b_norm * urx; vy += b_norm * ury

        # LEDs
        near_soft = abs(b_norm) > 1e-6
        on_ring = abs(R_TARGET - r) < 0.04
        
        if collision_detected:
            robot.set_led(100, 50, 0)  # Orange for collision avoidance
        elif near_soft:
            robot.set_led(100, 60, 0)   # Yellow for near boundary
        elif on_ring:
            robot.set_led(0, 100, 0)    # Green when on target ring
        else:
            robot.set_led(0, 70, 80)    # Cyan when moving to position

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
        if near_soft or collision_detected: fwd *= 0.8

        turn = clamp(TURN_K * err, -1.5, 1.5)
        lcmd = clamp(int(MAX_WHEEL*0.9*(fwd - 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
        rcmd = clamp(int(MAX_WHEEL*0.9*(fwd + 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)

        left  = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
        right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
        lastL, lastR = left, right
        robot.set_vel(left, right)

        # occasional logw
        if now - last_logw > logw_PERIOD:
            logw(f"[velocity_spacing] id={rid} neighbors={len(nbrs)} radius_error={R_TARGET - r:.3f}")
            last_logw = now

        robot.delay(DT_MS)

