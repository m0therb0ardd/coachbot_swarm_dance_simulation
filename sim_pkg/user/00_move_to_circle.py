
# -*- coding: utf-8 -*-
# Concentric Circle Formation (single ring)
# Python 2.7 + Coachbot messaging + lab logging pattern + separation

import math, struct

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

# ----------------- Testbed bounds & center -----------------
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35
CX, CY = (-0.1, 0.475)   # dancer center / nominal center

# ----------------- Dancer no-go circle -----------------
FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET   # ~0.1524 m
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
OBST_CX, OBST_CY = CX, CY  # obstacle centered on dancer center (fixed)

# ----------------- Circle parameters -----------------
R_TARGET = SAFE_BUBBLE + 0.33  # Target circle radius
DIRECTION = 1  # CCW rotation

# ----------------- Drive control -----------------
MAX_WHEEL = 35      # a bit softer than 40
TURN_K    = 3.0
FWD_FAST  = 0.75
FWD_SLOW  = 0.25
EPS       = 1e-3

# ----------------- Control gains -----------------
K_RADIAL    = 1.5    # Gain for radial positioning
K_ANGULAR   = 2.0    # Gain for angular positioning

# ----------------- Separation (collision avoidance) -----------------
MIN_SEP     = 0.18   # minimum desired distance between robots (meters)
SEP_GAIN    = 0.6    # strength of repulsion

def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def wrap_angle(a):
    while a >  math.pi:
        a -= 2*math.pi
    while a <= -math.pi:
        a += 2*math.pi
    return a

def safe_pose(robot):
    p = robot.get_pose()
    if isinstance(p, (list, tuple)) and len(p) >= 3:
        return float(p[0]), float(p[1]), float(p[2])
    return None

def get_id(robot):
    """
    Prefer hardware virtual_id(), fall back to sim .id.
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

def obstacle_distance(x, y):
    dx, dy = x - OBST_CX, y - OBST_CY
    return math.hypot(dx, dy)

def is_critical_obstacle(x, y, critical_margin=0.0):
    return obstacle_distance(x, y) < (OBST_RADIUS + critical_margin)

def soft_boundary_force(x, y, max_force=0.3, boundary_margin=0.08):
    fx = 0.0
    fy = 0.0
    if x < X_MIN + boundary_margin:
        fx += max_force * (1.0 - (x - X_MIN) / boundary_margin)
    elif x > X_MAX - boundary_margin:
        fx -= max_force * (1.0 - (X_MAX - x) / boundary_margin)
    if y < Y_MIN + boundary_margin:
        fy += max_force * (1.0 - (y - Y_MIN) / boundary_margin)
    elif y > Y_MAX - boundary_margin:
        fy -= max_force * (1.0 - (Y_MAX - y) / boundary_margin)
    return fx, fy

def usr(robot):
    global log, log_out 
    # ---- open log exactly as lab wants ----
    try:
        log = open("experiment_log", "wb")
    except:
        log = None

    my_id = get_id(robot)
    robot.set_led(0, 180, 180)  # cyan normal
    logw("Robot %d starting: Concentric Circle Formation" % my_id)

    # ----------------- PHASE 1: gossip to discover all robots -----------------
    POSE_FMT  = "ffi"  # float x, float y, int id
    POSE_SIZE = struct.calcsize(POSE_FMT)

    poses = {}  # id -> (x, y)

    t_start = robot.get_clock()
    gossip_duration = 6.0  # seconds to share poses

    while robot.get_clock() - t_start < gossip_duration:
        pose = safe_pose(robot)
        if pose is not None:
            x, y, _ = pose
            # broadcast my pose
            try:
                msg = struct.pack(POSE_FMT, x, y, my_id)
                robot.send_msg(msg)
            except:
                pass
            # store my own pose too
            poses[my_id] = (x, y)

        # receive and store others' poses
        msgs = robot.recv_msg()
        if msgs is not None:
            # msgs can be a list or a single message depending on API
            if not isinstance(msgs, list):
                msgs = [msgs]
            for m in msgs:
                if not m:
                    continue
                try:
                    px, py, pid = struct.unpack(POSE_FMT, m[:POSE_SIZE])
                    poses[int(pid)] = (float(px), float(py))
                except:
                    pass

        robot.delay(50)  # 50 ms as recommended between send/recv

    # ----------------- PHASE 2: Assign target positions on circle -----------------
    total_robots = len(poses)
    
    if total_robots == 0:
        logw("Robot %d: No robots discovered, using default position" % my_id)
        # Fallback: just go to some position on circle
        target_angle = 0.0
    else:
        # Sort robot IDs to get consistent ordering
        sorted_ids = sorted(poses.keys())
        # If my_id somehow not discovered, default index 0
        try:
            my_index = sorted_ids.index(my_id)
        except ValueError:
            my_index = 0
        
        # Calculate equal angular spacing
        angle_spacing = 2.0 * math.pi / float(total_robots)
        target_angle = my_index * angle_spacing
        
        logw("Robot %d: %d robots, my index=%d, target_angle=%.3f"
             % (my_id, total_robots, my_index, target_angle))

    # Calculate target position on circle
    target_x = CX + R_TARGET * math.cos(target_angle)
    target_y = CY + R_TARGET * math.sin(target_angle)
    
    logw("Robot %d target: (%.3f, %.3f)" % (my_id, target_x, target_y))

    # ----------------- PHASE 3: Move to target positions -----------------
    last_logw_time = 0.0
    logw_PERIOD = 2.0
    
    formation_achieved = False

    while True:
        pose = safe_pose(robot)
        if pose is None:
            robot.set_vel(0,0)
            robot.delay(20)
            continue
            
        x, y, th = pose
        
        # Safety check (always active)
        if is_critical_obstacle(x, y):
            robot.set_led(255, 0, 0)  # red for critical
            robot.set_vel(0, 0)
            robot.delay(40)
            continue

        # If formation is achieved, stop and skip control
        if formation_achieved:
            robot.set_vel(0, 0)
            robot.set_led(0, 255, 0)  # green
            robot.delay(40)
            continue

        # Calculate current position relative to center
        dx = x - CX
        dy = y - CY
        current_r = math.hypot(dx, dy)
        current_angle = math.atan2(dy, dx)
        
        # Calculate errors
        radial_error  = R_TARGET - current_r
        angular_error = wrap_angle(target_angle - current_angle)
        
        # Unit vectors for radial and tangential directions
        if current_r > 1e-6:
            ur = (dx / current_r, dy / current_r)    # radial unit vector (outward)
            ut = (-ur[1], ur[0])                     # tangential unit vector (CCW)
            if DIRECTION < 0:
                ut = (-ut[0], -ut[1])                # tangential unit vector (CW)
        else:
            ur = (1.0, 0.0)
            if DIRECTION > 0:
                ut = (0.0, 1.0)
            else:
                ut = (0.0, -1.0)

        # Base control: move toward target position
        v_radial     = K_RADIAL  * radial_error
        v_tangential = K_ANGULAR * angular_error
        
        vx = v_radial * ur[0] + v_tangential * ut[0]
        vy = v_radial * ur[1] + v_tangential * ut[1]
        
        # ----------------- PHASE 4: Neighbor info for spacing + separation -----------------
        msgs = robot.recv_msg()
        current_neighbors = {}
        if msgs is not None:
            if not isinstance(msgs, list):
                msgs = [msgs]
            for m in msgs:
                if not m:
                    continue
                try:
                    px, py, pid = struct.unpack(POSE_FMT, m[:POSE_SIZE])
                    if int(pid) != my_id:
                        current_neighbors[int(pid)] = (float(px), float(py))
                except:
                    pass
        
        # ----- ANGULAR SPACING (same as before) -----
        if len(current_neighbors) > 0 and total_robots > 0:
            neighbor_angles = []
            for nid in current_neighbors:
                nx, ny = current_neighbors[nid]
                ndx = nx - CX
                ndy = ny - CY
                n_angle = math.atan2(ndy, ndx)
                neighbor_angles.append(n_angle)
            
            # Find closest angular neighbor
            min_gap = float('inf')
            for n_angle in neighbor_angles:
                gap = abs(wrap_angle(n_angle - current_angle))
                if gap < min_gap:
                    min_gap = gap
            
            # Ideal gap is 2*pi / total_robots
            ideal_gap = 2.0 * math.pi / float(total_robots)
            
            # Adjust tangential velocity based on spacing
            if min_gap < ideal_gap * 0.7:
                # Too close to neighbor, slow down tangentially
                v_tangential = v_tangential * 0.6
            elif min_gap > ideal_gap * 1.3:
                # Too far from neighbor, speed up tangentially
                v_tangential = v_tangential * 1.4
            
            # Recalculate velocity with spacing adjustment
            vx = v_radial * ur[0] + v_tangential * ut[0]
            vy = v_radial * ur[1] + v_tangential * ut[1]

        # ----- COLLISION AVOIDANCE (new) -----
        collision_risk = False
        if len(current_neighbors) > 0:
            sep_x = 0.0
            sep_y = 0.0
            for nid in current_neighbors:
                nx, ny = current_neighbors[nid]
                ddx = x - nx
                ddy = y - ny
                d = math.hypot(ddx, ddy)
                if d < MIN_SEP and d > 1e-6:
                    collision_risk = True
                    # strength grows as robots get closer
                    s = SEP_GAIN * (MIN_SEP - d) / MIN_SEP
                    sep_x += s * (ddx / d)
                    sep_y += s * (ddy / d)
            vx += sep_x
            vy += sep_y

        # Add boundary cushion
        bfx, bfy = soft_boundary_force(x, y)
        vx += bfx
        vy += bfy

        # Check if we're close to target
        position_error = math.hypot(target_x - x, target_y - y)
        on_target = (position_error < 0.05) and (abs(radial_error) < 0.03)
        
        # LED feedback
        if on_target:
            formation_achieved = True
            robot.set_led(0, 255, 0)  # green when on target
            # Stop moving by setting velocity to zero
            vx = 0.0
            vy = 0.0
        else:
            if collision_risk:
                robot.set_led(255, 120, 0)  # orange when actively separating
            elif abs(radial_error) > 0.1:
                robot.set_led(255, 150, 0)  # orange - moving radially
            else:
                robot.set_led(0, 150, 255)  # blue - adjusting angle

        # Convert to wheel commands
        if abs(vx) + abs(vy) < EPS:
            robot.set_vel(0, 0)
        else:
            hdg = math.atan2(vy, vx)
            err = wrap_angle(hdg - th)

            abs_err = abs(err)
            if abs_err < 0.5:
                fwd = FWD_FAST
            elif abs_err < 1.2:
                fwd = FWD_FAST * 0.7
            else:
                fwd = FWD_SLOW

            # If we are in collision avoidance, be a bit more cautious
            if collision_risk:
                fwd = fwd * 0.7

            turn = clamp(TURN_K * err, -1.5, 1.5)

            left  = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
            right = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
            robot.set_vel(left, right)

        # Periodic status
        now = robot.get_clock()
        if now - last_logw_time > logw_PERIOD:
            logw("Robot %d: pos [%.3f, %.3f], err=%.3f, neighbors=%d, collision=%d"
                 % (my_id, x, y, position_error, len(current_neighbors),
                    1 if collision_risk else 0))
            last_logw_time = now

        robot.delay(40)
