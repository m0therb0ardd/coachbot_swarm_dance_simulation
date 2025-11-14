import math, struct, os

# ----------------- Logging (sim + hardware) -----------------
LOG = None   # global log handle

def init_log():
    """
    Try to open experiment_log.txt in a hardware-like way.
    If it fails (e.g., sim with no FS), we just fall back to print only.
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
    Write to log file (if available) AND print to stdout.
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

    # Always also print (sim / console)
    print(line.rstrip("\n"))
    
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
MAX_WHEEL = 40
TURN_K    = 3.0
FWD_FAST  = 0.8
FWD_SLOW  = 0.30
EPS       = 1e-3

# ----------------- Control gains -----------------
K_RADIAL = 1.5    # Gain for radial positioning
K_ANGULAR = 2.0   # Gain for angular positioning

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

def wrap_angle(a):
    while a >  math.pi: a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a

def safe_pose(robot):
    p = robot.get_pose()
    if isinstance(p,(list,tuple)) and len(p)>=3:
        return float(p[0]), float(p[1]), float(p[2])
    return None

def get_id(robot):
    """
    Prefer hardware virtual_id(), fall back to sim .id.
    """
    # Real hardware ID
    if hasattr(robot, "virtual_id") and callable(robot.virtual_id):
        try:
            return int(robot.virtual_id())
        except Exception:
            pass

    # Simulation ID
    if hasattr(robot, "id"):
        try:
            return int(robot.id)
        except Exception:
            pass

    return -1

def obstacle_distance(x, y):
    dx, dy = x - OBST_CX, y - OBST_CY
    return math.hypot(dx, dy)

def is_critical_obstacle(x, y, critical_margin=0.0):
    return obstacle_distance(x, y) < (OBST_RADIUS + critical_margin)

def soft_boundary_force(x, y, max_force=0.3, boundary_margin=0.08):
    fx, fy = 0.0, 0.0
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
    init_log()
    my_id = get_id(robot)
    robot.set_led(0, 180, 180)  # cyan normal
    logw(f"Robot {my_id} starting: Concentric Circle Formation")

    # -----------------= PHASE 1: Gossip to discover all robots -----------------=
    POSE_FMT  = "ffi"  # float x, float y, int id
    POSE_SIZE = struct.calcsize(POSE_FMT)

    poses = {}  # id -> (x, y)

    t_start = robot.get_clock()
    gossip_duration = 3.0  # seconds to share poses

    while robot.get_clock() - t_start < gossip_duration:
        pose = safe_pose(robot)
        if pose is not None:
            x, y, _ = pose
            # broadcast my pose
            msg = struct.pack(POSE_FMT, x, y, my_id)
            robot.send_msg(msg)
            # store my own pose too
            poses[my_id] = (x, y)

        # receive and store others' poses
        msgs = robot.recv_msg()
        if msgs:
            for m in msgs:
                if not m:
                    continue
                try:
                    px, py, pid = struct.unpack(POSE_FMT, m[:POSE_SIZE])
                    poses[int(pid)] = (float(px), float(py))
                except Exception:
                    pass

        robot.delay(50)

    # -----------------= PHASE 2: Assign target positions on circle -----------------=
    total_robots = len(poses)
    
    if total_robots == 0:
        logw(f"Robot {my_id}: No robots discovered, using default position")
        # Fallback: just go to some position on circle
        target_angle = 0.0
    else:
        # Sort robot IDs to get consistent ordering
        sorted_ids = sorted(poses.keys())
        my_index = sorted_ids.index(my_id)
        
        # Calculate equal angular spacing
        angle_spacing = 2 * math.pi / total_robots
        target_angle = my_index * angle_spacing
        
        logw(f"Robot {my_id}: {total_robots} robots, my index={my_index}, target_angle={target_angle:.3f}")

    # Calculate target position on circle
    target_x = CX + R_TARGET * math.cos(target_angle)
    target_y = CY + R_TARGET * math.sin(target_angle)
    
    logw(f"Robot {my_id} target: ({target_x:.3f}, {target_y:.3f})")

    # -----------------= PHASE 3: Move to target positions -----------------=
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
        dx, dy = x - CX, y - CY
        current_r = math.hypot(dx, dy)
        current_angle = math.atan2(dy, dx)
        
        # Calculate errors
        radial_error = R_TARGET - current_r
        angular_error = wrap_angle(target_angle - current_angle)
        
        # Unit vectors for radial and tangential directions
        if current_r > 1e-6:
            ur = dx / current_r, dy / current_r  # radial unit vector (outward)
            ut = -ur[1], ur[0]                  # tangential unit vector (CCW)
            if DIRECTION < 0:
                ut = -ut[0], -ut[1]             # tangential unit vector (CW)
        else:
            ur = (1.0, 0.0)
            ut = (0.0, 1.0) if DIRECTION > 0 else (0.0, -1.0)

        # Base control: move toward target position
        v_radial = K_RADIAL * radial_error
        v_tangential = K_ANGULAR * angular_error
        
        vx = v_radial * ur[0] + v_tangential * ut[0]
        vy = v_radial * ur[1] + v_tangential * ut[1]
        
        # -----------------= PHASE 4: Fine-tune spacing using neighbor information -----------------=
        # Comment out this entire block if you don't want continuous spacing adjustment
        # Receive current positions for spacing adjustment
        msgs = robot.recv_msg()
        current_neighbors = {}
        if msgs:
            for m in msgs:
                if not m:
                    continue
                try:
                    px, py, pid = struct.unpack(POSE_FMT, m[:POSE_SIZE])
                    if int(pid) != my_id:
                        current_neighbors[int(pid)] = (float(px), float(py))
                except Exception:
                    pass
        
        # If we have neighbor info, adjust spacing
        if len(current_neighbors) > 0:
            neighbor_angles = []
            for nid, (nx, ny) in current_neighbors.items():
                ndx, ndy = nx - CX, ny - CY
                n_angle = math.atan2(ndy, ndx)
                neighbor_angles.append(n_angle)
            
            # Find closest neighbors
            min_gap = float('inf')
            for n_angle in neighbor_angles:
                gap = abs(wrap_angle(n_angle - current_angle))
                if gap < min_gap:
                    min_gap = gap
            
            # Ideal gap is 2*pi / total_robots
            ideal_gap = 2 * math.pi / total_robots if total_robots > 0 else math.pi/4
            
            # Adjust tangential velocity based on spacing
            if min_gap < ideal_gap * 0.7:
                # Too close to neighbor, slow down
                v_tangential *= 0.6
            elif min_gap > ideal_gap * 1.3:
                # Too far from neighbor, speed up
                v_tangential *= 1.4
            
            # Recalculate velocity with spacing adjustment
            vx = v_radial * ur[0] + v_tangential * ut[0]
            vy = v_radial * ur[1] + v_tangential * ut[1]

        # Add boundary cushion
        bfx, bfy = soft_boundary_force(x, y)
        vx += bfx
        vy += bfy

        # Check if we're close to target
        position_error = math.hypot(target_x - x, target_y - y)
        on_target = position_error < 0.05 and abs(radial_error) < 0.03
        
        # LED feedback
        if on_target:
            formation_achieved = True
            robot.set_led(0, 255, 0)  # green when on target
            # Stop moving by setting velocity to zero
            vx, vy = 0, 0
        else:
            if abs(radial_error) > 0.1:
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
            if abs_err < 0.5:      fwd = FWD_FAST
            elif abs_err < 1.2:    fwd = FWD_FAST * 0.7
            else:                  fwd = FWD_SLOW

            turn = clamp(TURN_K * err, -1.5, 1.5)

            left  = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
            right = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
            robot.set_vel(left, right)

        # Continue broadcasting position for spacing adjustment (only if not achieved, but if we stop adjusting, we can stop broadcasting)
        # If you want to stop broadcasting after formation, condition on `not formation_achieved`
        if not formation_achieved:
            msg = struct.pack(POSE_FMT, x, y, my_id)
            robot.send_msg(msg)

        # Periodic status
        now = robot.get_clock()
        if now - last_logw_time > logw_PERIOD:
            logw(f"Robot {my_id}: pos [{x:.3f}, {y:.3f}], error={position_error:.3f}, neighbors={len(current_neighbors)}")
            last_logw_time = now

        robot.delay(40)