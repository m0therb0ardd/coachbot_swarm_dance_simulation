# -*- coding: utf-8 -*-
import math
import random
import struct
import time
import os 

# ----------------- Logging (sim + hardware) -----------------
log = None       # /home/pi/control/experiment_log
log_out = None   # /home/pi/experiment_output

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
REPULSE_RADIUS  = 0.75
REPULSE_GAIN    = 0.12 
NOISE_GAIN      = 0.12
FWD_GAIN        = 0.95
LEFT_BIAS_VX    = 0.00

# --- communication ---
POSE_FMT = "ffi"  # x, y, id
POSE_SIZE = struct.calcsize(POSE_FMT)
NEIGHBOR_TIMEOUT = 1.0  # seconds before considering neighbor data stale

# dancer no-go circle (meters)
FEET = 0.6048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
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

def get_id(robot):
    vid_attr = getattr(robot, "virtual_id", None)
    try:
        return vid_attr() if callable(vid_attr) else int(vid_attr or 0)
    except:
        return -1

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

# ----------------- main -----------------
def usr(robot):
    init_log()
    robot.delay(3000)
    my_id = get_id(robot)
    robot.set_led(0, 180, 180)
    logw(f"Robot {my_id} starting JITTERY REPULSION WITH P2P COMMS")
    
    # Dictionary to store neighbor positions with timestamps
    neighbors = {}  # format: {robot_id: (x, y, timestamp)}
    
    start_time = robot.get_clock()
    last_logw = start_time
    last_broadcast = start_time
    broadcast_interval = 0.1  # broadcast every 100ms

    while True:
        current_time = robot.get_clock()
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
            logw(f"ROBOT {my_id} OBSTACLE STOP at [{x:.3f}, {y:.3f}]")
            robot.delay(40)
            continue

        # 2) emergency boundary hold
        if is_critical_boundary(x, y):
            robot.set_vel(0, 0)
            robot.set_led(255, 50, 0)
            logw(f"ROBOT {my_id} CRITICAL STOP at [{x:.3f}, {y:.3f}]")
            robot.delay(40)
            continue

        # 3) BROADCAST our position periodically
        if current_time - last_broadcast > broadcast_interval:
            msg = struct.pack(POSE_FMT, x, y, my_id)
            robot.send_msg(msg)
            last_broadcast = current_time

        # 4) RECEIVE neighbor positions
        msgs = robot.recv_msg()
        if msgs:
            for m in msgs:
                if not m:
                    continue
                try:
                    nx, ny, nid = struct.unpack(POSE_FMT, m[:POSE_SIZE])
                    nid = int(nid)
                    if nid != my_id:  # don't store our own messages
                        neighbors[nid] = (float(nx), float(ny), current_time)
                except Exception as e:
                    pass

        # 5) Remove stale neighbors (older than NEIGHBOR_TIMEOUT)
        stale_neighbors = []
        for nid, (nx, ny, timestamp) in neighbors.items():
            if current_time - timestamp > NEIGHBOR_TIMEOUT:
                stale_neighbors.append(nid)
        for nid in stale_neighbors:
            del neighbors[nid]

        # 6) base velocity from soft boundary + optional left bias
        bfx, bfy = soft_boundary_force(x, y)
        boundary_warn = (abs(bfx) + abs(bfy)) > 1e-6
        if boundary_warn: 
            robot.set_led(255, 150, 0)  # orange near boundary
        else:             
            robot.set_led(0, 180, 180)  # cyan normal

        vx = bfx + LEFT_BIAS_VX
        vy = bfy

        # 7) add soft obstacle repulsion
        obx, oby = soft_obstacle_force(x, y)
        vx += obx
        vy += oby

        # 8) PEER-TO-PEER REPULSION from neighbors
        repulsion_count = 0
        for nid, (nx, ny, timestamp) in neighbors.items():
            dx, dy = x - nx, y - ny
            r2 = dx*dx + dy*dy
            if 1e-5 < r2 < (REPULSE_RADIUS * REPULSE_RADIUS):
                inv = 1.0 / r2
                s = REPULSE_GAIN * inv
                vx += s * dx
                vy += s * dy
                repulsion_count += 1

        # 9) always add jitter to feel glitchy
        ang1 = random.uniform(-math.pi, math.pi)
        ang2 = random.uniform(-math.pi, math.pi)
        vx += NOISE_GAIN * math.cos(ang1) + 0.6 * NOISE_GAIN * math.cos(ang2)
        vy += NOISE_GAIN * math.sin(ang1) + 0.6 * NOISE_GAIN * math.sin(ang2)

        # 10) map (vx,vy) -> differential drive
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

        # 11) light console output without spamming
        now = robot.get_clock()
        if now - last_logw > 2.0:
            logw(f"Robot {my_id} pos [{x:.3f}, {y:.3f}] - neighbors: {len(neighbors)}")
            last_logw = now

        robot.delay(40)