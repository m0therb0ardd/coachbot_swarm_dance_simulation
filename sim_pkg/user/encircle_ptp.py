# -*- coding: utf-8 -*-
from __future__ import division
import math
import struct

# --- field bounds (meters) ---
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

# --- dancer no-go circle (meters) ---
FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
OBST_CX, OBST_CY = (-0.1, 0.475)

# --- SINGLE CIRCLE parameters ---
CIRCLE_RADIUS = SAFE_BUBBLE + 0.35
CIRCLE_DIR = -1  # -1 = CW, +1 = CCW

# --- control gains ---
MAX_WHEEL = 35
TURN_K = 2.5
FWD_FAST = 0.70
FWD_SLOW = 0.35
FWD_MIN = 0.40
CMD_SMOOTH = 0.25
LOOP_DT_MS = 40

# --- formation control ---
K_RADIAL = 1.5           # Pull to correct radius
K_TARGET_ANGLE = 0.50    # Pull to target angle position
K_SPACING = 0.25         # Push away from neighbors if too close

# --- boundary ---
SOFT_MARGIN = 0.08
CRIT_MARGIN = 0.02
SOFT_MAX_F = 0.35

# --- messaging ---
HB_FMT = 'ffffi'    # x, y, theta, timestamp, vid
HB_BYTES = struct.calcsize(HB_FMT)
HB_DT = 0.10
STALE_S = 0.6

def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def wrap_angle(a):
    while a > math.pi:
        a -= 2.0*math.pi
    while a <= -math.pi:
        a += 2.0*math.pi
    return a

def soft_boundary_force(x, y):
    fx = 0.0
    fy = 0.0
    if x < X_MIN + SOFT_MARGIN:
        fx += SOFT_MAX_F * (1.0 - (x - X_MIN)/SOFT_MARGIN)
    elif x > X_MAX - SOFT_MARGIN:
        fx -= SOFT_MAX_F * (1.0 - (X_MAX - x)/SOFT_MARGIN)
    if y < Y_MIN + SOFT_MARGIN:
        fy += SOFT_MAX_F * (1.0 - (y - Y_MIN)/SOFT_MARGIN)
    elif y > Y_MAX - SOFT_MARGIN:
        fy -= SOFT_MAX_F * (1.0 - (Y_MAX - y)/SOFT_MARGIN)
    return fx, fy

def boundary_state(x, y):
    if (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
        y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN):
        return 2
    if (x < X_MIN + SOFT_MARGIN or x > X_MAX - SOFT_MARGIN or
        y < Y_MIN + SOFT_MARGIN or y > Y_MAX - SOFT_MARGIN):
        return 1
    return 0

def safe_pose(robot):
    p = robot.get_pose()
    if p and len(p) >= 3:
        return float(p[0]), float(p[1]), float(p[2])
    return None

def usr(robot):
    robot.delay(500)
    
    try:
        vid = robot.virtual_id()
    except:
        vid = 0
    
    print('Robot %d: Equidistant circle formation starting' % vid)
    
    neighbors = {}      # nid -> (x, y, theta, timestamp)
    last_seen = {}
    last_hb = -1e9
    last_left = 0
    last_right = 0
    last_print = 0.0
    
    # Wake up
    robot.set_vel(20, 20)
    robot.delay(150)
    
    while True:
        pose = safe_pose(robot)
        if not pose:
            robot.set_vel(0, 0)
            robot.delay(LOOP_DT_MS)
            continue
        
        x = pose[0]
        y = pose[1]
        th = pose[2]
        now = robot.get_clock()
        
        # Calculate position relative to obstacle center
        dx = x - OBST_CX
        dy = y - OBST_CY
        r = math.sqrt(dx*dx + dy*dy)
        
        # My angle around circle
        my_angle = math.atan2(dy, dx)
        
        # Boundary check
        b = boundary_state(x, y)
        if b == 2:
            robot.set_led(100, 0, 0)
            robot.set_vel(0, 0)
            robot.delay(LOOP_DT_MS)
            continue
        
        # Obstacle safety check
        if r < OBST_RADIUS:
            print('Robot %d: Too close to obstacle!' % vid)
            robot.set_led(100, 0, 0)
            robot.set_vel(0, 0)
            robot.delay(LOOP_DT_MS)
            continue
        
        # --- Heartbeat send ---
        if now - last_hb >= HB_DT:
            try:
                robot.send_msg(struct.pack(HB_FMT, float(x), float(y), float(th), float(now), int(vid)))
                last_hb = now
            except:
                pass
        
        # --- Receive heartbeats ---
        msgs = robot.recv_msg()
        if msgs:
            for m in msgs:
                try:
                    nx, ny, nth, ts, nid = struct.unpack(HB_FMT, m[:HB_BYTES])
                    if int(nid) != vid:
                        neighbors[int(nid)] = (nx, ny, nth, ts)
                        last_seen[int(nid)] = now
                except:
                    pass
        
        # Prune stale neighbors
        cut = now - STALE_S
        for nid in list(neighbors.keys()):
            if last_seen.get(nid, 0.0) < cut:
                neighbors.pop(nid, None)
                last_seen.pop(nid, None)
        
        # --- EQUIDISTANT FORMATION LOGIC ---
        
        # Total number of robots (me + neighbors)
        total_robots = len(neighbors) + 1
        
        # Create sorted list of all robot IDs (including me)
        all_robot_ids = sorted([vid] + list(neighbors.keys()))
        
        # My index in sorted list (determines my target slot)
        my_index = all_robot_ids.index(vid)
        
        # Calculate target angle for perfect spacing
        # Divide circle into N equal segments
        angle_spacing = (2.0 * math.pi) / total_robots
        
        # My target angle (starting from 0, going CW or CCW)
        if CIRCLE_DIR < 0:  # CW
            target_angle = -my_index * angle_spacing
        else:  # CCW
            target_angle = my_index * angle_spacing
        
        target_angle = wrap_angle(target_angle)
        
        # Angular error (how far am I from my target position?)
        angle_error = wrap_angle(target_angle - my_angle)
        
        # --- Formation control forces ---
        
        # 1. Radial component - maintain circle radius
        if r < 1e-6:
            urx, ury = 1.0, 0.0
        else:
            urx, ury = dx/r, dy/r
        
        radial_error = CIRCLE_RADIUS - r
        radial_force = K_RADIAL * radial_error
        radial_force = clamp(radial_force, -0.15, 0.15)
        
        vx = radial_force * urx
        vy = radial_force * ury
        
        # 2. Tangential component - move toward target angle
        # Tangent direction
        utx, uty = -ury, urx
        if CIRCLE_DIR < 0:
            utx, uty = -utx, -uty
        
        # Convert angular error to tangential force
        # Positive angle_error means I need to move in positive tangent direction
        tangent_force = K_TARGET_ANGLE * angle_error
        tangent_force = clamp(tangent_force, -0.30, 0.30)
        
        vx += tangent_force * utx
        vy += tangent_force * uty
        
        # 3. Local spacing - avoid getting too close to immediate neighbors
        # This prevents collisions while moving to target positions
        for nid in neighbors:
            nx, ny, nth, ts = neighbors[nid]
            
            # Distance to neighbor
            ndx = x - nx
            ndy = y - ny
            dist = math.sqrt(ndx*ndx + ndy*ndy)
            
            # If too close, add repulsive force
            min_distance = 0.20
            if dist < min_distance and dist > 1e-6:
                repel_strength = K_SPACING * (min_distance - dist) / min_distance
                vx += repel_strength * (ndx / dist)
                vy += repel_strength * (ndy / dist)
        
        # 4. Boundary forces
        bfx, bfy = soft_boundary_force(x, y)
        vx += bfx
        vy += bfy
        
        # --- Convert to wheel commands ---
        speed = math.sqrt(vx*vx + vy*vy)
        if speed < 1e-6:
            vx = 0.05 * utx
            vy = 0.05 * uty
        
        desired_heading = math.atan2(vy, vx)
        heading_error = wrap_angle(desired_heading - th)
        
        # Speed control based on heading error
        ae = abs(heading_error)
        if ae < 0.5:
            fwd = FWD_FAST
        elif ae < 1.0:
            fwd = FWD_FAST * 0.7
        else:
            fwd = FWD_SLOW
        
        # Slow down when close to target position
        if abs(angle_error) < 0.2 and abs(radial_error) < 0.05:
            fwd = fwd * 0.5
        
        if b == 1:
            fwd = fwd * 0.75
        fwd = max(fwd, FWD_MIN)
        
        turn = clamp(TURN_K * heading_error, -1.5, 1.5)
        lcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
        rcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
        
        # Smooth commands
        left = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * last_left)
        right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * last_right)
        last_left = left
        last_right = right
        
        robot.set_vel(left, right)
        
        # LED feedback based on formation quality
        radius_error = abs(r - CIRCLE_RADIUS)
        angle_error_abs = abs(angle_error)
        
        if radius_error < 0.03 and angle_error_abs < 0.1:
            # Perfect position - green
            robot.set_led(0, 100, 0)
        elif radius_error < 0.08 and angle_error_abs < 0.3:
            # Close - cyan
            robot.set_led(0, 80, 80)
        elif b == 1:
            # Near boundary - amber
            robot.set_led(100, 60, 0)
        else:
            # Moving to position - blue
            robot.set_led(0, 50, 100)
        
        # Status output
        if now - last_print > 2.0:
            print('Robot %d: slot=%d/%d target_ang=%.2f my_ang=%.2f err=%.2f r=%.3f' % 
                  (vid, my_index, total_robots, target_angle, my_angle, angle_error, r))
            last_print = now
        
        robot.delay(LOOP_DT_MS)