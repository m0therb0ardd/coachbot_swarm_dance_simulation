import math, struct, random

# ===== Field & center =====
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35
CX, CY = (-0.1, 0.475)

# ===== Dancer no-go circle =====
FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
SPIRAL_STOP_DISTANCE = SAFE_BUBBLE + 0.15  # Stop before entering dancer zone

# ===== Motion parameters =====
MAX_WHEEL = 40

def clamp(v, lo, hi): return max(lo, min(v, hi))
def wrap_angle(a):
    while a > math.pi: a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a
def safe_pose(robot):
    p = robot.get_pose()
    if p and len(p) >= 3: return float(p[0]), float(p[1]), float(p[2])
    return None
def get_id(robot):
    vid_attr = getattr(robot, "virtual_id", None)
    return vid_attr() if callable(vid_attr) else int(vid_attr or 0)

def boundary_force(x, y):
    """Keep robots in bounds"""
    fx, fy = 0.0, 0.0
    boundary_margin = 0.1
    
    if x < X_MIN + boundary_margin:
        strength = 2.0 * (1.0 - (x - X_MIN) / boundary_margin)
        fx += strength
    elif x > X_MAX - boundary_margin:
        strength = 2.0 * (1.0 - (X_MAX - x) / boundary_margin)  
        fx -= strength
        
    if y < Y_MIN + boundary_margin:
        strength = 2.0 * (1.0 - (y - Y_MIN) / boundary_margin)
        fy += strength
    elif y > Y_MAX - boundary_margin:
        strength = 2.0 * (1.0 - (Y_MAX - y) / boundary_margin)
        fy -= strength
        
    return fx, fy

def strong_dancer_repulsion(x, y):
    """STRONG repulsion from dancer zone"""
    dx, dy = x - CX, y - CY
    dist = math.hypot(dx, dy)
    safe_dist = SAFE_BUBBLE + 0.2
    
    if dist < safe_dist:
        strength = 8.0 * ((safe_dist - dist) / safe_dist) ** 2
        fx = (dx/dist) * strength if dist > 0 else 1.0
        fy = (dy/dist) * strength if dist > 0 else 1.0
        return fx, fy
    return 0, 0

def usr(robot):
    my_id = get_id(robot)
    random.seed(my_id * 42)
    
    print(f"Robot {my_id} - SPIRALING INWARD")
    
    # Spiral parameters
    phase = 'spiral_inward'
    spiral_start_time = robot.get_clock()
    
    # Each robot has unique spiral parameters for visual variety
    spiral_angle_offset = (my_id % 10) * 0.4  # Stagger start angles
    spiral_speed = 0.12 + (my_id % 5) * 0.02  # Slightly different speeds
    rotation_speed = 1.8 + (my_id % 3) * 0.3   # Different rotation rates
    
    # Starting radius based on initial position
    start_x, start_y, _ = safe_pose(robot) or (CX + 0.8, CY, 0)
    start_radius = math.hypot(start_x - CX, start_y - CY)
    
    robot.set_led(150, 0, 150)  # Start purple - spiraling
    
    while True:
        pose = safe_pose(robot)
        if not pose:
            robot.set_vel(0, 0)
            robot.delay(20)
            continue
            
        x, y, th = pose
        t = robot.get_clock()
        
        dist_from_center = math.hypot(x - CX, y - CY)
        spiral_time = t - spiral_start_time
        
        # Check if we've reached the safe stopping distance
        if dist_from_center <= SPIRAL_STOP_DISTANCE:
            phase = 'orbit'
            robot.set_led(0, 150, 150)  # Teal - orbiting
        
        if phase == 'spiral_inward':
            # Beautiful spiral equation: radius decreases, angle increases
            progress = min(1.0, spiral_time / 15.0)  # 15 second spiral duration
            target_radius = start_radius * (1.0 - progress) + SPIRAL_STOP_DISTANCE * progress
            
            # Spiral angle increases over time with offset
            target_angle = spiral_time * rotation_speed + spiral_angle_offset
            
            # Target position on spiral
            target_x = CX + target_radius * math.cos(target_angle)
            target_y = CY + target_radius * math.sin(target_angle)
            
            # Move toward spiral target
            tx, ty = target_x, target_y
            ex, ey = tx - x, ty - y
            
            # Smooth movement toward target
            vx = 1.8 * ex
            vy = 1.8 * ey
            
            # Add tangent force for smooth spiraling (perpendicular to radius)
            tangent_angle = target_angle + math.pi/2
            tangent_strength = 0.4
            vx += tangent_strength * math.cos(tangent_angle)
            vy += tangent_strength * math.sin(tangent_angle)
            
            # Convert to wheel commands
            if abs(vx) + abs(vy) > 1e-3:
                hdg = math.atan2(vy, vx)
                err = wrap_angle(hdg - th)
                
                turn = clamp(2.2 * err, -1.8, 1.8)
                
                left = clamp(int(MAX_WHEEL * (0.15 - 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
                right = clamp(int(MAX_WHEEL * (0.15 + 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
                
                robot.set_vel(left, right)
            else:
                robot.set_vel(0, 0)
            
            # Beautiful spiral LED effect - pulsing purple
            spiral_pulse = math.sin(t * 2.5) * 0.4 + 0.6
            r = int(150 * spiral_pulse)
            g = int(50 * (1.0 - spiral_pulse * 0.3))
            b = int(150 * spiral_pulse)
            robot.set_led(r, g, b)
            
            # Progress reporting
            if int(t) % 4 == 0:
                progress_pct = (1.0 - (dist_from_center - SPIRAL_STOP_DISTANCE) / (start_radius - SPIRAL_STOP_DISTANCE)) * 100
                print(f"Robot {my_id} spiral progress: {progress_pct:.0f}%")
        
        elif phase == 'orbit':
            # Orbit around dancer at safe distance
            orbit_time = t - spiral_start_time
            orbit_angle = orbit_time * 1.2 + spiral_angle_offset
            
            target_x = CX + SPIRAL_STOP_DISTANCE * math.cos(orbit_angle)
            target_y = CY + SPIRAL_STOP_DISTANCE * math.sin(orbit_angle)
            
            # Move toward orbit position
            tx, ty = target_x, target_y
            ex, ey = tx - x, ty - y
            
            vx = 1.2 * ex
            vy = 1.2 * ey
            
            if abs(vx) + abs(vy) > 1e-3:
                hdg = math.atan2(vy, vx)
                err = wrap_angle(hdg - th)
                
                turn = clamp(1.8 * err, -1.5, 1.5)
                
                left = clamp(int(MAX_WHEEL * (0.08 - 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
                right = clamp(int(MAX_WHEEL * (0.08 + 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
                
                robot.set_vel(left, right)
            else:
                robot.set_vel(0, 0)
            
            # Orbit LED effect - pulsing teal
            orbit_pulse = math.sin(t * 3) * 0.5 + 0.5
            r = int(50 * (1.0 - orbit_pulse * 0.5))
            g = int(150 * orbit_pulse)
            b = int(150 * orbit_pulse)
            robot.set_led(r, g, b)
        
        # Safety forces
        boundary_fx, boundary_fy = boundary_force(x, y)
        dancer_fx, dancer_fy = strong_dancer_repulsion(x, y)
        
        if abs(boundary_fx) + abs(boundary_fy) + abs(dancer_fx) + abs(dancer_fy) > 0.5:
            safety_heading = math.atan2(boundary_fy + dancer_fy, boundary_fx + dancer_fx)
            current_heading = th
            safety_error = wrap_angle(safety_heading - current_heading)
            
            left_vel, right_vel = robot.get_vel() if hasattr(robot, 'get_vel') else (0, 0)
            safety_turn = clamp(2.0 * safety_error, -1.0, 1.0)
            left_vel = clamp(left_vel - int(MAX_WHEEL * 0.3 * safety_turn), -MAX_WHEEL, MAX_WHEEL)
            right_vel = clamp(right_vel + int(MAX_WHEEL * 0.3 * safety_turn), -MAX_WHEEL, MAX_WHEEL)
            robot.set_vel(left_vel, right_vel)
        
        robot.delay(20)