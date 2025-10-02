# # sim_pkg/user/translate_circle_left_center_static.py
# # Keep a circular formation (IDs 1..29) and translate it left.
# # ID 0 = dancer/center: stays stationary. Formation stops at left wall
# # OR before any follower would enter the dancer's safety bubble.

# import math, struct, random

# # ===== IDs & formation =====
# CENTER_ID = 0
# FORMATION_RADIUS = 0.50    # meters — set to your ring radius from init CSV

# # ===== Motion plan (left translation) =====
# SHIFT_PER_TICK = 0.010     # meters per control tick (~10 mm/tick)
# STOP_MARGIN    = 0.08      # stop when circle's left edge is this far from wall

# # ===== Center safety bubble (HARD STOP vs dancer) =====
# CENTER_STOP_R  = 0.09      # followers must NOT come within this radius of the dancer

# # ===== Comm & spacing =====
# COMM_RADIUS  = 0.60        # <= config.json COMM_RANGE
# NEIGHBOR_MIN = 0.10        # soft separation radius
# SEPAR_W      = 0.90        # modest separation to avoid bumping

# # ===== Robot safety & arena (match your config.json) =====
# # ARENA       = (-1.5, 1.5, -1.0, 1.0)  # xmin, xmax, ymin, ymax
# ARENA = (-5.0, 5.0, -5.0, 5.0)
# ROBOT_RAD   = 0.06
# HARD_RADIUS = 0.09                    # gentle bubble to avoid freezes

# # ===== Control =====
# MAX_WHEEL = 40
# TURN_K    = 2.5
# FWD_FAST  = 0.8
# FWD_SLOW  = 0.35
# FWD_MIN_WHEN_THREAT = 0.20
# EPS = 1e-3

# # +1 = move right, -1 = move left
# MOVE_DIR = 1


# # ===== Utils =====
# def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
# def wrap_angle(a):
#     while a >  math.pi: a -= 2*math.pi
#     while a <= -math.pi: a += 2*math.pi
#     return a
# def get_id(robot):
#     vid_attr = getattr(robot, "virtual_id", None)
#     return vid_attr() if callable(vid_attr) else int(vid_attr)

# def separation_push(x, y, nbrs):
#     vx = vy = 0.0
#     for nx, ny, _ in nbrs:
#         dx, dy = x - nx, y - ny
#         d = math.hypot(dx, dy) + 1e-6
#         if d < NEIGHBOR_MIN:
#             s = (NEIGHBOR_MIN - d) / NEIGHBOR_MIN
#             vx += SEPAR_W * (dx/d) * s
#             vy += SEPAR_W * (dy/d) * s
#     return vx, vy

# def hard_bubble(x, y, nbrs):
#     ax = ay = 0.0; threat = False
#     for nx, ny, _ in nbrs:
#         dx, dy = x - nx, y - ny
#         d = math.hypot(dx, dy) + 1e-6
#         if d < HARD_RADIUS:
#             threat = True
#             push = (HARD_RADIUS - d) / HARD_RADIUS
#             ax += (dx/d) * 3.0 * push
#             ay += (dy/d) * 3.0 * push
#     return ax, ay, threat

# # ===== Center/dancer (ID 0): stationary, just broadcast pose =====
# def center_tick(robot, my_pose):
#     if my_pose: x, y, th = my_pose
#     else:       x, y, th = 0.0, 0.0, 0.0
#     robot.send_msg(struct.pack('fffI', x, y, th, CENTER_ID))
#     robot.set_led(255, 0, 180)  # magenta
#     robot.set_vel(0, 0)         # never moves
#     robot.delay(50)             # spacing for followers

# # ===== Main =====
# def usr(robot):
#     my_id = get_id(robot)

#     # LEDs: center magenta; followers cyan-ish
#     if my_id == CENTER_ID: robot.set_led(255, 0, 180)
#     else:                  robot.set_led(0, 180, 180)

#     # Per-follower memory
#     rel_offset = None      # my initial offset from the dancer (x0 - cx0, y0 - cy0)
#     center0 = None         # dancer's initial pose (cx0, cy0, th0)
#     ticks = 0              # local tick counter (~shared rate)
#     s_stop_wall = None     # wall-based stop shift
#     s_stop_center = None   # center-safety stop shift
#     s_stop = None          # final stop shift = min(wall, center)

#     while True:
#         my_pose = robot.get_pose()

#         # === Center robot stays put ===
#         if my_id == CENTER_ID:
#             center_tick(robot, my_pose)
#             continue

#         # Followers broadcast their pose
#         if my_pose:
#             x, y, th = my_pose
#             robot.send_msg(struct.pack('fffI', x, y, th, my_id))

#         # Send -> short delay -> receive
#         robot.delay(50)
#         msgs = robot.recv_msg() or []

#         # Parse neighbors & dancer pose
#         dancer = None; nbrs = []
#         if my_pose:
#             x, y, th = my_pose
#             for m in msgs:
#                 try:
#                     nx, ny, nth, nid = struct.unpack('fffI', m)
#                     if nid == CENTER_ID:
#                         dancer = (nx, ny, nth)
#                         continue  # don't include in nbrs
#                     dx, dy = nx - x, ny - y
#                     if dx*dx + dy*dy <= COMM_RADIUS*COMM_RADIUS:
#                         nbrs.append((nx, ny, nth))
#                 except Exception:
#                     pass

#         # Wait until we've seen the dancer once (to lock the reference)
#         if my_pose and dancer and center0 is None:
#             center0 = (dancer[0], dancer[1], dancer[2])
#             # my initial offset from dancer (defines my place on the ring)
#             rel_offset = (x - center0[0], y - center0[1])

#             # 1) wall-based max shift: circle left edge at xmin + STOP_MARGIN
#             # xmin, _, _, _ = ARENA
#             # s_stop_wall = max(0.0, center0[0] - (xmin + STOP_MARGIN + FORMATION_RADIUS))
#             xmin, xmax, ymin, ymax = ARENA
#             if MOVE_DIR < 0:
#                 # moving left (old behavior)
#                 s_stop_wall = max(0.0, center0[0] - (xmin + STOP_MARGIN + FORMATION_RADIUS))
#             else:
#                 # moving right (NEW)
#                 s_stop_wall = max(0.0, (xmax - STOP_MARGIN - FORMATION_RADIUS) - center0[0])


#             # 2) center-safety max shift (global; same for all):
#             # For the worst-case robot (rightmost point), distance to center is |R - s|.
#             # Solve |R - s| >= CENTER_STOP_R  =>  s <= R - sqrt(R^2 - CENTER_STOP_R^2)
#             # delta = FORMATION_RADIUS*FORMATION_RADIUS - CENTER_STOP_R*CENTER_STOP_R
#             # delta = max(0.0, delta)
#             # s_stop_center = FORMATION_RADIUS - math.sqrt(delta)
#             s_stop_center = max(0.0, FORMATION_RADIUS - CENTER_STOP_R)

#             # Final shared stop
#             s_stop = min(s_stop_wall, s_stop_center)

#         # If we still don't know the dancer pose, idle gently forward
#         if center0 is None or rel_offset is None or s_stop is None:
#             robot.set_vel(10, 10)
#             robot.delay()
#             continue

#         # Shared left-shift amount (same for all robots)
#         s = min(ticks * SHIFT_PER_TICK, s_stop)

#         # Target for me = dancer initial center shifted left by s, plus my saved offset
#         # tx = center0[0] - s + rel_offset[0]
#         tx = center0[0] + MOVE_DIR * s + rel_offset[0]
#         ty = center0[1]      + rel_offset[1]

#         # Build desired velocity toward target
#         ex, ey = tx - x, ty - y
#         kp = 1.6
#         vx = kp * ex
#         vy = kp * ey

#         # Gentle spacing and safety between followers
#         sx, sy = separation_push(x, y, nbrs)
#         vx += sx; vy += sy
#         ax, ay, threat = hard_bubble(x, y, nbrs)
#         vx += ax; vy += ay

#         # EXTRA safety: if I'm already within the dancer bubble now, stop me
#         if dancer:
#             dx_c, dy_c = x - dancer[0], y - dancer[1]
#             if math.hypot(dx_c, dy_c) < CENTER_STOP_R:
#                 robot.set_vel(0, 0)
#                 robot.delay()
#                 ticks += 1
#                 continue

#         # If finished shifting and I'm essentially at target, stop
#         if abs(s - s_stop) < 1e-6 and (abs(ex) + abs(ey) < 0.01):
#             robot.set_vel(0, 0)
#             robot.delay()
#             # do not advance ticks further; everyone will stay put
#             continue

#         # Map (vx,vy) to wheels
#         if abs(vx) + abs(vy) < EPS:
#             robot.set_vel(0, 0)
#         else:
#             hdg = math.atan2(vy, vx)
#             err = wrap_angle(hdg - th)
#             fwd  = FWD_FAST if abs(err) < 0.9 else FWD_SLOW
#             if threat: fwd = max(FWD_MIN_WHEN_THREAT, 0.5 * fwd)
#             turn = clamp(TURN_K * err, -1.0, 1.0)
#             left  = clamp(int(MAX_WHEEL * (fwd - 0.8*turn)), -50, 50)
#             right = clamp(int(MAX_WHEEL * (fwd + 0.8*turn)), -50, 50)
#             robot.set_vel(left, right)

#         ticks += 1
#         robot.delay()  # ~20 ms




# ######################## NEW STUFFFFFFFFFFFFFFFFFFFF
# sim_pkg/user/translate_circle_left.py
# Keep a rigid 5-robot ring and translate it screen-left until:
#  - the ring edge is within STOP_MARGIN of the left wall, OR
#  - the ring is within SAFE_BUBBLE of the dancer circle.
# Uses a shared field center and sim clock so bots stay in formation.
# sim_pkg/user/translate_circle_left_with_boundaries.py
# Keep a rigid 5-robot ring and translate it screen-left with boundary enforcement
# sim_pkg/user/translate_circle_left_smooth.py
# Smooth version with boundary enforcement

import math, struct

# ===== Testbed bounds & center =====
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35
CX, CY = (-0.1, 0.475)

# ===== Dancer no-go circle =====
FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN

# ===== Translation plan =====
MOVE_DIR    = -1        # -1 = screen-left, +1 = screen-right
SHIFT_RATE  = 0.18     # m/s center velocity
STOP_MARGIN = 0.08     # keep ring this far from wall at stop

# ===== Drive control =====
MAX_WHEEL = 40
TURN_K    = 3.0
FWD_FAST  = 0.8
FWD_SLOW  = 0.30
EPS       = 1e-3

# ===== Rigid-body locking gains =====
KX = 1.2
KY = 2.0
KR = 2.6

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
    vid_attr = getattr(robot, "virtual_id", None)
    return vid_attr() if callable(vid_attr) else int(vid_attr or 0)

def soft_boundary_force(x, y, max_force=0.5, boundary_margin=0.1):
    """Apply soft repulsive force near boundaries instead of hard stops"""
    fx, fy = 0.0, 0.0
    
    # X boundaries
    if x < X_MIN + boundary_margin:
        strength = 1.0 - (x - X_MIN) / boundary_margin
        fx += max_force * strength
    elif x > X_MAX - boundary_margin:
        strength = 1.0 - (X_MAX - x) / boundary_margin  
        fx -= max_force * strength
        
    # Y boundaries  
    if y < Y_MIN + boundary_margin:
        strength = 1.0 - (y - Y_MIN) / boundary_margin
        fy += max_force * strength
    elif y > Y_MAX - boundary_margin:
        strength = 1.0 - (Y_MAX - y) / boundary_margin
        fy -= max_force * strength
        
    return fx, fy

def is_critical_boundary(x, y, critical_margin=0.03):
    """Only emergency stop when dangerously close"""
    return (x < X_MIN + critical_margin or x > X_MAX - critical_margin or 
            y < Y_MIN + critical_margin or y > Y_MAX - critical_margin)

def usr(robot):
    robot.set_led(0, 180, 180)  # cyan
    my_id = get_id(robot)
    
    print(f"Robot {my_id} starting - Smooth boundary enforcement")

    # Per-robot state
    rel_off = None
    R_form  = None
    s_stop  = None
    t0      = None
    started = False
    emergency_stop = False
    last_print_time = 0

    # slight spawn settle
    robot.delay(200)

    while True:
        pose = safe_pose(robot)
        if pose is None:
            robot.set_vel(0,0)
            robot.delay(20)
            continue
        x, y, th = pose

        # CRITICAL BOUNDARY CHECK - Only emergency stop when absolutely necessary
        if is_critical_boundary(x, y, 0.02):
            if not emergency_stop:
                print(f"ROBOT {my_id} CRITICAL STOP! Position: [{x:.3f}, {y:.3f}]")
                emergency_stop = True
                robot.set_led(255, 50, 0)  # Orange = critical stop
            
            robot.set_vel(0, 0)
            robot.delay(20)
            continue
        elif emergency_stop:
            # Recovered from critical boundary
            emergency_stop = False
            robot.set_led(0, 180, 180)
            print(f"Robot {my_id} recovered from critical boundary")

        # Lock initial offset & compute stop distance once
        if rel_off is None:
            rel_off = (x - CX, y - CY)
            R_form  = math.hypot(*rel_off)

            # Wall-limited shift with reasonable safety margin
            safety_buffer = 0.05  # Reduced from 0.1 for smoother movement
            if MOVE_DIR < 0:  # moving left
                s_wall = max(0.0, CX - (X_MIN + STOP_MARGIN + safety_buffer + R_form))
            else:  # moving right
                s_wall = max(0.0, (X_MAX - STOP_MARGIN - safety_buffer - R_form) - CX)
                
            s_obst = max(0.0, R_form - SAFE_BUBBLE)
            s_stop = min(s_wall, s_obst)

            # Use full rate for smooth movement
            sim_safe_rate = SHIFT_RATE  # 100% of original speed

            # synced start
            now = robot.get_clock()
            t0 = math.floor(now * 2.0) / 2.0 + 0.5

            print(f"Robot {my_id} locked: R={R_form:.3f}, s_stop={s_stop:.3f}")

        # Wait for the shared start time
        t = robot.get_clock()
        if not started:
            if t < t0:
                robot.set_vel(0,0)
                robot.delay(10)
                continue
            started = True
            print(f"Robot {my_id} started moving")

        # Planned center shift with full speed
        s = min(max(0.0, t - t0) * SHIFT_RATE, s_stop)

        # Moving center and rigid target for me
        Cx = CX + MOVE_DIR * s
        Cy = CY
        
        # SOFTER center boundaries
        Cx = max(X_MIN + R_form + 0.08, min(X_MAX - R_form - 0.08, Cx))
        Cy = max(Y_MIN + R_form + 0.08, min(Y_MAX - R_form - 0.08, Cy))
        
        tx = Cx + rel_off[0]
        ty = Cy + rel_off[1]

        # Position error → velocity field
        ex, ey = tx - x, ty - y
        vx = KX * ex
        vy = KY * ey

        # Feed-forward at full rate
        vx += MOVE_DIR * SHIFT_RATE

        # Radial lock to initial offset
        rx, ry = x - Cx, y - Cy
        vx += KR * (rel_off[0] - rx)
        vy += KR * (rel_off[1] - ry)

        # SOFT BOUNDARY FORCE - instead of hard stops
        boundary_fx, boundary_fy = soft_boundary_force(x, y, max_force=0.3, boundary_margin=0.08)
        vx += boundary_fx
        vy += boundary_fy
        
        # Visual feedback for boundary proximity (no stopping)
        boundary_warning = is_critical_boundary(x, y, 0.05)  # Wider margin for warning
        if boundary_warning:
            robot.set_led(255, 150, 0)  # Yellow = boundary warning
        else:
            robot.set_led(0, 180, 180)  # Cyan = normal

        # Less frequent printing for smoother performance
        current_time = robot.get_clock()
        if current_time - last_print_time > 3.0:  # Every 3 seconds
            print(f"Robot {my_id} at [{x:.3f}, {y:.3f}], s={s:.3f}")
            last_print_time = current_time

        # Finished? stop & hold
        if abs(s - s_stop) < 1e-6 and (abs(ex) + abs(ey) < 0.01):
            robot.set_vel(0, 0)
            robot.set_led(0, 255, 0)  # Green = completed safely
            robot.delay()
            continue

        # Map (vx,vy) → wheels with SMOOTHER settings
        if abs(vx) + abs(vy) < EPS:
            robot.set_vel(0, 0)
        else:
            hdg = math.atan2(vy, vx)
            err = wrap_angle(hdg - th)
            
            # Smoother forward speed selection
            abs_err = abs(err)
            if abs_err < 0.5:
                fwd = FWD_FAST  # Fast when well-aligned
            elif abs_err < 1.2:
                fwd = FWD_FAST * 0.7  # Medium when somewhat misaligned  
            else:
                fwd = FWD_SLOW  # Slow when very misaligned
                
            # Smoother turning with less aggressive response
            turn = clamp(TURN_K * err, -1.5, 1.5)
            
            # Use 90% of max wheel for smoothness
            left  = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
            right = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
            
            robot.set_vel(left, right)

        robot.delay(20)