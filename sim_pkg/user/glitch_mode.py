
# import math, struct, random

# # ===== Field & center =====
# X_MIN, X_MAX = -1.2, 1.0
# Y_MIN, Y_MAX = -1.4, 2.35
# CX, CY = (-0.1, 0.475)

# # ===== Dancer no-go circle =====
# FEET = 0.3048
# OBST_DIAM_FT = 1.0
# OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
# OBST_MARGIN  = 0.03
# SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN

# # ===== Glitch parameters =====
# MAX_WHEEL = 40
# GLITCH_SPEED = 0.20

# def clamp(v, lo, hi): return max(lo, min(v, hi))
# def wrap_angle(a):
#     while a > math.pi: a -= 2*math.pi
#     while a <= -math.pi: a += 2*math.pi
#     return a
# def safe_pose(robot):
#     p = robot.get_pose()
#     if p and len(p) >= 3: return float(p[0]), float(p[1]), float(p[2])
#     return None
# def get_id(robot):
#     vid_attr = getattr(robot, "virtual_id", None)
#     return vid_attr() if callable(vid_attr) else int(vid_attr or 0)

# def glitch_boundary_force(x, y):
#     """Glitchy boundary response"""
#     fx, fy = 0.0, 0.0
#     boundary_margin = 0.1
    
#     if random.random() > 0.15:
#         if x < X_MIN + boundary_margin:
#             strength = 1.5 * (1.0 - (x - X_MIN) / boundary_margin)
#             fx += strength
#         elif x > X_MAX - boundary_margin:
#             strength = 1.5 * (1.0 - (X_MAX - x) / boundary_margin)  
#             fx -= strength
            
#         if y < Y_MIN + boundary_margin:
#             strength = 1.5 * (1.0 - (y - Y_MIN) / boundary_margin)
#             fy += strength
#         elif y > Y_MAX - boundary_margin:
#             strength = 1.5 * (1.0 - (Y_MAX - y) / boundary_margin)
#             fy -= strength
        
#     return fx, fy

# def glitch_dancer_repulsion(x, y):
#     """Glitchy dancer avoidance"""
#     dx, dy = x - CX, y - CY
#     dist = math.hypot(dx, dy)
#     safe_dist = SAFE_BUBBLE + 0.1
    
#     if random.random() < 0.2:
#         return 0, 0
        
#     if dist < safe_dist:
#         strength = 2.0 * ((safe_dist - dist) / safe_dist)
#         fx = (dx/dist) * strength if dist > 0 else random.random() - 0.5
#         fy = (dy/dist) * strength if dist > 0 else random.random() - 0.5
#         return fx, fy
#     return 0, 0

# def get_chaotic_oscillation(glitch_timer, my_id, phase):
#     """Generate chaotic oscillation patterns"""
#     base_freq = 2.0 + random.random() * 3.0  # Different frequency per robot
    
#     if phase == 'glitch_breakout':
#         # Very chaotic during breakout
#         x_osc = (math.sin(glitch_timer * base_freq) + 
#                 0.5 * math.sin(glitch_timer * base_freq * 2.3) +
#                 0.3 * math.sin(glitch_timer * base_freq * 0.7))
        
#         y_osc = (math.cos(glitch_timer * base_freq * 1.7) + 
#                 0.6 * math.sin(glitch_timer * base_freq * 3.1) +
#                 0.4 * math.cos(glitch_timer * base_freq * 0.9))
        
#         # Add random glitch spikes
#         if random.random() < 0.1:
#             x_osc += (random.random() - 0.5) * 2.0
#         if random.random() < 0.1:
#             y_osc += (random.random() - 0.5) * 2.0
            
#     elif phase == 'split':
#         # Still oscillating but more directional
#         x_osc = 0.7 * math.sin(glitch_timer * base_freq * 1.5)
#         y_osc = 0.5 * math.cos(glitch_timer * base_freq * 2.0)
        
#     else:  # hold
#         # Gentle wandering
#         x_osc = 0.2 * math.sin(glitch_timer * base_freq * 0.5)
#         y_osc = 0.2 * math.cos(glitch_timer * base_freq * 0.7)
    
#     return x_osc * 0.15, y_osc * 0.15  # Scale oscillations

# def get_glitch_color(phase, glitch_timer, target_side):
#     """Glitchy, flickering colors with oscillation patterns"""
#     if phase == 'glitch_breakout':
#         # Rapid color cycling with oscillation
#         r = int(128 + 127 * math.sin(glitch_timer * 5))
#         g = int(128 + 127 * math.sin(glitch_timer * 7))
#         b = int(128 + 127 * math.sin(glitch_timer * 3))
        
#         # Occasional blackouts
#         if random.random() < 0.05:
#             return (0, 0, 0)
#         return (r, g, b)
        
#     elif phase == 'split':
#         if target_side == 'left':
#             # Oscillating orange
#             pulse = math.sin(glitch_timer * 3) * 0.3 + 0.7
#             if random.random() < 0.1:
#                 return (0, 0, 0)
#             return (int(255 * pulse), int(100 * pulse), 0)
#         else:  # right
#             # Oscillating blue
#             pulse = math.cos(glitch_timer * 4) * 0.3 + 0.7
#             if random.random() < 0.1:
#                 return (255, 255, 255)
#             return (0, int(100 * pulse), int(255 * pulse))
#     else:  # hold
#         if target_side == 'left':
#             return (255, 150, 0)
#         else:
#             return (0, 150, 255)

# def generate_chaotic_target(x, y, my_id, glitch_timer, phase, target_side):
#     """Generate targets with chaotic oscillations"""
#     base_angle = math.atan2(y - CY, x - CX)
    
#     if phase == 'glitch_breakout':
#         # Chaotic breakout with oscillations
#         if random.random() < 0.25:  # Frequent target changes
#             # Add oscillation to direction
#             osc_angle = base_angle + (math.sin(glitch_timer * 4) * 0.8)
#             distance = 0.6 + random.random() * 0.4
            
#             target_x = CX + distance * math.cos(osc_angle)
#             target_y = CY + distance * math.sin(osc_angle)
            
#             # Add secondary oscillation to position
#             target_x += math.sin(glitch_timer * 6) * 0.2
#             target_y += math.cos(glitch_timer * 5) * 0.2
            
#         else:
#             # Keep current target but add oscillation
#             return None  # Signal to keep current target
            
#     elif phase == 'split':
#         # Oscillating movement toward final side
#         if target_side == 'left':
#             target_x = X_MIN + 0.25
#         else:
#             target_x = X_MAX - 0.25
            
#         # Vertical oscillation during horizontal movement
#         y_center = (Y_MIN + Y_MAX) / 2
#         y_osc = math.sin(glitch_timer * 3) * 0.3
#         target_y = y_center + y_osc
        
#     else:  # hold
#         # Gentle wandering around final position
#         if target_side == 'left':
#             base_x = X_MIN + 0.25
#         else:
#             base_x = X_MAX - 0.25
            
#         target_x = base_x + math.sin(glitch_timer * 2) * 0.1
#         target_y = y + math.cos(glitch_timer * 1.5) * 0.08
    
#     return target_x, target_y

# def usr(robot):
#     my_id = get_id(robot)
#     random.seed(my_id * 1337)
    
#     # Determine final side
#     if my_id % 2 == 0:
#         target_side = 'left'
#         final_x = X_MIN + 0.25
#     else:
#         target_side = 'right'
#         final_x = X_MAX - 0.25
    
#     print(f"Robot {my_id} CHAOTIC OSCILLATION MODE - Destination: {target_side}")
    
#     # State
#     phase = 'glitch_breakout'
#     current_target = None
#     glitch_timer = 0
#     breakout_start_time = 0
    
#     # Oscillation parameters (different for each robot)
#     osc_params = {
#         'freq_x': 2.0 + random.random() * 4.0,
#         'freq_y': 1.5 + random.random() * 3.5,
#         'amp_x': 0.1 + random.random() * 0.1,
#         'amp_y': 0.1 + random.random() * 0.1,
#     }
    
#     robot.set_led(255, 0, 0)
    
#     while True:
#         pose = safe_pose(robot)
#         if not pose:
#             robot.set_vel(0, 0)
#             robot.delay(20)
#             continue
            
#         x, y, th = pose
#         t = robot.get_clock()
#         glitch_timer += 0.02
        
#         if breakout_start_time == 0:
#             breakout_start_time = t
        
#         # Calculate distance from center
#         dist_from_center = math.hypot(x - CX, y - CY)
        
#         # Phase transitions
#         if phase == 'glitch_breakout':
#             if dist_from_center > 0.5 or t - breakout_start_time > 6.0:
#                 phase = 'split'
#                 print(f"Robot {my_id} breakout complete, oscillating to {target_side}")
        
#         elif phase == 'split':
#             if (target_side == 'left' and x < X_MIN + 0.35) or (target_side == 'right' and x > X_MAX - 0.35):
#                 phase = 'hold'
#                 print(f"Robot {my_id} reached {target_side} side with oscillations")
        
#         # Generate chaotic targets
#         new_target = generate_chaotic_target(x, y, my_id, glitch_timer, phase, target_side)
#         if new_target:
#             current_target = new_target
        
#         # Get oscillation forces
#         osc_x, osc_y = get_chaotic_oscillation(glitch_timer, my_id, phase)
        
#         # Calculate movement with OSCILLATIONS
#         if current_target:
#             tx, ty = current_target
            
#             # Add oscillations to target
#             if phase == 'glitch_breakout':
#                 tx += osc_x * 0.5
#                 ty += osc_y * 0.5
            
#             ex, ey = tx - x, ty - y
            
#             # GLITCH: Sometimes ignore target for oscillation-only movement
#             if random.random() < 0.15:
#                 vx = osc_x * 2.0
#                 vy = osc_y * 2.0
#             else:
#                 vx = 1.6 * ex + osc_x
#                 vy = 1.6 * ey + osc_y
            
#             # Add random directional glitches
#             if random.random() < 0.1:
#                 vx = -vx * 0.5
#             if random.random() < 0.1:
#                 vy = -vy * 0.5
            
#             # Add boundary and dancer forces
#             boundary_fx, boundary_fy = glitch_boundary_force(x, y)
#             dancer_fx, dancer_fy = glitch_dancer_repulsion(x, y)
            
#             vx += boundary_fx + dancer_fx + osc_x
#             vy += boundary_fy + dancer_fy + osc_y
            
#             # Convert to wheel commands with oscillating behavior
#             if abs(vx) + abs(vy) > 1e-3:
#                 hdg = math.atan2(vy, vx)
#                 err = wrap_angle(hdg - th)
                
#                 # Oscillating turn gain
#                 turn_gain = 2.5 + math.sin(glitch_timer * 4) * 1.0
                
#                 turn = clamp(turn_gain * err, -2.5, 2.5)
                
#                 # Oscillating speed
#                 base_speed = GLITCH_SPEED
#                 speed_osc = math.sin(glitch_timer * 5) * 0.3 + 1.0
#                 if random.random() < 0.1:
#                     speed_osc *= random.uniform(0.3, 2.0)
                
#                 speed_mult = base_speed * speed_osc
                
#                 left = clamp(int(MAX_WHEEL * (speed_mult - 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
#                 right = clamp(int(MAX_WHEEL * (speed_mult + 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
                
#                 # Motor swapping glitch
#                 if random.random() < 0.05:
#                     left, right = right, left
                
#                 robot.set_vel(left, right)
#             else:
#                 # Oscillating in place
#                 if random.random() < 0.3:
#                     osc_strength = 0.1 + random.random() * 0.2
#                     left = int(MAX_WHEEL * (osc_x * osc_strength))
#                     right = int(MAX_WHEEL * (osc_y * osc_strength))
#                     robot.set_vel(left, right)
#                 else:
#                     robot.set_vel(0, 0)
#         else:
#             robot.set_vel(0, 0)
        
#         # Oscillating LED colors
#         robot.set_led(*get_glitch_color(phase, glitch_timer, target_side))
        
#         # Chaotic console messages
#         if random.random() < 0.008:
#             chaotic_messages = [
#                 "OSCILLATING UNCONTROLLABLY!", "FORWARD-BACKWARD GLITCH!", 
#                 "UP-DOWN MOTION DETECTED!", "CHAOTIC TRAJECTORY!",
#                 "MOVEMENT PATTERN CORRUPTED!", "BOUNCING OFF NOTHING!",
#                 "ZIGZAG PROTOCOL ACTIVATED!", "ERRATIC NAVIGATION!",
#                 "WOBBLE WOBBLE WOBBLE!", "SPIRALING OUT OF CONTROL!",
#                 "BACK AND FORTH FOREVER!", "VERTICAL OSCILLATION MAXIMUM!"
#             ]
#             print(f"Robot {my_id}: {random.choice(chaotic_messages)}")
        
#         robot.delay(20)






# import math, struct, random

# # ===== Field & center =====
# X_MIN, X_MAX = -1.2, 1.0
# Y_MIN, Y_MAX = -1.4, 2.35
# CX, CY = (-0.1, 0.475)

# # ===== Dancer no-go circle =====
# FEET = 0.3048
# OBST_DIAM_FT = 1.0
# OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
# OBST_MARGIN  = 0.03
# SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN

# # ===== Motion parameters =====
# MAX_WHEEL = 40
# OUTWARD_SPEED = 30  # Direct wheel speed for outward movement

# def clamp(v, lo, hi): return max(lo, min(v, hi))
# def wrap_angle(a):
#     while a > math.pi: a -= 2*math.pi
#     while a <= -math.pi: a += 2*math.pi
#     return a
# def safe_pose(robot):
#     p = robot.get_pose()
#     if p and len(p) >= 3: return float(p[0]), float(p[1]), float(p[2])
#     return None
# def get_id(robot):
#     vid_attr = getattr(robot, "virtual_id", None)
#     return vid_attr() if callable(vid_attr) else int(vid_attr or 0)

# def boundary_force(x, y):
#     """Keep robots in bounds"""
#     fx, fy = 0.0, 0.0
#     boundary_margin = 0.1
    
#     if x < X_MIN + boundary_margin:
#         strength = 2.0 * (1.0 - (x - X_MIN) / boundary_margin)
#         fx += strength
#     elif x > X_MAX - boundary_margin:
#         strength = 2.0 * (1.0 - (X_MAX - x) / boundary_margin)  
#         fx -= strength
        
#     if y < Y_MIN + boundary_margin:
#         strength = 2.0 * (1.0 - (y - Y_MIN) / boundary_margin)
#         fy += strength
#     elif y > Y_MAX - boundary_margin:
#         strength = 2.0 * (1.0 - (Y_MAX - y) / boundary_margin)
#         fy -= strength
        
#     return fx, fy

# def strong_dancer_repulsion(x, y):
#     """STRONG repulsion from dancer zone"""
#     dx, dy = x - CX, y - CY
#     dist = math.hypot(dx, dy)
#     safe_dist = SAFE_BUBBLE + 0.2
    
#     if dist < safe_dist:
#         strength = 6.0 * ((safe_dist - dist) / safe_dist) ** 2
#         fx = (dx/dist) * strength if dist > 0 else 1.0
#         fy = (dy/dist) * strength if dist > 0 else 1.0
#         return fx, fy
#     return 0, 0

# def usr(robot):
#     my_id = get_id(robot)
#     random.seed(my_id * 42)
    
#     # SIMPLE: Half go left, half go right
#     if my_id % 2 == 0:
#         target_side = 'left'
#         final_x = X_MIN + 0.25
#         color = (255, 150, 0)  # Orange
#     else:
#         target_side = 'right'
#         final_x = X_MAX - 0.25
#         color = (0, 150, 255)  # Blue
    
#     print(f"Robot {my_id} - Final destination: {target_side} side")
    
#     # State machine
#     phase = 'face_outward'
#     start_time = robot.get_clock()
    
#     robot.set_led(255, 255, 0)  # Yellow - preparing
    
#     while True:
#         pose = safe_pose(robot)
#         if not pose:
#             robot.set_vel(0, 0)
#             robot.delay(20)
#             continue
            
#         x, y, th = pose
#         t = robot.get_clock()
        
#         # Calculate distance from center and angle TO center
#         dist_from_center = math.hypot(x - CX, y - CY)
#         angle_to_center = math.atan2(CY - y, CX - x)  # Angle FROM robot TO center
        
#         # Phase 1: Face directly AWAY from center
#         if phase == 'face_outward':
#             # Face 180 degrees OPPOSITE from center direction
#             target_heading = angle_to_center + math.pi  # This points AWAY from center
            
#             # Check if we're facing the right direction
#             heading_error = wrap_angle(target_heading - th)
            
#             if abs(heading_error) > 0.3:  # Not yet facing outward
#                 # Rotate in place toward outward direction
#                 turn_speed = clamp(3.0 * heading_error, -1.5, 1.5)
#                 left_wheel = int(-MAX_WHEEL * 0.8 * turn_speed)
#                 right_wheel = int(MAX_WHEEL * 0.8 * turn_speed)
#                 robot.set_vel(left_wheel, right_wheel)
#                 robot.set_led(255, 255, 0)  # Yellow - rotating
#                 print(f"Robot {my_id} rotating to face AWAY from center...")
#             else:
#                 # Facing correct direction! Start moving OUTWARD
#                 phase = 'move_outward'
#                 start_time = t
#                 print(f"Robot {my_id} NOW FACING AWAY FROM CENTER - MOVING OUTWARD!")
#                 robot.set_led(255, 0, 0)  # Red - moving outward
        
#         # Phase 2: Move STRAIGHT OUTWARD from center
#         elif phase == 'move_outward':
#             # Keep moving outward until we're far from center
#             target_distance = 1.2  # Move far out before scattering
            
#             if dist_from_center > target_distance or t - start_time > 6.0:
#                 # Far enough out, now scatter to sides
#                 phase = 'scatter_to_side'
#                 print(f"Robot {my_id} REACHED OUTER EDGE - SCATTERING TO {target_side}")
#                 robot.set_led(255, 100, 0) if target_side == 'left' else robot.set_led(0, 100, 255)
#             else:
#                 # SIMPLE: Just move FORWARD (they're already facing outward)
#                 robot.set_vel(OUTWARD_SPEED, OUTWARD_SPEED)
#                 robot.set_led(255, 0, 0)  # Red - moving outward
                
#                 # Print progress
#                 if int(t) % 2 == 0:
#                     print(f"Robot {my_id} moving OUTWARD: {dist_from_center:.2f}m from center")
        
#         # Phase 3: Scatter to assigned side
#         elif phase == 'scatter_to_side':
#             # Check if we've reached our final position
#             if target_side == 'left' and x < final_x + 0.05:
#                 phase = 'final_position'
#                 print(f"Robot {my_id} REACHED LEFT SIDE!")
#             elif target_side == 'right' and x > final_x - 0.05:
#                 phase = 'final_position'
#                 print(f"Robot {my_id} REACHED RIGHT SIDE!")
            
#             # Move toward our assigned side
#             if target_side == 'left':
#                 target_x = final_x
#                 # Add vertical spread
#                 vertical_spread = ((my_id % 5) - 2) * 0.2
#                 target_y = CY + vertical_spread
#             else:  # right
#                 target_x = final_x
#                 vertical_spread = ((my_id % 5) - 2) * 0.2
#                 target_y = CY + vertical_spread
                
#             # Calculate movement toward side
#             tx, ty = target_x, target_y
#             ex, ey = tx - x, ty - y
            
#             vx = 2.0 * ex
#             vy = 2.0 * ey
            
#             # Add safety forces
#             boundary_fx, boundary_fy = boundary_force(x, y)
#             dancer_fx, dancer_fy = strong_dancer_repulsion(x, y)
            
#             vx += boundary_fx + dancer_fx
#             vy += boundary_fy + dancer_fy
            
#             # Convert to wheel commands
#             if abs(vx) + abs(vy) > 1e-3:
#                 hdg = math.atan2(vy, vx)
#                 err = wrap_angle(hdg - th)
                
#                 turn = clamp(2.5 * err, -2.0, 2.0)
                
#                 left = clamp(int(MAX_WHEEL * (0.15 - 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
#                 right = clamp(int(MAX_WHEEL * (0.15 + 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
                
#                 robot.set_vel(left, right)
#             else:
#                 robot.set_vel(0, 0)
            
#             robot.set_led(255, 100, 0) if target_side == 'left' else robot.set_led(0, 100, 255)
        
#         # Phase 4: Final position with GLITCHING
#         elif phase == 'final_position':
#             # We've reached our side - now add glitching!
#             glitch_pattern = my_id % 4
            
#             if glitch_pattern == 0:
#                 # Forward/backward oscillation
#                 if int(t * 4) % 2 == 0:
#                     robot.set_vel(15, 15)  # Forward
#                 else:
#                     robot.set_vel(-10, -10)  # Backward
                    
#             elif glitch_pattern == 1:
#                 # Side-to-side wobble
#                 wobble = math.sin(t * 5) * 0.3
#                 left = int(15 * (1 - wobble))
#                 right = int(15 * (1 + wobble))
#                 robot.set_vel(left, right)
                    
#             elif glitch_pattern == 2:
#                 # Random twitching
#                 if random.random() < 0.3:
#                     left = random.randint(-10, 15)
#                     right = random.randint(-10, 15)
#                     robot.set_vel(left, right)
#                 else:
#                     robot.set_vel(0, 0)
                    
#             else:  # pattern 3
#                 # Gentle wandering around final position
#                 wander_x = final_x + math.sin(t * 2) * 0.1
#                 wander_y = y + math.cos(t * 1.5) * 0.08
                
#                 tx, ty = wander_x, wander_y
#                 ex, ey = tx - x, ty - y
#                 vx = 1.0 * ex
#                 vy = 1.0 * ey
                
#                 if abs(vx) + abs(vy) > 1e-3:
#                     hdg = math.atan2(vy, vx)
#                     err = wrap_angle(hdg - th)
#                     turn = clamp(2.0 * err, -1.5, 1.5)
#                     left = clamp(int(MAX_WHEEL * (0.08 - 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
#                     right = clamp(int(MAX_WHEEL * (0.08 + 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
#                     robot.set_vel(left, right)
#                 else:
#                     robot.set_vel(0, 0)
            
#             # Glitchy LED colors
#             if int(t * 6) % 3 == 0:
#                 robot.set_led(255, 255, 255)  # White flash
#             else:
#                 robot.set_led(*color)
        
#         # ALWAYS check dancer safety
#         dancer_fx, dancer_fy = strong_dancer_repulsion(x, y)
#         if abs(dancer_fx) + abs(dancer_fy) > 1.0:
#             print(f"Robot {my_id} TOO CLOSE TO DANCER - EMERGENCY STOP!")
#             robot.set_vel(0, 0)
#             robot.set_led(255, 255, 255)
        
#         robot.delay(20)


# import math, struct, random

# # ===== Field & center =====
# X_MIN, X_MAX = -1.2, 1.0
# Y_MIN, Y_MAX = -1.4, 2.35
# CX, CY = (-0.1, 0.475)

# # ===== Dancer no-go circle =====
# FEET = 0.3048
# OBST_DIAM_FT = 1.0
# OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
# OBST_MARGIN  = 0.03
# SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN

# # ===== Motion parameters =====
# MAX_WHEEL = 40
# FORWARD_SPEED = 35
# BACKWARD_SPEED = 25
# TURN_SPEED = 30

# def clamp(v, lo, hi): return max(lo, min(v, hi))
# def wrap_angle(a):
#     while a > math.pi: a -= 2*math.pi
#     while a <= -math.pi: a += 2*math.pi
#     return a
# def safe_pose(robot):
#     p = robot.get_pose()
#     if p and len(p) >= 3: return float(p[0]), float(p[1]), float(p[2])
#     return None
# def get_id(robot):
#     vid_attr = getattr(robot, "virtual_id", None)
#     return vid_attr() if callable(vid_attr) else int(vid_attr or 0)

# def boundary_force(x, y):
#     """Keep robots in bounds"""
#     fx, fy = 0.0, 0.0
#     boundary_margin = 0.1
    
#     if x < X_MIN + boundary_margin:
#         strength = 2.0 * (1.0 - (x - X_MIN) / boundary_margin)
#         fx += strength
#     elif x > X_MAX - boundary_margin:
#         strength = 2.0 * (1.0 - (X_MAX - x) / boundary_margin)  
#         fx -= strength
        
#     if y < Y_MIN + boundary_margin:
#         strength = 2.0 * (1.0 - (y - Y_MIN) / boundary_margin)
#         fy += strength
#     elif y > Y_MAX - boundary_margin:
#         strength = 2.0 * (1.0 - (Y_MAX - y) / boundary_margin)
#         fy -= strength
        
#     return fx, fy

# def strong_dancer_repulsion(x, y):
#     """STRONG repulsion from dancer zone"""
#     dx, dy = x - CX, y - CY
#     dist = math.hypot(dx, dy)
#     safe_dist = SAFE_BUBBLE + 0.2
    
#     if dist < safe_dist:
#         strength = 6.0 * ((safe_dist - dist) / safe_dist) ** 2
#         fx = (dx/dist) * strength if dist > 0 else 1.0
#         fy = (dy/dist) * strength if dist > 0 else 1.0
#         return fx, fy
#     return 0, 0

# def usr(robot):
#     my_id = get_id(robot)
#     random.seed(my_id * 42)
    
#     print(f"Robot {my_id} - GLITCHY STEPPING OUTWARD")
    
#     # State for step pattern
#     step_phase = 'forward_1'  # forward_1, forward_2, backward, turn
#     step_timer = 0
#     step_duration = 0.8  # seconds per step
#     turn_direction = 1 if random.random() > 0.5 else -1  # Random initial turn direction
    
#     # Overall progress
#     start_time = robot.get_clock()
#     target_distance = 0.8  # How far to move outward
    
#     robot.set_led(255, 255, 0)  # Start yellow
    
#     while True:
#         pose = safe_pose(robot)
#         if not pose:
#             robot.set_vel(0, 0)
#             robot.delay(20)
#             continue
            
#         x, y, th = pose
#         t = robot.get_clock()
#         step_timer += 0.02
        
#         # Calculate distance from center
#         dist_from_center = math.hypot(x - CX, y - CY)
#         angle_to_center = math.atan2(CY - y, CX - x)
#         outward_heading = angle_to_center + math.pi  # Direction AWAY from center
        
#         # Check if we've reached target distance
#         if dist_from_center > target_distance:
#             # Reached destination - do final glitching
#             if int(t * 5) % 2 == 0:
#                 robot.set_vel(15, 15)
#                 robot.set_led(0, 255, 0)  # Green
#             else:
#                 robot.set_vel(-10, -10)
#                 robot.set_led(0, 200, 0)  # Darker green
#             robot.delay(20)
#             continue
        
#         # GLITCHY STEP PATTERN
#         if step_phase == 'forward_1':
#             # First forward step - fast and determined
#             if step_timer < step_duration * 0.8:  # 80% of step time
#                 robot.set_vel(FORWARD_SPEED, FORWARD_SPEED)
#                 robot.set_led(255, 0, 0)  # Red - moving forward
#             else:
#                 # Brief pause between forward steps
#                 robot.set_vel(0, 0)
#                 robot.set_led(255, 100, 0)  # Orange - pausing
                
#             if step_timer > step_duration:
#                 step_phase = 'forward_2'
#                 step_timer = 0
#                 # 20% chance to skip second forward step (GLITCH!)
#                 if random.random() < 0.2:
#                     step_phase = 'backward'
        
#         elif step_phase == 'forward_2':
#             # Second forward step - slightly erratic
#             if step_timer < step_duration * 0.7:  # Shorter forward time
#                 # Add some wobble to second step
#                 wobble = math.sin(t * 10) * 0.2
#                 left = int(FORWARD_SPEED * (1 - wobble))
#                 right = int(FORWARD_SPEED * (1 + wobble))
#                 robot.set_vel(left, right)
#                 robot.set_led(255, 50, 0)  # Darker red
#             else:
#                 robot.set_vel(0, 0)
#                 robot.set_led(255, 150, 0)  # Light orange
                
#             if step_timer > step_duration:
#                 step_phase = 'backward'
#                 step_timer = 0
#                 # 30% chance to skip backward step (GLITCH!)
#                 if random.random() < 0.3:
#                     step_phase = 'turn'
        
#         elif step_phase == 'backward':
#             # One step back - slower and more hesitant
#             if step_timer < step_duration * 0.6:  # Shorter backward time
#                 robot.set_vel(-BACKWARD_SPEED, -BACKWARD_SPEED)
#                 robot.set_led(0, 0, 255)  # Blue - moving backward
#             else:
#                 robot.set_vel(0, 0)
#                 robot.set_led(0, 100, 255)  # Light blue
                
#             if step_timer > step_duration:
#                 step_phase = 'turn'
#                 step_timer = 0
#                 # Randomly change turn direction
#                 if random.random() < 0.4:
#                     turn_direction *= -1
        
#         elif step_phase == 'turn':
#             # Turn left or right - erratic turning
#             if step_timer < step_duration * 0.5:  # Shorter turn time
#                 # Erratic turning speed
#                 turn_variation = math.sin(t * 8) * 0.3 + 1.0
#                 turn_strength = TURN_SPEED * turn_variation * turn_direction
                
#                 left = int(-turn_strength)
#                 right = int(turn_strength)
#                 robot.set_vel(left, right)
#                 robot.set_led(255, 255, 0)  # Yellow - turning
#             else:
#                 robot.set_vel(0, 0)
#                 robot.set_led(200, 200, 0)  # Dark yellow
                
#             if step_timer > step_duration:
#                 step_phase = 'forward_1'
#                 step_timer = 0
#                 # 10% chance for extra turn (GLITCH!)
#                 if random.random() < 0.1:
#                     step_phase = 'turn'
#                     turn_direction *= -1  # Turn the other way
        
#         # Occasionally add random glitches that interrupt the pattern
#         if random.random() < 0.05:  # 5% chance per frame for random glitch
#             glitch_type = random.randint(0, 3)
#             if glitch_type == 0:
#                 # Sudden burst forward
#                 robot.set_vel(FORWARD_SPEED + 10, FORWARD_SPEED + 10)
#                 robot.set_led(255, 255, 255)  # White flash
#                 step_timer = 0  # Reset step timer
#             elif glitch_type == 1:
#                 # Spin in place
#                 robot.set_vel(-TURN_SPEED, TURN_SPEED)
#                 robot.set_led(255, 0, 255)  # Magenta
#                 step_timer = 0
#             elif glitch_type == 2:
#                 # Freeze
#                 robot.set_vel(0, 0)
#                 robot.set_led(0, 0, 0)  # Black
#                 step_timer += 0.5  # Add extra time to current step
#             else:
#                 # Motor swap
#                 left, right = FORWARD_SPEED, FORWARD_SPEED
#                 if random.random() < 0.5:
#                     left, right = right, left
#                 robot.set_vel(left, right)
#                 robot.set_led(255, 100, 100)  # Pink
        
#         # Safety forces (always active)
#         boundary_fx, boundary_fy = boundary_force(x, y)
#         dancer_fx, dancer_fy = strong_dancer_repulsion(x, y)
        
#         # Convert safety forces to wheel adjustments
#         if abs(boundary_fx) + abs(boundary_fy) + abs(dancer_fx) + abs(dancer_fy) > 0.5:
#             safety_heading = math.atan2(boundary_fy + dancer_fy, boundary_fx + dancer_fx)
#             current_heading = th
#             safety_error = wrap_angle(safety_heading - current_heading)
            
#             # Add safety correction to current movement
#             left_vel, right_vel = robot.get_vel() if hasattr(robot, 'get_vel') else (0, 0)
#             safety_turn = clamp(2.0 * safety_error, -1.0, 1.0)
#             left_vel = clamp(left_vel - int(MAX_WHEEL * 0.3 * safety_turn), -MAX_WHEEL, MAX_WHEEL)
#             right_vel = clamp(right_vel + int(MAX_WHEEL * 0.3 * safety_turn), -MAX_WHEEL, MAX_WHEEL)
#             robot.set_vel(left_vel, right_vel)
        
#         # Progress reporting
#         if int(t) % 3 == 0 and int(t) > int(t - 0.02):
#             progress = (dist_from_center / target_distance) * 100
#             print(f"Robot {my_id} outward progress: {progress:.0f}% - Step: {step_phase}")
        
#         robot.delay(20)


# import math, struct, random

# # ===== Field & center =====
# X_MIN, X_MAX = -1.2, 1.0
# Y_MIN, Y_MAX = -1.4, 2.35
# CX, CY = (-0.1, 0.475)

# # ===== Dancer no-go circle =====
# FEET = 0.3048
# OBST_DIAM_FT = 1.0
# OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
# OBST_MARGIN  = 0.03
# SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN

# # ===== Motion parameters =====
# MAX_WHEEL = 40
# FORWARD_SPEED = 35
# BACKWARD_SPEED = 25
# TURN_SPEED = 30

# def clamp(v, lo, hi): return max(lo, min(v, hi))
# def wrap_angle(a):
#     while a > math.pi: a -= 2*math.pi
#     while a <= -math.pi: a += 2*math.pi
#     return a
# def safe_pose(robot):
#     p = robot.get_pose()
#     if p and len(p) >= 3: return float(p[0]), float(p[1]), float(p[2])
#     return None
# def get_id(robot):
#     vid_attr = getattr(robot, "virtual_id", None)
#     return vid_attr() if callable(vid_attr) else int(vid_attr or 0)

# def boundary_force(x, y):
#     """Keep robots in bounds"""
#     fx, fy = 0.0, 0.0
#     boundary_margin = 0.1
    
#     if x < X_MIN + boundary_margin:
#         strength = 2.0 * (1.0 - (x - X_MIN) / boundary_margin)
#         fx += strength
#     elif x > X_MAX - boundary_margin:
#         strength = 2.0 * (1.0 - (X_MAX - x) / boundary_margin)  
#         fx -= strength
        
#     if y < Y_MIN + boundary_margin:
#         strength = 2.0 * (1.0 - (y - Y_MIN) / boundary_margin)
#         fy += strength
#     elif y > Y_MAX - boundary_margin:
#         strength = 2.0 * (1.0 - (Y_MAX - y) / boundary_margin)
#         fy -= strength
        
#     return fx, fy

# def strong_dancer_repulsion(x, y):
#     """STRONG repulsion from dancer zone"""
#     dx, dy = x - CX, y - CY
#     dist = math.hypot(dx, dy)
#     safe_dist = SAFE_BUBBLE + 0.2
    
#     if dist < safe_dist:
#         strength = 6.0 * ((safe_dist - dist) / safe_dist) ** 2
#         fx = (dx/dist) * strength if dist > 0 else 1.0
#         fy = (dy/dist) * strength if dist > 0 else 1.0
#         return fx, fy
#     return 0, 0

# def usr(robot):
#     my_id = get_id(robot)
#     random.seed(my_id * 42)
    
#     print(f"Robot {my_id} - FACING OUTWARD THEN GLITCHY STEPPING")
    
#     # PHASE 1: Face outward first
#     phase = 'face_outward'
#     outward_heading = None
    
#     # PHASE 2: Glitchy stepping (will be set later)
#     step_phase = 'forward_1'
#     step_timer = 0
#     step_duration = 0.8
#     turn_direction = 1 if random.random() > 0.5 else -1
    
#     # Overall progress
#     start_time = robot.get_clock()
#     target_distance = 0.8
    
#     robot.set_led(255, 255, 0)  # Start yellow - facing outward phase
    
#     while True:
#         pose = safe_pose(robot)
#         if not pose:
#             robot.set_vel(0, 0)
#             robot.delay(20)
#             continue
            
#         x, y, th = pose
#         t = robot.get_clock()
#         step_timer += 0.02
        
#         # Calculate distance from center and outward direction
#         dist_from_center = math.hypot(x - CX, y - CY)
#         angle_to_center = math.atan2(CY - y, CX - x)
#         current_outward_heading = angle_to_center + math.pi  # Direction AWAY from center
        
#         # PHASE 1: Face outward first
#         if phase == 'face_outward':
#             # Store the outward heading once we calculate it
#             if outward_heading is None:
#                 outward_heading = current_outward_heading
            
#             # Check if we're facing the right direction
#             heading_error = wrap_angle(outward_heading - th)
            
#             if abs(heading_error) > 0.2:  # Not yet facing outward
#                 # Rotate in place toward outward direction
#                 turn_speed = clamp(3.0 * heading_error, -1.5, 1.5)
#                 left_wheel = int(-MAX_WHEEL * 0.8 * turn_speed)
#                 right_wheel = int(MAX_WHEEL * 0.8 * turn_speed)
#                 robot.set_vel(left_wheel, right_wheel)
#                 robot.set_led(255, 255, 0)  # Yellow - rotating to face outward
#                 print(f"Robot {my_id} rotating to face outward... error: {heading_error:.2f}")
#             else:
#                 # Successfully facing outward! Start glitchy stepping
#                 phase = 'glitchy_stepping'
#                 start_time = t
#                 print(f"Robot {my_id} NOW FACING OUTWARD - STARTING GLITCHY STEPS!")
#                 robot.set_led(255, 0, 0)  # Red - starting to move
        
#         # PHASE 2: Glitchy stepping outward (only after facing correct direction)
#         elif phase == 'glitchy_stepping':
#             # Check if we've reached target distance
#             if dist_from_center > target_distance:
#                 # Reached destination - do final glitching
#                 if int(t * 5) % 2 == 0:
#                     robot.set_vel(15, 15)
#                     robot.set_led(0, 255, 0)  # Green
#                 else:
#                     robot.set_vel(-10, -10)
#                     robot.set_led(0, 200, 0)  # Darker green
#                 robot.delay(20)
#                 continue
            
#             # GLITCHY STEP PATTERN
#             if step_phase == 'forward_1':
#                 # First forward step - fast and determined
#                 if step_timer < step_duration * 0.8:
#                     robot.set_vel(FORWARD_SPEED, FORWARD_SPEED)
#                     robot.set_led(255, 0, 0)  # Red - moving forward
#                 else:
#                     # Brief pause between forward steps
#                     robot.set_vel(0, 0)
#                     robot.set_led(255, 100, 0)  # Orange - pausing
                    
#                 if step_timer > step_duration:
#                     step_phase = 'forward_2'
#                     step_timer = 0
#                     # 20% chance to skip second forward step (GLITCH!)
#                     if random.random() < 0.2:
#                         step_phase = 'backward'

#             elif step_phase == 'forward_2':
#                 # Second forward step - slightly erratic
#                 if step_timer < step_duration * 0.7:
#                     # Add some wobble to second step
#                     wobble = math.sin(t * 10) * 0.2
#                     left = int(FORWARD_SPEED * (1 - wobble))
#                     right = int(FORWARD_SPEED * (1 + wobble))
#                     robot.set_vel(left, right)
#                     robot.set_led(255, 50, 0)  # Darker red
#                 else:
#                     robot.set_vel(0, 0)
#                     robot.set_led(255, 150, 0)  # Light orange
                    
#                 if step_timer > step_duration:
#                     step_phase = 'backward'
#                     step_timer = 0
#                     # 30% chance to skip backward step (GLITCH!)
#                     if random.random() < 0.3:
#                         step_phase = 'turn'

#             elif step_phase == 'backward':
#                 # One step back - slower and more hesitant
#                 if step_timer < step_duration * 0.6:
#                     robot.set_vel(-BACKWARD_SPEED, -BACKWARD_SPEED)
#                     robot.set_led(0, 0, 255)  # Blue - moving backward
#                 else:
#                     robot.set_vel(0, 0)
#                     robot.set_led(0, 100, 255)  # Light blue
                    
#                 if step_timer > step_duration:
#                     step_phase = 'turn'
#                     step_timer = 0
#                     # Randomly change turn direction
#                     if random.random() < 0.4:
#                         turn_direction *= -1

#             elif step_phase == 'turn':
#                 # Turn left or right - but try to maintain overall outward direction
#                 if step_timer < step_duration * 0.5:
#                     # Check current heading vs outward direction
#                     current_heading_error = wrap_angle(outward_heading - th)
                    
#                     # Mix between random turning and correction toward outward direction
#                     if abs(current_heading_error) > 0.5:
#                         # Too far off course, correct toward outward
#                         correction_turn = clamp(2.0 * current_heading_error, -1.0, 1.0)
#                         turn_strength = TURN_SPEED * correction_turn
#                     else:
#                         # Close enough, do random turning
#                         turn_variation = math.sin(t * 8) * 0.3 + 1.0
#                         turn_strength = TURN_SPEED * turn_variation * turn_direction
                    
#                     left = int(-turn_strength)
#                     right = int(turn_strength)
#                     robot.set_vel(left, right)
#                     robot.set_led(255, 255, 0)  # Yellow - turning
#                 else:
#                     robot.set_vel(0, 0)
#                     robot.set_led(200, 200, 0)  # Dark yellow
                    
#                 if step_timer > step_duration:
#                     step_phase = 'forward_1'
#                     step_timer = 0
#                     # 10% chance for extra turn (GLITCH!)
#                     if random.random() < 0.1:
#                         step_phase = 'turn'
#                         turn_direction *= -1

#             # Occasionally add random glitches that interrupt the pattern
#             if random.random() < 0.05:
#                 glitch_type = random.randint(0, 3)
#                 if glitch_type == 0:
#                     # Sudden burst forward
#                     robot.set_vel(FORWARD_SPEED + 10, FORWARD_SPEED + 10)
#                     robot.set_led(255, 255, 255)  # White flash
#                     step_timer = 0
#                 elif glitch_type == 1:
#                     # Spin in place
#                     robot.set_vel(-TURN_SPEED, TURN_SPEED)
#                     robot.set_led(255, 0, 255)  # Magenta
#                     step_timer = 0
#                 elif glitch_type == 2:
#                     # Freeze
#                     robot.set_vel(0, 0)
#                     robot.set_led(0, 0, 0)  # Black
#                     step_timer += 0.5
#                 else:
#                     # Motor swap
#                     left, right = FORWARD_SPEED, FORWARD_SPEED
#                     if random.random() < 0.5:
#                         left, right = right, left
#                     robot.set_vel(left, right)
#                     robot.set_led(255, 100, 100)  # Pink

#         # Safety forces (always active)
#         boundary_fx, boundary_fy = boundary_force(x, y)
#         dancer_fx, dancer_fy = strong_dancer_repulsion(x, y)
        
#         # Apply safety corrections
#         if abs(boundary_fx) + abs(boundary_fy) + abs(dancer_fx) + abs(dancer_fy) > 0.5:
#             safety_heading = math.atan2(boundary_fy + dancer_fy, boundary_fx + dancer_fx)
#             current_heading = th
#             safety_error = wrap_angle(safety_heading - current_heading)
            
#             left_vel, right_vel = robot.get_vel() if hasattr(robot, 'get_vel') else (0, 0)
#             safety_turn = clamp(2.0 * safety_error, -1.0, 1.0)
#             left_vel = clamp(left_vel - int(MAX_WHEEL * 0.3 * safety_turn), -MAX_WHEEL, MAX_WHEEL)
#             right_vel = clamp(right_vel + int(MAX_WHEEL * 0.3 * safety_turn), -MAX_WHEEL, MAX_WHEEL)
#             robot.set_vel(left_vel, right_vel)
        
#         # Progress reporting
#         if int(t) % 3 == 0 and int(t) > int(t - 0.02):
#             if phase == 'face_outward':
#                 heading_error = wrap_angle(outward_heading - th) if outward_heading else 999
#                 print(f"Robot {my_id} facing outward... error: {heading_error:.2f}")
#             else:
#                 progress = (dist_from_center / target_distance) * 100
#                 print(f"Robot {my_id} outward progress: {progress:.0f}% - Step: {step_phase}")
        
#         robot.delay(20)

# -*- coding: utf-8 -*-
import math
import random

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
REPULSE_RADIUS  = 0.35
REPULSE_GAIN    = 0.60
NOISE_GAIN      = 0.25
FWD_GAIN        = 0.95
LEFT_BIAS_VX    = 0.00  # set to -0.06 for slow drift left

# dancer no-go circle (meters)
FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET   # ~0.1524
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
    names = ('get_swarm_poses', 'get_all_poses', 'get_poses', 'swarm_poses')
    for nm in names:
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
    vid_attr = getattr(robot, "virtual_id", None)
    try:
        return vid_attr() if callable(vid_attr) else int(vid_attr or 0)
    except:
        return -1

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
    robot.delay(3000)
    my_id = get_id(robot)
    robot.set_led(0, 180, 180)
    print("Robot %s starting JITTERY REPULSION" % str(my_id))

    start_time = robot.get_clock()
    last_print = start_time
    told_no_swarm_api = False

    while True:
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
            print("ROBOT %s OBSTACLE STOP at [%.3f, %.3f]" % (str(my_id), x, y))
            robot.delay(40)
            continue

        # 2) emergency boundary hold
        if is_critical_boundary(x, y):
            robot.set_vel(0, 0)
            robot.set_led(255, 50, 0)
            print("ROBOT %s CRITICAL STOP at [%.3f, %.3f]" % (str(my_id), x, y))
            robot.delay(40)
            continue

        # 3) base velocity from soft boundary + optional left bias
        bfx, bfy = soft_boundary_force(x, y)
        boundary_warn = (abs(bfx) + abs(bfy)) > 1e-6
        if boundary_warn: robot.set_led(255, 150, 0)
        else:             robot.set_led(0, 180, 180)

        vx = bfx + LEFT_BIAS_VX
        vy = bfy

        # 4) add soft obstacle repulsion
        obx, oby = soft_obstacle_force(x, y)
        vx += obx
        vy += oby

        # 5) add neighbor repulsion if API exists
        neighbors = try_get_swarm_poses(robot)
        if neighbors:
            for item in neighbors:
                if isinstance(item, (list, tuple)) and len(item) >= 3:
                    if len(item) == 4:
                        nid, nx, ny, nth = item
                    else:
                        nx, ny, nth = item[0], item[1], item[2]
                        nid = None
                    if (nid is not None) and (str(nid) == str(my_id)):
                        continue
                    dx, dy = x - nx, y - ny
                    r2 = dx*dx + dy*dy
                    if 1e-5 < r2 < (REPULSE_RADIUS*REPULSE_RADIUS):
                        inv = 1.0 / r2
                        s = REPULSE_GAIN * inv
                        vx += s * dx
                        vy += s * dy
        else:
            if not told_no_swarm_api:
                print("Robot %s: no swarm pose API; using jitter fallback" % str(my_id))
                told_no_swarm_api = True

        # 6) always add jitter to feel glitchy
        ang1 = random.uniform(-math.pi, math.pi)
        ang2 = random.uniform(-math.pi, math.pi)
        vx += NOISE_GAIN * math.cos(ang1) + 0.6 * NOISE_GAIN * math.cos(ang2)
        vy += NOISE_GAIN * math.sin(ang1) + 0.6 * NOISE_GAIN * math.sin(ang2)

        # 7) map (vx,vy) -> differential drive
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

        # 8) light console output without spamming
        now = robot.get_clock()
        if now - last_print > 2.0:
            print("Robot %s pos [%.3f, %.3f]" % (str(my_id), x, y))
            last_print = now

        robot.delay(40)
