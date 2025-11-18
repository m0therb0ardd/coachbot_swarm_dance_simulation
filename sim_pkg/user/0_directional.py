# import math, struct, os 

# # ----------------- LOGGING (sim and hardware) -----------------
# LOG = None

# # ----------------- Testbed bounds & center -----------------
# X_MIN, X_MAX = -1.2, 1.0
# Y_MIN, Y_MAX = -1.4, 2.35
# CX, CY = (-0.1, 0.475)   # dancer center / nominal center

# # ----------------- Dancer no-go circle -----------------
# FEET = 0.3048
# OBST_DIAM_FT = 1.0
# OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET   # ~0.1524 m
# OBST_MARGIN  = 0.03
# SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
# OBST_CX, OBST_CY = CX, CY  # obstacle centered on dancer center (fixed)
# STOP_MSG = b"S"  # global emergency stop message


# # ----------------- Translation plan -----------------
# # MOVE_DIR -1 means screen up (viewer left), +1 screen down
# MOVE_DIR    = 1
# SHIFT_RATE  = 0.18      # m/s center velocity
# STOP_MARGIN = 0.08      # keep ring this far from wall at stop

# # ----------------- Drive control -----------------
# MAX_WHEEL = 40
# TURN_K    = 3.0
# FWD_FAST  = 0.8
# FWD_SLOW  = 0.30
# EPS       = 1e-3

# # ----------------- Rigid-body locking gains -----------------
# KX = 1.2
# KY = 2.0
# KR = 2.6

# # ----------------- “GUI parity” options -----------------
# WARN_MARGIN_BOUNDARY = 0.05   # yellow warn margin (matches glitch feel)
# CRIT_MARGIN_BOUNDARY = 0.02   # hard-stop margin
# USE_SOFT_OBST_FORCE  = False  # keep False to preserve your original trajectory
# OBST_WARN_BUFFER     = 0.10   # soft obstacle buffer for warning/optional push
# OBST_MAX_SOFT_FORCE  = 0.6    # only used if USE_SOFT_OBST_FORCE = True
# logw_PERIOD         = 2.0    # seconds (like glitch)

# def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

# def wrap_angle(a):
#     while a >  math.pi: a -= 2*math.pi
#     while a <= -math.pi: a += 2*math.pi
#     return a

# def safe_pose(robot):
#     p = robot.get_pose()
#     if isinstance(p,(list,tuple)) and len(p)>=3:
#         return float(p[0]), float(p[1]), float(p[2])
#     return None

# def get_id(robot):
#     # Prefer real hardware ID when available
#     if hasattr(robot, "virtual_id") and callable(robot.virtual_id):
#         try:
#             return int(robot.virtual_id())
#         except Exception:
#             pass

#     # Simulation fallback
#     if hasattr(robot, "id"):
#         try:
#             return int(robot.id)
#         except Exception:
#             pass

#     return -1


# def init_log():
#     """
#     Try to open experiment_log.txt in a hardware-like way.
#     If it fails (e.g., sim with no FS), we just fall back to logw only.
#     """
#     global LOG
#     try:
#         # line-buffered, Py2-safe style like FLOAT
#         LOG = open("experiment_log.txt", "a", 1)
#     except Exception:
#         LOG = None

# def logw(msg):
#     """
#     Write to log file (if available) AND print to stdout.
#     Safe in both sim and hardware.
#     """
#     if not isinstance(msg, str):
#         msg = str(msg)
#     line = msg if msg.endswith("\n") else msg + "\n"

#     # Try log file first
#     if LOG is not None:
#         try:
#             LOG.write(line)
#             LOG.flush()
#             os.fsync(LOG.fileno())
#         except Exception:
#             pass

#     # Always also print (so sim output still works)
#     print(line.rstrip("\n"))




# # ---------- GUI-parity helpers  ----------
# def soft_boundary_force(x, y, max_force=0.3, boundary_margin=0.08):
#     fx, fy = 0.0, 0.0
#     if x < X_MIN + boundary_margin:
#         fx += max_force * (1.0 - (x - X_MIN) / boundary_margin)
#     elif x > X_MAX - boundary_margin:
#         fx -= max_force * (1.0 - (X_MAX - x) / boundary_margin)
#     if y < Y_MIN + boundary_margin:
#         fy += max_force * (1.0 - (y - Y_MIN) / boundary_margin)
#     elif y > Y_MAX - boundary_margin:
#         fy -= max_force * (1.0 - (Y_MAX - y) / boundary_margin)
#     return fx, fy

# def is_critical_boundary(x, y, critical_margin=CRIT_MARGIN_BOUNDARY):
#     return (x < X_MIN + critical_margin or x > X_MAX - critical_margin or
#             y < Y_MIN + critical_margin or y > Y_MAX - critical_margin)

# def obstacle_distance(x, y):
#     dx, dy = x - OBST_CX, y - OBST_CY
#     return math.hypot(dx, dy)

# def is_critical_obstacle(x, y, critical_margin=0.0):
#     return obstacle_distance(x, y) < (OBST_RADIUS + critical_margin)

# def soft_obstacle_force(x, y, max_force=OBST_MAX_SOFT_FORCE, buffer_width=OBST_WARN_BUFFER):
#     dx, dy = x - OBST_CX, y - OBST_CY
#     r = math.hypot(dx, dy)
#     if r < SAFE_BUBBLE + buffer_width:
#         if r < 1e-6:
#             return max_force, 0.0
#         strength = max(0.0, (SAFE_BUBBLE + buffer_width - r) / buffer_width)
#         s = max_force * strength
#         return s * (dx / r), s * (dy / r)
#     return 0.0, 0.0

# def usr(robot):
#     init_log()
#     my_id = get_id(robot)
#     robot.set_led(0, 180, 180)  # cyan normal
#     logw(f"Robot {my_id} starting: Translate with centroid-based center")
#     global_stop = False  # if True, this robot will permanently stop


#     # -----------------= PHASE 0: gossip to estimate formation center -----------------=
#     POSE_FMT  = "ffi"                     # float x, float y, int id
#     POSE_SIZE = struct.calcsize(POSE_FMT)

#     poses = {}  # id -> (x, y)

#     t_start = robot.get_clock()
#     gossip_duration = 2.0  # seconds to share poses

#     while robot.get_clock() - t_start < gossip_duration:
#         pose = safe_pose(robot)
#         if pose is not None:
#             x, y, _ = pose

#             # broadcast my pose (best-effort; ignore return value)
#             msg = struct.pack(POSE_FMT, x, y, my_id)
#             robot.send_msg(msg)

#             # store my own pose too
#             poses[my_id] = (x, y)

#         # receive and store others' poses
#         msgs = robot.recv_msg()
#         if msgs:
#             for m in msgs:
#                 if not m:
#                     continue
#                 try:
#                     px, py, pid = struct.unpack(POSE_FMT, m[:POSE_SIZE])
#                     poses[int(pid)] = (float(px), float(py))
#                 except Exception:
#                     # ignore malformed messages
#                     pass

#         robot.delay(50)

#     # compute formation center; fall back to dancer center if no info
#     if poses:
#         xs = [p[0] for p in poses.values()]
#         ys = [p[1] for p in poses.values()]
#         form_cx = sum(xs) / len(xs)
#         form_cy = sum(ys) / len(ys)
#     else:
#         form_cx, form_cy = CX, CY

#     logw(f"Robot {my_id}: formation center = ({form_cx:.3f}, {form_cy:.3f}), from {len(poses)} robots")

#     # -----------------= PHASE 1: the original translate-in-formation behavior -----------------=

#     # Per-robot state
#     rel_off = None
#     R_form  = None
#     s_stop  = None
#     t0      = None
#     started = False
#     last_logw_time = 0.0

#     robot.delay(300)  # small settle

#     while True:

#          # --- listen for global STOP from any robot ---
#         msgs = robot.recv_msg()
#         if msgs:
#             for m in msgs:
#                 if m and m[:1] == STOP_MSG:  # first byte is 'S'
#                     global_stop = True

#         if global_stop:
#             # Hard global stop: hold position, maybe green to indicate "frozen"
#             robot.set_vel(0, 0)
#             robot.set_led(0, 255, 0)
#             robot.delay(40)
#             continue

#         pose = safe_pose(robot)
#         if pose is None:
#             robot.set_vel(0,0)
#             robot.delay(20)
#             continue
#         x, y, th = pose

#         # --- CRITICAL STOPS (GUI parity) ---
#         if is_critical_obstacle(x, y) or is_critical_boundary(x, y):
#             # Broadcast an emergency STOP to the swarm
#             robot.send_msg(STOP_MSG)
#             global_stop = True

#             robot.set_vel(0, 0)
#             robot.set_led(255, 50, 0)  # orange/red = critical
#             logw(f"ROBOT {my_id} EMERGENCY STOP at [{x:.3f}, {y:.3f}]")
#             robot.delay(40)
#             continue


#         # --- WARNINGS (yellow), otherwise cyan ---
#         obst_r = obstacle_distance(x, y)
#         near_obstacle = obst_r < (SAFE_BUBBLE + OBST_WARN_BUFFER)
#         near_boundary = (x < X_MIN + WARN_MARGIN_BOUNDARY or x > X_MAX - WARN_MARGIN_BOUNDARY or
#                          y < Y_MIN + WARN_MARGIN_BOUNDARY or y > Y_MAX - WARN_MARGIN_BOUNDARY)
#         if near_obstacle or near_boundary:
#             robot.set_led(255, 150, 0)  # yellow warn
#         else:
#             robot.set_led(0, 180, 180)  # cyan normal

#         # Lock initial offset & compute stop distance once
#         if rel_off is None:
#             # offset relative to *formation* center, not dancer center
#             rel_off = (x - form_cx, y - form_cy)
#             R_form  = math.hypot(*rel_off)

#             # Wall-limited shift, now in Y (vertical), based on formation center
#             # safety_buffer = 0.05
#             # if MOVE_DIR < 0:  # moving up (toward Y_MAX)
#             #     s_wall = max(0.0, (Y_MAX - STOP_MARGIN - safety_buffer - R_form) - form_cy)
#             # else:  # moving down (toward Y_MIN)
#             #     s_wall = max(0.0, form_cy - (Y_MIN + STOP_MARGIN + safety_buffer + R_form))

#             # # Obstacle limit based on bubble vs radius
#             # s_obst = max(0.0, R_form - SAFE_BUBBLE)
#             # s_stop = min(s_wall, s_obst)

#             # Wall-limited shift, now in Y (vertical), based on formation center
#             safety_buffer = 0.05
#             if MOVE_DIR < 0:  # moving up (toward Y_MAX)
#                 s_wall = max(0.0, (Y_MAX - STOP_MARGIN - safety_buffer - R_form) - form_cy)
#             else:  # moving down (toward Y_MIN)
#                 s_wall = max(0.0, form_cy - (Y_MIN + STOP_MARGIN + safety_buffer + R_form))

#             # For this translation, let the walls be the only limiter.
#             # Obstacle safety is handled by is_critical_obstacle(...) and soft forces.
#             s_stop = s_wall
#             # synced start half-second grid
#             now = robot.get_clock()
#             t0 = math.floor(now * 2.0) / 2.0 + 0.5

#             logw(f"Robot {my_id} locked: R={R_form:.3f}, s_stop={s_stop:.3f}")

#         # Wait for shared start
#         t = robot.get_clock()
#         if not started:
#             if t < t0:
#                 robot.set_vel(0,0)
#                 robot.delay(10)
#                 continue
#             started = True
#             logw(f"Robot {my_id} started moving")

#         # Center shift
#         s = min(max(0.0, t - t0) * SHIFT_RATE, s_stop)

#         # Moving center + rigid target (vertical translation) from formation center
#         Cx = form_cx
#         Cy = form_cy - MOVE_DIR * s   # MOVE_DIR=-1 => Cy increases (up)

#         # keep center valid
#         Cx = max(X_MIN + R_form + 0.08, min(X_MAX - R_form - 0.08, Cx))
#         Cy = max(Y_MIN + R_form + 0.08, min(Y_MAX - R_form - 0.08, Cy))

#         tx = Cx + rel_off[0]
#         ty = Cy + rel_off[1]

#         # Position error → base velocity
#         ex, ey = tx - x, ty - y
#         vx = KX * ex
#         vy = KY * ey

#         # feed-forward along the translate direction (now vertical)
#         # vy += -MOVE_DIR * SHIFT_RATE   # MOVE_DIR=-1 => vy += +SHIFT_RATE (up)

#         # Radial lock to initial offset
#         # rx, ry = x - Cx, y - Cy
#         # vx += KR * (rel_off[0] - rx)
#         # vy += KR * (rel_off[1] - ry)

#         # Soft boundary cushion (visual parity; mild effect)
#         bfx, bfy = soft_boundary_force(x, y, max_force=0.3, boundary_margin=0.08)
#         vx += bfx; vy += bfy

#         # Optional: soft obstacle push to mirror glitch feel (off by default)
#         if USE_SOFT_OBST_FORCE:
#             ofx, ofy = soft_obstacle_force(x, y)
#             vx += ofx; vy += ofy

#         # Finished? stop & hold
#         if abs(s - s_stop) < 1e-6 and (abs(ex) + abs(ey) < 0.01):
#             robot.set_vel(0, 0)
#             robot.set_led(0, 255, 0)  # green = completed
#             robot.delay(20)
#             # keep holding position but continue the GUI loop
#             continue

#         # Map (vx,vy) → wheels
#         if abs(vx) + abs(vy) < EPS:
#             robot.set_vel(0, 0)
#         else:
#             hdg = math.atan2(vy, vx)
#             err = wrap_angle(hdg - th)

#             abs_err = abs(err)
#             if abs_err < 0.5:      fwd = FWD_FAST
#             elif abs_err < 1.2:    fwd = FWD_FAST * 0.7
#             else:                  fwd = FWD_SLOW

#             turn = clamp(TURN_K * err, -1.5, 1.5)

#             left  = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
#             right = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
#             robot.set_vel(left, right)

#         # glitch-like periodic console status
#         now = robot.get_clock()
#         if now - last_logw_time > logw_PERIOD:
#             logw(f"Robot {my_id} pos [{x:.3f}, {y:.3f}], s={s:.3f}")
#             last_logw_time = now

#         robot.delay(20)


# ######################################################

# # -*- coding: utf-8 -*-
# import math, struct, os

# # ----------------- LOGGING (lab style, sim + hardware) -----------------
# log = None       # /home/pi/control/experiment_log
# log_out = None   # /home/pi/experiment_output

# def logw(msg):
#     """Write to experiment_log (lab spec) AND /home/pi/experiment_output for fetch_logs."""
#     global log, log_out
#     try:
#         s = str(msg)
#     except:
#         s = repr(msg)
#     if not s.endswith("\n"):
#         s = s + "\n"

#     # 1) standard experiment_log in current dir
#     if log is None:
#         try:
#             log = open("experiment_log", "wb")  # lab requirement
#         except:
#             log = None
#     if log is not None:
#         try:
#             log.write(s)
#             log.flush()
#         except:
#             pass

#     # 2) extra file where cctl.fetch_output_handler expects things
#     if log_out is None:
#         try:
#             # absolute path to match scp pi@ip:/home/pi/experiment_output ...
#             log_out = open("/home/pi/experiment_output", "wb")
#         except:
#             log_out = None
#     if log_out is not None:
#         try:
#             log_out.write(s)
#             log_out.flush()
#         except:
#             pass

#     # Optional: also print to console (sim)
#     try:
#         print(s.rstrip("\n"))
#     except:
#         pass


# # ----------------- Testbed bounds & center -----------------
# X_MIN, X_MAX = -1.2, 1.0
# Y_MIN, Y_MAX = -1.4, 2.35
# CX, CY = (-0.1, 0.475)   # dancer center / nominal center

# # ----------------- Dancer no-go circle -----------------
# FEET = 0.3048
# OBST_DIAM_FT = 1.0
# OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET   # ~0.1524 m
# OBST_MARGIN  = 0.03
# SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
# OBST_CX, OBST_CY = CX, CY  # obstacle centered on dancer center (fixed)
# STOP_MSG = b"S"  # global emergency stop message

# # ----------------- Translation plan -----------------
# # MOVE_DIR -1 means screen up (viewer left), +1 screen down
# MOVE_DIR    = 1
# SHIFT_RATE  = 0.18      # m/s center velocity
# STOP_MARGIN = 0.08      # keep ring this far from wall at stop

# # ----------------- Drive control -----------------
# MAX_WHEEL = 40
# TURN_K    = 3.0
# FWD_FAST  = 0.8
# FWD_SLOW  = 0.30
# EPS       = 1e-3

# # ----------------- Rigid-body locking gains -----------------
# KX = 1.2
# KY = 2.0
# KR = 2.6

# # ----------------- “GUI parity” options -----------------
# WARN_MARGIN_BOUNDARY = 0.05   # yellow warn margin (matches glitch feel)
# CRIT_MARGIN_BOUNDARY = 0.02   # hard-stop margin
# USE_SOFT_OBST_FORCE  = False  # keep False to preserve your original trajectory
# OBST_WARN_BUFFER     = 0.10   # soft obstacle buffer for warning/optional push
# OBST_MAX_SOFT_FORCE  = 0.6    # only used if USE_SOFT_OBST_FORCE = True
# logw_PERIOD          = 2.0    # seconds (like glitch)

# def clamp(v, lo, hi):
#     return lo if v < lo else hi if v > hi else v

# def wrap_angle(a):
#     while a >  math.pi:
#         a -= 2*math.pi
#     while a <= -math.pi:
#         a += 2*math.pi
#     return a

# def safe_pose(robot):
#     p = robot.get_pose()
#     if isinstance(p,(list,tuple)) and len(p)>=3:
#         return float(p[0]), float(p[1]), float(p[2])
#     return None

# def get_id(robot):
#     """
#     Prefer hardware virtual_id(), fall back to sim .id.
#     (virtual_id commented out so we currently force robot.id, like the model script)
#     """
#     # # Real hardware ID
#     # if hasattr(robot, "virtual_id") and callable(robot.virtual_id):
#     #     try:
#     #         return int(robot.virtual_id())
#     #     except Exception:
#     #         pass

#     # Simulation ID
#     if hasattr(robot, "id"):
#         try:
#             return int(robot.id)
#         except Exception:
#             pass

#     return -1

# # ---------- GUI-parity helpers  ----------
# def soft_boundary_force(x, y, max_force=0.3, boundary_margin=0.08):
#     fx, fy = 0.0, 0.0
#     if x < X_MIN + boundary_margin:
#         fx += max_force * (1.0 - (x - X_MIN) / boundary_margin)
#     elif x > X_MAX - boundary_margin:
#         fx -= max_force * (1.0 - (X_MAX - x) / boundary_margin)
#     if y < Y_MIN + boundary_margin:
#         fy += max_force * (1.0 - (y - Y_MIN) / boundary_margin)
#     elif y > Y_MAX - boundary_margin:
#         fy -= max_force * (1.0 - (Y_MAX - y) / boundary_margin)
#     return fx, fy

# def is_critical_boundary(x, y, critical_margin=CRIT_MARGIN_BOUNDARY):
#     return (x < X_MIN + critical_margin or x > X_MAX - critical_margin or
#             y < Y_MIN + critical_margin or y > Y_MAX - critical_margin)

# def obstacle_distance(x, y):
#     dx, dy = x - OBST_CX, y - OBST_CY
#     return math.hypot(dx, dy)

# def is_critical_obstacle(x, y, critical_margin=0.0):
#     return obstacle_distance(x, y) < (OBST_RADIUS + critical_margin)

# def soft_obstacle_force(x, y, max_force=OBST_MAX_SOFT_FORCE, buffer_width=OBST_WARN_BUFFER):
#     dx, dy = x - OBST_CX, y - OBST_CY
#     r = math.hypot(dx, dy)
#     if r < SAFE_BUBBLE + buffer_width:
#         if r < 1e-6:
#             return max_force, 0.0
#         strength = max(0.0, (SAFE_BUBBLE + buffer_width - r) / buffer_width)
#         s = max_force * strength
#         return s * (dx / r), s * (dy / r)
#     return 0.0, 0.0

# def usr(robot):
#     global log, log_out
#     # open log exactly as lab wants
#     try:
#         log = open("experiment_log", "wb")
#     except:
#         log = None

#     my_id = get_id(robot)
#     robot.set_led(0, 180, 180)  # cyan normal
#     logw("Robot %d starting: Translate with centroid-based center" % my_id)
#     global_stop = False  # if True, this robot will permanently stop

#     # ----------------- PHASE 0: gossip to estimate formation center -----------------
#     POSE_FMT  = "ffi"                     # float x, float y, int id
#     POSE_SIZE = struct.calcsize(POSE_FMT)

#     poses = {}  # id -> (x, y)

#     t_start = robot.get_clock()
#     gossip_duration = 2.0  # seconds to share poses

#     while robot.get_clock() - t_start < gossip_duration:
#         pose = safe_pose(robot)
#         if pose is not None:
#             x, y, _ = pose

#             # broadcast my pose (best-effort; ignore return value)
#             try:
#                 msg = struct.pack(POSE_FMT, x, y, my_id)
#                 robot.send_msg(msg)
#             except:
#                 pass

#             # store my own pose too
#             poses[my_id] = (x, y)

#         # receive and store others' poses
#         msgs = robot.recv_msg()
#         if msgs is not None:
#             if not isinstance(msgs, list):
#                 msgs = [msgs]
#             for m in msgs:
#                 if not m:
#                     continue
#                 try:
#                     px, py, pid = struct.unpack(POSE_FMT, m[:POSE_SIZE])
#                     poses[int(pid)] = (float(px), float(py))
#                 except Exception:
#                     # ignore malformed messages
#                     pass

#         robot.delay(50)

#     # compute formation center; fall back to dancer center if no info
#     if poses:
#         xs = [p[0] for p in poses.values()]
#         ys = [p[1] for p in poses.values()]
#         form_cx = sum(xs) / float(len(xs))
#         form_cy = sum(ys) / float(len(ys))
#     else:
#         form_cx, form_cy = CX, CY

#     logw("Robot %d: formation center = (%.3f, %.3f), from %d robots"
#          % (my_id, form_cx, form_cy, len(poses)))

#     # ----------------- PHASE 1: original translate-in-formation behavior -----------------
#     rel_off = None
#     R_form  = None
#     s_stop  = None
#     t0      = None
#     started = False
#     last_logw_time = 0.0

#     robot.delay(300)  # small settle

#     while True:

#         # --- listen for global STOP from any robot ---
#         msgs = robot.recv_msg()
#         if msgs is not None:
#             if not isinstance(msgs, list):
#                 msgs = [msgs]
#             for m in msgs:
#                 if m and m[:1] == STOP_MSG:  # first byte is 'S'
#                     global_stop = True

#         if global_stop:
#             # Hard global stop: hold position, maybe green to indicate "frozen"
#             robot.set_vel(0, 0)
#             robot.set_led(0, 255, 0)
#             robot.delay(40)
#             continue

#         pose = safe_pose(robot)
#         if pose is None:
#             robot.set_vel(0,0)
#             robot.delay(20)
#             continue
#         x, y, th = pose

#         # --- CRITICAL STOPS (GUI parity) ---
#         if is_critical_obstacle(x, y) or is_critical_boundary(x, y):
#             # Broadcast an emergency STOP to the swarm
#             try:
#                 robot.send_msg(STOP_MSG)
#             except:
#                 pass
#             global_stop = True

#             robot.set_vel(0, 0)
#             robot.set_led(255, 50, 0)  # orange/red = critical
#             logw("ROBOT %d EMERGENCY STOP at [%.3f, %.3f]" % (my_id, x, y))
#             robot.delay(40)
#             continue

#         # --- WARNINGS (yellow), otherwise cyan ---
#         obst_r = obstacle_distance(x, y)
#         near_obstacle = obst_r < (SAFE_BUBBLE + OBST_WARN_BUFFER)
#         near_boundary = (x < X_MIN + WARN_MARGIN_BOUNDARY or x > X_MAX - WARN_MARGIN_BOUNDARY or
#                          y < Y_MIN + WARN_MARGIN_BOUNDARY or y > Y_MAX - WARN_MARGIN_BOUNDARY)
#         if near_obstacle or near_boundary:
#             robot.set_led(255, 150, 0)  # yellow warn
#         else:
#             robot.set_led(0, 180, 180)  # cyan normal

#         # Lock initial offset & compute stop distance once
#         if rel_off is None:
#             # offset relative to *formation* center, not dancer center
#             rel_off = (x - form_cx, y - form_cy)
#             R_form  = math.hypot(rel_off[0], rel_off[1])

#             safety_buffer = 0.05
#             if MOVE_DIR < 0:  # moving up (toward Y_MAX)
#                 s_wall = max(0.0, (Y_MAX - STOP_MARGIN - safety_buffer - R_form) - form_cy)
#             else:  # moving down (toward Y_MIN)
#                 s_wall = max(0.0, form_cy - (Y_MIN + STOP_MARGIN + safety_buffer + R_form))

#             # For this translation, let the walls be the only limiter.
#             # Obstacle safety is handled by is_critical_obstacle(...) and soft forces.
#             s_stop = s_wall

#             # synced start half-second grid
#             now = robot.get_clock()
#             t0 = math.floor(now * 2.0) / 2.0 + 0.5

#             logw("Robot %d locked: R=%.3f, s_stop=%.3f" % (my_id, R_form, s_stop))

#         # Wait for shared start
#         t = robot.get_clock()
#         if not started:
#             if t < t0:
#                 robot.set_vel(0,0)
#                 robot.delay(10)
#                 continue
#             started = True
#             logw("Robot %d started moving" % my_id)

#         # Center shift
#         s = min(max(0.0, t - t0) * SHIFT_RATE, s_stop)

#         # Moving center + rigid target (vertical translation) from formation center
#         Cx = form_cx
#         Cy = form_cy - MOVE_DIR * s   # MOVE_DIR=-1 => Cy increases (up)

#         # keep center valid
#         Cx = max(X_MIN + R_form + 0.08, min(X_MAX - R_form - 0.08, Cx))
#         Cy = max(Y_MIN + R_form + 0.08, min(Y_MAX - R_form - 0.08, Cy))

#         tx = Cx + rel_off[0]
#         ty = Cy + rel_off[1]

#         # Position error → base velocity
#         ex, ey = tx - x, ty - y
#         vx = KX * ex
#         vy = KY * ey

#         # Soft boundary cushion (visual parity; mild effect)
#         bfx, bfy = soft_boundary_force(x, y, max_force=0.3, boundary_margin=0.08)
#         vx += bfx
#         vy += bfy

#         # Optional: soft obstacle push to mirror glitch feel (off by default)
#         if USE_SOFT_OBST_FORCE:
#             ofx, ofy = soft_obstacle_force(x, y)
#             vx += ofx
#             vy += ofy

#         # Finished? stop & hold
#         if abs(s - s_stop) < 1e-6 and (abs(ex) + abs(ey) < 0.01):
#             robot.set_vel(0, 0)
#             robot.set_led(0, 255, 0)  # green = completed
#             robot.delay(20)
#             # keep holding position but continue the GUI loop
#             continue

#         # Map (vx,vy) → wheels
#         if abs(vx) + abs(vy) < EPS:
#             robot.set_vel(0, 0)
#         else:
#             hdg = math.atan2(vy, vx)
#             err = wrap_angle(hdg - th)

#             abs_err = abs(err)
#             if abs_err < 0.5:
#                 fwd = FWD_FAST
#             elif abs_err < 1.2:
#                 fwd = FWD_FAST * 0.7
#             else:
#                 fwd = FWD_SLOW

#             turn = clamp(TURN_K * err, -1.5, 1.5)

#             left  = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
#             right = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
#             robot.set_vel(left, right)

#         # glitch-like periodic console status
#         now = robot.get_clock()
#         if now - last_logw_time > logw_PERIOD:
#             logw("Robot %d pos [%.3f, %.3f], s=%.3f" % (my_id, x, y, s))
#             last_logw_time = now

#         robot.delay(20)


# -*- coding: utf-8 -*-
import math
import os
import struct

# --- field (arena bounds) ---
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

# --- Dancer no-go circle (meters) ---
FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN

# --- Motion & control (MATCH SIM) ---
# MOVE_DIR = -1 => move UP (toward +Y)
# MOVE_DIR = +1 => move DOWN (toward -Y)
MOVE_DIR        = 1
BASE_SHIFT_RATE = 0.18
STOP_MARGIN     = 0.08
MAX_WHEEL       = 35
TURN_K          = 3.0
FWD_FAST        = 0.8
FWD_SLOW        = 0.30
EPS             = 1e-3
KX              = 1.2
KY              = 2.0
KR              = 2.6

# --- center-estimation messaging ---
POSE_FMT   = 'ffi'   # x, y, id
POSE_BYTES = struct.calcsize(POSE_FMT)
CENTER_GATHER_TIME = 1.5  # seconds to gather positions

def clamp(v, lo, hi):
    return max(lo, min(v, hi))

def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a

def safe_pose(robot):
    p = robot.get_pose()
    if p and len(p) >= 3:
        return float(p[0]), float(p[1]), float(p[2])
    return None

def usr(robot):
    robot.delay(3000)

    # get id first so we can use a per-robot log
    try:
        vid = int(robot.virtual_id())
    except:
        vid = -1

    log_main = open("experiment_log.txt", "a")
    def logw(s):
        if not s.endswith("\n"):
            s += "\n"
        log_main.write(s)
        log_main.flush()
        try:
            os.fsync(log_main.fileno())
        except:
            pass

    try:
        logw("I am robot %s" % str(vid))

        def soft_boundary_check(x, y):
            warning_margin  = 0.08
            critical_margin = 0.02
            if (x < X_MIN + critical_margin or x > X_MAX - critical_margin or
                y < Y_MIN + critical_margin or y > Y_MAX - critical_margin):
                return 2
            elif (x < X_MIN + warning_margin or x > X_MAX - warning_margin or
                  y < Y_MIN + warning_margin or y > Y_MAX - warning_margin):
                return 1
            return 0

        # --- STATE ---
        # center estimation
        center_ready      = False
        center_start_time = None
        sum_x = 0.0
        sum_y = 0.0
        count = 0
        C0x = None
        C0y = None

        # ring translation state
        rel_off   = None
        R_form    = None
        s_stop    = None
        t0        = None
        started   = False
        last_log_sec = -1
        last_pose = None  # for final log

        start_time = robot.get_clock()
        max_runtime = 55.0

        # small nudge to wake localization
        robot.set_vel(20, 20)
        robot.delay(150)

        while (robot.get_clock() - start_time) < max_runtime:
            pose = safe_pose(robot)
            if pose is None:
                robot.set_vel(0, 0)
                robot.delay(20)
                continue

            x, y, th = pose
            last_pose = (x, y)
            now = robot.get_clock()

            # Boundary light + protection
            bstat = soft_boundary_check(x, y)
            if bstat == 2:
                logw("CRITICAL: Robot %s at boundary [%.3f, %.3f]" % (str(vid), x, y))
                robot.set_vel(0, 0)
                robot.set_led(255, 0, 0)
                break
            elif bstat == 1:
                robot.set_led(255, 150, 0)  # warn
            else:
                robot.set_led(0, 180, 180)  # normal

            # --- PHASE 1: estimate ring center from robot positions ---
            if not center_ready:
                if center_start_time is None:
                    center_start_time = now
                    logw("Robot %s: starting center-gather phase" % str(vid))
                    # slight LED code to show "gathering"
                    robot.set_led(180, 180, 0)

                # add our own pose to running sums
                sum_x += x
                sum_y += y
                count += 1

                # broadcast our pose
                try:
                    pkt = struct.pack(POSE_FMT, x, y, vid)
                    robot.send_msg(pkt)
                except:
                    pass

                # receive others' poses
                msgs = robot.recv_msg()
                if msgs:
                    for m in msgs:
                        try:
                            rx, ry, rid = struct.unpack(POSE_FMT, m[:POSE_BYTES])
                            # we don't de-duplicate; multiple samples just average out
                            sum_x += rx
                            sum_y += ry
                            count += 1
                        except:
                            pass

                # after CENTER_GATHER_TIME seconds, compute center
                if now - center_start_time >= CENTER_GATHER_TIME and count > 0:
                    C0x = sum_x / float(count)
                    C0y = sum_y / float(count)
                    center_ready = True
                    logw("Robot %s: estimated center (C0x,C0y)=(%.3f, %.3f) from %d samples"
                         % (str(vid), C0x, C0y, count))
                    robot.set_led(0, 200, 200)  # ready
                else:
                    # hold still during center estimation
                    robot.set_vel(0, 0)
                    robot.delay(20)
                    continue

            # --- PHASE 2: ring translation using estimated center C0x, C0y ---
            if rel_off is None:
                # offset from estimated ring center
                rel_off = (x - C0x, y - C0y)
                R_form  = math.hypot(rel_off[0], rel_off[1])

                safety_buffer = 0.05
                if MOVE_DIR < 0:  # UP (toward Y_MAX)
                    s_wall = max(0.0, (Y_MAX - STOP_MARGIN - safety_buffer - R_form) - C0y)
                else:             # DOWN (toward Y_MIN)
                    s_wall = max(0.0, C0y - (Y_MIN + STOP_MARGIN + safety_buffer + R_form))

                # obstacle-limited shift (stay outside SAFE_BUBBLE)
                s_obst = max(0.0, R_form - SAFE_BUBBLE)
                s_stop = min(s_wall, s_obst)

                t0 = now + 1.0  # small sync delay
                logw("Robot %s: R=%.3f, s_stop=%.3f, rate=%.3f, center=(%.3f, %.3f)"
                     % (str(vid), R_form, s_stop, BASE_SHIFT_RATE, C0x, C0y))
                if s_stop <= 0.0:
                    logw("Robot %s: s_stop=0, no safe translation; holding position" % str(vid))
                robot.set_led(255, 200, 0)

            # Wait for synchronized start
            if not started:
                if now < t0:
                    robot.set_vel(0, 0)
                    robot.delay(10)
                    continue
                started = True
                robot.set_led(0, 200, 0)
                logw("Robot %s started ring translation" % str(vid))

            # Compute center shift along Y (up/down)
            s = max(0.0, (now - t0) * BASE_SHIFT_RATE)
            if s_stop is not None:
                s = min(s, s_stop)

            # shifted center (only Y moves from C0y)
            Cx = C0x
            Cy = C0y + MOVE_DIR * s

            # Keep ring center valid in the box
            Cx = max(X_MIN + R_form + 0.08, min(X_MAX - R_form - 0.08, Cx))
            Cy = max(Y_MIN + R_form + 0.08, min(Y_MAX - R_form - 0.08, Cy))

            # Target is ring-offset from shifted center
            tx = Cx + rel_off[0]
            ty = Cy + rel_off[1]

            ex = tx - x
            ey = ty - y

            # If reached the planned shift and position error is tiny -> done
            if (s_stop is not None) and (abs(s_stop - s) < 1e-6) and (math.hypot(ex, ey) < 0.02):
                robot.set_vel(0, 0)
                robot.set_led(0, 80, 255)
                logw("Robot %s completed mission" % str(vid))
                break

            # Smooth heading + speed control toward (tx, ty)
            vx = KX * ex + KR * (rel_off[0] - (x - Cx))
            vy = KY * ey + MOVE_DIR * BASE_SHIFT_RATE + KR * (rel_off[1] - (y - Cy))

            if abs(vx) + abs(vy) > EPS:
                hdg = math.atan2(vy, vx)
                err = wrap_angle(hdg - th)

                abs_err = abs(err)
                if abs_err < 0.5:
                    fwd = FWD_FAST
                elif abs_err < 1.2:
                    fwd = FWD_FAST * 0.7
                else:
                    fwd = FWD_SLOW

                if bstat == 1:
                    fwd *= 0.7

                turn = clamp(TURN_K * err, -1.5, 1.5)

                left  = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)),
                              -MAX_WHEEL,  MAX_WHEEL)
                right = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)),
                              -MAX_WHEEL,  MAX_WHEEL)
                robot.set_vel(left, right)
            else:
                robot.set_vel(0, 0)

            robot.delay(20)

    except Exception as e:
        logw("ERROR: %s" % str(e))
        try:
            robot.set_vel(0, 0)
            robot.set_led(255, 0, 0)
        except:
            pass
        raise
    finally:
        final_time = robot.get_clock()
        if last_pose:
            lx, ly = last_pose
        else:
            lx = ly = float('nan')
        try:
            robot.set_vel(0, 0)
        except:
            pass
        logw("Robot %s finished at [%.3f, %.3f] after %.1fs"
             % (str(vid), lx, ly, final_time - start_time))
        log_main.close()



