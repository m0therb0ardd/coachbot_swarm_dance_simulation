# # -*- coding: utf-8 -*-
# # Concentric Circle Formation (multi-ring, NO dancer no-go zone)
# # Python 2.7 + Coachbot messaging + lab logging pattern + separation

# import math, struct

# # ----------------- USER INPUT: RINGS -----------------
# NUM_RINGS = 2   # <-- change to 1, 2, 3, ... as needed

# # ----------------- Logging -----------------
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

# # ----------------- Circle parameters (NO no-go zone) -----------------
# # Previously: R_TARGET_OUTER = SAFE_BUBBLE + 0.33
# # SAFE_BUBBLE ≈ 0.18, so effective radius was ~0.51
# R_TARGET_OUTER = 0.52
# RING_SPACING   = 0.3   # radial spacing between rings (m)

# # DIRECTION per ring: inner CCW, outer CW, alternating
# # +1 = CCW, -1 = CW
# def build_ring_directions(num_rings):
#     dirs = []
#     for r in range(num_rings):
#         if r % 2 == 0:
#             dirs.append(1)   # inner, ring 0: CCW
#         else:
#             dirs.append(-1)  # ring 1: CW
#     return dirs

# # ----------------- Drive control -----------------
# MAX_WHEEL = 35
# TURN_K    = 3.0
# FWD_FAST  = 0.75
# FWD_SLOW  = 0.25
# EPS       = 1e-3

# # ----------------- Control gains -----------------
# K_RADIAL    = 1.5
# K_ANGULAR   = 1.4   # slightly softer to help settling

# # ----------------- Separation (collision avoidance) -----------------
# MIN_SEP     = 0.18
# SEP_GAIN    = 0.6

# def clamp(v, lo, hi):
#     return lo if v < lo else (hi if v > hi else v)

# def wrap_angle(a):
#     while a >  math.pi:
#         a -= 2*math.pi
#     while a <= -math.pi:
#         a += 2*math.pi
#     return a

# def safe_pose(robot):
#     p = robot.get_pose()
#     if isinstance(p, (list, tuple)) and len(p) >= 3:
#         return float(p[0]), float(p[1]), float(p[2])
#     return None

# def get_id(robot):
#     """
#     Prefer sim .id (works in sim); on hardware you can swap in virtual_id().
#     """
#     if hasattr(robot, "id"):
#         try:
#             return int(robot.id)
#         except:
#             pass
#     return -1

# def soft_boundary_force(x, y, max_force=0.3, boundary_margin=0.08):
#     fx = 0.0
#     fy = 0.0
#     if x < X_MIN + boundary_margin:
#         fx += max_force * (1.0 - (x - X_MIN) / boundary_margin)
#     elif x > X_MAX - boundary_margin:
#         fx -= max_force * (1.0 - (X_MAX - x) / boundary_margin)
#     if y < Y_MIN + boundary_margin:
#         fy += max_force * (1.0 - (y - Y_MIN) / boundary_margin)
#     elif y > Y_MAX - boundary_margin:
#         fy -= max_force * (1.0 - (Y_MAX - y) / boundary_margin)
#     return fx, fy

# def compute_ring_assignments(total_robots, num_rings):
#     """
#     Split total_robots into num_rings buckets.
#     Returns:
#         counts:  [n0, n1, ...] robots per ring (inner to outer)
#         offsets: [o0, o1, ...] starting global index for each ring
#     """
#     if num_rings < 1:
#         num_rings = 1
#     if num_rings > total_robots:
#         num_rings = total_robots

#     base = total_robots // num_rings
#     extra = total_robots % num_rings

#     counts = []
#     offsets = []
#     offset = 0
#     r = 0
#     while r < num_rings:
#         n = base
#         if r < extra:
#             n += 1
#         counts.append(n)
#         offsets.append(offset)
#         offset += n
#         r += 1
#     return counts, offsets, num_rings

# # def usr(robot):
# #     global log, log_out 

# #     # ---- open log exactly as lab wants ----
# #     try:
# #         log = open("experiment_log", "wb")
# #     except:
# #         log = None

# #     my_id = get_id(robot)
# #     robot.set_led(0, 180, 180)  # cyan normal
# #     logw("Robot %d starting: Concentric Circle Formation (NUM_RINGS=%d)" % (my_id, NUM_RINGS))

# #     # ----------------- PHASE 1: gossip to discover all robots -----------------
# #     POSE_FMT  = "ffi"  # float x, float y, int id
# #     POSE_SIZE = struct.calcsize(POSE_FMT)

# #     poses = {}  # id -> (x, y)

# #     t_start = robot.get_clock()
# #     gossip_duration = 2.0  # seconds to share poses

# #     while robot.get_clock() - t_start < gossip_duration:
# #         pose = safe_pose(robot)
# #         if pose is not None:
# #             x, y, _ = pose
# #             # broadcast my pose
# #             try:
# #                 msg = struct.pack(POSE_FMT, x, y, my_id)
# #                 robot.send_msg(msg)
# #             except:
# #                 pass
# #             # store my own pose too
# #             poses[my_id] = (x, y)

# #         # receive and store others' poses
# #         msgs = robot.recv_msg()
# #         if msgs is not None:
# #             if not isinstance(msgs, list):
# #                 msgs = [msgs]
# #             for m in msgs:
# #                 if not m:
# #                     continue
# #                 try:
# #                     px, py, pid = struct.unpack(POSE_FMT, m[:POSE_SIZE])
# #                     poses[int(pid)] = (float(px), float(py))
# #                 except:
# #                     pass

# #         robot.delay(50)

# #     # ----------------- PHASE 2: Assign target positions on rings -----------------
# #     total_robots = len(poses)

# #     # Precompute ring radii (inner to outer)
# #     ring_radii = []
# #     r_idx = 0
# #     while r_idx < max(1, min(NUM_RINGS, total_robots)):
# #         n_rings_active = max(1, min(NUM_RINGS, total_robots))
# #         step_index = r_idx
# #         radius = R_TARGET_OUTER - (n_rings_active - 1 - step_index) * RING_SPACING
# #         ring_radii.append(radius)
# #         r_idx += 1

# #     ring_dirs = build_ring_directions(len(ring_radii))

# #     if total_robots == 0:
# #         logw("Robot %d: No robots discovered, using default position" % my_id)
# #         target_angle    = 0.0
# #         my_ring_id      = 0
# #         my_ring_count   = 1
# #         R_TARGET_LOCAL  = R_TARGET_OUTER
# #         DIRECTION_LOCAL = 1
# #     else:
# #         # Sort robot IDs for consistent ordering
# #         sorted_ids = sorted(poses.keys())
# #         try:
# #             my_index = sorted_ids.index(my_id)
# #         except ValueError:
# #             my_index = 0

# #         # Compute ring sizes + offsets
# #         counts, offsets, num_rings_active = compute_ring_assignments(total_robots, NUM_RINGS)

# #         ring_radii = ring_radii[:num_rings_active]
# #         ring_dirs  = ring_dirs[:num_rings_active]

# #         # Which ring am I in?
# #         my_ring_id = 0
# #         my_ring_index_in_ring = 0
# #         my_ring_count = total_robots
# #         r = 0
# #         while r < num_rings_active:
# #             start_idx = offsets[r]
# #             end_idx   = offsets[r] + counts[r]
# #             if my_index >= start_idx and my_index < end_idx:
# #                 my_ring_id = r
# #                 my_ring_index_in_ring = my_index - start_idx
# #                 my_ring_count = counts[r]
# #                 break
# #             r += 1

# #         # Angular spacing in my ring
# #         if my_ring_count > 0:
# #             angle_spacing = 2.0 * math.pi / float(my_ring_count)
# #         else:
# #             angle_spacing = 2.0 * math.pi

# #         target_angle = my_ring_index_in_ring * angle_spacing

# #         # Ring-specific radius + direction
# #         if my_ring_id < len(ring_radii):
# #             R_TARGET_LOCAL = ring_radii[my_ring_id]
# #         else:
# #             R_TARGET_LOCAL = R_TARGET_OUTER

# #         if my_ring_id < len(ring_dirs):
# #             DIRECTION_LOCAL = ring_dirs[my_ring_id]
# #         else:
# #             DIRECTION_LOCAL = 1

# #         logw("Robot %d: %d robots total, my_index=%d, ring_id=%d, ring_index=%d, ring_count=%d"
# #              % (my_id, total_robots, my_index, my_ring_id, my_ring_index_in_ring, my_ring_count))
# #         logw("Robot %d: ring_radius=%.3f, target_angle=%.3f, direction=%d"
# #              % (my_id, R_TARGET_LOCAL, target_angle, DIRECTION_LOCAL))

# #     # Target position on my ring
# #     target_x = CX + R_TARGET_LOCAL * math.cos(target_angle)
# #     target_y = CY + R_TARGET_LOCAL * math.sin(target_angle)

# #     logw("Robot %d target: (%.3f, %.3f)" % (my_id, target_x, target_y))

# #     # ----------------- PHASE 3: Move to target positions -----------------
# #     last_logw_time = 0.0
# #     logw_PERIOD = 2.0

# #     formation_achieved = False

# #     while True:
# #         pose = safe_pose(robot)
# #         if pose is None:
# #             robot.set_vel(0,0)
# #             robot.delay(20)
# #             continue

# #         x, y, th = pose

# #         # If formation is achieved, stop and skip control
# #         if formation_achieved:
# #             robot.set_vel(0, 0)
# #             robot.set_led(0, 255, 0)  # green
# #             robot.delay(40)
# #             continue

# #         # Position relative to center
# #         dx = x - CX
# #         dy = y - CY
# #         current_r = math.hypot(dx, dy)
# #         current_angle = math.atan2(dy, dx)

# #         # Errors
# #         radial_error  = R_TARGET_LOCAL - current_r
# #         angular_error = wrap_angle(target_angle - current_angle)

# #         # Unit vectors
# #         if current_r > 1e-6:
# #             ur = (dx / current_r, dy / current_r)    # radial
# #             ut = (-ur[1], ur[0])                     # CCW tangential
# #             if DIRECTION_LOCAL < 0:
# #                 ut = (-ut[0], -ut[1])                # CW
# #         else:
# #             ur = (1.0, 0.0)
# #             if DIRECTION_LOCAL > 0:
# #                 ut = (0.0, 1.0)
# #             else:
# #                 ut = (0.0, -1.0)

# #         # Base control toward ring position
# #         v_radial     = K_RADIAL  * radial_error
# #         v_tangential = K_ANGULAR * angular_error

# #         vx = v_radial * ur[0] + v_tangential * ut[0]
# #         vy = v_radial * ur[1] + v_tangential * ut[1]

# #         # ----------------- Neighbor info for spacing + separation -----------------
# #         msgs = robot.recv_msg()
# #         current_neighbors = {}
# #         if msgs is not None:
# #             if not isinstance(msgs, list):
# #                 msgs = [msgs]
# #             for m in msgs:
# #                 if not m:
# #                     continue
# #                 try:
# #                     px, py, pid = struct.unpack(POSE_FMT, m[:POSE_SIZE])
# #                     if int(pid) != my_id:
# #                         current_neighbors[int(pid)] = (float(px), float(py))
# #                 except:
# #                     pass

# #         collision_risk = False

# #         # ----- ANGULAR SPACING (same ring only) -----
# #         if len(current_neighbors) > 0 and my_ring_count > 0:
# #             neighbor_angles = []
# #             # tolerance for same ring
# #             RING_RADIUS_TOL = max(0.04, 0.6 * RING_SPACING)
# #             for nid in current_neighbors:
# #                 nx, ny = current_neighbors[nid]
# #                 ndx = nx - CX
# #                 ndy = ny - CY
# #                 n_r = math.hypot(ndx, ndy)
# #                 if abs(n_r - R_TARGET_LOCAL) <= RING_RADIUS_TOL:
# #                     n_angle = math.atan2(ndy, ndx)
# #                     neighbor_angles.append(n_angle)

# #             if len(neighbor_angles) > 0:
# #                 min_gap = float('inf')
# #                 for n_angle in neighbor_angles:
# #                     gap = abs(wrap_angle(n_angle - current_angle))
# #                     if gap < min_gap:
# #                         min_gap = gap

# #                 ideal_gap = 2.0 * math.pi / float(my_ring_count)

# #                 if min_gap < ideal_gap * 0.7:
# #                     v_tangential = v_tangential * 0.6
# #                 elif min_gap > ideal_gap * 1.3:
# #                     v_tangential = v_tangential * 1.4

# #                 vx = v_radial * ur[0] + v_tangential * ut[0]
# #                 vy = v_radial * ur[1] + v_tangential * ut[1]

# #                # ----- COLLISION AVOIDANCE (same ring only) -----
# #             if len(current_neighbors) > 0:
# #                 sep_x = 0.0
# #                 sep_y = 0.0

# #                 # use same-ring radius tolerance (fallback if spacing is tiny)
# #                 RING_RADIUS_TOL = max(0.04, 0.6 * RING_SPACING)

# #                 for nid in current_neighbors:
# #                     nx, ny = current_neighbors[nid]
# #                     ndx = nx - CX
# #                     ndy = ny - CY
# #                     n_r = math.hypot(ndx, ndy)

# #                     # only repel neighbors that are effectively on *my* ring
# #                     if abs(n_r - R_TARGET_LOCAL) > RING_RADIUS_TOL:
# #                         continue

# #                     ddx = x - nx
# #                     ddy = y - ny
# #                     d = math.hypot(ddx, ddy)
# #                     if d < MIN_SEP and d > 1e-6:
# #                         collision_risk = True
# #                         s = SEP_GAIN * (MIN_SEP - d) / MIN_SEP
# #                         sep_x += s * (ddx / d)
# #                         sep_y += s * (ddy / d)

# #                 vx += sep_x
# #                 vy += sep_y


# #         # Add boundary cushion
# #         bfx, bfy = soft_boundary_force(x, y)
# #         vx += bfx
# #         vy += bfy

# #         # Are we close enough to target?
# #         position_error = math.hypot(target_x - x, target_y - y)
# #         on_target = (position_error < 0.06) and (abs(radial_error) < 0.04)

# #         # LEDs
# #         if on_target:
# #             formation_achieved = True
# #             robot.set_led(0, 255, 0)  # green locked
# #             vx = 0.0
# #             vy = 0.0
# #         else:
# #             if collision_risk:
# #                 robot.set_led(255, 120, 0)  # orange
# #             elif abs(radial_error) > 0.1:
# #                 robot.set_led(255, 150, 0)  # orange-ish
# #             else:
# #                 robot.set_led(0, 150, 255)  # blue

# #         # Convert to wheel commands
# #         if abs(vx) + abs(vy) < EPS:
# #             robot.set_vel(0, 0)
# #         else:
# #             hdg = math.atan2(vy, vx)
# #             err = wrap_angle(hdg - th)

# #             abs_err = abs(err)
# #             if abs_err < 0.5:
# #                 fwd = FWD_FAST
# #             elif abs_err < 1.2:
# #                 fwd = FWD_FAST * 0.7
# #             else:
# #                 fwd = FWD_SLOW

# #             if collision_risk:
# #                 fwd = fwd * 0.7

# #             turn = clamp(TURN_K * err, -1.5, 1.5)

# #             left  = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
# #             right = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
# #             robot.set_vel(left, right)

# #         # Periodic status
# #         now = robot.get_clock()
# #         if now - last_logw_time > logw_PERIOD:
# #             logw("Robot %d: pos [%.3f, %.3f], err=%.3f, neighbors=%d, collision=%d"
# #                  % (my_id, x, y, position_error, len(current_neighbors),
# #                     1 if collision_risk else 0))
# #             last_logw_time = now

# #         robot.delay(40)

# def usr(robot):
#     global log, log_out 

#     # ---- open log exactly as lab wants ----
#     try:
#         log = open("experiment_log", "wb")
#     except:
#         log = None

#     my_id = get_id(robot)
#     robot.set_led(0, 180, 180)  # cyan normal
#     logw("Robot %d starting: Concentric Circle Formation (NUM_RINGS=%d)" % (my_id, NUM_RINGS))

#     # ----------------- PHASE 1: gossip to discover all robots -----------------
#     POSE_FMT  = "ffi"  # float x, float y, int id
#     POSE_SIZE = struct.calcsize(POSE_FMT)

#     poses = {}  # id -> (x, y)

#     t_start = robot.get_clock()
#     gossip_duration = 2.0  # seconds to share poses

#     while robot.get_clock() - t_start < gossip_duration:
#         pose = safe_pose(robot)
#         if pose is not None:
#             x, y, _ = pose
#             # broadcast my pose
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
#             # msgs can be a list or a single message depending on API
#             if not isinstance(msgs, list):
#                 msgs = [msgs]
#             for m in msgs:
#                 if not m:
#                     continue
#                 try:
#                     px, py, pid = struct.unpack(POSE_FMT, m[:POSE_SIZE])
#                     poses[int(pid)] = (float(px), float(py))
#                 except:
#                     pass

#         robot.delay(50)  # 50 ms as recommended between send/recv

#     # ----------------- PHASE 2: Assign target positions on rings -----------------
#     total_robots = len(poses)

#     # Precompute ring radii (inner to outer)
#     # Inner radius is stepped inwards from the outer R_TARGET_OUTER
#     # ring_radii[0] = inner; ring_radii[-1] = outer
#     ring_radii = []
#     r_idx = 0
#     n_rings_active = max(1, min(NUM_RINGS, total_robots))
#     while r_idx < n_rings_active:
#         # number 0 = innermost, number (n-1) = outermost
#         radius = R_TARGET_OUTER - (n_rings_active - 1 - r_idx) * RING_SPACING
#         ring_radii.append(radius)
#         r_idx += 1

#     ring_dirs = build_ring_directions(len(ring_radii))

#     if total_robots == 0:
#         logw("Robot %d: No robots discovered, using default position" % my_id)
#         target_angle = 0.0
#         my_ring_id = 0
#         my_ring_count = 1
#         R_TARGET_LOCAL = R_TARGET_OUTER
#         DIRECTION_LOCAL = 1
#     else:
#         # Sort robot IDs for consistent ordering
#         sorted_ids = sorted(poses.keys())
#         # If my_id somehow not discovered, default index 0
#         try:
#             my_index = sorted_ids.index(my_id)
#         except ValueError:
#             my_index = 0

#         # Compute ring sizes + offsets
#         counts, offsets, num_rings_active = compute_ring_assignments(total_robots, NUM_RINGS)

#         # Clip ring_radii / ring_dirs to active ring count
#         ring_radii = ring_radii[:num_rings_active]
#         ring_dirs  = ring_dirs[:num_rings_active]

#         # ---------- NEW: LOG FULL RING ASSIGNMENT MAP ----------
#         logw("----- RING ASSIGNMENT MAP -----")
#         logw("Total robots discovered: %d, active rings: %d" % (total_robots, num_rings_active))
#         logw("Ring radii (inner->outer): " + ", ".join(["%.3f" % r for r in ring_radii]))
#         logw("Ring dirs  (inner->outer): " + ", ".join(["%+d" % d for d in ring_dirs]))

#         # For each robot in global order, figure out its ring, slot, and target
#         for g_idx, rid in enumerate(sorted_ids):
#             # find which ring this global index belongs to
#             ring_id = 0
#             ring_slot = 0
#             ring_count = total_robots
#             r = 0
#             while r < num_rings_active:
#                 start_idx = offsets[r]
#                 end_idx   = offsets[r] + counts[r]
#                 if g_idx >= start_idx and g_idx < end_idx:
#                     ring_id = r
#                     ring_slot = g_idx - start_idx
#                     ring_count = counts[r]
#                     break
#                 r += 1

#             if ring_count > 0:
#                 angle_spacing = 2.0 * math.pi / float(ring_count)
#             else:
#                 angle_spacing = 2.0 * math.pi

#             if ring_id < len(ring_radii):
#                 r_local = ring_radii[ring_id]
#             else:
#                 r_local = R_TARGET_OUTER

#             if ring_id < len(ring_dirs):
#                 d_local = ring_dirs[ring_id]
#             else:
#                 d_local = 1

#             targ_angle = ring_slot * angle_spacing
#             tx = CX + r_local * math.cos(targ_angle)
#             ty = CY + r_local * math.sin(targ_angle)
#             mark = " <== ME" if rid == my_id else ""

#             logw("slot %2d | id=%2d | ring=%d | idx_in_ring=%2d/%2d | "
#                  "angle=%.2f rad | radius=%.3f | dir=%+d | target=(%.3f, %.3f)%s"
#                  % (g_idx, rid, ring_id, ring_slot, ring_count,
#                     targ_angle, r_local, d_local, tx, ty, mark))
#         logw("----- END RING ASSIGNMENT MAP -----")

#         # ---------- ORIGINAL PER-ROBOT ASSIGNMENT (unchanged logic) ----------
#         # Figure out which ring I'm in and my index within that ring
#         my_ring_id = 0
#         my_ring_index_in_ring = 0
#         my_ring_count = total_robots  # fallback
#         r = 0
#         while r < num_rings_active:
#             start_idx = offsets[r]
#             end_idx   = offsets[r] + counts[r]  # exclusive
#             if my_index >= start_idx and my_index < end_idx:
#                 my_ring_id = r
#                 my_ring_index_in_ring = my_index - start_idx
#                 my_ring_count = counts[r]
#                 break
#             r += 1

#         # Angular spacing in *my ring only*
#         if my_ring_count > 0:
#             angle_spacing = 2.0 * math.pi / float(my_ring_count)
#         else:
#             angle_spacing = 2.0 * math.pi

#         target_angle = my_ring_index_in_ring * angle_spacing

#         # Ring-specific radius + direction
#         if my_ring_id < len(ring_radii):
#             R_TARGET_LOCAL = ring_radii[my_ring_id]
#         else:
#             R_TARGET_LOCAL = R_TARGET_OUTER

#         if my_ring_id < len(ring_dirs):
#             DIRECTION_LOCAL = ring_dirs[my_ring_id]
#         else:
#             DIRECTION_LOCAL = 1

#         logw("Robot %d: %d robots total, my_index=%d, ring_id=%d, ring_index=%d, ring_count=%d"
#              % (my_id, total_robots, my_index, my_ring_id, my_ring_index_in_ring, my_ring_count))
#         logw("Robot %d: ring_radius=%.3f, target_angle=%.3f, direction=%d"
#              % (my_id, R_TARGET_LOCAL, target_angle, DIRECTION_LOCAL))

#     # Calculate target position on my ring
#     target_x = CX + R_TARGET_LOCAL * math.cos(target_angle)
#     target_y = CY + R_TARGET_LOCAL * math.sin(target_angle)

#     logw("Robot %d target: (%.3f, %.3f)" % (my_id, target_x, target_y))

#     # ----------------- PHASE 3: Move to target positions -----------------
#     last_logw_time = 0.0
#     logw_PERIOD = 2.0

#     formation_achieved = False

#     while True:
#         pose = safe_pose(robot)
#         if pose is None:
#             robot.set_vel(0,0)
#             robot.delay(20)
#             continue

#         x, y, th = pose

#         # Safety check (always active)
#         if is_critical_obstacle(x, y):
#             robot.set_led(255, 0, 0)  # red for critical
#             robot.set_vel(0, 0)
#             robot.delay(40)
#             continue

#         # If formation is achieved, stop and skip control
#         if formation_achieved:
#             robot.set_vel(0, 0)
#             robot.set_led(0, 255, 0)  # green
#             robot.delay(40)
#             continue

#         # Calculate current position relative to center
#         dx = x - CX
#         dy = y - CY
#         current_r = math.hypot(dx, dy)
#         current_angle = math.atan2(dy, dx)

#         # Calculate errors
#         radial_error  = R_TARGET_LOCAL - current_r
#         angular_error = wrap_angle(target_angle - current_angle)

#         # Unit vectors for radial and tangential directions
#         if current_r > 1e-6:
#             ur = (dx / current_r, dy / current_r)    # radial unit vector (outward)
#             ut = (-ur[1], ur[0])                     # tangential unit vector (CCW)
#             if DIRECTION_LOCAL < 0:
#                 ut = (-ut[0], -ut[1])                # tangential unit vector (CW)
#         else:
#             ur = (1.0, 0.0)
#             if DIRECTION_LOCAL > 0:
#                 ut = (0.0, 1.0)
#             else:
#                 ut = (0.0, -1.0)

#         # Base control: move toward target position
#         v_radial     = K_RADIAL  * radial_error
#         v_tangential = K_ANGULAR * angular_error

#         vx = v_radial * ur[0] + v_tangential * ut[0]
#         vy = v_radial * ur[1] + v_tangential * ut[1]

#         # ----------------- PHASE 4: Neighbor info for spacing + separation -----------------
#         msgs = robot.recv_msg()
#         current_neighbors = {}
#         if msgs is not None:
#             if not isinstance(msgs, list):
#                 msgs = [msgs]
#             for m in msgs:
#                 if not m:
#                     continue
#                 try:
#                     px, py, pid = struct.unpack(POSE_FMT, m[:POSE_SIZE])
#                     if int(pid) != my_id:
#                         current_neighbors[int(pid)] = (float(px), float(py))
#                 except:
#                     pass

#         # ----- ANGULAR SPACING (only with neighbors on *my ring*) -----
#         collision_risk = False
#         if len(current_neighbors) > 0 and my_ring_count > 0:
#             neighbor_angles = []
#             # we only treat robots at similar radius as "same ring" for spacing
#             RING_RADIUS_TOL = RING_SPACING * 0.6 if RING_SPACING > 0.0 else 0.12
#             for nid in current_neighbors:
#                 nx, ny = current_neighbors[nid]
#                 ndx = nx - CX
#                 ndy = ny - CY
#                 n_r = math.hypot(ndx, ndy)
#                 # same-ring filter
#                 if abs(n_r - R_TARGET_LOCAL) <= RING_RADIUS_TOL:
#                     n_angle = math.atan2(ndy, ndx)
#                     neighbor_angles.append(n_angle)

#             if len(neighbor_angles) > 0:
#                 # Find closest angular neighbor
#                 min_gap = float('inf')
#                 for n_angle in neighbor_angles:
#                     gap = abs(wrap_angle(n_angle - current_angle))
#                     if gap < min_gap:
#                         min_gap = gap

#                 # Ideal gap is 2*pi / my_ring_count
#                 ideal_gap = 2.0 * math.pi / float(my_ring_count)

#                 # Adjust tangential velocity based on spacing
#                 if min_gap < ideal_gap * 0.7:
#                     # Too close to neighbor, slow down tangentially
#                     v_tangential = v_tangential * 0.6
#                 elif min_gap > ideal_gap * 1.3:
#                     # Too far from neighbor, speed up tangentially
#                     v_tangential = v_tangential * 1.4

#                 # Recalculate velocity with spacing adjustment
#                 vx = v_radial * ur[0] + v_tangential * ut[0]
#                 vy = v_radial * ur[1] + v_tangential * ut[1]

#         # ----- COLLISION AVOIDANCE (all neighbors, across rings) -----
#         if len(current_neighbors) > 0:
#             sep_x = 0.0
#             sep_y = 0.0
#             for nid in current_neighbors:
#                 nx, ny = current_neighbors[nid]
#                 ddx = x - nx
#                 ddy = y - ny
#                 d = math.hypot(ddx, ddy)
#                 if d < MIN_SEP and d > 1e-6:
#                     collision_risk = True
#                     # strength grows as robots get closer
#                     s = SEP_GAIN * (MIN_SEP - d) / MIN_SEP
#                     sep_x += s * (ddx / d)
#                     sep_y += s * (ddy / d)
#             vx += sep_x
#             vy += sep_y

#         # Add boundary cushion
#         bfx, bfy = soft_boundary_force(x, y)
#         vx += bfx
#         vy += bfy

#         # Check if we're close to target
#         position_error = math.hypot(target_x - x, target_y - y)
#         on_target = (position_error < 0.05) and (abs(radial_error) < 0.03)

#         # LED feedback
#         if on_target:
#             formation_achieved = True
#             robot.set_led(0, 255, 0)  # green when on target
#             # Stop moving by setting velocity to zero
#             vx = 0.0
#             vy = 0.0
#         else:
#             if collision_risk:
#                 robot.set_led(255, 120, 0)  # orange when actively separating
#             elif abs(radial_error) > 0.1:
#                 robot.set_led(255, 150, 0)  # orange - moving radially
#             else:
#                 robot.set_led(0, 150, 255)  # blue - adjusting angle

#         # Convert to wheel commands
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

#             # If we are in collision avoidance, be a bit more cautious
#             if collision_risk:
#                 fwd = fwd * 0.7

#             turn = clamp(TURN_K * err, -1.5, 1.5)

#             left  = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
#             right = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
#             robot.set_vel(left, right)

#         # Periodic status
#         now = robot.get_clock()
#         if now - last_logw_time > logw_PERIOD:
#             logw("Robot %d: pos [%.3f, %.3f], err=%.3f, neighbors=%d, collision=%d"
#                  % (my_id, x, y, position_error, len(current_neighbors),
#                     1 if collision_risk else 0))
#             last_logw_time = now

#         robot.delay(40)


# -*- coding: utf-8 -*-
# Concentric Circle Formation with Centralized Collision-Free Movement
# Python 2.7 + Coachbot messaging + lab logging pattern

import math, struct

# ----------------- USER INPUT: RINGS -----------------
NUM_RINGS = 2   # <-- change to 1, 2, 3, ... as needed

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

    # Optional: also print to console (sim)
    try:
        print(s.rstrip("\n"))
    except:
        pass

# ----------------- Testbed bounds & center -----------------
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35
CX, CY = (-0.1, 0.475)   # dancer center / nominal center

# ----------------- Circle parameters -----------------
R_TARGET_OUTER = 0.52
RING_SPACING   = 0.3   # radial spacing between rings (m)

# ----------------- Drive control -----------------
MAX_WHEEL = 35
TURN_K    = 3.0
FWD_FAST  = 0.75
FWD_SLOW  = 0.25
EPS       = 1e-3

# ----------------- Control gains -----------------
K_RADIAL    = 1.5
K_ANGULAR   = 1.4

# ----------------- Collision avoidance -----------------
MIN_SEP     = 0.25  # Increased for safety
SEP_GAIN    = 1.2   # Increased for stronger avoidance

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
    if hasattr(robot, "id"):
        try:
            return int(robot.id)
        except:
            pass
    return -1

def soft_boundary_force(x, y, max_force=0.4, boundary_margin=0.12):  # Increased margin
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

def compute_ring_assignments(total_robots, num_rings):
    if num_rings < 1:
        num_rings = 1
    if num_rings > total_robots:
        num_rings = total_robots

    base = total_robots // num_rings
    extra = total_robots % num_rings

    counts = []
    offsets = []
    offset = 0
    r = 0
    while r < num_rings:
        n = base
        if r < extra:
            n += 1
        counts.append(n)
        offsets.append(offset)
        offset += n
        r += 1
    return counts, offsets, num_rings

def build_ring_directions(num_rings):
    dirs = []
    for r in range(num_rings):
        if r % 2 == 0:
            dirs.append(1)   # inner, ring 0: CCW
        else:
            dirs.append(-1)  # ring 1: CW
    return dirs

def compute_target_positions(poses, num_rings, center_x, center_y, outer_radius, ring_spacing):
    """Compute collision-free target positions for all robots"""
    total_robots = len(poses)
    if total_robots == 0:
        return {}
    
    # Sort robot IDs for consistent ordering
    sorted_ids = sorted(poses.keys())
    
    # Compute ring assignments
    counts, offsets, num_rings_active = compute_ring_assignments(total_robots, num_rings)
    
    # Precompute ring radii (inner to outer)
    ring_radii = []
    for r_idx in range(num_rings_active):
        radius = outer_radius - (num_rings_active - 1 - r_idx) * ring_spacing
        ring_radii.append(radius)
    
    ring_dirs = build_ring_directions(num_rings_active)
    
    # Assign targets
    targets = {}
    for global_idx, robot_id in enumerate(sorted_ids):
        # Find which ring this robot belongs to
        ring_id = 0
        ring_index = 0
        r = 0
        while r < num_rings_active:
            start_idx = offsets[r]
            end_idx = offsets[r] + counts[r]
            if global_idx >= start_idx and global_idx < end_idx:
                ring_id = r
                ring_index = global_idx - start_idx
                break
            r += 1
        
        # Calculate target position
        if counts[ring_id] > 0:
            angle_spacing = 2.0 * math.pi / counts[ring_id]
        else:
            angle_spacing = 2.0 * math.pi
            
        target_angle = ring_index * angle_spacing
        
        # Apply ring direction for final orientation
        if ring_id < len(ring_dirs):
            final_orientation = target_angle + (math.pi/2 if ring_dirs[ring_id] > 0 else -math.pi/2)
        else:
            final_orientation = target_angle + math.pi/2
            
        # Wrap orientation to [-pi, pi]
        final_orientation = wrap_angle(final_orientation)
        
        radius = ring_radii[ring_id] if ring_id < len(ring_radii) else outer_radius
        target_x = center_x + radius * math.cos(target_angle)
        target_y = center_y + radius * math.sin(target_angle)
        
        targets[robot_id] = (target_x, target_y, final_orientation)
    
    return targets

def usr(robot):
    global log, log_out 

    # ---- open log exactly as lab wants ----
    try:
        log = open("experiment_log", "wb")
    except:
        log = None

    my_id = get_id(robot)
    robot.set_led(0, 180, 180)  # cyan normal
    logw("Robot %d starting: Improved Concentric Circle Formation" % my_id)

    # ----------------- PHASE 1: Discover all robots and compute targets -----------------
    POSE_FMT  = "ffi"  # float x, float y, int id
    POSE_SIZE = struct.calcsize(POSE_FMT)

    poses = {}  # id -> (x, y)
    target_positions = {}  # id -> (target_x, target_y, target_theta)

    t_start = robot.get_clock()
    gossip_duration = 3.0  # Increased for better discovery

    # Extended gossip phase to ensure all robots know each other
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

        robot.delay(50)

    # Compute target positions for all robots
    target_positions = compute_target_positions(poses, NUM_RINGS, CX, CY, R_TARGET_OUTER, RING_SPACING)
    
    if my_id in target_positions:
        my_target_x, my_target_y, my_target_theta = target_positions[my_id]
        logw("Robot %d: Assigned target (%.3f, %.3f, %.3f)" % (my_id, my_target_x, my_target_y, my_target_theta))
    else:
        logw("Robot %d: No target assigned, using default" % my_id)
        my_target_x, my_target_y, my_target_theta = CX + R_TARGET_OUTER, CY, 0.0

    # Broadcast target assignment for coordination
    TARGET_FMT = "fffi"  # x, y, theta, id
    try:
        target_msg = struct.pack(TARGET_FMT, my_target_x, my_target_y, my_target_theta, my_id)
        robot.send_msg(target_msg)
    except:
        pass

    # ----------------- PHASE 2: Collision-Aware Movement -----------------
    last_logw_time = 0.0
    logw_PERIOD = 2.0
    formation_achieved = False
    collision_count = 0
    MAX_COLLISIONS = 5  # If too many collisions, try different approach

    while True:
        pose = safe_pose(robot)
        if pose is None:
            robot.set_vel(0,0)
            robot.delay(20)
            continue

        x, y, th = pose

        # If formation is achieved, maintain position with gentle corrections
        if formation_achieved:
            position_error = math.hypot(my_target_x - x, my_target_y - y)
            if position_error > 0.08:  # If drifted too far, correct
                formation_achieved = False
            else:
                # Gentle orientation correction while maintaining position
                orientation_error = wrap_angle(my_target_theta - th)
                if abs(orientation_error) > 0.2:
                    turn = clamp(TURN_K * 0.3 * orientation_error, -1.0, 1.0)
                    left = clamp(int(MAX_WHEEL * 0.2 * (-0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
                    right = clamp(int(MAX_WHEEL * 0.2 * (0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
                    robot.set_vel(left, right)
                else:
                    robot.set_vel(0, 0)
                robot.set_led(0, 255, 0)  # green
                robot.delay(40)
                continue

        # Calculate errors
        dx = my_target_x - x
        dy = my_target_y - y
        position_error = math.hypot(dx, dy)
        orientation_error = wrap_angle(my_target_theta - th)

        # Check if we've reached the target
        if position_error < 0.06 and abs(orientation_error) < 0.3:
            formation_achieved = True
            robot.set_led(0, 255, 0)  # green
            robot.set_vel(0, 0)
            robot.delay(40)
            continue

        # Base velocity towards target
        if position_error > 1e-6:
            direction = math.atan2(dy, dx)
            heading_error = wrap_angle(direction - th)
        else:
            heading_error = orientation_error

        # Adaptive speed based on heading alignment and distance
        abs_heading_error = abs(heading_error)
        if abs_heading_error < 0.3:
            fwd = FWD_FAST
        elif abs_heading_error < 1.0:
            fwd = FWD_FAST * 0.5
        else:
            fwd = FWD_SLOW

        # Reduce speed when close to target to prevent overshoot
        if position_error < 0.2:
            fwd = fwd * (position_error / 0.2)

        turn = clamp(TURN_K * heading_error, -1.5, 1.5)

        # ----------------- Enhanced Collision Avoidance -----------------
        msgs = robot.recv_msg()
        current_neighbors = {}
        if msgs is not None:
            if not isinstance(msgs, list):
                msgs = [msgs]
            for m in msgs:
                if not m:
                    continue
                # Try to parse both pose and target messages
                try:
                    # Pose message
                    if len(m) >= POSE_SIZE:
                        px, py, pid = struct.unpack(POSE_FMT, m[:POSE_SIZE])
                        current_neighbors[int(pid)] = ('pose', float(px), float(py))
                except:
                    pass
                try:
                    # Target message  
                    if len(m) >= struct.calcsize(TARGET_FMT):
                        tx, ty, tt, tid = struct.unpack(TARGET_FMT, m)
                        current_neighbors[int(tid)] = ('target', float(tx), float(ty))
                except:
                    pass

        collision_risk = False
        avoidance_x, avoidance_y = 0.0, 0.0

        # Enhanced collision checking with both current and target positions
        for nid, data in current_neighbors.items():
            if nid == my_id:
                continue
                
            if data[0] == 'pose':
                nx, ny = data[1], data[2]
            else:  # 'target'
                nx, ny = data[1], data[2]
            
            # Calculate distance to neighbor
            dist = math.hypot(x - nx, y - ny)
            
            if dist < MIN_SEP:
                collision_risk = True
                collision_count += 1
                
                # Strong repulsive force
                if dist > 1e-6:
                    repulse_strength = SEP_GAIN * (MIN_SEP - dist) / MIN_SEP
                    avoidance_x += repulse_strength * (x - nx) / dist
                    avoidance_y += repulse_strength * (y - ny) / dist
                
                # If too many collisions, try a different approach
                if collision_count > MAX_COLLISIONS:
                    # Move perpendicular to the line to target to avoid deadlock
                    perp_dx = -dy
                    perp_dy = dx
                    perp_len = math.hypot(perp_dx, perp_dy)
                    if perp_len > 1e-6:
                        avoidance_x += 0.8 * perp_dx / perp_len
                        avoidance_y += 0.8 * perp_dy / perp_len

        # Apply avoidance forces
        if collision_risk:
            # Reduce forward speed and apply avoidance
            fwd = fwd * 0.5
            turn = turn * 0.7
            
            # Blend avoidance with target direction
            blend_factor = 0.7  # How much to prioritize avoidance
            total_dx = (1 - blend_factor) * dx + blend_factor * avoidance_x
            total_dy = (1 - blend_factor) * dy + blend_factor * avoidance_y
            
            # Recalculate direction with avoidance
            if math.hypot(total_dx, total_dy) > 1e-6:
                direction = math.atan2(total_dy, total_dx)
                heading_error = wrap_angle(direction - th)

        # Add boundary forces
        bfx, bfy = soft_boundary_force(x, y)
        total_dx = dx + bfx + (avoidance_x if collision_risk else 0)
        total_dy = dy + bfy + (avoidance_y if collision_risk else 0)
        
        if math.hypot(total_dx, total_dy) > 1e-6:
            direction = math.atan2(total_dy, total_dx)
            heading_error = wrap_angle(direction - th)

        # LED feedback
        if collision_risk:
            robot.set_led(255, 100, 0)  # orange for collision avoidance
        elif position_error > 0.15:
            robot.set_led(255, 150, 0)  # yellow/orange for moving
        else:
            robot.set_led(0, 150, 255)  # blue for fine adjustment

        # Final wheel commands
        left = clamp(int(MAX_WHEEL * (fwd - 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
        right = clamp(int(MAX_WHEEL * (fwd + 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
        robot.set_vel(left, right)

        # Periodic logging
        now = robot.get_clock()
        if now - last_logw_time > logw_PERIOD:
            logw("Robot %d: pos [%.3f, %.3f], target [%.3f, %.3f], error=%.3f, collision=%d" 
                 % (my_id, x, y, my_target_x, my_target_y, position_error, 1 if collision_risk else 0))
            last_logw_time = now

        robot.delay(40)