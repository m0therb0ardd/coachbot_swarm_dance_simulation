# # -*- coding: utf-8 -*-
# # Performance Sequencer (SIM, v4)
# # Phases: translate-left → translate-right (orient→shift) → glitch → encircle → float → punch
# # - Uses a GLOBAL obstacle-safe shift so the whole ring translates rigidly.
# # - move_right: all bots first orient to +x, then sync-shift together.

# import math, struct, random

# # ----------------- Testbed bounds & dancer -----------------
# X_MIN, X_MAX = -1.2, 1.0
# Y_MIN, Y_MAX = -1.4, 2.35
# CX, CY = (-0.1, 0.475)

# FEET = 0.3048
# OBST_DIAM_FT = 1.0
# OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET   # ~0.1524 m
# OBST_MARGIN  = 0.03
# SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
# OBST_CX, OBST_CY = CX, CY

# # ----------------- Drive / loop (global defaults) -----------------
# MAX_WHEEL   = 40
# TURN_K      = 3.0
# CMD_SMOOTH  = 0.20
# DT_MS       = 40
# FWD_MIN     = 0.38
# FWD_SLOW    = 0.55
# FWD_BASE    = 0.75
# PRINT_PERIOD= 2.0
# EPS         = 1e-3

# # ----------------- Boundary UI parity -----------------
# WARN_MARGIN_BOUNDARY = 0.05
# CRIT_MARGIN_BOUNDARY = 0.02

# # ----------------- Soft obstacle buffer (used in every phase) -----------------
# OBST_WARN_BUFFER   = 0.10
# OBST_MAX_SOFT_FORCE= 0.55

# # ----------------- Translate (left/right) gains -----------------
# SHIFT_RATE  = 0.18
# STOP_MARGIN = 0.08
# KX, KY, KR  = 1.2, 2.0, 2.6

# # --- orientation params for move_right ---
# RIGHT_HEADING = 0.0          # +x direction
# ORIENT_TOL    = 0.18         # rad (~10°) target tolerance
# ORIENT_HOLD   = 0.25         # s: stay within tol before we launch
# ORIENT_FWD    = 0.00         # forward factor during orient (0 = turn-in-place)

# # ----------------- Glitch -----------------
# JITTER   = 0.45
# NEAR_SEP = 0.20

# # ----------------- Encircle (single ring) -----------------
# R_RING       = SAFE_BUBBLE + 0.34
# DIR          = +1                    # +1 CCW, -1 CW
# SAME_RING_TOL= 0.12
# V_TANGENT    = 0.26
# K_R_HOLD     = 1.3
# RADIAL_CLMP  = 0.12
# ANG_GAIN     = 0.30
# ANG_POW      = 1.2
# ANG_CUTOFF   = 1.4
# LIN_MIN_SEP  = 0.18

# # P2P heartbeats for encircle
# HB_FMT   = 'fffI'            # x,y,theta,id
# HB_BYTES = struct.calcsize(HB_FMT)
# HB_DT, STALE_S = 0.12, 0.8

# # ----------------- FLOAT (sim-friendly) -----------------
# FLOAT_MAX_WHEEL = 35
# FLOAT_TURN_K    = 2.2
# FLOAT_FWD_BASE  = 0.65
# FLOAT_FWD_MIN   = 0.35
# FLOAT_CMD_SMOOTH= 0.35
# FLOAT_VEL_SLEW  = 8

# K_MIG_FLOAT   = 0.08
# K_SEP_FLOAT   = 0.20
# K_ALI_FLOAT   = 0.18
# K_COH_FLOAT   = 0.10
# CURVE_NOISE   = 0.20
# SEP_RADIUS    = 0.26
# NEIGH_RADIUS  = 0.75

# SOFT_MARGIN = 0.08
# CRIT_MARGIN = 0.02
# SOFT_MAX_F  = 0.35

# # ----------------- Punch -----------------
# WAVE_PERIOD = 3.0
# AIM_TIME    = 0.18
# CHARGE_TIME = 0.55
# HOLD_TIME   = 0.10
# TAP_TIME    = 0.08
# RECOIL_TIME = 0.30
# FWD_AIM     = 0.40
# FWD_CHARGE  = 1.00
# FWD_TAP     = 0.70
# FWD_RECOIL  = 0.55
# PANIC_R     = 0.18
# SEP_R_PUNCH = 0.28
# K_SEP_PUNCH = 0.48
# K_ALI_PUNCH = 0.10

# # ----------------- Phase schedule (seconds) -----------------
# PHASES = [
#     ("move_left",  20.0),
#     ("move_right", 20.0),
#     ("glitch",     20.0),
#     ("encircle",   35.0),
#     ("float",      35.0),
#     ("punch",      35.0),
# ]

# # ---------- helpers ----------
# def clamp(v, lo, hi): return lo if v < lo else (hi if v > hi else v)
# def wrap(a):
#     while a >  math.pi: a -= 2.0*math.pi
#     while a <= -math.pi: a += 2.0*math.pi
#     return a

# def soft_boundary_force(x, y, max_force=SOFT_MAX_F, boundary_margin=SOFT_MARGIN):
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

# def try_get_swarm_poses(robot):
#     for nm in ('get_swarm_poses','get_all_poses','get_poses','swarm_poses'):
#         fn = getattr(robot, nm, None)
#         if callable(fn):
#             try:
#                 poses = fn()
#                 if poses: return poses
#             except: pass
#     return []

# def get_id(robot):
#     try: return int(robot.id())
#     except: return 0

# # ---- global obstacle-safe shift limits (keep ring rigid) ----
# def global_s_limit_right(snapshot):
#     """
#     Largest s≥0 so that shifting center by +s (to +x) keeps all robots ≥ SAFE_BUBBLE from dancer.
#     Uses closed-form roots of (rx + s)^2 + ry^2 = SAFE_BUBBLE^2 for each robot.
#     """
#     if not snapshot: return float('inf')
#     s_lim = float('inf')
#     for (x0, y0, _th0) in snapshot:
#         rx, ry = x0 - CX, y0 - CY
#         if abs(ry) >= SAFE_BUBBLE:  # no crossing of bubble for any s
#             continue
#         delta = SAFE_BUBBLE*SAFE_BUBBLE - ry*ry
#         if delta <= 0:  # tangential or outside
#             continue
#         r = math.sqrt(delta)
#         lo = -rx - r
#         hi = -rx + r
#         if hi > 0:  # forbidden band intersects positive s
#             if lo <= 0:
#                 s_lim = min(s_lim, 0.0)
#             else:
#                 s_lim = min(s_lim, lo)
#     return max(0.0, s_lim if s_lim != float('inf') else float('inf'))

# def global_s_limit_left(snapshot):
#     """
#     Largest s≥0 so that shifting center by -s (to -x) keeps all robots ≥ SAFE_BUBBLE from dancer.
#     Uses roots of (rx - s)^2 + ry^2 = SAFE_BUBBLE^2.
#     """
#     if not snapshot: return float('inf')
#     s_lim = float('inf')
#     for (x0, y0, _th0) in snapshot:
#         rx, ry = x0 - CX, y0 - CY
#         if abs(ry) >= SAFE_BUBBLE:
#             continue
#         delta = SAFE_BUBBLE*SAFE_BUBBLE - ry*ry
#         if delta <= 0:
#             continue
#         r = math.sqrt(delta)
#         lo =  rx - r
#         hi =  rx + r
#         if hi > 0:
#             if lo <= 0:
#                 s_lim = min(s_lim, 0.0)
#             else:
#                 s_lim = min(s_lim, lo)
#     return max(0.0, s_lim if s_lim != float('inf') else float('inf'))

# # ---------- main ----------
# def usr(robot):
#     robot.delay(800)
#     rid = get_id(robot)
#     random.seed(rid*1103515245 & 0xFFFFFFFF)

#     # global timebase
#     t_start = robot.get_clock()
#     last_print = 0.0

#     # --- per-phase state ---
#     phase_name_prev = None

#     # translate state (re-init on phase entry)
#     rel_off = None
#     R_form  = None
#     s_stop  = None
#     t0_sync = None
#     started = False
#     swarm_snapshot = None  # filled on translate phase entry

#     # move_right orientation sub-state
#     trans_stage = None        # None | "orient" | "shift"
#     orient_ok_since = None

#     # encircle state
#     p2p_ever_seen = False
#     nbrs = {}                     # id -> (x,y,theta,last_t)
#     last_hb = -1e9

#     # float state
#     drift_phase = random.uniform(-math.pi, math.pi)
#     float_lastL = float_lastR = 0

#     # punch state
#     punch_state = "reset"
#     aim_until = charge_until = hold_until = tap_until = recoil_until = 0.0

#     # drive smoothing for non-float phases
#     lastL = lastR = 0

#     # wake localization a touch
#     robot.set_vel(20,20); robot.delay(150)

#     while True:
#         pose = robot.get_pose()
#         if not pose or len(pose)<3:
#             robot.set_vel(0,0); robot.delay(DT_MS); continue
#         x, y, th = float(pose[0]), float(pose[1]), float(pose[2])
#         now = robot.get_clock()
#         t = now - t_start

#         # choose phase
#         acc = 0.0
#         phase_name = PHASES[-1][0]
#         for name, dur in PHASES:
#             if t < acc + dur:
#                 phase_name = name; break
#             acc += dur

#         # phase change reset
#         if phase_name != phase_name_prev:
#             rel_off = R_form = s_stop = t0_sync = None
#             started = False
#             trans_stage = None
#             orient_ok_since = None
#             swarm_snapshot = None
#             if phase_name == "float":
#                 drift_phase = random.uniform(-math.pi, math.pi)
#                 float_lastL = float_lastR = 0
#             if phase_name == "punch":
#                 punch_state = "reset"
#             print(f"[seq] id={rid} → phase={phase_name}")
#             phase_name_prev = phase_name

#         # ----------------- hard safety (applies to all phases) -----------------
#         if is_critical_obstacle(x, y) or is_critical_boundary(x, y):
#             robot.set_led(100,0,0); robot.set_vel(0,0); robot.delay(DT_MS); continue

#         # shared soft fields
#         bfx, bfy = soft_boundary_force(x, y)
#         ofx, ofy = soft_obstacle_force(x, y)

#         # default outputs
#         vx = vy = 0.0
#         fwd = FWD_BASE
#         led = (0, 180, 180)

#         # -----------------=== MOVE LEFT (GLOBAL rigid-body shift) -----------------===
#         if phase_name == "move_left":
#             MOVE_DIR = -1

#             near_obst = obstacle_distance(x, y) < (SAFE_BUBBLE + OBST_WARN_BUFFER)
#             near_bound= (x < X_MIN + WARN_MARGIN_BOUNDARY or x > X_MAX - WARN_MARGIN_BOUNDARY or
#                          y < Y_MIN + WARN_MARGIN_BOUNDARY or y > Y_MAX - WARN_MARGIN_BOUNDARY)
#             led = (255,150,0) if (near_obst or near_bound) else (0,180,180)

#             if rel_off is None:
#                 rel_off = (x - CX, y - CY)
#                 R_form  = math.hypot(*rel_off)

#                 # snapshot the swarm once (shared s_stop)
#                 swarm_snapshot = []
#                 poses = try_get_swarm_poses(robot) or []
#                 if poses:
#                     for it in poses:
#                         if isinstance(it,(list,tuple)) and len(it)>=3:
#                             if len(it)>=4: _nid, px, py, pth = it
#                             else: px, py, pth = it[0], it[1], it[2]
#                             swarm_snapshot.append((float(px), float(py), float(pth)))
#                 if not swarm_snapshot:
#                     swarm_snapshot = [(x, y, th)]
#                 elif all(abs(px-x)>1e-9 or abs(py-y)>1e-9 for (px,py,_t) in swarm_snapshot):
#                     swarm_snapshot.append((x, y, th))

#                 # wall & obstacle global limits
#                 safety_buffer = 0.05
#                 s_wall = max(0.0, CX - (X_MIN + STOP_MARGIN + safety_buffer + R_form))
#                 s_obst_global = global_s_limit_left(swarm_snapshot)

#                 s_stop = min(s_wall, s_obst_global)

#                 # synchronized start
#                 now0 = robot.get_clock()
#                 t0_sync = math.floor(now0 * 2.0) / 2.0 + 0.5
#                 print(f"[translate-L] id={rid} lock R={R_form:.3f}, s_wall={s_wall:.3f}, s_obst={s_obst_global:.3f}, s_stop={s_stop:.3f}")

#             if not started:
#                 if now < t0_sync:
#                     robot.set_vel(0,0); robot.delay(10); continue
#                 started = True
#                 print(f"[translate-L] id={rid} START")

#             s = min(max(0.0, now - t0_sync) * SHIFT_RATE, s_stop)
#             Cx = CX + MOVE_DIR * s
#             Cy = CY
#             Cx = max(X_MIN + R_form + 0.08, min(X_MAX - R_form - 0.08, Cx))
#             Cy = max(Y_MIN + R_form + 0.08, min(Y_MAX - R_form - 0.08, Cy))

#             tx = Cx + rel_off[0]; ty = Cy + rel_off[1]
#             ex, ey = tx - x, ty - y

#             vx = KX * ex + MOVE_DIR * SHIFT_RATE + KR * ((rel_off[0] - (x - Cx)))
#             vy = KY * ey + KR * ((rel_off[1] - (y - Cy)))

#             vx += 0.3 * bfx + 0.25 * ofx
#             vy += 0.3 * bfy + 0.25 * ofy

#             if abs(s - s_stop) < 1e-6 and (abs(ex) + abs(ey) < 0.01):
#                 robot.set_vel(0,0); robot.set_led(0,255,0); robot.delay(20); continue

#             hdg = math.atan2(vy, vx); err = wrap(hdg - th)
#             ae = abs(err)
#             spd = FWD_BASE if ae < 0.5 else (FWD_BASE*0.75 if ae < 1.2 else FWD_SLOW)
#             spd = max(spd, FWD_MIN)
#             turn = clamp(TURN_K * err, -1.8, 1.8)
#             lcmd = clamp(int(MAX_WHEEL*0.9*(spd - 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
#             rcmd = clamp(int(MAX_WHEEL*0.9*(spd + 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
#             left  = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
#             right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
#             lastL, lastR = left, right
#             robot.set_vel(left, right)

#         # -----------------=== MOVE RIGHT (ORIENT → GLOBAL SHIFT) -----------------===
#         elif phase_name == "move_right":
#             MOVE_DIR = +1

#             near_obst = obstacle_distance(x, y) < (SAFE_BUBBLE + OBST_WARN_BUFFER)
#             near_bound= (x < X_MIN + WARN_MARGIN_BOUNDARY or x > X_MAX - WARN_MARGIN_BOUNDARY or
#                          y < Y_MIN + WARN_MARGIN_BOUNDARY or y > Y_MAX - WARN_MARGIN_BOUNDARY)
#             led = (255,150,0) if (near_obst or near_bound) else (0,180,180)

#             if rel_off is None:
#                 rel_off = (x - CX, y - CY)
#                 R_form  = math.hypot(*rel_off)

#                 # snapshot the swarm once (shared s_stop)
#                 swarm_snapshot = []
#                 poses = try_get_swarm_poses(robot) or []
#                 if poses:
#                     for it in poses:
#                         if isinstance(it,(list,tuple)) and len(it)>=3:
#                             if len(it)>=4: _nid, px, py, pth = it
#                             else: px, py, pth = it[0], it[1], it[2]
#                             swarm_snapshot.append((float(px), float(py), float(pth)))
#                 if not swarm_snapshot:
#                     swarm_snapshot = [(x, y, th)]
#                 elif all(abs(px-x)>1e-9 or abs(py-y)>1e-9 for (px,py,_t) in swarm_snapshot):
#                     swarm_snapshot.append((x, y, th))

#                 # wall & obstacle global limits
#                 safety_buffer = 0.05
#                 s_wall = max(0.0, (X_MAX - STOP_MARGIN - safety_buffer - R_form) - CX)
#                 s_obst_global = global_s_limit_right(swarm_snapshot)
#                 s_stop = min(s_wall, s_obst_global)

#                 # orientation stage first
#                 trans_stage = "orient"
#                 orient_ok_since = None
#                 print(f"[translate-R] id={rid} lock R={R_form:.3f}, s_wall={s_wall:.3f}, s_obst={s_obst_global:.3f}, s_stop={s_stop:.3f} → ORIENT")

#             # Stage 1: ORIENT
#             if trans_stage == "orient":
#                 target = RIGHT_HEADING
#                 err = wrap(target - th)
#                 ae  = abs(err)

#                 if ae <= ORIENT_TOL:
#                     if orient_ok_since is None:
#                         orient_ok_since = now
#                     if (now - orient_ok_since) >= ORIENT_HOLD:
#                         t0_sync = math.floor(now * 2.0) / 2.0 + 0.5
#                         started = False
#                         trans_stage = "shift"
#                         print(f"[translate-R] id={rid} ORIENT OK → SHIFT at t0={t0_sync:.2f}")
#                 else:
#                     orient_ok_since = None

#                 fwd = ORIENT_FWD
#                 turn = clamp(TURN_K * err, -1.8, 1.8)
#                 lcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.9 * turn)), -MAX_WHEEL, MAX_WHEEL)
#                 rcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.9 * turn)), -MAX_WHEEL, MAX_WHEEL)
#                 left  = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
#                 right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
#                 lastL, lastR = left, right
#                 robot.set_vel(left, right)
#                 robot.set_led(*[int(clamp(c,0,100)) for c in led])
#                 robot.delay(DT_MS)
#                 continue

#             # Stage 2: SHIFT
#             if trans_stage == "shift":
#                 if not started:
#                     if now < t0_sync:
#                         robot.set_vel(0,0); robot.delay(10); continue
#                     started = True
#                     print(f"[translate-R] id={rid} START SHIFT")

#                 s = min(max(0.0, now - t0_sync) * SHIFT_RATE, s_stop)
#                 Cx = CX + MOVE_DIR * s
#                 Cy = CY
#                 Cx = max(X_MIN + R_form + 0.08, min(X_MAX - R_form - 0.08, Cx))
#                 Cy = max(Y_MIN + R_form + 0.08, min(Y_MAX - R_form - 0.08, Cy))

#                 tx = Cx + rel_off[0]; ty = Cy + rel_off[1]
#                 ex, ey = tx - x, ty - y

#                 vx = KX * ex + MOVE_DIR * SHIFT_RATE + KR * ((rel_off[0] - (x - Cx)))
#                 vy = KY * ey + KR * ((rel_off[1] - (y - Cy)))

#                 vx += 0.3 * bfx + 0.25 * ofx
#                 vy += 0.3 * bfy + 0.25 * ofy

#                 if abs(s - s_stop) < 1e-6 and (abs(ex) + abs(ey) < 0.01):
#                     robot.set_vel(0,0); robot.set_led(0,255,0); robot.delay(20); continue

#                 hdg = math.atan2(vy, vx); err = wrap(hdg - th)
#                 ae = abs(err)
#                 spd = FWD_BASE if ae < 0.5 else (FWD_BASE*0.75 if ae < 1.2 else FWD_SLOW)
#                 spd = max(spd, FWD_MIN)
#                 turn = clamp(TURN_K * err, -1.8, 1.8)
#                 lcmd = clamp(int(MAX_WHEEL*0.9*(spd - 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
#                 rcmd = clamp(int(MAX_WHEEL*0.9*(spd + 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
#                 left  = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
#                 right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
#                 lastL, lastR = left, right
#                 robot.set_vel(left, right)

#         # -----------------=== GLITCH -----------------===
#         elif phase_name == "glitch":
#             led = (100,0,100)
#             vx = bfx + ofx + random.uniform(-JITTER, JITTER)
#             vy = bfy + ofy + random.uniform(-JITTER, JITTER)
#             for item in (try_get_swarm_poses(robot) or []):
#                 if isinstance(item,(list,tuple)) and len(item)>=3:
#                     if len(item)>=4: nid,nx,ny,_ = int(item[0]), float(item[1]), float(item[2]), item[3]
#                     else: nx,ny = float(item[0]), float(item[1])
#                     dx,dy = x-nx, y-ny; d2 = dx*dx + dy*dy
#                     if 1e-9 < d2 < NEAR_SEP*NEAR_SEP:
#                         s = (NEAR_SEP*NEAR_SEP - d2)/(NEAR_SEP*NEAR_SEP)
#                         dd = math.sqrt(d2); vx += s*(dx/dd); vy += s*(dy/dd)
#             hdg = math.atan2(vy, vx); err = wrap(hdg - th)
#             ae = abs(err)
#             spd = FWD_BASE if ae < 0.5 else (FWD_BASE*0.75 if ae < 1.2 else FWD_SLOW)
#             spd = max(spd, FWD_MIN)
#             turn = clamp(TURN_K * err, -1.8, 1.8)
#             lcmd = clamp(int(MAX_WHEEL*0.9*(spd - 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
#             rcmd = clamp(int(MAX_WHEEL*0.9*(spd + 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
#             left  = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
#             right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
#             lastL, lastR = left, right
#             robot.set_vel(left, right)

#         # -----------------=== ENCIRCLE -----------------===
#         elif phase_name == "encircle":
#             if (now - last_hb) >= HB_DT:
#                 try: robot.send_msg(struct.pack(HB_FMT, float(x), float(y), float(th), int(rid)))
#                 except: pass
#                 last_hb = now
#             msgs = robot.recv_msg() or []
#             for m in msgs:
#                 try:
#                     nx, ny, nth, nid = struct.unpack(HB_FMT, m[:HB_BYTES])
#                     if int(nid) != rid:
#                         nbrs[int(nid)] = (float(nx), float(ny), float(nth), now)
#                         p2p_ever_seen = True
#                 except: pass
#             if not p2p_ever_seen:
#                 sim_poses = try_get_swarm_poses(robot)
#                 if sim_poses:
#                     nbrs.clear()
#                     for it in sim_poses:
#                         if isinstance(it,(list,tuple)) and len(it)>=3:
#                             if len(it)>=4: nid, nx, ny, nth = int(it[0]), float(it[1]), float(it[2]), float(it[3])
#                             else: nx, ny, nth = float(it[0]), float(it[1]), float(it[2]); nid = 10_000 + int(1000*nx) ^ int(1000*ny)
#                             if nid == rid: continue
#                             nbrs[int(nid)] = (nx, ny, nth, now)
#             cutoff = now - STALE_S
#             for nid in list(nbrs.keys()):
#                 if nbrs[nid][3] < cutoff:
#                     nbrs.pop(nid, None)

#             rdx, rdy = x - CX, y - CY
#             r = math.hypot(rdx, rdy)
#             urx, ury = (1.0,0.0) if r<1e-6 else (rdx/r, rdy/r)
#             utx, uty = -ury, urx
#             if DIR < 0: utx, uty = -utx, -uty

#             vx = V_TANGENT*utx
#             vy = V_TANGENT*uty
#             radial = clamp(K_R_HOLD*(R_RING - r), -RADIAL_CLMP, RADIAL_CLMP)
#             vx += radial*urx; vy += radial*ury

#             theta = math.atan2(rdy, rdx)
#             for _, (nx, ny, nth, tlast) in nbrs.items():
#                 nr = math.hypot(nx - CX, ny - CY)
#                 if abs(nr - R_RING) <= SAME_RING_TOL:
#                     ddx, ddy = x - nx, y - ny
#                     d2 = ddx*ddx + ddy*ddy
#                     if d2 < LIN_MIN_SEP*LIN_MIN_SEP:
#                         s = (LIN_MIN_SEP*LIN_MIN_SEP - d2)/(LIN_MIN_SEP*LIN_MIN_SEP)
#                         vx += s*urx; vy += s*ury
#                     ntheta = math.atan2(ny - CY, nx - CX)
#                     dtheta = wrap(theta - ntheta)
#                     ad = abs(dtheta)
#                     if 1e-3 < ad <= ANG_CUTOFF:
#                         strength = ANG_GAIN / (ad ** ANG_POW)
#                         push = strength * (1.0 if dtheta > 0 else -1.0)
#                         vx += push * utx; vy += push * uty

#             bfx2, bfy2 = soft_boundary_force(x, y)
#             b_norm = bfx2*urx + bfy2*ury
#             vx += b_norm * urx; vy += b_norm * ury
#             vx += 0.25 * ofx; vy += 0.25 * ofy

#             led = (0,100,0) if DIR>0 else (0,0,100)

#             hdg = math.atan2(vy, vx)
#             err = wrap(hdg - th)
#             ae = abs(err)
#             spd = FWD_BASE if ae < 0.5 else (FWD_BASE*0.75 if ae < 1.2 else FWD_SLOW)
#             spd = max(spd, FWD_MIN)
#             turn = clamp(TURN_K * err, -1.8, 1.8)
#             lcmd = clamp(int(MAX_WHEEL*0.9*(spd - 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
#             rcmd = clamp(int(MAX_WHEEL*0.9*(spd + 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
#             left  = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
#             right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
#             lastL, lastR = left, right
#             robot.set_vel(left, right)

#         # -----------------=== FLOAT (low-jerk) -----------------===
#         elif phase_name == "float":
#             # boundary LEDs
#             bstat = 2 if is_critical_boundary(x,y,CRIT_MARGIN) else (1 if (
#                 x < X_MIN + SOFT_MARGIN or x > X_MAX - SOFT_MARGIN or
#                 y < Y_MIN + SOFT_MARGIN or y > Y_MAX - SOFT_MARGIN) else 0)
#             if bstat == 2:
#                 robot.set_led(100,0,0); robot.set_vel(0,0); robot.delay(DT_MS); continue
#             elif bstat == 1: robot.set_led(100,60,0)
#             else:            robot.set_led(0,70,80)

#             # env
#             ex, ey = soft_boundary_force(x, y)
#             ox, oy = soft_obstacle_force(x, y)
#             drift_phase += 0.03
#             curl = CURVE_NOISE*math.sin(drift_phase)
#             mx,my = -K_MIG_FLOAT, 0.0

#             # neighbors
#             repx=repy=cx=cy=ax=ay=0.0; n=0
#             for item in (try_get_swarm_poses(robot) or []):
#                 if isinstance(item,(list,tuple)) and len(item)>=3:
#                     if len(item)>=4: nid,nx,ny,nth = int(item[0]), float(item[1]), float(item[2]), float(item[3])
#                     else: nx,ny,nth = float(item[0]), float(item[1]), float(item[2])
#                     dx,dy = x-nx, y-ny
#                     d2 = dx*dx+dy*dy
#                     if d2>1e-9:
#                         d = math.sqrt(d2)
#                         if d < SEP_RADIUS:
#                             s = K_SEP_FLOAT*(SEP_RADIUS-d)/SEP_RADIUS
#                             repx += s*(dx/d); repy += s*(dy/d)
#                         if d <= NEIGH_RADIUS:
#                             cx += nx; cy += ny; ax += math.cos(nth); ay += math.sin(nth); n+=1
#             cohx=cohy=alx=aly=0.0
#             if n>0:
#                 cx/=n; cy/=n
#                 cohx = K_COH_FLOAT*(cx-x); cohy = K_COH_FLOAT*(cy-y)
#                 ah = math.atan2(ay,ax)
#                 alx = K_ALI_FLOAT*math.cos(ah); aly = K_ALI_FLOAT*math.sin(ah)

#             vx = ex + ox + mx + repx + cohx + alx
#             vy = ey + oy + my + repy + cohy + aly + curl
#             if abs(vx)<1e-6 and abs(vy)<1e-6: vx = 1e-3

#             hdg = math.atan2(vy, vx)
#             err = wrap(hdg - th)
#             fwd = FLOAT_FWD_BASE
#             if bstat == 1: fwd *= 0.8
#             fwd = max(FLOAT_FWD_MIN, fwd)

#             turn = clamp(FLOAT_TURN_K*err, -1.2, 1.2)
#             lcmd = clamp(int(FLOAT_MAX_WHEEL*0.9*(fwd - 0.75*turn)), -FLOAT_MAX_WHEEL, FLOAT_MAX_WHEEL)
#             rcmd = clamp(int(FLOAT_MAX_WHEEL*0.9*(fwd + 0.75*turn)), -FLOAT_MAX_WHEEL, FLOAT_MAX_WHEEL)

#             if lcmd > float_lastL + FLOAT_VEL_SLEW: lcmd = float_lastL + FLOAT_VEL_SLEW
#             if lcmd < float_lastL - FLOAT_VEL_SLEW: lcmd = float_lastL - FLOAT_VEL_SLEW
#             if rcmd > float_lastR + FLOAT_VEL_SLEW: rcmd = float_lastR + FLOAT_VEL_SLEW
#             if rcmd < float_lastR - FLOAT_VEL_SLEW: rcmd = float_lastR - FLOAT_VEL_SLEW

#             left  = int((1.0 - FLOAT_CMD_SMOOTH) * lcmd + FLOAT_CMD_SMOOTH * float_lastL)
#             right = int((1.0 - FLOAT_CMD_SMOOTH) * rcmd + FLOAT_CMD_SMOOTH * float_lastR)
#             float_lastL, float_lastR = left, right
#             robot.set_vel(left, right)

#         # -----------------=== PUNCH -----------------===
#         else:
#             led = (100,0,70) if punch_state=="charge" else (100,100,0)
#             robot.set_led(*[int(clamp(c,0,100)) for c in led])

#             if punch_state == "reset":
#                 punch_state = "aim"; aim_until = now + AIM_TIME

#             repx=repy=ax=ay=0.0; n=0; nearest=1e9; nn=None
#             for item in (try_get_swarm_poses(robot) or []):
#                 if isinstance(item,(list,tuple)) and len(item)>=3:
#                     if len(item)>=4: nid,nx,ny,nth = int(item[0]), float(item[1]), float(item[2]), float(item[3])
#                     else: nx,ny,nth = float(item[0]), float(item[1]), float(item[2])
#                     dx,dy = x-nx, y-ny; d = math.hypot(dx,dy)
#                     if d < nearest: nearest, nn = d, (nx,ny)
#                     if d>1e-9 and d<SEP_R_PUNCH:
#                         s = K_SEP_PUNCH*(SEP_R_PUNCH-d)/SEP_R_PUNCH
#                         repx += s*(dx/d); repy += s*(dy/d)
#                     if d<=0.75:
#                         ax += math.cos(nth); ay += math.sin(nth); n+=1
#             alx=aly=0.0
#             if n>0:
#                 ah = math.atan2(ay,ax)
#                 alx = K_ALI_PUNCH*math.cos(ah); aly = K_ALI_PUNCH*math.sin(ah)

#             evade_h = None
#             if nearest < PANIC_R and nn:
#                 nx,ny = nn
#                 evade_h = math.atan2(y-ny, x-nx)

#             if punch_state == "aim":
#                 fwd = FWD_AIM
#                 if now >= aim_until:
#                     punch_state = "charge"; charge_until = now + CHARGE_TIME
#                 vx = vy = 0.0

#             elif punch_state == "charge":
#                 if now >= charge_until:
#                     punch_state = "hold"; hold_until = now + HOLD_TIME
#                     robot.set_vel(0,0); lastL=lastR=0; robot.delay(DT_MS); continue
#                 vx = 1.0 + repx + 0.4*bfx + 0.4*ofx + 0.2*alx
#                 vy = 0.0 + repy + 0.4*bfy + 0.4*ofy + 0.2*aly
#                 if evade_h is not None:
#                     vx += 0.2*math.cos(evade_h); vy += 0.2*math.sin(evade_h)
#                 fwd = FWD_CHARGE

#             elif punch_state == "hold":
#                 if now >= hold_until:
#                     punch_state = "tap"; tap_until = now + TAP_TIME
#                 robot.set_vel(0,0); lastL=lastR=0; robot.delay(DT_MS); continue

#             elif punch_state == "tap":
#                 if now >= tap_until:
#                     punch_state = "recoil"; recoil_until = now + RECOIL_TIME
#                 vx = 1.0 + 0.3*bfx + 0.3*ofx; vy = 0.0 + 0.3*bfy + 0.3*ofy
#                 fwd = FWD_TAP

#             elif punch_state == "recoil":
#                 if now >= recoil_until:
#                     punch_state = "aim"; aim_until = now + AIM_TIME
#                 rc = math.pi - 0.35
#                 vx = math.cos(rc) + bfx + ofx; vy = math.sin(rc) + bfy + ofy
#                 fwd = FWD_RECOIL

#             hdg = math.atan2(vy, vx) if (abs(vx)>1e-6 or abs(vy)>1e-6) else th
#             err = wrap(hdg - th)
#             ae = abs(err)
#             spd = fwd if ae < 0.5 else (fwd*0.75 if ae < 1.2 else FWD_SLOW)
#             spd = max(spd, FWD_MIN)
#             turn = clamp(TURN_K * err, -1.8, 1.8)
#             lcmd = clamp(int(MAX_WHEEL*0.9*(spd - 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
#             rcmd = clamp(int(MAX_WHEEL*0.9*(spd + 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
#             left  = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
#             right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
#             lastL, lastR = left, right
#             robot.set_vel(left, right)

#         # ----------------- LEDs + sparse logging -----------------
#         robot.set_led(*[int(clamp(c,0,100)) for c in led])
#         if now - last_print > PRINT_PERIOD:
#             print(f"[seq] id={rid} phase={phase_name} t={t:5.1f}s")
#             last_print = now

#         robot.delay(DT_MS)




# # -*- coding: utf-8 -*-
# # Single-Ring Encircle: Bots assigned fixed equidistant positions on circle
# # Uses P2P to coordinate target assignments and move to fixed positions

# import math, struct, random

# # ----------------- Arena -----------------
# X_MIN, X_MAX = -1.2, 1.0
# Y_MIN, Y_MAX = -1.4, 2.35

# # ----------------- Dancer no-go circle -----------------
# FEET = 0.3048
# OBST_DIAM_FT = 1.0
# OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
# OBST_MARGIN  = 0.03
# SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
# CX, CY = (-0.1, 0.475)

# # ----------------- Single ring parameters -----------------
# R_TARGET = SAFE_BUBBLE + 0.33  # Single target radius
# DIRECTION = +1  # CCW rotation

# # ----------------- Motion / spacing -----------------
# V_TANGENT_BASE = 0.26
# K_R, RADIAL_CLAMP = 1.3, 0.12
# ANG_P_GAIN, ANG_D_GAIN = 2.0, 0.5  # PD control for angle
# MIN_LINEAR_SEP = 0.15
# COLLISION_SEP = 0.10

# # ----------------- Boundary cushion -----------------
# SOFT_MARGIN, CRIT_MARGIN, SOFT_MAX_FORCE = 0.08, 0.02, 0.35

# # ----------------- Drive model -----------------
# MAX_WHEEL, TURN_K = 35, 3.0
# FWD_FAST, FWD_SLOW, FWD_MIN = 0.80, 0.30, 0.40
# CMD_SMOOTH, EPS, DT_MS = 0.20, 1e-3, 40
# PRINT_PERIOD = 2.0

# # ----------------- Heartbeats (pose+target_angle+state) -----------------
# HB_FMT = 'fffifi'  # x,y,theta, target_angle, vid, state
# HB_BYTES = struct.calcsize(HB_FMT)
# HB_DT, STALE_S = 0.12, 0.8

# # Bot states
# STATE_MOVING_TO_TARGET = 0
# STATE_IN_POSITION = 1
# STATE_MOVING_WITH_RING = 2

# def clamp(v, lo, hi): return lo if v < lo else (hi if v > hi else v)
# def wrap(a):
#     while a >  math.pi: a -= 2*math.pi
#     while a <= -math.pi: a += 2*math.pi
#     return a

# def safe_pose(robot):
#     p = robot.get_pose()
#     if p and len(p) >= 3: return float(p[0]), float(p[1]), float(p[2])
#     return None

# def soft_boundary_force(x, y):
#     fx = fy = 0.0
#     if x < X_MIN + SOFT_MARGIN:        fx += SOFT_MAX_FORCE * (1 - (x - X_MIN)/SOFT_MARGIN)
#     elif x > X_MAX - SOFT_MARGIN:      fx -= SOFT_MAX_FORCE * (1 - (X_MAX - x)/SOFT_MARGIN)
#     if y < Y_MIN + SOFT_MARGIN:        fy += SOFT_MAX_FORCE * (1 - (y - Y_MIN)/SOFT_MARGIN)
#     elif y > Y_MAX - SOFT_MARGIN:      fy -= SOFT_MAX_FORCE * (1 - (Y_MAX - y)/SOFT_MARGIN)
#     return fx, fy

# def is_critical_boundary(x, y):
#     return (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
#             y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN)

# def try_get_swarm_poses(robot):
#     for nm in ('get_swarm_poses','get_all_poses','get_poses','swarm_poses'):
#         fn = getattr(robot, nm, None)
#         if callable(fn):
#             try:
#                 poses = fn()
#                 if poses: return poses
#             except: pass
#     return []

# # --- sim shims ---
# def get_vid(robot):
#     try: return int(robot.id())
#     except: return 0
# def logw(msg): print(msg)

# def usr(robot):
#     robot.delay(800)
#     rid = get_vid(robot)
#     random.seed((rid if rid is not None else 0)*1103515245 & 0xFFFFFFFF)

#     # neighbors: id -> (x,y,theta,target_angle,state,t_last)
#     nbrs = {}
#     last_hb = -1e9
#     last_print = 0.0
#     lastL = lastR = 0
    
#     # Target assignment
#     my_target_angle = None
#     my_state = STATE_MOVING_TO_TARGET
#     last_angle_error = 0.0

#     # detection flags
#     p2p_ever_seen = False
#     ids_seen = set()

#     # nudge localization
#     robot.set_vel(20,20); robot.delay(150)

#     while True:
#         pose = safe_pose(robot)
#         if not pose:
#             robot.set_vel(0,0); robot.delay(DT_MS); continue
#         x,y,th = pose
#         now = robot.get_clock()

#         # safety
#         rdx, rdy = x - CX, y - CY
#         r = math.hypot(rdx, rdy)
#         if r < OBST_RADIUS or is_critical_boundary(x, y):
#             robot.set_led(100,0,0); robot.set_vel(0,0); robot.delay(DT_MS); continue

#         theta = math.atan2(rdy, rdx)

#         # unit frames
#         if r < 1e-6: urx, ury = 1.0, 0.0
#         else:        urx, ury = rdx/r, rdy/r
#         utx, uty = -ury, urx
#         if DIRECTION < 0: utx, uty = -utx, -uty

#         # --- TARGET ANGLE ASSIGNMENT ---
#         if my_target_angle is None:
#             # Initial random target angle
#             my_target_angle = random.uniform(-math.pi, math.pi)
        
#         # Receive heartbeats and coordinate target angles
#         msgs = robot.recv_msg() or []
#         for m in msgs:
#             try:
#                 nx, ny, nth, n_target_angle, nid, nstate = struct.unpack(HB_FMT, m[:HB_BYTES])
#                 if int(nid) != rid:
#                     nbrs[int(nid)] = (float(nx), float(ny), float(nth), float(n_target_angle), int(nstate), now)
#                     ids_seen.add(int(nid))
#                     p2p_ever_seen = True
#             except: pass

#         # Fallback to sim API if no P2P
#         if not p2p_ever_seen:
#             sim_poses = try_get_swarm_poses(robot)
#             if sim_poses:
#                 nbrs.clear()
#                 for item in sim_poses:
#                     if isinstance(item,(list,tuple)) and len(item)>=3:
#                         if len(item)>=4:
#                             nid, nx, ny, nth = int(item[0]), float(item[1]), float(item[2]), float(item[3])
#                         else:
#                             nx, ny, nth = float(item[0]), float(item[1]), float(item[2])
#                             nid = 10_000 + int(1000*nx) ^ int(1000*ny)
#                         if nid == rid: continue
#                         ntheta = math.atan2(ny - CY, nx - CX)
#                         # Assign temporary target for neighbors
#                         nbrs[int(nid)] = (nx, ny, nth, ntheta, STATE_MOVING_TO_TARGET, now)

#         # prune stale
#         cutoff = now - STALE_S
#         for nid in list(nbrs.keys()):
#             if nbrs[nid][5] < cutoff:
#                 nbrs.pop(nid, None)

#         # --- COORDINATE TARGET ANGLES ---
#         # If we have neighbor info, adjust targets to be equidistant
#         if len(nbrs) > 0:
#             # Collect all target angles (ours and neighbors')
#             all_targets = [my_target_angle]
#             all_ids = [rid]
            
#             for nid, (nx, ny, nth, n_target, nstate, tlast) in nbrs.items():
#                 all_targets.append(n_target)
#                 all_ids.append(nid)
            
#             # Sort by target angle to find gaps
#             sorted_indices = sorted(range(len(all_targets)), key=lambda i: all_targets[i])
#             sorted_targets = [all_targets[i] for i in sorted_indices]
#             sorted_ids = [all_ids[i] for i in sorted_indices]
            
#             # Find our position in the sorted list
#             our_index = sorted_ids.index(rid)
            
#             # Calculate ideal spacing (2π / total_bots)
#             total_bots = len(all_targets)
#             ideal_spacing = 2 * math.pi / total_bots
            
#             # Calculate what our target angle SHOULD be based on ideal spacing
#             # Start from the smallest current target angle
#             base_angle = sorted_targets[0]
#             ideal_target = base_angle + our_index * ideal_spacing
#             ideal_target = wrap(ideal_target)
            
#             # Smoothly move our target toward the ideal position
#             target_error = wrap(ideal_target - my_target_angle)
#             my_target_angle = wrap(my_target_angle + 0.1 * target_error)

#         # --- CONTROL STRATEGY BASED ON STATE ---
#         angle_error = wrap(my_target_angle - theta)
#         radial_error = R_TARGET - r
        
#         vx, vy = 0.0, 0.0
        
#         if my_state == STATE_MOVING_TO_TARGET:
#             # PD control for angular position
#             angle_derivative = (angle_error - last_angle_error) / (DT_MS/1000.0) if abs(last_angle_error) > 0 else 0
#             angular_velocity = ANG_P_GAIN * angle_error + ANG_D_GAIN * angle_derivative
#             angular_velocity = clamp(angular_velocity, -1.0, 1.0)
            
#             # Move tangentially to correct angle
#             vx = angular_velocity * utx * V_TANGENT_BASE
#             vy = angular_velocity * uty * V_TANGENT_BASE
            
#             # Also maintain radial position
#             radial = clamp(K_R * radial_error, -RADIAL_CLAMP, RADIAL_CLAMP)
#             vx += radial * urx
#             vy += radial * ury
            
#             # Check if we're close enough to target
#             if abs(angle_error) < 0.1 and abs(radial_error) < 0.03:
#                 my_state = STATE_IN_POSITION
#                 logw(f"Bot {rid} reached target position!")
        
#         elif my_state == STATE_IN_POSITION:
#             # Stay in place, just maintain position
#             radial = clamp(K_R * radial_error, -RADIAL_CLAMP, RADIAL_CLAMP)
#             vx = radial * urx
#             vy = radial * ury
            
#             # Check if all neighbors are also in position
#             all_in_position = True
#             for nid, (nx, ny, nth, n_target, nstate, tlast) in nbrs.items():
#                 if nstate != STATE_IN_POSITION and nstate != STATE_MOVING_WITH_RING:
#                     all_in_position = False
#                     break
            
#             if all_in_position and len(nbrs) > 0:
#                 my_state = STATE_MOVING_WITH_RING
#                 logw(f"All bots in position! Starting coordinated motion.")
        
#         elif my_state == STATE_MOVING_WITH_RING:
#             # Coordinated motion around the circle
#             vx = V_TANGENT_BASE * utx
#             vy = V_TANGENT_BASE * uty
            
#             # Maintain radial position
#             radial = clamp(K_R * radial_error, -RADIAL_CLAMP, RADIAL_CLAMP)
#             vx += radial * urx
#             vy += radial * ury

#         last_angle_error = angle_error

#         # --- COLLISION AVOIDANCE (safety net) ---
#         collision_detected = False
#         for _, (nx, ny, nth, n_target, nstate, tlast) in nbrs.items():
#             ddx, ddy = x - nx, y - ny
#             d_linear = math.hypot(ddx, ddy)
            
#             if d_linear < COLLISION_SEP:
#                 collision_detected = True
#                 repulsion_strength = 0.5 * (COLLISION_SEP - d_linear) / COLLISION_SEP
#                 if d_linear > 1e-6:
#                     repulse_x = ddx / d_linear
#                     repulse_y = ddy / d_linear
#                     vx += repulsion_strength * repulse_x
#                     vy += repulsion_strength * repulse_y

#         # boundary cushion
#         bfx, bfy = soft_boundary_force(x, y)
#         b_norm = bfx*urx + bfy*ury
#         vx += b_norm * urx
#         vy += b_norm * ury

#         # --- send heartbeat ---
#         if now - last_hb >= HB_DT:
#             try:
#                 robot.send_msg(struct.pack(HB_FMT, float(x), float(y), float(th),
#                                            float(my_target_angle), int(rid), int(my_state)))
#             except: pass
#             last_hb = now

#         # LEDs based on state
#         near_soft = abs(b_norm) > 1e-6
#         if collision_detected:          robot.set_led(100, 0, 0)     # Red for collision
#         elif my_state == STATE_MOVING_TO_TARGET: robot.set_led(100, 60, 0)  # Orange - moving to target
#         elif my_state == STATE_IN_POSITION:      robot.set_led(0, 100, 0)   # Green - in position
#         elif my_state == STATE_MOVING_WITH_RING: robot.set_led(0, 0, 100)   # Blue - moving together
#         else:                          robot.set_led(0, 70, 80)      # Cyan - unknown

#         # map to wheels
#         spd = math.hypot(vx, vy)
#         if spd < EPS:
#             vx += 0.05 * utx
#             vy += 0.05 * uty
#             spd = math.hypot(vx, vy)
        
#         hdg = math.atan2(vy, vx)
#         err = wrap(hdg - th)

#         ae = abs(err)
#         if ae < 0.5:    fwd = FWD_FAST
#         elif ae < 1.2:  fwd = FWD_FAST * 0.7
#         else:           fwd = FWD_SLOW
#         fwd = max(fwd, FWD_MIN)
#         if near_soft or collision_detected: fwd *= 0.7

#         turn = clamp(TURN_K * err, -1.5, 1.5)
#         lcmd = clamp(int(MAX_WHEEL*0.9*(fwd - 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
#         rcmd = clamp(int(MAX_WHEEL*0.9*(fwd + 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)

#         left  = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
#         right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
#         lastL, lastR = left, right
#         robot.set_vel(left, right)

#         # occasional print
#         if now - last_print > PRINT_PERIOD:
#             states_count = {STATE_MOVING_TO_TARGET: 0, STATE_IN_POSITION: 0, STATE_MOVING_WITH_RING: 0}
#             for nbr in nbrs.values():
#                 states_count[nbr[4]] += 1
#             states_count[my_state] += 1  # Include self
            
#             logw(f"[structured_ring] id={rid} state={my_state} "
#                  f"target_err={angle_error:.3f} radial_err={radial_error:.3f} "
#                  f"states[M:{states_count[0]} I:{states_count[1]} R:{states_count[2]}]")
#             last_print = now

#         robot.delay(DT_MS)




# -*- coding: utf-8 -*-
# Single-Ring Encircle: Bots assigned fixed equidistant positions on circle
# Uses P2P to coordinate target assignments and move to fixed positions
# SIMULATION VERSION - uses robot.id() and print()

import math, struct, random

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
DIRECTION = +1  # CCW rotation

# ----------------- Motion / spacing -----------------
V_TANGENT_BASE = 0.26
K_R, RADIAL_CLAMP = 1.3, 0.12
ANG_P_GAIN, ANG_D_GAIN = 2.0, 0.5  # PD control for angle
MIN_LINEAR_SEP = 0.15
COLLISION_SEP = 0.10

# ----------------- Boundary cushion -----------------
SOFT_MARGIN, CRIT_MARGIN, SOFT_MAX_FORCE = 0.08, 0.02, 0.35

# ----------------- Drive model -----------------
MAX_WHEEL, TURN_K = 35, 3.0
FWD_FAST, FWD_SLOW, FWD_MIN = 0.80, 0.30, 0.40
CMD_SMOOTH, EPS, DT_MS = 0.20, 1e-3, 40
PRINT_PERIOD = 2.0

# ----------------- Heartbeats (pose+target_angle+state) -----------------
HB_FMT = 'fffifi'  # x,y,theta, target_angle, vid, state
HB_BYTES = struct.calcsize(HB_FMT)
HB_DT, STALE_S = 0.12, 0.8

# Bot states
STATE_MOVING_TO_TARGET = 0
STATE_IN_POSITION = 1
STATE_MOVING_WITH_RING = 2

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
    robot.delay(800)
    
    # SIMULATION: Use robot.id() directly
    try:
        rid = int(robot.id())
    except:
        rid = 0
        
    random.seed((rid if rid is not None else 0)*1103515245 & 0xFFFFFFFF)

    # neighbors: id -> (x,y,theta,target_angle,state,t_last)
    nbrs = {}
    last_hb = -1e9
    last_print = 0.0
    lastL = lastR = 0
    
    # Target assignment
    my_target_angle = None
    my_state = STATE_MOVING_TO_TARGET
    last_angle_error = 0.0

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

        # safety
        rdx, rdy = x - CX, y - CY
        r = math.hypot(rdx, rdy)
        if r < OBST_RADIUS or is_critical_boundary(x, y):
            robot.set_led(100,0,0); robot.set_vel(0,0); robot.delay(DT_MS); continue

        theta = math.atan2(rdy, rdx)

        # unit frames
        if r < 1e-6: urx, ury = 1.0, 0.0
        else:        urx, ury = rdx/r, rdy/r
        utx, uty = -ury, urx
        if DIRECTION < 0: utx, uty = -utx, -uty

        # --- TARGET ANGLE ASSIGNMENT ---
        if my_target_angle is None:
            # Initial target angle based on current position
            my_target_angle = theta
            print(f"Bot {rid} initial target angle: {my_target_angle:.3f}")
        
        # --- RECEIVE HEARTBEATS (P2P) ---
        msgs = robot.recv_msg()
        if msgs is None:
            msgs = []
        elif not isinstance(msgs, list):
            msgs = [msgs]
            
        for m in msgs:
            try:
                if len(m) >= HB_BYTES:
                    nx, ny, nth, n_target_angle, nid, nstate = struct.unpack(HB_FMT, m[:HB_BYTES])
                    if int(nid) != rid:
                        nbrs[int(nid)] = (float(nx), float(ny), float(nth), float(n_target_angle), int(nstate), now)
                        ids_seen.add(int(nid))
                        p2p_ever_seen = True
            except Exception as e:
                print(f"Bot {rid} heartbeat unpack error: {e}")

        # Fallback to sim API if no P2P seen after some time
        if not p2p_ever_seen and now > 5.0:  # Wait 5 seconds before using fallback
            sim_poses = try_get_swarm_poses(robot)
            if sim_poses:
                nbrs.clear()
                for item in sim_poses:
                    if isinstance(item,(list,tuple)) and len(item)>=3:
                        if len(item)>=4:
                            nid, nx, ny, nth = int(item[0]), float(item[1]), float(item[2]), float(item[3])
                        else:
                            nx, ny, nth = float(item[0]), float(item[1]), float(item[2])
                            nid = 10000 + int(1000*abs(nx) + int(1000*abs(ny))  # More stable ID generation
                        if nid == rid: continue
                        ntheta = math.atan2(ny - CY, nx - CX)
                        # Assign temporary target for neighbors
                        nbrs[int(nid)] = (nx, ny, nth, ntheta, STATE_MOVING_TO_TARGET, now)

        # prune stale neighbors
        cutoff = now - STALE_S
        for nid in list(nbrs.keys()):
            if nbrs[nid][5] < cutoff:
                nbrs.pop(nid, None)

        # --- COORDINATE TARGET ANGLES ---
        # If we have neighbor info, adjust targets to be equidistant
        if len(nbrs) > 0:
            # Collect all target angles (ours and neighbors')
            all_targets = [my_target_angle]
            all_ids = [rid]
            
            for nid, (nx, ny, nth, n_target, nstate, tlast) in nbrs.items():
                all_targets.append(n_target)
                all_ids.append(nid)
            
            # Sort by target angle to find gaps
            sorted_pairs = sorted(zip(all_targets, all_ids))
            sorted_targets = [pair[0] for pair in sorted_pairs]
            sorted_ids = [pair[1] for pair in sorted_pairs]
            
            # Find our position in the sorted list
            our_index = sorted_ids.index(rid)
            
            # Calculate ideal spacing (2π / total_bots)
            total_bots = len(all_targets)
            ideal_spacing = 2 * math.pi / total_bots
            
            # Calculate what our target angle SHOULD be based on ideal spacing
            # Start from the smallest current target angle
            base_angle = sorted_targets[0]
            ideal_target = base_angle + our_index * ideal_spacing
            ideal_target = wrap(ideal_target)
            
            # Smoothly move our target toward the ideal position
            target_error = wrap(ideal_target - my_target_angle)
            my_target_angle = wrap(my_target_angle + 0.15 * target_error)

        # --- CONTROL STRATEGY BASED ON STATE ---
        angle_error = wrap(my_target_angle - theta)
        radial_error = R_TARGET - r
        
        vx, vy = 0.0, 0.0
        
        if my_state == STATE_MOVING_TO_TARGET:
            # PD control for angular position
            angle_derivative = (angle_error - last_angle_error) / (DT_MS/1000.0) if abs(last_angle_error) > 0 else 0
            angular_velocity = ANG_P_GAIN * angle_error + ANG_D_GAIN * angle_derivative
            angular_velocity = clamp(angular_velocity, -1.5, 1.5)
            
            # Move tangentially to correct angle
            vx = angular_velocity * utx * V_TANGENT_BASE
            vy = angular_velocity * uty * V_TANGENT_BASE
            
            # Also maintain radial position
            radial = clamp(K_R * radial_error, -RADIAL_CLAMP, RADIAL_CLAMP)
            vx += radial * urx
            vy += radial * ury
            
            # Check if we're close enough to target
            if abs(angle_error) < 0.08 and abs(radial_error) < 0.03:
                my_state = STATE_IN_POSITION
                print(f"Bot {rid} reached target position!")
        
        elif my_state == STATE_IN_POSITION:
            # Stay in place, just maintain position
            radial = clamp(K_R * radial_error, -RADIAL_CLAMP, RADIAL_CLAMP)
            vx = radial * urx
            vy = radial * ury
            
            # Small tangential movement to fine-tune position
            vx += 0.1 * angle_error * utx
            vy += 0.1 * angle_error * uty
            
            # Check if all neighbors are also in position
            all_in_position = True
            neighbors_in_pos = 0
            for nid, (nx, ny, nth, n_target, nstate, tlast) in nbrs.items():
                if nstate != STATE_IN_POSITION and nstate != STATE_MOVING_WITH_RING:
                    all_in_position = False
                else:
                    neighbors_in_pos += 1
            
            # Start moving if most neighbors are ready
            if (all_in_position and len(nbrs) > 0) or (neighbors_in_pos >= len(nbrs) * 0.8 and len(nbrs) > 2):
                my_state = STATE_MOVING_WITH_RING
                print(f"Bot {rid} starting coordinated motion with {neighbors_in_pos}/{len(nbrs)} neighbors ready")
        
        elif my_state == STATE_MOVING_WITH_RING:
            # Coordinated motion around the circle
            vx = V_TANGENT_BASE * utx
            vy = V_TANGENT_BASE * uty
            
            # Maintain radial position
            radial = clamp(K_R * radial_error, -RADIAL_CLAMP, RADIAL_CLAMP)
            vx += radial * urx
            vy += radial * ury

        last_angle_error = angle_error

        # --- COLLISION AVOIDANCE (safety net) ---
        collision_detected = False
        for _, (nx, ny, nth, n_target, nstate, tlast) in nbrs.items():
            ddx, ddy = x - nx, y - ny
            d_linear = math.hypot(ddx, ddy)
            
            if d_linear < COLLISION_SEP:
                collision_detected = True
                repulsion_strength = 0.8 * (COLLISION_SEP - d_linear) / COLLISION_SEP
                if d_linear > 1e-6:
                    repulse_x = ddx / d_linear
                    repulse_y = ddy / d_linear
                    vx += repulsion_strength * repulse_x
                    vy += repulsion_strength * repulse_y

        # boundary cushion
        bfx, bfy = soft_boundary_force(x, y)
        b_norm = bfx*urx + bfy*ury
        vx += b_norm * urx
        vy += b_norm * ury

        # --- SEND HEARTBEAT (P2P) ---
        if now - last_hb >= HB_DT:
            try:
                msg = struct.pack(HB_FMT, float(x), float(y), float(th),
                                 float(my_target_angle), int(rid), int(my_state))
                robot.send_msg(msg)
            except Exception as e:
                print(f"Bot {rid} heartbeat send error: {e}")
            last_hb = now

        # LEDs based on state
        near_soft = abs(b_norm) > 1e-6
        if collision_detected:          
            robot.set_led(100, 0, 0)     # Red for collision
        elif my_state == STATE_MOVING_TO_TARGET: 
            robot.set_led(100, 60, 0)    # Orange - moving to target
        elif my_state == STATE_IN_POSITION:      
            robot.set_led(0, 100, 0)     # Green - in position
        elif my_state == STATE_MOVING_WITH_RING: 
            robot.set_led(0, 0, 100)     # Blue - moving together
        else:                          
            robot.set_led(0, 70, 80)     # Cyan - unknown

        # map to wheels
        spd = math.hypot(vx, vy)
        if spd < EPS:
            vx += 0.05 * utx
            vy += 0.05 * uty
            spd = math.hypot(vx, vy)
        
        hdg = math.atan2(vy, vx)
        err = wrap(hdg - th)

        ae = abs(err)
        if ae < 0.5:    fwd = FWD_FAST
        elif ae < 1.2:  fwd = FWD_FAST * 0.7
        else:           fwd = FWD_SLOW
        fwd = max(fwd, FWD_MIN)
        if near_soft or collision_detected: fwd *= 0.7

        turn = clamp(TURN_K * err, -1.5, 1.5)
        lcmd = clamp(int(MAX_WHEEL*0.9*(fwd - 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
        rcmd = clamp(int(MAX_WHEEL*0.9*(fwd + 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)

        left  = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
        right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
        lastL, lastR = left, right
        robot.set_vel(left, right)

        # occasional print
        if now - last_print > PRINT_PERIOD:
            states_count = {STATE_MOVING_TO_TARGET: 0, STATE_IN_POSITION: 0, STATE_MOVING_WITH_RING: 0}
            for nbr in nbrs.values():
                states_count[nbr[4]] += 1
            states_count[my_state] += 1  # Include self
            
            print(f"[structured_ring] id={rid} state={my_state} "
                 f"target_err={angle_error:.3f} radial_err={radial_error:.3f} "
                 f"neighbors={len(nbrs)} "
                 f"states[M:{states_count[0]} I:{states_count[1]} R:{states_count[2]}]")
            last_print = now
 
        robot.delay(DT_MS) 