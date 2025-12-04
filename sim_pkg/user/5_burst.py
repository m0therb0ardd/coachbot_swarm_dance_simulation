# # # # # -*- coding: utf-8 -*-
# # # # # Jitter P2P (SIM) — short random darts with peer-aware avoidance + dancer ring safety.
# # # # # SIM NOTES:
# # # # #   - Uses robot.id (attribute) for ID in simulation.
# # # # #   - Uses send_msg / recv_msg for P2P heartbeats.
# # # # #   - To port to physical testbed, change get_id() to prefer robot.virtual_id()
# # # # #     and change logw() to use the provided log.write(...).

# # # # import math, struct, random, os

# # # # # ----------------- Logging (sim + hardware) -----------------
# # # # LOG = None   # global log handle

# # # # def init_log():
# # # #     """
# # # #     Try to open experiment_log.txt in a hardware-like way.
# # # #     If it fails (e.g., sim with no FS), we just fall back to print only.
# # # #     """
# # # #     global LOG
# # # #     if LOG is not None:
# # # #         return
# # # #     try:
# # # #         # line-buffered like your FLOAT HW script
# # # #         LOG = open("experiment_log.txt", "a", 1)
# # # #     except Exception:
# # # #         LOG = None

# # # # def logw(msg):
# # # #     """
# # # #     Write to log file (if available) AND print to stdout.
# # # #     Safe in both sim and hardware.
# # # #     """
# # # #     if not isinstance(msg, str):
# # # #         msg = str(msg)
# # # #     line = msg if msg.endswith("\n") else msg + "\n"

# # # #     # Log file (hardware) if available
# # # #     if LOG is not None:
# # # #         try:
# # # #             LOG.write(line)
# # # #             LOG.flush()
# # # #             os.fsync(LOG.fileno())
# # # #         except Exception:
# # # #             pass

# # # #     # Always also print (sim / console)
# # # #     print(line.rstrip("\n"))
    

# # # # # -------- arena & dancer obstacle (meters) --------
# # # # X_MIN, X_MAX = -1.2, 1.0
# # # # Y_MIN, Y_MAX = -1.4, 2.35
# # # # FEET = 0.3048
# # # # OBST_DIAM_FT = 1.0
# # # # OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
# # # # OBST_MARGIN  = 0.03
# # # # SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
# # # # OBST_CX, OBST_CY = (-0.1, 0.475)

# # # # # -------- drive / loop --------
# # # # MAX_WHEEL   = 35
# # # # TURN_K      = 3.0
# # # # FWD_JITTER  = 0.95     # forward during dart
# # # # FWD_AIM     = 0.35     # forward while aiming
# # # # FWD_MIN     = 0.38
# # # # CMD_SMOOTH  = 0.20
# # # # DT_MS       = 40

# # # # # -------- jitter timing (seconds) --------
# # # # JITTER_LO   = 0.55
# # # # JITTER_HI   = 1.20
# # # # AIM_TIME    = 0.20
# # # # DART_TIME   = 0.35
# # # # COOLDOWN    = 0.25

# # # # # -------- collision safety --------
# # # # NEIGH_RADIUS = 0.75
# # # # SEP_RADIUS   = 0.24
# # # # PANIC_RADIUS = 0.18
# # # # WALL_MARGIN  = 0.08
# # # # WALL_CRIT    = 0.02
# # # # MAX_SOFT_F   = 0.50

# # # # # -------- heading sampling --------
# # # # SAMPLES      = 20
# # # # BIAS_STD     = 0.9  # radians; set 0 for uniform

# # # # # -------- messaging (heartbeats) --------
# # # # # (x, y, th, vx, vy, vid) -> 5 floats + 1 int
# # # # HB_FMT   = 'fffffi'
# # # # HB_BYTES = struct.calcsize(HB_FMT)
# # # # HB_DT    = 0.12        # ~8–9 Hz
# # # # STALE_S  = 0.7

# # # # # ---------- small helpers ----------
# # # # def clamp(v, lo, hi):
# # # #     return lo if v < lo else (hi if v > hi else v)

# # # # def wrap_angle(a):
# # # #     while a >  math.pi:  a -= 2.0 * math.pi
# # # #     while a <= -math.pi: a += 2.0 * math.pi
# # # #     return a

# # # # def soft_boundary_force(x, y):
# # # #     fx = fy = 0.0
# # # #     if x < X_MIN + WALL_MARGIN:
# # # #         fx += MAX_SOFT_F * (1.0 - (x - X_MIN) / WALL_MARGIN)
# # # #     elif x > X_MAX - WALL_MARGIN:
# # # #         fx -= MAX_SOFT_F * (1.0 - (X_MAX - x) / WALL_MARGIN)
# # # #     if y < Y_MIN + WALL_MARGIN:
# # # #         fy += MAX_SOFT_F * (1.0 - (y - Y_MIN) / WALL_MARGIN)
# # # #     elif y > Y_MAX - WALL_MARGIN:
# # # #         fy -= MAX_SOFT_F * (1.0 - (Y_MAX - y) / WALL_MARGIN)
# # # #     return fx, fy

# # # # def boundary_state(x, y):
# # # #     # 2 = critical, 1 = warning, 0 = safe
# # # #     if (x < X_MIN + WALL_CRIT or x > X_MAX - WALL_CRIT or
# # # #         y < Y_MIN + WALL_CRIT or y > Y_MAX - WALL_CRIT):
# # # #         return 2
# # # #     if (x < X_MIN + WALL_MARGIN or x > X_MAX - WALL_MARGIN or
# # # #         y < Y_MIN + WALL_MARGIN or y > Y_MAX - WALL_MARGIN):
# # # #         return 1
# # # #     return 0

# # # # def obstacle_push(x, y, max_force=0.8, buffer_width=0.12):
# # # #     dx = x - OBST_CX
# # # #     dy = y - OBST_CY
# # # #     r  = math.hypot(dx, dy)
# # # #     if r < SAFE_BUBBLE + buffer_width:
# # # #         if r < 1e-6:
# # # #             return max_force, 0.0
# # # #         s = max(0.0, (SAFE_BUBBLE + buffer_width - r) / buffer_width) * max_force
# # # #         return s * (dx / r), s * (dy / r)
# # # #     return 0.0, 0.0

# # # # def safe_pose(robot):
# # # #     """Return (x,y,th) as floats, or None if pose invalid."""
# # # #     p = robot.get_pose()
# # # #     if isinstance(p, (list, tuple)) and len(p) >= 3:
# # # #         return float(p[0]), float(p[1]), float(p[2])
# # # #     return None

# # # # def heading_to_wheels(err, fwd, lastL, lastR):
# # # #     turn = clamp(TURN_K * err, -1.5, 1.5)
# # # #     lcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL,  MAX_WHEEL)
# # # #     rcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL,  MAX_WHEEL)
# # # #     left  = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
# # # #     right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
# # # #     return left, right

# # # # # --- portability shims (SIM ⇄ PHYSICAL) ---
# # # # def get_id(robot):
# # # #     """
# # # #     SIM: use robot.id (attribute).
# # # #     PHYSICAL: prefer robot.virtual_id(), fall back to robot.id if present.
# # # #     """
# # # #     # simulation: robot.id is an attribute
# # # #     if hasattr(robot, "id"):
# # # #         try:
# # # #             return int(robot.id)
# # # #         except:
# # # #             pass

# # # #     # physical testbed: often robot.virtual_id() exists
# # # #     if hasattr(robot, "virtual_id"):
# # # #         try:
# # # #             return int(robot.virtual_id())
# # # #         except:
# # # #             pass

# # # #     return -1


# # # # # ---------- heading chooser ----------
# # # # def choose_safe_heading(x, y, th, neighbors):
# # # #     best_h, best_score = th, -1e9
# # # #     for _ in range(SAMPLES):
# # # #         if BIAS_STD > 0:
# # # #             cand = wrap_angle(th + random.gauss(0.0, BIAS_STD))
# # # #         else:
# # # #             cand = random.uniform(-math.pi, math.pi)

# # # #         step = 0.18
# # # #         px = x + step * math.cos(cand)
# # # #         py = y + step * math.sin(cand)

# # # #         score = 0.0
# # # #         # neighbor avoidance / spacing
# # # #         for _, (nx, ny, _, _, _) in neighbors.items():
# # # #             d = math.hypot(px - nx, py - ny)
# # # #             if d < PANIC_RADIUS:
# # # #                 score -= 1000.0
# # # #             elif d < SEP_RADIUS:
# # # #                 score -= 200.0 * (SEP_RADIUS - d)
# # # #             else:
# # # #                 score += min(d, 0.6)

# # # #         # dancer bubble avoidance
# # # #         od = math.hypot(px - OBST_CX, py - OBST_CY)
# # # #         if od < SAFE_BUBBLE:
# # # #             score -= 800.0
# # # #         # else:
# # # #         #     score += min(od - SAFE_BUBBLE, 0.5)

# # # #         # wall margin penalty
# # # #         if (px < X_MIN + WALL_MARGIN or px > X_MAX - WALL_MARGIN or
# # # #             py < Y_MIN + WALL_MARGIN or py > Y_MAX - WALL_MARGIN):
# # # #             score -= 500.0

# # # #         if score > best_score:
# # # #             best_score, best_h = score, cand
# # # #     return best_h

# # # # # ---------------- main entry ----------------
# # # # def usr(robot):
# # # #     init_log()
# # # #     robot.delay(400)  # sim settle
# # # #     vid = get_id(robot)
# # # #     random.seed((vid if vid is not None else 0) * 1103515245 & 0xFFFFFFFF)

    
# # # #     neighbors = {}   # vid -> (x,y,th,vx,vy)
# # # #     last_seen = {}   # vid -> t
# # # #     last_hb   = -1e9

# # # #     # --- desynchronize first dart per robot ---
# # # #     now = robot.get_clock()
# # # #     initial_wiggle    = random.uniform(0.0, 2.0)          # each bot waits 0–2s before first dart cycle
# # # #     cooldown_until    = now + initial_wiggle
# # # #     next_dart_at      = cooldown_until + random.uniform(JITTER_LO, JITTER_HI)
# # # #     aim_until         = 0.0
# # # #     dart_until        = 0.0
# # # #     evasive_until     = 0.0
# # # #     target_h          = 0.0
# # # #     lastL = lastR     = 0
# # # #     last_log          = 0.0


# # # #     # kick once so pose starts updating
# # # #     robot.set_vel(20, 20)
# # # #     robot.delay(120)

# # # #     while True:
# # # #         pose = safe_pose(robot)
# # # #         if pose is None:
# # # #             robot.set_vel(0, 0)
# # # #             robot.delay(DT_MS)
# # # #             continue
# # # #         x, y, th = pose
# # # #         now = robot.get_clock()

# # # #         # boundary LEDs (sim-safe colors)
# # # #         b = boundary_state(x, y)
# # # #         if b == 2:
# # # #             robot.set_led(100, 0, 0)
# # # #             robot.set_vel(0, 0)
# # # #             robot.delay(DT_MS)
# # # #             continue
# # # #         elif b == 1:
# # # #             robot.set_led(120, 60, 0)
# # # #         else:
# # # #             robot.set_led(40, 120, 200)

# # # #         # --- heartbeat send (finite diff for vx,vy) ---
# # # #         if now - last_hb >= HB_DT:
# # # #             x1, y1, _ = pose
# # # #             t1 = now
# # # #             robot.delay(50)
# # # #             p2 = safe_pose(robot)
# # # #             if p2:
# # # #                 x2, y2, th2 = p2
# # # #                 t2 = robot.get_clock()
# # # #                 dt = max(1e-3, t2 - t1)
# # # #                 vx = (x2 - x1) / dt
# # # #                 vy = (y2 - y1) / dt
# # # #                 try:
# # # #                     hb = struct.pack(
# # # #                         HB_FMT,
# # # #                         float(x2), float(y2), float(th2),
# # # #                         float(vx), float(vy),
# # # #                         int(vid),
# # # #                     )
# # # #                     robot.send_msg(hb)
# # # #                 except:
# # # #                     pass
# # # #                 last_hb = t2
# # # #                 x, y, th = x2, y2, th2
# # # #             else:
# # # #                 last_hb = now

# # # #         # --- receive heartbeats ---
# # # #         msgs = robot.recv_msg() or []
# # # #         for m in msgs:
# # # #             try:
# # # #                 nx, ny, nth, nvx, nvy, nid = struct.unpack(HB_FMT, m[:HB_BYTES])
# # # #                 nid = int(nid)
# # # #                 if nid != int(vid):
# # # #                     neighbors[nid] = (nx, ny, nth, nvx, nvy)
# # # #                     last_seen[nid] = now
# # # #             except:
# # # #                 # ignore malformed / short messages
# # # #                 pass

# # # #         # prune stale neighbors
# # # #         cutoff = now - STALE_S
# # # #         for nid in list(neighbors.keys()):
# # # #             if last_seen.get(nid, 0.0) < cutoff:
# # # #                 neighbors.pop(nid, None)
# # # #                 last_seen.pop(nid, None)

# # # #         # environmental nudges (walls + dancer bubble)
# # # #         fx, fy = soft_boundary_force(x, y)
# # # #         ox, oy = obstacle_push(x, y)
# # # #         if fx*fx + fy*fy + ox*ox + oy*oy > 1e-9:
# # # #             env_hdg = math.atan2(fy + oy, fx + ox)
# # # #         else:
# # # #             env_hdg = th

# # # #         # PANIC evade if closest neighbor too near
# # # #         nearest_d, nx, ny = 1e9, None, None
# # # #         for _, (qx, qy, _, _, _) in neighbors.items():
# # # #             d = math.hypot(x - qx, y - qy)
# # # #             if d < nearest_d:
# # # #                 nearest_d, nx, ny = d, qx, qy
# # # #         if nearest_d < PANIC_RADIUS and nx is not None:
# # # #             evasive_until = now + 0.40
# # # #             away = math.atan2(y - ny, x - nx)
# # # #             da = wrap_angle(env_hdg - away)
# # # #             target_h = wrap_angle(away + 0.25 * da)

# # # #         # schedule a new dart if free
# # # #         if (now >= cooldown_until and now >= evasive_until and
# # # #             now >= next_dart_at and now >= dart_until):
# # # #             target_h = choose_safe_heading(x, y, th, neighbors)
# # # #             aim_until      = now + AIM_TIME
# # # #             dart_until     = aim_until + DART_TIME
# # # #             cooldown_until = dart_until + COOLDOWN
# # # #             next_dart_at   = cooldown_until + random.uniform(JITTER_LO, JITTER_HI)
# # # #             robot.set_led(0, 180, 80)  # mint during dart cycle

# # # #         # control mode priority: evade > dart > aim > idle
# # # #         if now < evasive_until:
# # # #             err = wrap_angle(target_h - th)
# # # #             fwd = FWD_AIM
# # # #         elif now < aim_until:
# # # #             err = wrap_angle(target_h - th)
# # # #             fwd = FWD_AIM
# # # #         elif now < dart_until:
# # # #             err = wrap_angle(target_h - th)
# # # #             fwd = FWD_JITTER
# # # #         else:
# # # #             err = wrap_angle(env_hdg - th)
# # # #             fwd = max(FWD_MIN, 0.45)

# # # #         if b == 1:
# # # #             fwd *= 0.75
# # # #         if fwd < FWD_MIN:
# # # #             fwd = FWD_MIN

# # # #         left, right = heading_to_wheels(err, fwd, lastL, lastR)
# # # #         lastL, lastR = left, right
# # # #         robot.set_vel(left, right)

# # # #         if now - last_log > 2.0:
# # # #             state = ("evade" if now < evasive_until else
# # # #                      "dart"  if now < dart_until else
# # # #                      "aim"   if now < aim_until else
# # # #                      "idle")
# # # #             logw(f"[SIM jitter_p2p] id={vid} n={len(neighbors)} state={state}")
# # # #             last_log = now

# # # #         robot.delay(DT_MS)


# # # # -*- coding: utf-8 -*-
# # # # SPIRAL_FLOW (CoachBot): winding + unwinding spiral around center
# # # # - Bots orbit center in a ring
# # # # - Each bot's radius breathes in/out with a phase offset
# # # #   => global shape looks like a spiral that winds and unwinds
# # # from __future__ import division
# # # import math
# # # import random

# # # # --- field (meters) ---
# # # X_MIN, X_MAX = -1.2, 1.0
# # # Y_MIN, Y_MAX = -1.4, 2.35

# # # # --- center (same as your other scripts) ---
# # # CX = (X_MIN + X_MAX) / 2.0   # -0.1
# # # CY = (Y_MIN + Y_MAX) / 2.0   #  0.475

# # # # --- dancer bubble (if you want to avoid center region, optional) ---
# # # FEET = 0.3048
# # # OBST_DIAM_FT = 1.0
# # # OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
# # # OBST_MARGIN  = 0.03
# # # SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN

# # # # --- ring / spiral geometry ---
# # # R_IN   = SAFE_BUBBLE + 0.20   # inner radius of motion
# # # R_OUT  = SAFE_BUBBLE + 0.55   # outer radius of motion
# # # R_MID  = 0.5 * (R_IN + R_OUT)

# # # # how strongly we pull toward the instantaneous desired radius
# # # K_R         = 1.2
# # # RADIAL_MAX  = 0.20

# # # # tangential speed (how fast around the circle)
# # # V_TANGENT   = 0.28   # m/s-ish in field coordinates

# # # # spiral breathing:
# # # SPIRAL_AMP    = 0.18   # amplitude of radius oscillation
# # # SPIRAL_OMEGA  = 0.30   # rad/s (how fast it winds/unwinds)
# # # PHASE_STEP    = 0.9    # per-virtual-id phase offset

# # # # optional global drift (moves whole spiral slowly)
# # # DRIFT_X = 0.0         # e.g. 0.03 to drift right
# # # DRIFT_Y = 0.02        # small upward drift

# # # # --- boundary softness (walls only) ---
# # # SOFT_MARGIN    = 0.08
# # # CRIT_MARGIN    = 0.02
# # # SOFT_MAX_FORCE = 0.35

# # # # --- drive model ---
# # # MAX_WHEEL  = 35
# # # TURN_K     = 2.8
# # # FWD_MIN    = 0.30
# # # FWD_MED    = 0.70
# # # CMD_SMOOTH = 0.22
# # # DT_MS      = 40
# # # EPS        = 1e-3

# # # def clamp(v, lo, hi):
# # #     return lo if v < lo else (hi if v > hi else v)

# # # def wrap(a):
# # #     # wrap to (-pi, pi]
# # #     while a > math.pi:
# # #         a -= 2.0 * math.pi
# # #     while a <= -math.pi:
# # #         a += 2.0 * math.pi
# # #     return a

# # # def safe_pose(robot):
# # #     p = robot.get_pose()
# # #     if p and len(p) >= 3:
# # #         return float(p[0]), float(p[1]), float(p[2])
# # #     return None

# # # def soft_boundary_force(x, y):
# # #     fx = fy = 0.0
# # #     if x < X_MIN + SOFT_MARGIN:
# # #         fx += SOFT_MAX_FORCE * (1.0 - (x - X_MIN) / SOFT_MARGIN)
# # #     elif x > X_MAX - SOFT_MARGIN:
# # #         fx -= SOFT_MAX_FORCE * (1.0 - (X_MAX - x) / SOFT_MARGIN)
# # #     if y < Y_MIN + SOFT_MARGIN:
# # #         fy += SOFT_MAX_FORCE * (1.0 - (y - Y_MIN) / SOFT_MARGIN)
# # #     elif y > Y_MAX - SOFT_MARGIN:
# # #         fy -= SOFT_MAX_FORCE * (1.0 - (Y_MAX - y) / SOFT_MARGIN)
# # #     return fx, fy

# # # def boundary_state(x, y):
# # #     if (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
# # #         y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN):
# # #         return 2
# # #     if (x < X_MIN + SOFT_MARGIN or x > X_MAX - SOFT_MARGIN or
# # #         y < Y_MIN + SOFT_MARGIN or y > Y_MAX - SOFT_MARGIN):
# # #         return 1
# # #     return 0

# # # def usr(robot):
# # #     robot.delay(400)

# # #     # ID + per-bot phase
# # #     try:
# # #         vid = int(robot.virtual_id())
# # #     except:
# # #         vid = 0

# # #     random.seed((vid * 1103515245) & 0xFFFFFFFF)
# # #     phase0 = (vid % 16) * PHASE_STEP

# # #     print("Robot %d starting SPIRAL_FLOW choreography" % vid)

# # #     lastL = 0
# # #     lastR = 0

# # #     # wake localization a bit
# # #     robot.set_vel(20, 20)
# # #     robot.delay(150)

# # #     try:
# # #         while True:
# # #             pose = safe_pose(robot)
# # #             if not pose:
# # #                 robot.set_vel(0, 0)
# # #                 robot.delay(DT_MS)
# # #                 continue

# # #             x, y, th = pose
# # #             now = robot.get_clock()

# # #             # --- boundary LEDs / safety ---
# # #             b = boundary_state(x, y)
# # #             if b == 2:
# # #                 robot.set_led(100, 0, 0)
# # #                 robot.set_vel(0, 0)
# # #                 robot.delay(DT_MS)
# # #                 continue
# # #             elif b == 1:
# # #                 robot.set_led(100, 60, 0)  # amber near wall
# # #             else:
# # #                 robot.set_led(40, 0, 80)   # purple spiral vibe

# # #             # --- polar coords around center ---
# # #             rx = x - CX
# # #             ry = y - CY
# # #             r  = math.hypot(rx, ry)
# # #             if r < 1e-6:
# # #                 urx, ury = 1.0, 0.0
# # #             else:
# # #                 urx, ury = rx / r, ry / r   # radial unit vector
# # #             utx, uty = -ury, urx           # CCW tangential unit vector

# # #             # --- spiral breathing: target radius over time ---
# # #             # r_des oscillates between R_IN and R_OUT, phase-shifted per bot
# # #             r_des = R_MID + SPIRAL_AMP * math.sin(SPIRAL_OMEGA * now + phase0)
# # #             r_des = clamp(r_des, R_IN, R_OUT)

# # #             # radial velocity to chase r_des
# # #             radial_err = r_des - r
# # #             vr = clamp(K_R * radial_err, -RADIAL_MAX, RADIAL_MAX)

# # #             # tangential velocity: constant CCW
# # #             vt = V_TANGENT

# # #             # optional global drift so the whole spiral slowly moves
# # #             vx = vr * urx + vt * utx + DRIFT_X
# # #             vy = vr * ury + vt * uty + DRIFT_Y

# # #             # soft wall pushback
# # #             bfx, bfy = soft_boundary_force(x, y)
# # #             vx += bfx
# # #             vy += bfy

# # #             # if we somehow stall numerically, nudge along tangential
# # #             if abs(vx) < EPS and abs(vy) < EPS:
# # #                 vx += 0.05 * utx
# # #                 vy += 0.05 * uty

# # #             # --- map to heading / wheels ---
# # #             hdg = math.atan2(vy, vx)
# # #             err = wrap(hdg - th)

# # #             # forward speed: bigger when well-aligned
# # #             ae = abs(err)
# # #             if ae < 0.5:
# # #                 fwd = FWD_MED
# # #             elif ae < 1.2:
# # #                 fwd = 0.6 * FWD_MED
# # #             else:
# # #                 fwd = 0.4 * FWD_MED

# # #             # slow slightly near walls
# # #             if b == 1:
# # #                 fwd *= 0.75

# # #             if fwd < FWD_MIN:
# # #                 fwd = FWD_MIN

# # #             turn = clamp(TURN_K * err, -1.5, 1.5)

# # #             lcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)),
# # #                          -MAX_WHEEL, MAX_WHEEL)
# # #             rcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)),
# # #                          -MAX_WHEEL, MAX_WHEEL)

# # #             # simple slew + smoothing
# # #             if lcmd > lastL + 10: lcmd = lastL + 10
# # #             if lcmd < lastL - 10: lcmd = lastL - 10
# # #             if rcmd > lastR + 10: rcmd = lastR + 10
# # #             if rcmd < lastR - 10: rcmd = lastR - 10

# # #             left  = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
# # #             right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
# # #             lastL, lastR = left, right

# # #             robot.set_vel(left, right)
# # #             robot.delay(DT_MS)

# # #     except Exception as e:
# # #         try:
# # #             robot.set_vel(0, 0)
# # #             robot.set_led(100, 0, 0)
# # #         except:
# # #             pass
# # #         print("[spiral_flow] ERROR id=%d: %s" % (vid, repr(e)))
# # #         raise
# # #     finally:
# # #         try:
# # #             robot.set_vel(0, 0)
# # #         except:
# # #             pass


# # # -*- coding: utf-8 -*-
# # # SPIRAL_FLOW_MOVING_CENTER (CoachBot)
# # # Bots orbit a MOVING center point:
# # #   - Center moves straight upward at constant speed (no sinusoid)
# # #   - Each bot's radius breathes in/out with a phase offset
# # #   - Global pattern looks like a spiral that winds & unwinds around this moving point
# # from __future__ import division
# # import math
# # import random

# # # ---------- Field (meters) ----------
# # X_MIN, X_MAX = -1.2, 1.0
# # Y_MIN, Y_MAX = -1.4, 2.35

# # # Base center (same as other scripts)
# # CX0 = (X_MIN + X_MAX) / 2.0   # -0.1
# # CY0 = (Y_MIN + Y_MAX) / 2.0   #  0.475

# # # ---------- Optional dancer bubble (NOT used directly here, just for reference) ----------
# # FEET = 0.3048
# # OBST_DIAM_FT = 1.0
# # OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
# # OBST_MARGIN  = 0.03
# # SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN

# # # ---------- Spiral band (radius zone around moving center) ----------
# # R_IN   = SAFE_BUBBLE + 0.20   # inner radius of motion
# # R_OUT  = SAFE_BUBBLE + 0.55   # outer radius of motion
# # R_MID  = 0.5 * (R_IN + R_OUT)

# # K_R         = 1.2     # how strongly to pull toward r_des
# # RADIAL_MAX  = 0.20    # clamp radial velocity

# # V_TANGENT   = 0.28    # speed along the ring (CCW)

# # # Spiral "breathing" parameters (per robot phase)
# # SPIRAL_AMP    = 0.18   # how far in/out they oscillate
# # SPIRAL_OMEGA  = 0.30   # rad/s: how fast the spiral winds/unwinds
# # PHASE_STEP    = 0.9    # phase offset per virtual_id

# # # ---------- Moving center parameters (STRAIGHT LINE ONLY) ----------
# # CENTER_DRIFT_Y   = 0.05   # m/s upward drift speed
# # CENTER_MARGIN    = 0.25   # keep center off walls

# # # ---------- Boundary softness (walls only) ----------
# # SOFT_MARGIN    = 0.08
# # CRIT_MARGIN    = 0.02
# # SOFT_MAX_FORCE = 0.35

# # # ---------- Drive model ----------
# # MAX_WHEEL  = 35
# # TURN_K     = 2.8
# # FWD_MIN    = 0.30
# # FWD_MED    = 0.70
# # CMD_SMOOTH = 0.22
# # DT_MS      = 40
# # EPS        = 1e-3

# # def clamp(v, lo, hi):
# #     return lo if v < lo else (hi if v > hi else v)

# # def wrap(a):
# #     # wrap to (-pi, pi]
# #     while a > math.pi:
# #         a -= 2.0 * math.pi
# #     while a <= -math.pi:
# #         a += 2.0 * math.pi
# #     return a

# # def safe_pose(robot):
# #     p = robot.get_pose()
# #     if p and len(p) >= 3:
# #         return float(p[0]), float(p[1]), float(p[2])
# #     return None

# # def soft_boundary_force(x, y):
# #     fx = fy = 0.0
# #     if x < X_MIN + SOFT_MARGIN:
# #         fx += SOFT_MAX_FORCE * (1.0 - (x - X_MIN) / SOFT_MARGIN)
# #     elif x > X_MAX - SOFT_MARGIN:
# #         fx -= SOFT_MAX_FORCE * (1.0 - (X_MAX - x)/SOFT_MARGIN)
# #     if y < Y_MIN + SOFT_MARGIN:
# #         fy += SOFT_MAX_FORCE * (1.0 - (y - Y_MIN)/SOFT_MARGIN)
# #     elif y > Y_MAX - SOFT_MARGIN:
# #         fy -= SOFT_MAX_FORCE * (1.0 - (Y_MAX - y)/SOFT_MARGIN)
# #     return fx, fy

# # def boundary_state(x, y):
# #     if (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
# #         y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN):
# #         return 2
# #     if (x < X_MIN + SOFT_MARGIN or x > X_MAX - SOFT_MARGIN or
# #         y < Y_MIN + SOFT_MARGIN or y > Y_MAX - SOFT_MARGIN):
# #         return 1
# #     return 0

# # def moving_center(now):
# #     """
# #     Moving center in a *straight vertical line*:
# #     - x is fixed at CX0
# #     - y increases linearly with time at CENTER_DRIFT_Y
# #     """
# #     cx = CX0
# #     cy = CY0 + CENTER_DRIFT_Y * now

# #     cx = clamp(cx, X_MIN + CENTER_MARGIN, X_MAX - CENTER_MARGIN)
# #     cy = clamp(cy, Y_MIN + CENTER_MARGIN, Y_MAX - CENTER_MARGIN)
# #     return cx, cy

# # def usr(robot):
# #     robot.delay(400)

# #     # ID + per-bot spiral phase
# #     try:
# #         vid = int(robot.virtual_id())
# #     except:
# #         vid = 0

# #     random.seed((vid * 1103515245) & 0xFFFFFFFF)
# #     phase0 = (vid % 16) * PHASE_STEP

# #     print("Robot %d starting SPIRAL_FLOW_MOVING_CENTER choreography (straight-line center)" % vid)

# #     lastL = 0
# #     lastR = 0

# #     # wake localization
# #     robot.set_vel(20, 20)
# #     robot.delay(150)

# #     try:
# #         while True:
# #             pose = safe_pose(robot)
# #             if not pose:
# #                 robot.set_vel(0, 0)
# #                 robot.delay(DT_MS)
# #                 continue

# #             x, y, th = pose
# #             now = robot.get_clock()

# #             # --- moving center (straight vertical drift) ---
# #             cx, cy = moving_center(now)

# #             # --- boundary LEDs / safety ---
# #             b = boundary_state(x, y)
# #             if b == 2:
# #                 robot.set_led(100, 0, 0)
# #                 robot.set_vel(0, 0)
# #                 robot.delay(DT_MS)
# #                 continue
# #             elif b == 1:
# #                 robot.set_led(100, 60, 0)  # amber near wall
# #             else:
# #                 robot.set_led(60, 0, 90)   # purple spiral vibe

# #             # --- polar coordinates around *moving* center ---
# #             rx = x - cx
# #             ry = y - cy
# #             r  = math.hypot(rx, ry)
# #             if r < 1e-6:
# #                 urx, ury = 1.0, 0.0
# #             else:
# #                 urx, ury = rx / r, ry / r   # radial unit vector
# #             utx, uty = -ury, urx           # CCW tangential unit vector

# #             # --- spiral breathing: time-varying desired radius per bot ---
# #             r_des = R_MID + SPIRAL_AMP * math.sin(SPIRAL_OMEGA * now + phase0)
# #             r_des = clamp(r_des, R_IN, R_OUT)

# #             radial_err = r_des - r
# #             vr = clamp(K_R * radial_err, -RADIAL_MAX, RADIAL_MAX)
# #             vt = V_TANGENT

# #             # base velocity in world frame (around moving center)
# #             vx = vr * urx + vt * utx
# #             vy = vr * ury + vt * uty

# #             # soft wall pushback in world frame
# #             bfx, bfy = soft_boundary_force(x, y)
# #             vx += bfx
# #             vy += bfy

# #             # avoid stall
# #             if abs(vx) < EPS and abs(vy) < EPS:
# #                 vx += 0.05 * utx
# #                 vy += 0.05 * uty

# #             # --- map desired velocity to heading / wheels ---
# #             hdg = math.atan2(vy, vx)
# #             err = wrap(hdg - th)

# #             ae = abs(err)
# #             if ae < 0.5:
# #                 fwd = FWD_MED
# #             elif ae < 1.2:
# #                 fwd = 0.6 * FWD_MED
# #             else:
# #                 fwd = 0.4 * FWD_MED

# #             if b == 1:
# #                 fwd *= 0.75
# #             if fwd < FWD_MIN:
# #                 fwd = FWD_MIN

# #             turn = clamp(TURN_K * err, -1.5, 1.5)

# #             lcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)),
# #                          -MAX_WHEEL, MAX_WHEEL)
# #             rcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)),
# #                          -MAX_WHEEL, MAX_WHEEL)

# #             # simple slew + smoothing
# #             if lcmd > lastL + 10: lcmd = lastL + 10
# #             if lcmd < lastL - 10: lcmd = lastL - 10
# #             if rcmd > lastR + 10: rcmd = lastR + 10
# #             if rcmd < lastR - 10: rcmd = lastR - 10

# #             left  = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
# #             right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
# #             lastL, lastR = left, right

# #             robot.set_vel(left, right)
# #             robot.delay(DT_MS)

# #     except Exception as e:
# #         try:
# #             robot.set_vel(0, 0)
# #             robot.set_led(100, 0, 0)
# #         except:
# #             pass
# #         print("[spiral_flow_moving_center] ERROR id=%d: %s" % (vid, repr(e)))
# #         raise
# #     finally:
# #         try:
# #             robot.set_vel(0, 0)
# #         except:
# #             pass


# # -*- coding: utf-8 -*-
# # SPIRAL_UPWARD (CoachBot): big dramatic inward/outward spiral + CW rotation + upward drift
# # - One ring
# # - Staggered phases
# # - No sinusoidal float, no lateral wave
# # - Decentralized (neighbors for safety/spacing)

# from __future__ import division
# import math, struct, random

# # ---- Arena ----
# X_MIN, X_MAX = -1.2, 1.0
# Y_MIN, Y_MAX = -1.4, 2.35

# # Upward-moving center
# CX0, CY0 = (-0.1, -1.2)      # starting center
# UP_SPEED = 0.11              # upward migration (tune)

# # ---- Spiral parameters ----
# R0       = 0.42              # baseline radius
# AMP      = 0.22              # big spiral amplitude
# FREQ     = 0.22              # Hz breathing rate
# ANG_SPEED = -1.4             # CW rotation (negative = CW)
# PHASE_STEP = 0.7             # staggered breathing per-robot

# # ---- Control ----
# MAX_WHEEL = 35
# TURN_K    = 2.8
# CMD_SMOOTH = 0.18
# VEL_SLEW   = 10
# DT_MS      = 40

# K_R = 1.6                     # radial correction gain
# SEP_RADIUS = 0.28
# K_SEP = 0.20
# NEIGH_RADIUS = 0.75
# K_ALI = 0.14
# K_COH = 0.10

# # ---- Heartbeats ----
# HB_FMT = 'fffffi'
# HB_BYTES = struct.calcsize(HB_FMT)
# HB_DT   = 0.12
# STALE_S = 0.7

# def clamp(v, lo, hi): return lo if v < lo else (hi if v > hi else v)

# def wrap(a):
#     while a > math.pi: a -= 2*math.pi
#     while a <= -math.pi: a += 2*math.pi
#     return a

# def safe_pose(robot):
#     p = robot.get_pose()
#     if p and len(p) >= 3:
#         return float(p[0]), float(p[1]), float(p[2])
#     return None

# def soft_boundary(x, y):
#     fx = fy = 0.0
#     SM = 0.08
#     MF = 0.35
#     if x < X_MIN + SM: fx += MF * (1 - (x - X_MIN)/SM)
#     elif x > X_MAX - SM: fx -= MF * (1 - (X_MAX - x)/SM)
#     if y < Y_MIN + SM: fy += MF * (1 - (y - Y_MIN)/SM)
#     elif y > Y_MAX - SM: fy -= MF * (1 - (Y_MAX - y)/SM)
#     return fx, fy

# def usr(robot):
#     robot.delay(300)

#     try:
#         vid = int(robot.virtual_id())
#     except:
#         vid = 0

#     random.seed(vid*1103515245 & 0xFFFFFFFF)

#     print("Robot %d starting SPIRAL_UPWARD" % vid)

#     phase0 = (vid % 20) * PHASE_STEP

#     neighbors = {}
#     last_seen = {}
#     last_hb = -1e9
#     lastL = lastR = 0

#     robot.set_vel(15,15)
#     robot.delay(200)

#     while True:
#         pose = safe_pose(robot)
#         if not pose:
#             robot.set_vel(0,0); robot.delay(DT_MS); continue

#         x, y, th = pose
#         now = robot.get_clock()

#         # ---- Upward-moving center ----
#         cx = CX0
#         cy = CY0 + UP_SPEED * now

#         dx = x - cx
#         dy = y - cy
#         r  = math.hypot(dx, dy)
#         theta = math.atan2(dy, dx)

#         # ---- Heartbeats ----
#         if now - last_hb >= HB_DT:
#             try:
#                 vx_hb = 0.0; vy_hb = 0.0
#                 robot.send_msg(struct.pack(HB_FMT, x, y, th, vx_hb, vy_hb, vid))
#             except:
#                 pass
#             last_hb = now

#         msgs = robot.recv_msg()
#         if msgs:
#             for m in msgs:
#                 try:
#                     nx, ny, nth, nvx, nvy, nid = struct.unpack(HB_FMT, m[:HB_BYTES])
#                     if nid != vid:
#                         neighbors[nid] = (nx, ny, nth, now)
#                         last_seen[nid] = now
#                 except:
#                     pass

#         cutoff = now - STALE_S
#         for nid in list(neighbors.keys()):
#             if last_seen.get(nid,0) < cutoff:
#                 neighbors.pop(nid,None)
#                 last_seen.pop(nid,None)

#         # ---- Spiral target radius ----
#         target_r = R0 + AMP * math.sin(2*math.pi*FREQ*now + phase0)

#         # Radial velocity
#         vr = K_R * (target_r - r)

#         # Angular velocity
#         vt = ANG_SPEED * target_r

#         # ---- Convert polar → Cartesian ----
#         ct = math.cos(theta)
#         st = math.sin(theta)

#         vx = vr*ct - vt*st
#         vy = vr*st + vt*ct

#         # ---- Add upward translation ----
#         vy += UP_SPEED

#         # ---- Add safety & flocking ----
#         # Separation
#         for nid,(nx,ny,nth,t_seen) in neighbors.items():
#             dx2 = x - nx
#             dy2 = y - ny
#             d2 = dx2*dx2 + dy2*dy2
#             if d2 < SEP_RADIUS*SEP_RADIUS:
#                 d = math.sqrt(d2)
#                 if d > 1e-6:
#                     rep = K_SEP*(SEP_RADIUS - d)/SEP_RADIUS
#                     vx += rep*(dx2/d)
#                     vy += rep*(dy2/d)

#         # Cohesion + alignment
#         ax=ay=cx2=cy2=0.0
#         count=0
#         for nid,(nx,ny,nth,t_s) in neighbors.items():
#             cx2 += nx; cy2 += ny
#             ax  += math.cos(nth); ay += math.sin(nth)
#             count += 1

#         if count>0:
#             cx2/=count; cy2/=count
#             vx += K_COH*(cx2 - x)
#             vy += K_COH*(cy2 - y)
#             ah = math.atan2(ay,ax)
#             vx += K_ALI*math.cos(ah)
#             vy += K_ALI*math.sin(ah)

#         # Boundary
#         bfx, bfy = soft_boundary(x,y)
#         vx += bfx; vy += bfy

#         # ---- Wheels ----
#         hdg = math.atan2(vy,vx)
#         err = wrap(hdg - th)
#         fwd = 0.65
#         turn = clamp(TURN_K*err, -1.3, 1.3)

#         lcmd = clamp(int(MAX_WHEEL*(fwd - 0.8*turn)), -MAX_WHEEL,MAX_WHEEL)
#         rcmd = clamp(int(MAX_WHEEL*(fwd + 0.8*turn)), -MAX_WHEEL,MAX_WHEEL)

#         if lcmd > lastL + VEL_SLEW: lcmd = lastL + VEL_SLEW
#         if lcmd < lastL - VEL_SLEW: lcmd = lastL - VEL_SLEW
#         if rcmd > lastR + VEL_SLEW: rcmd = lastR + VEL_SLEW
#         if rcmd < lastR - VEL_SLEW: rcmd = lastR - VEL_SLEW

#         L = int((1-CMD_SMOOTH)*lcmd + CMD_SMOOTH*lastL)
#         R = int((1-CMD_SMOOTH)*rcmd + CMD_SMOOTH*lastR)
#         lastL,lastR = L,R

#         robot.set_led(70,10,85)
#         robot.set_vel(L,R)
#         robot.delay(DT_MS)

# -*- coding: utf-8 -*-
# SPIRAL_UPWARD + ORCA-Lite
#
# Dance-like spiral around a moving point.
# - CW rotation
# - Breathing radius (inward/outward spiral)
# - Upward drift of the center
# - ORCA-style avoidance prevents clumping
#
from __future__ import division
import math, struct, random

# ---- Arena ----
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

# ---- Spiral center motion ----
CX0, CY0 = (-0.1, -1.20)     # Start below the arena
UP_SPEED = 0.11              # Center drift upward

# ---- Spiral breathing ----
R0   = 0.42                  # baseline ring radius
AMP  = 0.22                  # breathing amplitude
FREQ = 0.22                  # breathing frequency (Hz)
ANG_SPEED = -1.4             # CW rotation (negative = clockwise)
PHASE_STEP = 0.7             # stagger breath timing

# ---- Control ----
MAX_WHEEL = 35
TURN_K    = 2.8
CMD_SMOOTH = 0.18
VEL_SLEW   = 10
DT_MS = 40
FWD = 0.65

# ---- Flocking ----
SEP_RADIUS = 0.28
K_SEP = 0.22
NEIGH_RADIUS = 0.80
K_ALI = 0.12
K_COH = 0.10

# ---- ORCA-lite ----
TAU = 0.8                # time horizon (sec)
AVOID_GAIN = 0.55        # strength of ORCA steering

# ---- Heartbeats ----
HB_FMT = 'fffffi'
HB_BYTES = struct.calcsize(HB_FMT)
HB_DT   = 0.12
STALE_S = 0.7

# ---- Helpers ----
def clamp(v, lo, hi): return lo if v < lo else (hi if v > hi else v)

def wrap(a):
    while a > math.pi: a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a

def safe_pose(robot):
    p = robot.get_pose()
    if p and len(p)>=3:
        return float(p[0]), float(p[1]), float(p[2])
    return None

def soft_boundary(x, y):
    fx = fy = 0.0
    SM = 0.08
    MF = 0.35
    if x < X_MIN + SM: fx += MF*(1 - (x - X_MIN)/SM)
    elif x > X_MAX - SM: fx -= MF*(1 - (X_MAX - x)/SM)
    if y < Y_MIN + SM: fy += MF*(1 - (y - Y_MIN)/SM)
    elif y > Y_MAX - SM: fy -= MF*(1 - (Y_MAX - y)/SM)
    return fx, fy

# ---- ORCA-lite avoidance ----
def orca_correction(x, y, vx, vy, neighbors):
    ax = ay = 0.0
    for (nx, ny, nth, t_last) in neighbors.values():
        dx = nx - x
        dy = ny - y
        dist2 = dx*dx + dy*dy
        if dist2 < 1e-6: 
            continue
        dist = math.sqrt(dist2)

        # Relative velocity (neighbor assumed stationary)
        rvx = vx
        rvy = vy

        # Predict collision time
        closing_speed = -(rvx*dx + rvy*dy) / (dist + 1e-6)
        if closing_speed <= 0:
            continue

        time_to_collision = dist / closing_speed

        if time_to_collision < TAU:
            # minimum adjustment direction (sideways)
            nxu = dx/dist
            nyu = dy/dist
            # perpendicular steering
            ax += -nyu * AVOID_GAIN
            ay += nxu * AVOID_GAIN

    return ax, ay


# ---- Main ----
def usr(robot):
    robot.delay(300)

    try:
        vid = int(robot.virtual_id())
    except:
        vid = 0

    random.seed(vid*1103515245 & 0xFFFFFFFF)
    phase0 = (vid % 16) * PHASE_STEP

    neighbors = {}
    last_seen = {}
    last_hb = -1e9
    lastL = lastR = 0

    robot.set_vel(15,15)
    robot.delay(200)

    print("Robot %d: SPIRAL_UPWARD + ORCA" % vid)

    while True:
        pose = safe_pose(robot)
        if not pose:
            robot.set_vel(0,0); robot.delay(DT_MS); continue

        x, y, th = pose
        now = robot.get_clock()

        # ---- Moving center ----
        cx = CX0
        cy = CY0 + UP_SPEED * now

        # ---- Relative polar coords ----
        dx = x - cx
        dy = y - cy
        r  = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)

        # ---- Heartbeats ----
        if now - last_hb >= HB_DT:
            robot.send_msg(struct.pack(HB_FMT, x, y, th, 0.0, 0.0, vid))
            last_hb = now

        msgs = robot.recv_msg()
        if msgs:
            for m in msgs:
                try:
                    nx, ny, nth, nvx, nvy, nid = struct.unpack(HB_FMT, m[:HB_BYTES])
                    if nid != vid:
                        neighbors[nid] = (nx, ny, nth, now)
                        last_seen[nid] = now
                except:
                    pass

        cutoff = now - STALE_S
        for nid in list(neighbors.keys()):
            if last_seen[nid] < cutoff:
                neighbors.pop(nid, None)
                last_seen.pop(nid, None)

        # ---- Breathing radius ----
        target_r = R0 + AMP * math.sin(2*math.pi*FREQ*now + phase0)
        vr = 1.4 * (target_r - r)      # radial motion

        # ---- Angular spiral ----
        vt = ANG_SPEED * max(target_r, 0.20)

        ct = math.cos(theta)
        st = math.sin(theta)

        vx = vr*ct - vt*st
        vy = vr*st + vt*ct

        # ---- Add upward drift ----
        vy += UP_SPEED

        # ---- ORCA-lite avoidance ----
        ax, ay = orca_correction(x, y, vx, vy, neighbors)
        vx += ax
        vy += ay

        # ---- Light flocking ----
        cx2 = cy2 = ax2 = ay2 = 0
        count = 0
        for (nx, ny, nth, t_s) in neighbors.values():
            cx2 += nx; cy2 += ny
            ax2 += math.cos(nth); ay2 += math.sin(nth)
            count += 1
        if count > 0:
            cx2 /= count; cy2 /= count
            vx += K_COH*(cx2 - x)
            vy += K_COH*(cy2 - y)
            ah = math.atan2(ay2, ax2)
            vx += K_ALI*math.cos(ah)
            vy += K_ALI*math.sin(ah)

        # ---- Walls ----
        bx, by = soft_boundary(x, y)
        vx += bx; vy += by

        # ---- Wheels ----
        hdg = math.atan2(vy, vx)
        err = wrap(hdg - th)
        turn = clamp(TURN_K * err, -1.3, 1.3)

        lcmd = clamp(int(MAX_WHEEL*(FWD - 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
        rcmd = clamp(int(MAX_WHEEL*(FWD + 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)

        if lcmd > lastL + VEL_SLEW: lcmd = lastL + VEL_SLEW
        if lcmd < lastL - VEL_SLEW: lcmd = lastL - VEL_SLEW
        if rcmd > lastR + VEL_SLEW: rcmd = lastR + VEL_SLEW
        if rcmd < lastR - VEL_SLEW: rcmd = lastR - VEL_SLEW

        L = int((1-CMD_SMOOTH)*lcmd + CMD_SMOOTH*lastL)
        R = int((1-CMD_SMOOTH)*rcmd + CMD_SMOOTH*lastR)
        lastL, lastR = L, R

        robot.set_led(80, 20, 90)
        robot.set_vel(L, R)
        robot.delay(DT_MS)


