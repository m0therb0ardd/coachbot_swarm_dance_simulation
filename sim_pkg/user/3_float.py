# # # -*- coding: utf-8 -*-
# # # FLOAT (SIM): light • sustained • indirect
# # # Smooth, curving motion with low jerk and gentle leftward migration.
# # import math, struct, random, os

# # # ----------------- Logging (sim + hardware) -----------------
# # LOG = None   # global log handle

# # def init_log():
# #     """
# #     Try to open experiment_log.txt in a hardware-like way.
# #     If it fails (e.g., sim with no FS), we just fall back to logw only.
# #     """
# #     global LOG
# #     if LOG is not None:
# #         return
# #     try:
# #         # line-buffered like your FLOAT HW script
# #         LOG = open("experiment_log.txt", "a", 1)
# #     except Exception:
# #         LOG = None

# # def logw(msg):
# #     """
# #     Write to log file (if available) AND logw to stdout.
# #     Safe in both sim and hardware.
# #     """
# #     if not isinstance(msg, str):
# #         msg = str(msg)
# #     line = msg if msg.endswith("\n") else msg + "\n"

# #     # Log file (hardware) if available
# #     if LOG is not None:
# #         try:
# #             LOG.write(line)
# #             LOG.flush()
# #             os.fsync(LOG.fileno())
# #         except Exception:
# #             pass

# #     # Always also logw (sim / console)
# #     print(line.rstrip("\n"))


# # # --- field & obstacle (meters) ---
# # X_MIN, X_MAX = -1.2, 1.0
# # Y_MIN, Y_MAX = -1.4, 2.35
# # FEET = 0.3048
# # OBST_DIAM_FT = 1.0
# # OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
# # OBST_MARGIN  = 0.03
# # SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
# # OBST_CX, OBST_CY = (-0.1, 0.475)

# # # --- control/loop ---
# # MAX_WHEEL   = 35
# # TURN_K      = 2.2
# # FWD_BASE    = 0.65      # nominal forward factor
# # FWD_MIN     = 0.35
# # DT_MS       = 40
# # CMD_SMOOTH  = 0.35      # higher smoothing → lower jerk
# # VEL_SLEW    = 8         # wheel cmd change limit per step (accel cap)

# # # --- float style gains ---
# # K_MIG   = 0.08          # gentle leftward drift
# # K_SEP   = 0.20          # spacing
# # K_ALI   = 0.18          # match headings (fluid flock)
# # K_COH   = 0.10          # gentle cohesion
# # CURVE_NOISE = 0.20      # adds “indirect” softly varying heading

# # SEP_RADIUS   = 0.26
# # NEIGH_RADIUS = 0.75

# # # --- boundary softness ---
# # SOFT_MARGIN = 0.08
# # CRIT_MARGIN = 0.02
# # SOFT_MAX_F  = 0.35

# # # --- P2P heartbeats ---
# # HB_FMT   = 'fffffi'     # x,y,th,vx,vy,id
# # HB_BYTES = struct.calcsize(HB_FMT)
# # HB_DT    = 0.12
# # STALE_S  = 0.7

# # def clamp(v, lo, hi): return lo if v < lo else (hi if v > hi else v)
# # def wrap(a):
# #     while a >  math.pi: a -= 2*math.pi
# #     while a <= -math.pi: a += 2*math.pi
# #     return a

# # def soft_boundary_force(x,y):
# #     fx=fy=0.0
# #     if x < X_MIN+SOFT_MARGIN: fx += SOFT_MAX_F*(1-(x-X_MIN)/SOFT_MARGIN)
# #     elif x > X_MAX-SOFT_MARGIN: fx -= SOFT_MAX_F*(1-(X_MAX-x)/SOFT_MARGIN)
# #     if y < Y_MIN+SOFT_MARGIN: fy += SOFT_MAX_F*(1-(y-Y_MIN)/SOFT_MARGIN)
# #     elif y > Y_MAX-SOFT_MARGIN: fy -= SOFT_MAX_F*(1-(Y_MAX-y)/SOFT_MARGIN)
# #     return fx,fy

# # def soft_obstacle_force(x, y, maxf=0.55, w=0.12):
# #     dx,dy = x-OBST_CX, y-OBST_CY; r=math.hypot(dx,dy)
# #     if r < SAFE_BUBBLE + w:
# #         if r<1e-6: return maxf,0.0
# #         s = max(0.0, (SAFE_BUBBLE+w-r)/w)*maxf
# #         return s*(dx/r), s*(dy/r)
# #     return 0.0,0.0

# # def boundary_state(x,y):
# #     if (x < X_MIN+CRIT_MARGIN or x > X_MAX-CRIT_MARGIN or
# #         y < Y_MIN+CRIT_MARGIN or y > Y_MAX-CRIT_MARGIN): return 2
# #     if (x < X_MIN+SOFT_MARGIN or x > X_MAX-SOFT_MARGIN or
# #         y < Y_MIN+SOFT_MARGIN or y > Y_MAX-SOFT_MARGIN): return 1
# #     return 0

# # def safe_pose(robot):
# #     p = robot.get_pose()
# #     if p and len(p)>=3: return float(p[0]),float(p[1]),float(p[2])
# #     return None

# # def usr(robot):
# #     init_log()
# #     robot.delay(400)
# #     try: vid = int(robot.id())
# #     except: vid = 0
# #     random.seed(vid*1103515245 & 0xFFFFFFFF)

# #     # state
# #     neighbors, last_seen = {}, {}
# #     last_hb = -1e9
# #     lastL = lastR = 0
# #     drift_phase = random.uniform(-math.pi, math.pi)

# #     # wake localization
# #     robot.set_vel(20,20); robot.delay(150)

# #     while True:
# #         pose = safe_pose(robot)
# #         if not pose:
# #             robot.set_vel(0,0); robot.delay(DT_MS); continue
# #         x,y,th = pose
# #         now = robot.get_clock()

# #         # LEDs
# #         b = boundary_state(x,y)
# #         if b==2:
# #             robot.set_led(100,0,0); robot.set_vel(0,0); robot.delay(DT_MS); continue
# #         elif b==1: robot.set_led(100,60,0) # amber
# #         else:      robot.set_led(0,70,80)  # calm teal

# #         # --- heartbeat send (finite diff for vx,vy) ---
# #         if now - last_hb >= HB_DT:
# #             x1,y1,_ = pose; t1 = now
# #             robot.delay(60)
# #             p2 = safe_pose(robot)
# #             if p2:
# #                 x2,y2,th2 = p2; t2 = robot.get_clock()
# #                 dt = max(1e-3, t2-t1)
# #                 vx=(x2-x1)/dt; vy=(y2-y1)/dt
# #                 try: robot.send_msg(struct.pack(HB_FMT, x2,y2,th2,vx,vy,vid))
# #                 except: pass
# #                 last_hb = t2; x,y,th = x2,y2,th2
# #             else:
# #                 last_hb = now

# #         # --- receive ---
# #         for m in (robot.recv_msg() or []):
# #             try:
# #                 nx,ny,nth,nvx,nvy,nid = struct.unpack(HB_FMT, m[:HB_BYTES])
# #                 if int(nid)!=vid:
# #                     neighbors[int(nid)] = (nx,ny,nth,nvx,nvy)
# #                     last_seen[int(nid)] = now
# #             except: pass
# #         # prune
# #         cut = now - STALE_S
# #         for nid in list(neighbors.keys()):
# #             if last_seen.get(nid,0) < cut:
# #                 neighbors.pop(nid,None); last_seen.pop(nid,None)

# #         # --- boidsy float field ---
# #         # environment
# #         ex,ey = soft_boundary_force(x,y)
# #         ox,oy = soft_obstacle_force(x,y)

# #         # migration upward (positive y)
# #         mx,my = 0.0, K_MIG

# #         # neighbor terms
# #         repx=0.0; repy=0.0; cx=cy=0.0; ax=ay=0.0; n=0
# #         for _,(nx,ny,nth,nvx,nvy) in neighbors.items():
# #             dx,dy = x-nx, y-ny
# #             d2 = dx*dx+dy*dy
# #             if d2>1e-9:
# #                 d = math.sqrt(d2)
# #                 if d<SEP_RADIUS:
# #                     s = K_SEP * (SEP_RADIUS - d)/SEP_RADIUS
# #                     repx += s*(dx/d); repy += s*(dy/d)
# #                 if d <= NEIGH_RADIUS:
# #                     cx += nx; cy += ny
# #                     ax += math.cos(nth); ay += math.sin(nth)
# #                     n+=1
# #         cohx=cohy=alx=aly=0.0
# #         if n>0:
# #             cx/=n; cy/=n
# #             cohx = K_COH*(cx-x); cohy = K_COH*(cy-y)
# #             ah = math.atan2(ay,ax)
# #             alx = K_ALI*math.cos(ah); aly = K_ALI*math.sin(ah)

# #         # softly varying curvature for “indirect”
# #         drift_phase += 0.03
# #         curl = CURVE_NOISE*math.sin(drift_phase)

# #         # now: sideways meander in x, clean upward drift in y
# #         curlx = curl
# #         curly = 0.0

# #         vx = ex+ox+mx+repx+cohx+alx+curlx
# #         vy = ey+oy+my+repy+cohy+aly+curly

# #         if abs(vx)<1e-6 and abs(vy)<1e-6: vx = 1e-3

# #         hdg = math.atan2(vy, vx)
# #         err = wrap(hdg - th)

# #         # speed: smooth, sustained
# #         fwd = FWD_BASE
# #         if b==1: fwd *= 0.8
# #         fwd = max(FWD_MIN, fwd)

# #         # map to wheels with *low jerk* (smoothing + slew rate)
# #         turn = clamp(TURN_K*err, -1.2, 1.2)
# #         lcmd = clamp(int(MAX_WHEEL*0.9*(fwd - 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
# #         rcmd = clamp(int(MAX_WHEEL*0.9*(fwd + 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)

# #         # slew limit
# #         if lcmd > lastL + VEL_SLEW: lcmd = lastL + VEL_SLEW
# #         if lcmd < lastL - VEL_SLEW: lcmd = lastL - VEL_SLEW
# #         if rcmd > lastR + VEL_SLEW: rcmd = lastR + VEL_SLEW
# #         if rcmd < lastR - VEL_SLEW: rcmd = lastR - VEL_SLEW

# #         left  = int((1-CMD_SMOOTH)*lcmd + CMD_SMOOTH*lastL)
# #         right = int((1-CMD_SMOOTH)*rcmd + CMD_SMOOTH*lastR)
# #         lastL, lastR = left, right
# #         robot.set_vel(left, right)

# #         robot.delay(DT_MS)






# # # -*- coding: utf-8 -*-
# # # FLOAT (CoachBot): light, sustained, indirect
# # # Smooth, curving motion with low jerk and gentle upward migration.
# # from __future__ import division
# # import math
# # import struct
# # import random

# # # --- field (meters) ---
# # X_MIN, X_MAX = -1.2, 1.0
# # Y_MIN, Y_MAX = -1.4, 2.35

# # # --- (unused now) dancer obstacle params kept for reference ---
# # FEET = 0.3048
# # OBST_DIAM_FT = 1.0
# # OBST_RADIUS = 0.5 * OBST_DIAM_FT * FEET
# # OBST_MARGIN = 0.03
# # SAFE_BUBBLE = OBST_RADIUS + OBST_MARGIN
# # OBST_CX, OBST_CY = (-0.1, 0.475)

# # # --- control/loop ---
# # MAX_WHEEL = 35
# # TURN_K = 2.2
# # FWD_BASE = 0.65  # nominal forward factor
# # FWD_MIN = 0.35
# # DT_MS = 40
# # CMD_SMOOTH = 0.35  # higher smoothing = lower jerk
# # VEL_SLEW = 8       # wheel cmd change limit per step (accel cap)

# # # --- float style gains ---
# # K_MIG = 0.08       # gentle upward drift (positive y)
# # K_SEP = 0.20       # spacing
# # K_ALI = 0.18       # match headings (fluid flock)
# # K_COH = 0.10       # gentle cohesion
# # CURVE_NOISE = 0.20 # adds "indirect" softly varying heading

# # SEP_RADIUS = 0.26
# # NEIGH_RADIUS = 0.75

# # # --- boundary softness (walls only) ---
# # SOFT_MARGIN = 0.08
# # CRIT_MARGIN = 0.02
# # SOFT_MAX_F = 0.35

# # # --- P2P heartbeats ---
# # HB_FMT = 'fffffi'  # x,y,th,vx,vy,id
# # HB_BYTES = struct.calcsize(HB_FMT)
# # HB_DT = 0.12
# # STALE_S = 0.7

# # def clamp(v, lo, hi):
# #     return lo if v < lo else (hi if v > hi else v)

# # def wrap(a):
# #     while a > math.pi:
# #         a -= 2*math.pi
# #     while a <= -math.pi:
# #         a += 2*math.pi
# #     return a

# # def soft_boundary_force(x, y):
# #     fx = fy = 0.0
# #     if x < X_MIN + SOFT_MARGIN:
# #         fx += SOFT_MAX_F * (1 - (x - X_MIN) / SOFT_MARGIN)
# #     elif x > X_MAX - SOFT_MARGIN:
# #         fx -= SOFT_MAX_F * (1 - (X_MAX - x) / SOFT_MARGIN)
# #     if y < Y_MIN + SOFT_MARGIN:
# #         fy += SOFT_MAX_F * (1 - (y - Y_MIN) / SOFT_MARGIN)
# #     elif y > Y_MAX - SOFT_MARGIN:
# #         fy -= SOFT_MAX_F * (1 - (Y_MAX - y) / SOFT_MARGIN)
# #     return fx, fy

# # def boundary_state(x, y):
# #     # NOTE: still only walls; dancer circle is not used here.
# #     if (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
# #         y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN):
# #         return 2
# #     if (x < X_MIN + SOFT_MARGIN or x > X_MAX - SOFT_MARGIN or
# #         y < Y_MIN + SOFT_MARGIN or y > Y_MAX - SOFT_MARGIN):
# #         return 1
# #     return 0

# # def safe_pose(robot):
# #     p = robot.get_pose()
# #     if p and len(p) >= 3:
# #         return float(p[0]), float(p[1]), float(p[2])
# #     return None

# # def usr(robot):
# #     robot.delay(400)

# #     # Use robot.virtual_id() for CoachBot
# #     try:
# #         vid = robot.virtual_id()
# #     except:
# #         vid = 0

# #     random.seed(vid*1103515245 & 0xFFFFFFFF)

# #     print('Robot %d starting FLOAT choreography (no dancer avoidance)' % vid)

# #     # state
# #     neighbors = {}
# #     last_seen = {}
# #     last_hb = -1e9
# #     lastL = 0
# #     lastR = 0
# #     drift_phase = random.uniform(-math.pi, math.pi)

# #     # wake localization
# #     robot.set_vel(20, 20)
# #     robot.delay(150)

# #     while True:
# #         pose = safe_pose(robot)
# #         if not pose:
# #             robot.set_vel(0, 0)
# #             robot.delay(DT_MS)
# #             continue

# #         x, y, th = pose
# #         now = robot.get_clock()

# #         # LEDs (only wall proximity)
# #         b = boundary_state(x, y)
# #         if b == 2:
# #             robot.set_led(100, 0, 0)
# #             robot.set_vel(0, 0)
# #             robot.delay(DT_MS)
# #             continue
# #         elif b == 1:
# #             robot.set_led(100, 60, 0)  # amber
# #         else:
# #             robot.set_led(0, 70, 80)   # calm teal

# #         # --- heartbeat send (finite diff for vx,vy) ---
# #         if now - last_hb >= HB_DT:
# #             x1 = x
# #             y1 = y
# #             t1 = now
# #             robot.delay(60)
# #             p2 = safe_pose(robot)
# #             if p2:
# #                 x2, y2, th2 = float(p2[0]), float(p2[1]), float(p2[2])
# #                 t2 = robot.get_clock()
# #                 dt = max(1e-3, t2 - t1)
# #                 vx = (x2 - x1) / dt
# #                 vy = (y2 - y1) / dt
# #                 try:
# #                     robot.send_msg(struct.pack(HB_FMT, x2, y2, th2, vx, vy, vid))
# #                 except:
# #                     pass
# #                 last_hb = t2
# #                 x, y, th = x2, y2, th2
# #             else:
# #                 last_hb = now

# #         # --- receive ---
# #         msgs = robot.recv_msg()
# #         if msgs:
# #             for m in msgs:
# #                 try:
# #                     nx, ny, nth, nvx, nvy, nid = struct.unpack(HB_FMT, m[:HB_BYTES])
# #                     if int(nid) != vid:
# #                         neighbors[int(nid)] = (nx, ny, nth, nvx, nvy)
# #                         last_seen[int(nid)] = now
# #                 except:
# #                     pass

# #         # prune stale neighbors
# #         cut = now - STALE_S
# #         for nid in list(neighbors.keys()):
# #             if last_seen.get(nid, 0) < cut:
# #                 neighbors.pop(nid, None)
# #                 last_seen.pop(nid, None)

# #         # --- boidsy float field ---
# #         # environment: only walls, NO dancer obstacle now
# #         ex, ey = soft_boundary_force(x, y)

# #         # migration UP (positive y)
# #         mx = 0.0
# #         my = K_MIG

# #         # neighbor terms
# #         repx = repy = 0.0
# #         cx = cy = 0.0
# #         ax = ay = 0.0
# #         n = 0

# #         for nid in neighbors:
# #             nx, ny, nth, nvx, nvy = neighbors[nid]
# #             dx = x - nx
# #             dy = y - ny
# #             d2 = dx*dx + dy*dy
# #             if d2 > 1e-9:
# #                 d = math.sqrt(d2)
# #                 if d < SEP_RADIUS:
# #                     s = K_SEP * (SEP_RADIUS - d) / SEP_RADIUS
# #                     repx += s * (dx / d)
# #                     repy += s * (dy / d)
# #                 if d <= NEIGH_RADIUS:
# #                     cx += nx
# #                     cy += ny
# #                     ax += math.cos(nth)
# #                     ay += math.sin(nth)
# #                     n += 1

# #         cohx = cohy = 0.0
# #         alx = aly = 0.0

# #         if n > 0:
# #             cx /= n
# #             cy /= n
# #             cohx = K_COH * (cx - x)
# #             cohy = K_COH * (cy - y)
# #             ah = math.atan2(ay, ax)
# #             alx = K_ALI * math.cos(ah)
# #             aly = K_ALI * math.sin(ah)

# #         # softly varying curvature for "indirect"
# #         drift_phase += 0.03
# #         curl = CURVE_NOISE * math.sin(drift_phase)

# #         # sideways meander in x, clean upward drift in y
# #         curlx = curl
# #         curly = 0.0

# #         vx = ex + mx + repx + cohx + alx + curlx
# #         vy = ey + my + repy + cohy + aly + curly

# #         if abs(vx) < 1e-6 and abs(vy) < 1e-6:
# #             vx = 1e-3

# #         hdg = math.atan2(vy, vx)
# #         err = wrap(hdg - th)

# #         # speed: smooth, sustained
# #         fwd = FWD_BASE
# #         if b == 1:
# #             fwd *= 0.8
# #         fwd = max(FWD_MIN, fwd)

# #         # map to wheels with *low jerk* (smoothing + slew rate)
# #         turn = clamp(TURN_K * err, -1.2, 1.2)
# #         lcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
# #         rcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)

# #         # slew limit
# #         if lcmd > lastL + VEL_SLEW: lcmd = lastL + VEL_SLEW
# #         if lcmd < lastL - VEL_SLEW: lcmd = lastL - VEL_SLEW
# #         if rcmd > lastR + VEL_SLEW: rcmd = lastR + VEL_SLEW
# #         if rcmd < lastR - VEL_SLEW: rcmd = lastR - VEL_SLEW

# #         left  = int((1 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
# #         right = int((1 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
# #         lastL, lastR = left, right

# #         robot.set_vel(left, right)
# #         robot.delay(DT_MS)


# # -*- coding: utf-8 -*-
# # GLIDE(CoachBot): light, sustained, indirect
# # Smooth, flocking, sinusoidal wave with gentle upward migration.
# from __future__ import division
# import math
# import struct
# import random

# # --- field (meters) ---
# X_MIN, X_MAX = -1.2, 1.0
# Y_MIN, Y_MAX = -1.4, 2.35

# # --- (unused) dancer obstacle params kept for reference ---
# FEET = 0.3048
# OBST_DIAM_FT = 1.0
# OBST_RADIUS = 0.5 * OBST_DIAM_FT * FEET
# OBST_MARGIN = 0.03
# SAFE_BUBBLE = OBST_RADIUS + OBST_MARGIN
# OBST_CX, OBST_CY = (-0.1, 0.475)

# # --- control/loop ---
# MAX_WHEEL   = 35
# TURN_K      = 2.2
# FWD_BASE    = 0.65  # nominal forward factor
# FWD_MIN     = 0.35
# DT_MS       = 40
# CMD_SMOOTH  = 0.38  # a bit more smoothing for calm motion
# VEL_SLEW    = 7     # wheel cmd change limit per step (accel cap)

# # --- flocking gains (soft) ---
# K_MIG   = 0.08      # gentle upward drift (positive y)
# K_SEP   = 0.16      # separation (slightly softer to avoid jitter)
# K_ALI   = 0.14      # match headings (fluid flock)
# K_COH   = 0.08      # gentle cohesion

# SEP_RADIUS   = 0.26
# NEIGH_RADIUS = 0.75

# # --- sinusoidal wave parameters ---
# WAVE_AMP   = 0.18   # lateral strength of sine field (tune if needed)
# WAVE_FREQ  = 0.05   # Hz (cycles per second)
# PHASE_STEP = 0.6    # phase offset per virtual_id

# # --- boundary softness (walls only) ---
# SOFT_MARGIN = 0.08
# CRIT_MARGIN = 0.02
# SOFT_MAX_F  = 0.35

# # --- P2P heartbeats ---
# HB_FMT   = 'fffffi'  # x,y,th,vx,vy,id
# HB_BYTES = struct.calcsize(HB_FMT)
# HB_DT    = 0.12
# STALE_S  = 0.7

# def clamp(v, lo, hi):
#     return lo if v < lo else (hi if v > hi else v)

# def wrap(a):
#     while a > math.pi:
#         a -= 2*math.pi
#     while a <= -math.pi:
#         a += 2*math.pi
#     return a

# def soft_boundary_force(x, y):
#     fx = fy = 0.0
#     if x < X_MIN + SOFT_MARGIN:
#         fx += SOFT_MAX_F * (1 - (x - X_MIN) / SOFT_MARGIN)
#     elif x > X_MAX - SOFT_MARGIN:
#         fx -= SOFT_MAX_F * (1 - (X_MAX - x) / SOFT_MARGIN)
#     if y < Y_MIN + SOFT_MARGIN:
#         fy += SOFT_MAX_F * (1 - (y - Y_MIN) / SOFT_MARGIN)
#     elif y > Y_MAX - SOFT_MARGIN:
#         fy -= SOFT_MAX_F * (1 - (Y_MAX - y) / SOFT_MARGIN)
#     return fx, fy

# def boundary_state(x, y):
#     # NOTE: only walls; dancer circle is not used here.
#     if (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
#         y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN):
#         return 2
#     if (x < X_MIN + SOFT_MARGIN or x > X_MAX - SOFT_MARGIN or
#         y < Y_MIN + SOFT_MARGIN or y > Y_MAX - SOFT_MARGIN):
#         return 1
#     return 0

# def safe_pose(robot):
#     p = robot.get_pose()
#     if p and len(p) >= 3:
#         return float(p[0]), float(p[1]), float(p[2])
#     return None

# def usr(robot):
#     robot.delay(400)

#     # Use robot.virtual_id() for CoachBot
#     try:
#         vid = int(robot.virtual_id())
#     except:
#         vid = 0

#     random.seed(vid*1103515245 & 0xFFFFFFFF)

#     # per-robot phase offset for the sine wave
#     phase0 = (vid % 12) * PHASE_STEP

#     print('Robot %d starting FLOAT choreography (sinusoidal flock, no dancer avoidance)' % vid)

#     # state
#     neighbors = {}
#     last_seen = {}
#     last_hb = -1e9
#     lastL = 0
#     lastR = 0

#     # wake localization
#     robot.set_vel(20, 20)
#     robot.delay(150)

#     try:
#         while True:
#             pose = safe_pose(robot)
#             if not pose:
#                 robot.set_vel(0, 0)
#                 robot.delay(DT_MS)
#                 continue

#             x, y, th = pose
#             now = robot.get_clock()

#             # LEDs (only wall proximity)
#             b = boundary_state(x, y)
#             if b == 2:
#                 robot.set_led(100, 0, 0)
#                 robot.set_vel(0, 0)
#                 robot.delay(DT_MS)
#                 continue
#             elif b == 1:
#                 robot.set_led(100, 60, 0)  # amber
#             else:
#                 robot.set_led(0, 70, 80)   # calm teal

#             # --- heartbeat send (finite diff for vx,vy) ---
#             if now - last_hb >= HB_DT:
#                 x1 = x
#                 y1 = y
#                 t1 = now
#                 robot.delay(60)
#                 p2 = safe_pose(robot)
#                 if p2:
#                     x2, y2, th2 = float(p2[0]), float(p2[1]), float(p2[2])
#                     t2 = robot.get_clock()
#                     dt = max(1e-3, t2 - t1)
#                     vx_hb = (x2 - x1) / dt
#                     vy_hb = (y2 - y1) / dt
#                     try:
#                         robot.send_msg(struct.pack(HB_FMT, x2, y2, th2,
#                                                    vx_hb, vy_hb, vid))
#                     except:
#                         pass
#                     last_hb = t2
#                     x, y, th = x2, y2, th2
#                 else:
#                     last_hb = now

#             # --- receive neighbor heartbeats ---
#             msgs = robot.recv_msg()
#             if msgs:
#                 for m in msgs:
#                     try:
#                         nx, ny, nth, nvx, nvy, nid = struct.unpack(HB_FMT,
#                                                                    m[:HB_BYTES])
#                         if int(nid) != vid:
#                             neighbors[int(nid)] = (nx, ny, nth, nvx, nvy)
#                             last_seen[int(nid)] = now
#                     except:
#                         pass

#             # prune stale neighbors
#             cut = now - STALE_S
#             for nid in list(neighbors.keys()):
#                 if last_seen.get(nid, 0) < cut:
#                     neighbors.pop(nid, None)
#                     last_seen.pop(nid, None)

#             # --- environment forces (walls only) ---
#             ex, ey = soft_boundary_force(x, y)

#             # --- sinusoidal wave field (main "float" feel) ---
#             # lateral oscillation in x, upward migration in y
#             wave_phase = 2.0 * math.pi * WAVE_FREQ * now + phase0
#             vx_wave = WAVE_AMP * math.sin(wave_phase)
#             vy_wave = 0.0

#             # --- upward migration ---
#             mx = 0.0
#             my = K_MIG

#             # --- neighbor terms (soft flocking) ---
#             repx = repy = 0.0
#             cx = cy = 0.0
#             ax = ay = 0.0
#             n = 0

#             for nid in neighbors:
#                 nx, ny, nth, nvx, nvy = neighbors[nid]
#                 dx = x - nx
#                 dy = y - ny
#                 d2 = dx*dx + dy*dy
#                 if d2 > 1e-9:
#                     d = math.sqrt(d2)
#                     if d < SEP_RADIUS:
#                         s = K_SEP * (SEP_RADIUS - d) / SEP_RADIUS
#                         repx += s * (dx / d)
#                         repy += s * (dy / d)
#                     if d <= NEIGH_RADIUS:
#                         cx += nx
#                         cy += ny
#                         ax += math.cos(nth)
#                         ay += math.sin(nth)
#                         n += 1

#             cohx = cohy = 0.0
#             alx = aly = 0.0

#             if n > 0:
#                 cx /= n
#                 cy /= n
#                 cohx = K_COH * (cx - x)
#                 cohy = K_COH * (cy - y)
#                 ah = math.atan2(ay, ax)
#                 alx = K_ALI * math.cos(ah)
#                 aly = K_ALI * math.sin(ah)

#             # --- combine fields ---
#             vx = ex + vx_wave + mx + repx + cohx + alx
#             vy = ey + vy_wave + my + repy + cohy + aly

#             if abs(vx) < 1e-6 and abs(vy) < 1e-6:
#                 vx = 1e-3

#             hdg = math.atan2(vy, vx)
#             err = wrap(hdg - th)

#             # speed: smooth, sustained
#             fwd = FWD_BASE
#             if b == 1:
#                 fwd *= 0.8
#             fwd = max(FWD_MIN, fwd)

#             # map to wheels with low jerk (smoothing + slew rate)
#             turn = clamp(TURN_K * err, -1.2, 1.2)
#             lcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)),
#                          -MAX_WHEEL, MAX_WHEEL)
#             rcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)),
#                          -MAX_WHEEL, MAX_WHEEL)

#             # slew limit
#             if lcmd > lastL + VEL_SLEW: lcmd = lastL + VEL_SLEW
#             if lcmd < lastL - VEL_SLEW: lcmd = lastL - VEL_SLEW
#             if rcmd > lastR + VEL_SLEW: rcmd = lastR - VEL_SLEW if rcmd < lastR - VEL_SLEW else rcmd
#             # (equivalently:)
#             if rcmd > lastR + VEL_SLEW: rcmd = lastR + VEL_SLEW
#             if rcmd < lastR - VEL_SLEW: rcmd = lastR - VEL_SLEW

#             left  = int((1 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
#             right = int((1 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
#             lastL, lastR = left, right

#             robot.set_vel(left, right)
#             robot.delay(DT_MS)

#     except Exception as e:
#         try:
#             robot.set_vel(0,0)
#             robot.set_led(100,0,0)
#         except:
#             pass
#         print('[float_sine] ERROR id=%d: %s' % (vid, repr(e)))
#         raise
#     finally:
#         try:
#             robot.set_vel(0,0)
#         except:
#             pass



# -*- coding: utf-8 -*-
# GLIDE CONTINUOUS (rigid ring + medium wave) UNTIL WALL
# GLOBAL STOP ADDED:
# - If ANY bot hits soft boundary: that bot turns YELLOW and broadcasts STOP_NOW
# - All bots receiving STOP_NOW turn WHITE and freeze
# - If any bot hits critical boundary: turns RED, broadcasts STOP_NOW, all freeze

from __future__ import division
import math, struct, os

# ----------------- Arena -----------------
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

# ----------------- Wave & motion -----------------
UPWARD_SPEED = 0.035     # m/s upward drift
WAVE_AMP     = 0.13      # lateral amplitude (medium)
WAVE_FREQ    = 0.12      # Hz

DT_MS        = 40
MAX_WHEEL    = 35
TURN_K       = 3.0
K_P_POS      = 1.8       # target correction gain

# ----------------- Boundary logic -----------------
SOFT_MARGIN = 0.08
CRIT_MARGIN = 0.02
SOFT_MAX_F  = 0.35

# ----------------- COM P2P -----------------
HB_FMT   = 'ffi'
HB_BYTES = struct.calcsize(HB_FMT)
HB_DT    = 0.15
STALE_S  = 1.0

# STOP NOW message (1 byte)
STOP_MSG = b'S'

# ----------------- Logging -----------------
LOG = None
LOG_OUT = None

def init_log():
    global LOG, LOG_OUT
    try:
        LOG = open("experiment_log.txt", "a", 1)
    except: LOG = None
    try:
        LOG_OUT = open("/home/pi/experiment_output", "a", 1)
    except: LOG_OUT = None

def logw(msg):
    if not isinstance(msg, str): msg = str(msg)
    if not msg.endswith("\n"): msg += "\n"
    try:
        if LOG: LOG.write(msg); LOG.flush()
    except: pass
    try:
        if LOG_OUT: LOG_OUT.write(msg); LOG_OUT.flush()
    except: pass

# ----------------- Helpers -----------------
def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def wrap_angle(a):
    while a > math.pi:  a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a

def safe_pose(robot):
    p = robot.get_pose()
    if p and len(p) >= 3:
        return float(p[0]), float(p[1]), float(p[2])
    return None

def boundary_state(x, y):
    if (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
        y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN):
        return 2
    if (x < X_MIN + SOFT_MARGIN or x > X_MAX - SOFT_MARGIN or
        y < Y_MIN + SOFT_MARGIN or y > Y_MAX - SOFT_MARGIN):
        return 1
    return 0

def soft_boundary_force(x, y):
    fx = fy = 0.0
    if x < X_MIN + SOFT_MARGIN:
        fx += SOFT_MAX_F * (1 - (x - X_MIN)/SOFT_MARGIN)
    elif x > X_MAX - SOFT_MARGIN:
        fx -= SOFT_MAX_F * (1 - (X_MAX - x)/SOFT_MARGIN)
    if y < Y_MIN + SOFT_MARGIN:
        fy += SOFT_MAX_F * (1 - (y - Y_MIN)/SOFT_MARGIN)
    elif y > Y_MAX - SOFT_MARGIN:
        fy -= SOFT_MAX_F * (1 - (Y_MAX - y)/SOFT_MARGIN)
    return fx, fy

# ----------------- MAIN -----------------
def usr(robot):
    init_log()

    try:
        vid = int(robot.id)
    except:
        vid = 0

    logw("glide_until_wall_global: START vid=%d" % vid)

    neighbors = {}
    last_hb = -1e9

    lastL = lastR = 0
    stop_flag = False      # If TRUE → Freeze and exit
    sentinel_active = False  # Ensures only ONE yellow bot

    # wake localization
    robot.set_vel(20,20); robot.delay(150)
    robot.set_vel(0,0);   robot.delay(150)

    # ---- Phase 1: COM estimation ----
    t0 = robot.get_clock()
    COM_PHASE_T = 3.0
    C0x = C0y = None
    rel_off = None

    while True:
        # If STOP_NOW received during COM → freeze immediately
        msgs = robot.recv_msg() or []
        for m in msgs:
            if m == STOP_MSG:
                robot.set_led(255,255,255)
                robot.set_vel(0,0)
                logw("glide_until_wall_global: vid=%d STOP_NOW during COM" % vid)
                return

        pose = safe_pose(robot)
        if not pose:
            robot.set_vel(0,0); robot.delay(DT_MS); continue
        x, y, th = pose
        now = robot.get_clock()

        b = boundary_state(x,y)
        if b == 2:
            # Hard boundary during COM → act as sentinel
            robot.set_led(255,0,0)
            try: robot.send_msg(STOP_MSG)
            except: pass
            return

        elif b == 1 and not sentinel_active:
            # SOFT boundary sentinel triggers STOP
            sentinel_active = True
            robot.set_led(255,200,0)
            try: robot.send_msg(STOP_MSG)
            except: pass
            continue

        elif b == 1:
            robot.set_led(255,200,0)
        else:
            robot.set_led(0,0,200)

        # COM heartbeats
        if now - last_hb >= HB_DT:
            try: robot.send_msg(struct.pack(HB_FMT, x, y, vid))
            except: pass
            last_hb = now

        for m in msgs:
            if len(m) >= HB_BYTES:
                try:
                    nx, ny, nid = struct.unpack(HB_FMT, m[:HB_BYTES])
                    nid = int(nid)
                    if nid != vid:
                        neighbors[nid] = (nx, ny, now)
                except: pass

        cut = now - STALE_S
        for nid in list(neighbors.keys()):
            if neighbors[nid][2] < cut:
                neighbors.pop(nid,None)

        # Compute COM
        xs = [x]; ys = [y]
        for (nx,ny,tlast) in neighbors.values():
            xs.append(nx); ys.append(ny)
        COMx = sum(xs)/len(xs)
        COMy = sum(ys)/len(ys)

        if now - t0 >= COM_PHASE_T:
            C0x = COMx; C0y = COMy
            rel_off = (x - C0x, y - C0y)
            break

        robot.set_vel(0,0); robot.delay(DT_MS)

    if rel_off is None:
        robot.set_led(255,0,0)
        return

    # ---- Phase 2: Continuous GLIDE ----
    t_wave0 = robot.get_clock()
    while True:

        # FIRST: Check for STOP_NOW
        msgs = robot.recv_msg() or []
        for m in msgs:
            if m == STOP_MSG:
                robot.set_led(255,255,255)
                robot.set_vel(0,0)
                logw("glide_until_wall_global: vid=%d RECEIVED STOP_NOW" % vid)
                return

        # Get pose
        pose = safe_pose(robot)
        if not pose:
            robot.set_vel(0,0); robot.delay(DT_MS); continue

        x, y, th = pose
        now = robot.get_clock()

        # Boundary handling
        b = boundary_state(x,y)

        if b == 2:
            # Hard boundary → TURN RED, broadcast STOP → freeze
            robot.set_led(255,0,0)
            try: robot.send_msg(STOP_MSG)
            except: pass
            robot.set_vel(0,0)
            logw("glide_until_wall_global: vid=%d HARD boundary" % vid)
            return

        if b == 1:
            if not sentinel_active:
                sentinel_active = True
                robot.set_led(255,200,0)  # sentinel yellow
                try: robot.send_msg(STOP_MSG)
                except: pass
                continue
            else:
                robot.set_led(255,200,0)
        else:
            robot.set_led(0,0,255)

        # Heartbeat for COM (unchanged)
        if now - last_hb >= HB_DT:
            try: robot.send_msg(struct.pack(HB_FMT, x, y, vid))
            except: pass
            last_hb = now

        # Compute moving center
        t_wave = now - t_wave0
        Cx = C0x + WAVE_AMP * math.sin(2*math.pi*WAVE_FREQ * t_wave)
        Cy = C0y + UPWARD_SPEED * t_wave

        # Rigid target
        tx = Cx + rel_off[0]
        ty = Cy + rel_off[1]

        bfx, bfy = soft_boundary_force(x, y)
        ex = tx - x
        ey = ty - y

        vx = K_P_POS*ex + bfx
        vy = K_P_POS*ey + bfy
        if abs(vx)<1e-6 and abs(vy)<1e-6:
            vx = 1e-3

        hdg = math.atan2(vy, vx)
        err = wrap_angle(hdg - th)

        fwd = 0.60
        turn = clamp(TURN_K * err, -1.5, 1.5)

        lcmd = clamp(int(MAX_WHEEL*(fwd - 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
        rcmd = clamp(int(MAX_WHEEL*(fwd + 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)

        slew = 10
        if lcmd > lastL+slew: lcmd = lastL+slew
        if lcmd < lastL-slew: lcmd = lastL-slew
        if rcmd > lastR+slew: rcmd = lastR+slew
        if rcmd < lastR-slew: rcmd = lastR-slew

        left  = int(0.82*lcmd + 0.18*lastL)
        right = int(0.82*rcmd + 0.18*lastR)

        lastL, lastR = left, right

        robot.set_vel(left, right)
        robot.delay(DT_MS)


