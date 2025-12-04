# # # -*- coding: utf-8 -*-

# # from __future__ import division
# # import math, struct, random, os

# # # ----------------- Logging (hardware safe) -----------------
# # log     = None     # experiment_log.txt
# # log_out = None     # /home/pi/experiment_output

# # def logw(msg):
# #     """Write to both required logs (lab spec)"""
# #     global log, log_out
# #     try:
# #         s = str(msg)
# #     except:
# #         s = repr(msg)
# #     if not s.endswith("\n"):
# #         s += "\n"
# #     try:
# #         if log:
# #             log.write(s)
# #             log.flush()
# #             os.fsync(log.fileno())
# #     except:
# #         pass
# #     try:
# #         if log_out:
# #             log_out.write(s)
# #             log_out.flush()
# #             os.fsync(log_out.fileno())
# #     except:
# #         pass


# # # ----------------- Field geometry -----------------
# # X_MIN, X_MAX = -1.2, 1.0
# # Y_MIN, Y_MAX = -1.4, 2.35

# # FEET = 0.3048
# # OBST_DIAM_FT = 1.0
# # OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
# # OBST_MARGIN  = 0.03
# # SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
# # OBST_CX, OBST_CY = (-0.1, 0.475)

# # # ----------------- Control -----------------
# # MAX_WHEEL  = 35
# # TURN_K     = 3.0
# # DT_MS      = 40
# # CMD_SMOOTH = 0.18
# # VEL_SLEW   = 10

# # FWD_BASE   = 0.70
# # FWD_MIN    = 0.08

# # # ----------------- Flocking -----------------
# # K_MIG = 0.13
# # K_SEP = 0.24
# # K_ALI = 0.12
# # K_COH = 0.08

# # SEP_RADIUS   = 0.28
# # NEIGH_RADIUS = 0.75

# # # ----------------- Slash timing -----------------
# # SLASH_FREQ  = 0.25
# # PHASE_STEP  = 0.40

# # # CHANGED: make drift downward instead of upward
# # UPWARD_DRIFT_ACTIVE = -0.11
# # UPWARD_DRIFT_IDLE   = -0.055

# # # ----------------- Ring split tuning -----------------
# # RING_SPLIT = 0.65     # inner vs outer ring

# # # ----------------- Boundary softness -----------------
# # SOFT_MARGIN = 0.08
# # CRIT_MARGIN = 0.02
# # SOFT_MAX_F  = 0.35

# # # ----------------- Heartbeats -----------------
# # HB_FMT   = 'fffffi'
# # HB_BYTES = struct.calcsize(HB_FMT)
# # HB_DT    = 0.12
# # STALE_S  = 0.7


# # # Helpers (Python 2.7-safe)
# # def clamp(v, lo, hi):
# #     if v < lo: return lo
# #     if v > hi: return hi
# #     return v

# # def wrap(a):
# #     while a > math.pi:  a -= 2.0*math.pi
# #     while a <= -math.pi: a += 2.0*math.pi
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


# # # Main behavior
# # def usr(robot):
# #     global log, log_out

# #     # ----- Initialize logs -----
# #     try:
# #         log     = open("experiment_log.txt", "a", 0)
# #     except:
# #         log = None
# #     try:
# #         log_out = open("/home/pi/experiment_output", "a", 0)
# #     except:
# #         log_out = None

# #     robot.delay(300)

# #     # get VID safely
# #     try:
# #         vid = int(robot.id)
# #     except:
# #         vid = 0

# #     random.seed((vid * 1103515245) & 0xFFFFFFFF)

# #     neighbors = {}
# #     last_seen = {}
# #     last_hb   = -1e9
# #     lastL = lastR = 0

# #     # Wake localization
# #     robot.set_vel(20,20)
# #     robot.delay(150)
# #     robot.set_vel(0,0)
# #     robot.delay(150)

# #     # Determine ring group
# #     pose0 = safe_pose(robot)
# #     if not pose0:
# #         ring_group = 1
# #         r0 = 0.0
# #     else:
# #         x0, y0, _ = pose0
# #         dx0 = x0 - OBST_CX
# #         dy0 = y0 - OBST_CY
# #         r0 = math.hypot(dx0, dy0)
# #         ring_group = 0 if r0 < RING_SPLIT else 1

# #     # per-robot timing phase
# #     base_phase = (ring_group * math.pi * 0.27) + ((vid % 10) * PHASE_STEP)

# #     logw("SLASH_ZIPPER_DOWN: vid=%d ring_group=%d r0=%.3f" %
# #          (vid, ring_group, r0))

# #     try:
# #         # MAIN LOOP
# #         while True:

# #             pose = safe_pose(robot)
# #             if not pose:
# #                 robot.set_vel(0,0)
# #                 robot.delay(DT_MS)
# #                 continue

# #             x, y, th = pose
# #             now = robot.get_clock()

# #             # ------ LED by group (unless boundary override) ------
# #             b = boundary_state(x, y)
# #             if b == 2:
# #                 robot.set_led(100,0,0)
# #                 robot.set_vel(0,0)
# #                 robot.delay(DT_MS)
# #                 continue
# #             elif b == 1:
# #                 robot.set_led(150,50,0)
# #             else:
# #                 if ring_group == 0:
# #                     robot.set_led(255,0,0)
# #                 else:
# #                     robot.set_led(40,180,255)

# #             # ------ Heartbeat send ------
# #             if (now - last_hb) >= HB_DT:
# #                 x1,y1,t1 = x,y,now
# #                 robot.delay(40)
# #                 p2 = safe_pose(robot)
# #                 if p2:
# #                     x2,y2,th2 = float(p2[0]), float(p2[1]), float(p2[2])
# #                     t2 = robot.get_clock()
# #                     dt = t2 - t1
# #                     if dt < 1e-3: dt = 1e-3
# #                     vx_hb = (x2 - x1) / dt
# #                     vy_hb = (y2 - y1) / dt
# #                     try:
# #                         robot.send_msg(struct.pack(HB_FMT,
# #                                                    x2,y2,th2,
# #                                                    vx_hb,vy_hb,
# #                                                    vid))
# #                     except:
# #                         pass
# #                     last_hb = t2
# #                     x,y,th = x2,y2,th2
# #                 else:
# #                     last_hb = now

# #             # ------ Heartbeat receive ------
# #             msgs = robot.recv_msg()
# #             if msgs:
# #                 for m in msgs:
# #                     try:
# #                         nx,ny,nth,nvx,nvy,nid = struct.unpack(HB_FMT,
# #                                                               m[:HB_BYTES])
# #                         nid = int(nid)
# #                         if nid != vid:
# #                             neighbors[nid] = (nx,ny,nth,nvx,nvy)
# #                             last_seen[nid] = now
# #                     except:
# #                         pass

# #             # prune stale neighbors
# #             cutoff = now - STALE_S
# #             for nid in list(neighbors.keys()):
# #                 if last_seen.get(nid,0.0) < cutoff:
# #                     neighbors.pop(nid,None)
# #                     last_seen.pop(nid,None)

# #             # ------ boundary soft force ------
# #             ex, ey = soft_boundary_force(x,y)

# #             # ------ slash timing ------
# #             wphase = 2.0*math.pi*SLASH_FREQ*now + base_phase
# #             phase_01 = (wphase/(2.0*math.pi)) % 1.0

# #             # active window by ring
# #             if ring_group == 0:
# #                 active = (phase_01 < 0.5)
# #             else:
# #                 active = (phase_01 >= 0.5)

# #             # ------ slash vectors, mirrored vertically (downward) ------
# #             if active:
# #                 if ring_group == 0:
# #                     bx =  1.0
# #                 else:
# #                     bx = -1.0

# #                 # CHANGED: make vertical part negative so pattern goes down
# #                 by = -(0.50 + 0.35*math.sin(2.0*wphase))
# #             else:
# #                 bx = 0.15*math.sin(wphase)
# #                 # CHANGED: idle drift downward
# #                 by = -(0.22 + 0.10*math.sin(2.0*wphase))

# #             nrm = math.sqrt(bx*bx + by*by)
# #             if nrm < 1e-6: nrm = 1.0
# #             dx = bx/nrm
# #             dy = by/nrm

# #             # migration field
# #             if active:
# #                 mx = K_MIG*dx
# #                 my = UPWARD_DRIFT_ACTIVE + K_MIG*dy
# #             else:
# #                 mx = 0.05*dx
# #                 my = UPWARD_DRIFT_IDLE

# #             # ------ flocking forces ------
# #             repx=repy=0.0
# #             cx=cy=ax=ay=0.0
# #             n=0

# #             for nid,(nx,ny,nth,nvx,nvy) in neighbors.items():
# #                 dxn = x - nx
# #                 dyn = y - ny
# #                 d2 = dxn*dxn + dyn*dyn
# #                 if d2 > 1e-12:
# #                     d = math.sqrt(d2)
# #                     if d < SEP_RADIUS:
# #                         s = K_SEP*(SEP_RADIUS-d)/SEP_RADIUS
# #                         repx += s*(dxn/d)
# #                         repy += s*(dyn/d)
# #                     if d <= NEIGH_RADIUS:
# #                         cx += nx
# #                         cy += ny
# #                         ax += math.cos(nth)
# #                         ay += math.sin(nth)
# #                         n += 1

# #             cohx=cohy=alx=aly=0.0
# #             if n>0:
# #                 cx/=n; cy/=n
# #                 cohx = K_COH*(cx-x)
# #                 cohy = K_COH*(cy-y)
# #                 ah = math.atan2(ay,ax)
# #                 alx = K_ALI*math.cos(ah)
# #                 aly = K_ALI*math.sin(ah)

# #             # Total field
# #             vx = ex + mx + repx + cohx + alx
# #             vy = ey + my + repy + cohy + aly

# #             if abs(vx)<1e-6 and abs(vy)<1e-6:
# #                 vx = 1e-3

# #             hdg = math.atan2(vy,vx)
# #             err = wrap(hdg - th)

# #             # forward speed
# #             if active:
# #                 fwd = FWD_BASE + 0.20
# #             else:
# #                 fwd = FWD_BASE * 0.45

# #             if b == 1:
# #                 fwd *= 0.7
# #             if fwd < FWD_MIN:
# #                 fwd = FWD_MIN

# #             # wheel commands
# #             turn = TURN_K*err
# #             turn = clamp(turn, -1.5, 1.5)

# #             lcmd = int(MAX_WHEEL*0.9*(fwd - 0.8*turn))
# #             rcmd = int(MAX_WHEEL*0.9*(fwd + 0.8*turn))

# #             # slew
# #             if lcmd > lastL+VEL_SLEW: lcmd = lastL+VEL_SLEW
# #             if lcmd < lastL-VEL_SLEW: lcmd = lastL-VEL_SLEW
# #             if rcmd > lastR+VEL_SLEW: rcmd = lastR+VEL_SLEW
# #             if rcmd < lastR-VEL_SLEW: rcmd = lastR-VEL_SLEW

# #             L = int((1.0-CMD_SMOOTH)*lcmd + CMD_SMOOTH*lastL)
# #             R = int((1.0-CMD_SMOOTH)*rcmd + CMD_SMOOTH*lastR)
# #             lastL, lastR = L, R

# #             robot.set_vel(L,R)
# #             robot.delay(DT_MS)

# #     except Exception as e:
# #         logw("[slash_zipper] ERROR vid=%d %s" % (vid, repr(e)))
# #         try:
# #             robot.set_led(100,0,0)
# #             robot.set_vel(0,0)
# #         except:
# #             pass
# #         raise

# #     finally:
# #         try:
# #             robot.set_vel(0,0)
# #         except:
# #             pass


# # -*- coding: utf-8 -*-

# from __future__ import division
# import math, struct, random, os

# # ----------------- Logging (hardware safe) -----------------
# log     = None     # experiment_log.txt
# log_out = None     # /home/pi/experiment_output

# def logw(msg):
#     """Write to both required logs (lab spec)"""
#     global log, log_out
#     try:
#         s = str(msg)
#     except:
#         s = repr(msg)
#     if not s.endswith("\n"):
#         s += "\n"
#     try:
#         if log:
#             log.write(s)
#             log.flush()
#             os.fsync(log.fileno())
#     except:
#         pass
#     try:
#         if log_out:
#             log_out.write(s)
#             log_out.flush()
#             os.fsync(log_out.fileno())
#     except:
#         pass


# # ----------------- Field geometry -----------------
# X_MIN, X_MAX = -1.2, 1.0
# Y_MIN, Y_MAX = -1.4, 2.35

# FEET = 0.3048
# OBST_DIAM_FT = 1.0
# OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
# OBST_MARGIN  = 0.03
# SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
# OBST_CX, OBST_CY = (-0.1, 0.475)

# # ----------------- Control -----------------
# MAX_WHEEL  = 35
# TURN_K     = 3.0
# DT_MS      = 40
# CMD_SMOOTH = 0.18
# VEL_SLEW   = 10

# FWD_BASE   = 0.70
# FWD_MIN    = 0.08

# # ----------------- Flocking -----------------
# K_MIG = 0.13
# K_SEP = 0.24
# K_ALI = 0.12
# K_COH = 0.08

# SEP_RADIUS   = 0.28
# NEIGH_RADIUS = 0.75

# # ----------------- Slash timing -----------------
# SLASH_FREQ  = 0.25
# PHASE_STEP  = 0.40

# # NOTE: we now treat these as "speed magnitudes" and apply a sign per ring
# UPWARD_DRIFT_ACTIVE = 0.11   # magnitude; sign set by ring_group
# UPWARD_DRIFT_IDLE   = 0.055  # magnitude; sign set by ring_group

# # ----------------- Ring split tuning -----------------
# RING_SPLIT = 0.65     # inner vs outer ring (radius from OBST center)

# # ----------------- Boundary softness -----------------
# SOFT_MARGIN = 0.08
# CRIT_MARGIN = 0.02
# SOFT_MAX_F  = 0.35

# # ----------------- Heartbeats -----------------
# HB_FMT   = 'fffffi'
# HB_BYTES = struct.calcsize(HB_FMT)
# HB_DT    = 0.12
# STALE_S  = 0.7


# # Helpers (Python 2.7-safe)
# def clamp(v, lo, hi):
#     if v < lo: return lo
#     if v > hi: return hi
#     return v

# def wrap(a):
#     while a > math.pi:  a -= 2.0*math.pi
#     while a <= -math.pi: a += 2.0*math.pi
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


# # Main behavior
# def usr(robot):
#     global log, log_out

#     # ----- Initialize logs -----
#     try:
#         log     = open("experiment_log.txt", "a", 0)
#     except:
#         log = None
#     try:
#         log_out = open("/home/pi/experiment_output", "a", 0)
#     except:
#         log_out = None

#     robot.delay(300)

#     # get VID safely
#     try:
#         vid = int(robot.id)
#     except:
#         vid = 0

#     random.seed((vid * 1103515245) & 0xFFFFFFFF)

#     neighbors = {}
#     last_seen = {}
#     last_hb   = -1e9
#     lastL = lastR = 0

#     # Wake localization
#     robot.set_vel(20,20)
#     robot.delay(150)
#     robot.set_vel(0,0)
#     robot.delay(150)

#     # Determine ring group (inner vs outer) based on distance to OBST center
#     pose0 = safe_pose(robot)
#     if not pose0:
#         ring_group = 1
#         r0 = 0.0
#     else:
#         x0, y0, _ = pose0
#         dx0 = x0 - OBST_CX
#         dy0 = y0 - OBST_CY
#         r0 = math.hypot(dx0, dy0)
#         ring_group = 0 if r0 < RING_SPLIT else 1

#     # Direction sign:
#     #   inner ring (0) moves UP (+y)
#     #   outer ring (1) moves DOWN (-y)
#     if ring_group == 0:
#         dir_sign = +1.0
#     else:
#         dir_sign = -1.0

#     # per-robot timing phase
#     base_phase = (ring_group * math.pi * 0.27) + ((vid % 10) * PHASE_STEP)

#     logw("SLASH_SPLIT: vid=%d ring_group=%d r0=%.3f dir_sign=%.1f" %
#          (vid, ring_group, r0, dir_sign))

#     try:
#         # MAIN LOOP
#         while True:

#             pose = safe_pose(robot)
#             if not pose:
#                 robot.set_vel(0,0)
#                 robot.delay(DT_MS)
#                 continue

#             x, y, th = pose
#             now = robot.get_clock()

#             # ------ LED by group (unless boundary override) ------
#             b = boundary_state(x, y)
#             if b == 2:
#                 robot.set_led(100,0,0)
#                 robot.set_vel(0,0)
#                 robot.delay(DT_MS)
#                 continue
#             elif b == 1:
#                 robot.set_led(150,50,0)
#             else:
#                 if ring_group == 0:
#                     robot.set_led(255,0,0)      # inner red
#                 else:
#                     robot.set_led(40,180,255)   # outer blue

#             # ------ Heartbeat send ------
#             if (now - last_hb) >= HB_DT:
#                 x1,y1,t1 = x,y,now
#                 robot.delay(40)
#                 p2 = safe_pose(robot)
#                 if p2:
#                     x2,y2,th2 = float(p2[0]), float(p2[1]), float(p2[2])
#                     t2 = robot.get_clock()
#                     dt = t2 - t1
#                     if dt < 1e-3: dt = 1e-3
#                     vx_hb = (x2 - x1) / dt
#                     vy_hb = (y2 - y1) / dt
#                     try:
#                         robot.send_msg(struct.pack(HB_FMT,
#                                                    x2,y2,th2,
#                                                    vx_hb,vy_hb,
#                                                    vid))
#                     except:
#                         pass
#                     last_hb = t2
#                     x,y,th = x2,y2,th2
#                 else:
#                     last_hb = now

#             # ------ Heartbeat receive ------
#             msgs = robot.recv_msg()
#             if msgs:
#                 for m in msgs:
#                     try:
#                         nx,ny,nth,nvx,nvy,nid = struct.unpack(HB_FMT,
#                                                               m[:HB_BYTES])
#                         nid = int(nid)
#                         if nid != vid:
#                             neighbors[nid] = (nx,ny,nth,nvx,nvy)
#                             last_seen[nid] = now
#                     except:
#                         pass

#             # prune stale neighbors
#             cutoff = now - STALE_S
#             for nid in list(neighbors.keys()):
#                 if last_seen.get(nid,0.0) < cutoff:
#                     neighbors.pop(nid,None)
#                     last_seen.pop(nid,None)

#             # ------ boundary soft force ------
#             ex, ey = soft_boundary_force(x,y)

#             # ------ slash timing ------
#             wphase = 2.0*math.pi*SLASH_FREQ*now + base_phase
#             phase_01 = (wphase/(2.0*math.pi)) % 1.0

#             # active window by ring (staggered in time)
#             if ring_group == 0:
#                 active = (phase_01 < 0.5)
#             else:
#                 active = (phase_01 >= 0.5)

#             # ------ slash vectors (zig-zag), vertical split by ring ------
#             if active:
#                 # lateral direction alternates per ring
#                 if ring_group == 0:
#                     bx =  1.0
#                 else:
#                     bx = -1.0

#                 base_mag = 0.50 + 0.35*math.sin(2.0*wphase)
#                 by = dir_sign * base_mag    # inner up, outer down
#             else:
#                 bx = 0.15*math.sin(wphase)
#                 base_mag = 0.22 + 0.10*math.sin(2.0*wphase)
#                 by = dir_sign * base_mag    # idle drift split

#             # normalize
#             nrm = math.sqrt(bx*bx + by*by)
#             if nrm < 1e-6: nrm = 1.0
#             dx_vec = bx/nrm
#             dy_vec = by/nrm

#             # migration field (split in y by dir_sign)
#             if active:
#                 mx = K_MIG*dx_vec
#                 my = dir_sign*UPWARD_DRIFT_ACTIVE + K_MIG*dy_vec
#             else:
#                 mx = 0.05*dx_vec
#                 my = dir_sign*UPWARD_DRIFT_IDLE

#             # ------ flocking forces ------
#             repx=repy=0.0
#             cx=cy=ax=ay=0.0
#             n=0

#             for nid,(nx,ny,nth,nvx,nvy) in neighbors.items():
#                 dxn = x - nx
#                 dyn = y - ny
#                 d2 = dxn*dxn + dyn*dyn
#                 if d2 > 1e-12:
#                     d = math.sqrt(d2)
#                     if d < SEP_RADIUS:
#                         s = K_SEP*(SEP_RADIUS-d)/SEP_RADIUS
#                         repx += s*(dxn/d)
#                         repy += s*(dyn/d)
#                     if d <= NEIGH_RADIUS:
#                         cx += nx
#                         cy += ny
#                         ax += math.cos(nth)
#                         ay += math.sin(nth)
#                         n += 1

#             cohx=cohy=alx=aly=0.0
#             if n>0:
#                 cx/=n; cy/=n
#                 cohx = K_COH*(cx-x)
#                 cohy = K_COH*(cy-y)
#                 ah = math.atan2(ay,ax)
#                 alx = K_ALI*math.cos(ah)
#                 aly = K_ALI*math.sin(ah)

#             # Total field
#             vx = ex + mx + repx + cohx + alx
#             vy = ey + my + repy + cohy + aly

#             if abs(vx)<1e-6 and abs(vy)<1e-6:
#                 vx = 1e-3

#             hdg = math.atan2(vy,vx)
#             err = wrap(hdg - th)

#             # forward speed
#             if active:
#                 fwd = FWD_BASE + 0.20
#             else:
#                 fwd = FWD_BASE * 0.45

#             if b == 1:
#                 fwd *= 0.7
#             if fwd < FWD_MIN:
#                 fwd = FWD_MIN

#             # wheel commands
#             turn = TURN_K*err
#             turn = clamp(turn, -1.5, 1.5)

#             lcmd = int(MAX_WHEEL*0.9*(fwd - 0.8*turn))
#             rcmd = int(MAX_WHEEL*0.9*(fwd + 0.8*turn))

#             # slew
#             if lcmd > lastL+VEL_SLEW: lcmd = lastL+VEL_SLEW
#             if lcmd < lastL-VEL_SLEW: lcmd = lastL-VEL_SLEW
#             if rcmd > lastR+VEL_SLEW: rcmd = lastR+VEL_SLEW
#             if rcmd < lastR-VEL_SLEW: rcmd = lastR-VEL_SLEW

#             L = int((1.0-CMD_SMOOTH)*lcmd + CMD_SMOOTH*lastL)
#             R = int((1.0-CMD_SMOOTH)*rcmd + CMD_SMOOTH*lastR)
#             lastL, lastR = L, R

#             robot.set_vel(L,R)
#             robot.delay(DT_MS)

#     except Exception as e:
#         logw("[slash_split] ERROR vid=%d %s" % (vid, repr(e)))
#         try:
#             robot.set_led(100,0,0)
#             robot.set_vel(0,0)
#         except:
#             pass
#         raise

#     finally:
#         try:
#             robot.set_vel(0,0)
#         except:
#             pass


# -*- coding: utf-8 -*-
# SLASH (Planned Two-Ring Split, Option C + Tangential Repulsion)
# - Compute COM via gossip
# - Dynamically find radius threshold using largest gap (true inner vs outer ring)
# - Lock each bot's initial radius + angle (formation preserved)
# - Inner ring migrates up, outer ring migrates down
# - Tangential peel-away to avoid head-on jams (tangential-only repulsion)
# - Jerky zig-zag along tangent (heavy, indirect, quick)
# - Final correction to clean rings

from __future__ import division
import math, struct, random, os

# ----------------- Logging (hardware safe) -----------------
log     = None     # experiment_log.txt
log_out = None     # /home/pi/experiment_output

def logw(msg):
    global log, log_out
    try:
        s = str(msg)
    except:
        s = repr(msg)
    if not s.endswith("\n"):
        s += "\n"
    try:
        if log:
            log.write(s)
            log.flush()
            os.fsync(log.fileno())
    except:
        pass
    try:
        if log_out:
            log_out.write(s)
            log_out.flush()
            os.fsync(log_out.fileno())
    except:
        pass

# ----------------- Field geometry -----------------
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

# dancer circle kept for reference (not used)
FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
OBST_CX, OBST_CY = (-0.1, 0.475)

# ----------------- Control -----------------
MAX_WHEEL  = 35
TURN_K     = 3.0
DT_MS      = 40
CMD_SMOOTH = 0.18
VEL_SLEW   = 10

# Migration gains
K_P_POS       = 1.8
FWD_MIG       = 0.75
FWD_CORR_MAX  = 0.45
FWD_CORR_MIN  = 0.10

# ----------------- Migration timing -----------------
MIG_DURATION   = 9.0     # main slash move
CORR_MAX_TIME  = 6.0     # correction lock-in
POS_TOL        = 0.02    # ~2 cm
VEL_TOL        = 0.01

# ----------------- Tangential + Zig-Zag (slash "feel") -----------------
ZIG_AMP     = 0.18       # oscillatory tangential amplitude
ZIG_FREQ    = 0.7        # Hz
DRIFT_AMP   = 0.14       # constant tangential drift (peel-away)

# ----------------- Boundary softness -----------------
SOFT_MARGIN = 0.08
CRIT_MARGIN = 0.02
SOFT_MAX_F  = 0.35

# ----------------- Gossip (COM) -----------------
POSE_FMT   = 'ffi'   # x, y, id
POSE_BYTES = struct.calcsize(POSE_FMT)
GOSSIP_T   = 2.0     # seconds

# ----------------- Neighbor heartbeats (for repulsion) -----------------
HB_DT   = 0.18       # how often we send our pose
STALE_S = 0.9        # neighbor timeout

# ----------------- Helpers -----------------
def clamp(v, lo, hi):
    if v < lo: return lo
    if v > hi: return hi
    return v

def wrap_angle(a):
    while a >  math.pi: a -= 2.0*math.pi
    while a <= -math.pi: a += 2.0*math.pi
    return a

def soft_boundary_force(x, y):
    fx = fy = 0.0
    if x < X_MIN + SOFT_MARGIN:
        fx += SOFT_MAX_F * (1.0 - (x - X_MIN) / SOFT_MARGIN)
    elif x > X_MAX - SOFT_MARGIN:
        fx -= SOFT_MAX_F * (1.0 - (X_MAX - x) / SOFT_MARGIN)
    if y < Y_MIN + SOFT_MARGIN:
        fy += SOFT_MAX_F * (1.0 - (y - Y_MIN) / SOFT_MARGIN)
    elif y > Y_MAX - SOFT_MARGIN:
        fy -= SOFT_MAX_F * (1.0 - (Y_MAX - y) / SOFT_MARGIN)
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

# ----------------- Main behavior -----------------
def usr(robot):
    global log, log_out

    # init logs
    try:
        log = open("experiment_log.txt", "a", 0)
    except:
        log = None
    try:
        log_out = open("/home/pi/experiment_output", "a", 0)
    except:
        log_out = None

    robot.delay(300)

    try:
        vid = int(robot.id)
    except:
        vid = 0

    random.seed((vid * 1103515245) & 0xFFFFFFFF)

    logw("SLASH_OPT_C: START vid=%d" % vid)

    # wake localization
    robot.set_vel(20,20)
    robot.delay(150)
    robot.set_vel(0,0)
    robot.delay(150)

    # ----------------- Phase 1: Gossip COM + radii -----------------
    poses = {}   # id -> (x,y)
    t0 = robot.get_clock()

    while (robot.get_clock() - t0) < GOSSIP_T:
        pose = safe_pose(robot)
        if pose:
            xg, yg, thg = pose
            poses[vid] = (xg, yg)
            try:
                robot.send_msg(struct.pack(POSE_FMT, xg, yg, vid))
            except:
                pass

        msgs = robot.recv_msg()
        if msgs:
            if not isinstance(msgs, list):
                msgs = [msgs]
            for m in msgs:
                if not m:
                    continue
                if len(m) < POSE_BYTES:
                    continue
                try:
                    px, py, pid = struct.unpack(POSE_FMT, m[:POSE_BYTES])
                    poses[int(pid)] = (float(px), float(py))
                except:
                    # ignore other message types
                    pass

        robot.set_vel(0,0)
        robot.delay(50)

    # compute COM
    if poses:
        xs = [p[0] for p in poses.values()]
        ys = [p[1] for p in poses.values()]
        CX = sum(xs) / float(len(xs))
        CY = sum(ys) / float(len(ys))
    else:
        pose0 = safe_pose(robot)
        if pose0:
            CX, CY = pose0[0], pose0[1]
        else:
            CX = CY = 0.0

    # compute radii for ring classification
    radii = []
    for pid, (px, py) in poses.items():
        dx = px - CX
        dy = py - CY
        r = math.hypot(dx, dy)
        radii.append((r, int(pid)))
    radii.sort()

    # dynamic threshold: largest gap between radii -> inner vs outer split
    r_thresh = 0.0
    if len(radii) >= 2:
        max_gap = 0.0
        gap_idx = 0
        for i in range(len(radii) - 1):
            g = radii[i+1][0] - radii[i][0]
            if g > max_gap:
                max_gap = g
                gap_idx = i
        if max_gap > 0.01:  # avoid degenerate all-same-radius case
            r_thresh = 0.5 * (radii[gap_idx][0] + radii[gap_idx+1][0])
        else:
            # fallback: use median radius
            mid = len(radii) // 2
            r_thresh = radii[mid][0]
    elif len(radii) == 1:
        r_thresh = radii[0][0]
    else:
        r_thresh = 0.0

    # my initial pose
    pose0 = safe_pose(robot)
    if pose0:
        x0, y0, th0 = pose0
    else:
        x0 = y0 = th0 = 0.0

    dx0 = x0 - CX
    dy0 = y0 - CY
    r0  = math.hypot(dx0, dy0)
    if r0 > 1e-6:
        ang0 = math.atan2(dy0, dx0)
    else:
        ang0 = 0.0

    # ring group: inner vs outer based on threshold radius
    if r0 <= r_thresh:
        ring_group = 0  # inner ring
    else:
        ring_group = 1  # outer ring

    # max radius -> safe SHIFT planning
    if radii:
        r_max = max(r for (r, _) in radii)
    else:
        r_max = r0

    margin = 0.06
    max_up   = (Y_MAX - margin - r_max) - CY
    max_down = CY - (Y_MIN + margin + r_max)
    max_up   = max(0.0, max_up)
    max_down = max(0.0, max_down)
    shift_mag = min(max_up, max_down)
    if shift_mag <= 0.0:
        SHIFT = 0.0
    else:
        SHIFT = 0.9 * shift_mag  # keep some margin

    # final ring centers
    Cfx = CX
    if ring_group == 0:
        Cfy = CY + SHIFT
        base_led = (255, 0, 0)     # inner ring = red
    else:
        Cfy = CY - SHIFT
        base_led = (40, 180, 255)  # outer ring = blue

    # radial and tangent directions based on initial angle
    if r0 > 1e-6:
        urx = math.cos(ang0)
        ury = math.sin(ang0)
        utx = -ury
        uty = urx
    else:
        urx, ury = 1.0, 0.0
        utx, uty = 0.0, 1.0

    # tangential split direction to avoid head-on jams
    # right half-plane (cos>0) drifts one way, left half-plane the other
    if math.cos(ang0) >= 0.0:
        tangential_sign = +1.0
    else:
        tangential_sign = -1.0

    logw("SLASH_OPT_C: vid=%d CX=%.3f CY=%.3f r0=%.3f r_thresh=%.3f ring=%d SHIFT=%.3f tangential_sign=%.1f"
         % (vid, CX, CY, r0, r_thresh, ring_group, SHIFT, tangential_sign))

    # states
    STATE_MIGRATE = 0
    STATE_CORRECT = 1

    state = STATE_MIGRATE
    t_mig_start = robot.get_clock()
    t_corr_start = None
    last_pose_for_vel = None

    lastL = lastR = 0

    # phase offset for jerky zig-zag
    zig_phase_offset = (vid % 17) * 0.73

    # neighbor tracking for tangential repulsion
    neighbors = {}   # nid -> (x,y,th,vx,vy)
    last_seen = {}
    last_hb   = -1e9

    try:
        while True:
            pose = safe_pose(robot)
            if not pose:
                robot.set_vel(0,0)
                robot.delay(DT_MS)
                continue

            x, y, th = pose
            now = robot.get_clock()

            # boundary
            b = boundary_state(x, y)
            if b == 2:
                robot.set_led(255,0,0)
                robot.set_vel(0,0)
                logw("SLASH_OPT_C: vid=%d CRIT boundary [%.3f,%.3f]" % (vid, x, y))
                break
            elif b == 1:
                robot.set_led(150, 50, 0)
            else:
                robot.set_led(base_led[0], base_led[1], base_led[2])

            # ----- heartbeat send for neighbors -----
            if (now - last_hb) >= HB_DT:
                x1, y1, t1 = x, y, now
                robot.delay(30)
                p2 = safe_pose(robot)
                if p2:
                    x2, y2, th2 = float(p2[0]), float(p2[1]), float(p2[2])
                    t2 = robot.get_clock()
                    dt = t2 - t1
                    if dt < 1e-3:
                        dt = 1e-3
                    vx_hb = (x2 - x1) / dt
                    vy_hb = (y2 - y1) / dt
                    try:
                        robot.send_msg(struct.pack('fffffi',
                                                   x2, y2, th2,
                                                   vx_hb, vy_hb,
                                                   vid))
                    except:
                        pass
                    last_hb = t2
                    x, y, th = x2, y2, th2
                else:
                    last_hb = now

            # ----- heartbeat receive -----
            msgs = robot.recv_msg()
            if msgs:
                if not isinstance(msgs, list):
                    msgs = [msgs]
                for m in msgs:
                    if not m or len(m) < struct.calcsize('fffffi'):
                        continue
                    try:
                        nx, ny, nth, nvx, nvy, nid = struct.unpack('fffffi',
                                                                   m[:struct.calcsize('fffffi')])
                        nid = int(nid)
                        if nid != vid:
                            neighbors[nid] = (nx, ny, nth, nvx, nvy)
                            last_seen[nid] = now
                    except:
                        pass

            # prune stale neighbors
            cutoff = now - STALE_S
            for nid in list(neighbors.keys()):
                if last_seen.get(nid, 0.0) < cutoff:
                    neighbors.pop(nid, None)
                    last_seen.pop(nid, None)

            # ------------ MIGRATION PHASE ------------
            if state == STATE_MIGRATE:
                t_rel = (now - t_mig_start) / max(1e-3, MIG_DURATION)
                if t_rel > 1.0:
                    t_rel = 1.0

                # ring center moves vertically from CY to Cfy
                Ctx = CX
                Cty = CY + (Cfy - CY) * t_rel

                # base ring target
                tx_nom = Ctx + r0 * urx
                ty_nom = Cty + r0 * ury

                # tangential jerk + drift (Option C: expressive but still controlled)
                zig = ZIG_AMP * (1.0 - 0.3 * t_rel) * math.sin(2.0 * math.pi * ZIG_FREQ * now + zig_phase_offset)
                drift = DRIFT_AMP * (1.0 - t_rel) * tangential_sign
                tangential_offset = zig + drift

                tx = tx_nom + tangential_offset * utx
                ty = ty_nom + tangential_offset * uty

                # wall push
                bfx, bfy = soft_boundary_force(x, y)

                ex = tx - x
                ey = ty - y

                vx = K_P_POS * ex + bfx
                vy = K_P_POS * ey + bfy

                # ======================================================
                # TANGENTIAL REPULSION (ANTI-JAM) — MIGRATE PHASE
                #   * pushes ONLY sideways around ring
                #   * no radial distortion
                #   * allows clean pass-by when head-to-head
                # ======================================================
                TREPEL_RAD  = 0.14      # 14 cm "too close"
                TREPEL_GAIN = 0.55      # fairly strong for SLASH

                dx_c = x - CX
                dy_c = y - CY
                r_now = math.sqrt(dx_c*dx_c + dy_c*dy_c)

                if r_now > 1e-6:
                    urx_c = dx_c / r_now
                    ury_c = dy_c / r_now
                    utx_c = -ury_c
                    uty_c =  urx_c
                else:
                    utx_c, uty_c = 0.0, 1.0

                t_rep_x = 0.0
                t_rep_y = 0.0

                for nid,(nx,ny,nth,nvx,nvy) in neighbors.items():
                    dxn = x - nx
                    dyn = y - ny
                    d2  = dxn*dxn + dyn*dyn
                    if d2 < 1e-12:
                        continue
                    d = math.sqrt(d2)
                    if d < TREPEL_RAD:
                        w = (TREPEL_RAD - d) / TREPEL_RAD
                        tang = dxn*utx_c + dyn*uty_c   # signed tangential projection
                        t_rep_x += TREPEL_GAIN * w * tang * utx_c
                        t_rep_y += TREPEL_GAIN * w * tang * uty_c

                vx += t_rep_x
                vy += t_rep_y

                if abs(vx) < 1e-6 and abs(vy) < 1e-6:
                    vx = 1e-3

                hdg = math.atan2(vy, vx)
                err = wrap_angle(hdg - th)

                fwd = FWD_MIG
                if b == 1:
                    fwd *= 0.7

                turn = clamp(TURN_K * err, -1.5, 1.5)

                lcmd = clamp(int(MAX_WHEEL * (fwd - 0.8 * turn)),
                             -MAX_WHEEL, MAX_WHEEL)
                rcmd = clamp(int(MAX_WHEEL * (fwd + 0.8 * turn)),
                             -MAX_WHEEL, MAX_WHEEL)

                # smoothing
                if lcmd > lastL + VEL_SLEW: lcmd = lastL + VEL_SLEW
                if lcmd < lastL - VEL_SLEW: lcmd = lastL - VEL_SLEW
                if rcmd > lastR + VEL_SLEW: rcmd = lastR + VEL_SLEW
                if rcmd < lastR - VEL_SLEW: rcmd = lastR - VEL_SLEW

                left  = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
                right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
                lastL, lastR = left, right

                robot.set_vel(left, right)

                # move to correction when time up
                if t_rel >= 1.0 or (now - t_mig_start) > (MIG_DURATION + 2.0):
                    state = STATE_CORRECT
                    t_corr_start = now
                    last_pose_for_vel = (x, y, now)
                    logw("SLASH_OPT_C: vid=%d entering CORRECT phase" % vid)

                robot.delay(DT_MS)
                continue

            # ------------ CORRECTION PHASE ------------
            # final clean ring, no more zig-zag or drift
            tx = Cfx + r0 * urx
            ty = Cfy + r0 * ury

            ex = tx - x
            ey = ty - y
            pos_err = math.hypot(ex, ey)

            # estimate speed
            v_est = 0.0
            if last_pose_for_vel is not None:
                lx, ly, lt = last_pose_for_vel
                dtv = max(1e-3, now - lt)
                dist = math.hypot(x - lx, y - ly)
                v_est = dist / dtv
            last_pose_for_vel = (x, y, now)

            done_geom = (pos_err < POS_TOL and v_est < VEL_TOL)
            done_time = (now - t_corr_start) > CORR_MAX_TIME

            if done_geom or done_time:
                robot.set_vel(0,0)
                robot.set_led(base_led[0], base_led[1], base_led[2])
                logw("SLASH_OPT_C: vid=%d LOCKED pos_err=%.3f v_est=%.3f (time_done=%s)"
                     % (vid, pos_err, v_est, str(done_time)))
                break

            bfx, bfy = soft_boundary_force(x, y)

            vx = K_P_POS * ex + bfx
            vy = K_P_POS * ey + bfy

            # ======================================================
            # TANGENTIAL REPULSION — CORRECTION PHASE (gentle)
            # keeps bots from bumping during ring lock-in
            # ======================================================
            TREPEL_RAD_C  = 0.12
            TREPEL_GAIN_C = 0.25

            dx_c = x - CX
            dy_c = y - CY
            r_now = math.sqrt(dx_c*dx_c + dy_c*dy_c)

            if r_now > 1e-6:
                urx_c = dx_c / r_now
                ury_c = dy_c / r_now
                utx_c = -ury_c
                uty_c =  urx_c
            else:
                utx_c, uty_c = 0.0, 1.0

            t_rep_x = 0.0
            t_rep_y = 0.0

            for nid,(nx,ny,nth,nvx,nvy) in neighbors.items():
                dxn = x - nx
                dyn = y - ny
                d2  = dxn*dxn + dyn*dyn
                if d2 < 1e-12:
                    continue
                d = math.sqrt(d2)
                if d < TREPEL_RAD_C:
                    w = (TREPEL_RAD_C - d) / TREPEL_RAD_C
                    tang = dxn*utx_c + dyn*uty_c
                    t_rep_x += TREPEL_GAIN_C * w * tang * utx_c
                    t_rep_y += TREPEL_GAIN_C * w * tang * uty_c

            vx += t_rep_x
            vy += t_rep_y

            if abs(vx) < 1e-6 and abs(vy) < 1e-6:
                vx = 1e-3

            hdg = math.atan2(vy, vx)
            err = wrap_angle(hdg - th)

            fwd = FWD_CORR_MAX * max(0.25, min(1.0, pos_err / 0.12))
            if b == 1:
                fwd *= 0.7
            if fwd < FWD_CORR_MIN:
                fwd = FWD_CORR_MIN

            turn = clamp(TURN_K * err, -1.5, 1.5)

            lcmd = clamp(int(MAX_WHEEL * (fwd - 0.8 * turn)),
                         -MAX_WHEEL, MAX_WHEEL)
            rcmd = clamp(int(MAX_WHEEL * (fwd + 0.8 * turn)),
                         -MAX_WHEEL, MAX_WHEEL)

            if lcmd > lastL + VEL_SLEW: lcmd = lastL + VEL_SLEW
            if lcmd < lastL - VEL_SLEW: lcmd = lastL - VEL_SLEW
            if rcmd > lastR + VEL_SLEW: rcmd = lastR + VEL_SLEW
            if rcmd < lastR - VEL_SLEW: rcmd = lastR - VEL_SLEW

            left  = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
            right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
            lastL, lastR = left, right

            robot.set_vel(left, right)
            robot.delay(DT_MS)

    except Exception as e:
        logw("SLASH_OPT_C: ERROR vid=%d %s" % (vid, repr(e)))
        try:
            robot.set_led(255,0,0)
            robot.set_vel(0,0)
        except:
            pass
        raise

    finally:
        try:
            robot.set_vel(0,0)
        except:
            pass
        logw("SLASH_OPT_C: FINISH vid=%d" % vid)
