# # -*- coding: utf-8 -*-
# """
# FLOAT mode — SIMULATION-FRIENDLY
# - In sim: uses robot.id and prints status lines
# - On hardware: uses robot.virtual_id() and writes to experiment_log.txt
# Behavior is otherwise identical to your physical-robot version.
# """

# import math
# import os
# import random

# # --- field bounds (meters) ---
# X_MIN, X_MAX = -1.2, 1.0
# Y_MIN, Y_MAX = -1.4, 2.35

# # --- dancer no-go circle (meters) ---
# FEET = 0.6048
# OBST_DIAM_FT = 1.0
# OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET   # ~0.1524
# OBST_MARGIN  = 0.03
# SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
# OBST_CX, OBST_CY = (-0.1, 0.475)

# # --- drive / control ---
# MAX_WHEEL = 35
# TURN_K    = 3.0
# FWD_FAST  = 0.80
# FWD_SLOW  = 0.35
# FWD_MIN   = 0.33
# EPS       = 1e-3

# # --- command smoothing (more smoothing for float) ---
# CMD_SMOOTH  = 0.30

# # --- boundary softness ---
# SOFT_MARGIN     = 0.08
# CRIT_MARGIN     = 0.02
# SOFT_MAX_FORCE  = 0.35

# # --- neighbor spacing ---
# REPULSE_RADIUS  = 0.75
# REPULSE_GAIN    = 0.10
# HARD_REP_RADIUS = 0.18
# HARD_REP_GAIN   = 0.26

# # --- FLOAT field params ---
# LEFT_DRIFT_VX   = -0.10  # constant gentle push left
# NOISE_GAIN      = 0.10
# ALIGN_GAIN      = 0.06
# COHERE_GAIN     = 0.04
# NOISE_SCALE     = 0.35
# NOISE_SPEED     = 0.05

# # --- timing ---
# PRINT_PERIOD = 2.0
# MAX_RUNTIME  = 55.0
# LOOP_DT_MS   = 40  # 25 Hz


# # ----------------- helpers -----------------

# def clamp(v, lo, hi):
#     if v < lo: return lo
#     if v > hi: return hi
#     return v

# def wrap_angle(a):
#     while a >  math.pi:
#         a -= 2.0*math.pi
#     while a <= -math.pi:
#         a += 2.0*math.pi
#     return a

# def safe_pose(robot):
#     p = robot.get_pose()
#     if p and len(p) >= 3:
#         return float(p[0]), float(p[1]), float(p[2])
#     return None

# def soft_boundary_check(x, y):
#     """Return 0=ok, 1=warn, 2=critical based on margins."""
#     if (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
#         y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN):
#         return 2
#     elif (x < X_MIN + SOFT_MARGIN or x > X_MAX - SOFT_MARGIN or
#           y < Y_MIN + SOFT_MARGIN or y > Y_MAX - SOFT_MARGIN):
#         return 1
#     return 0

# def soft_boundary_force(x, y):
#     """Soft push back toward interior near walls."""
#     fx = 0.0
#     fy = 0.0
#     if x < X_MIN + SOFT_MARGIN:
#         fx += SOFT_MAX_FORCE * (1.0 - (x - X_MIN)/SOFT_MARGIN)
#     elif x > X_MAX - SOFT_MARGIN:
#         fx -= SOFT_MAX_FORCE * (1.0 - (X_MAX - x)/SOFT_MARGIN)
#     if y < Y_MIN + SOFT_MARGIN:
#         fy += SOFT_MAX_FORCE * (1.0 - (y - Y_MIN)/SOFT_MARGIN)
#     elif y > Y_MAX - SOFT_MARGIN:
#         fy -= SOFT_MAX_FORCE * (1.0 - (Y_MAX - y)/SOFT_MARGIN)
#     return fx, fy

# def soft_obstacle_force(x, y, max_force=0.55, buffer_width=0.10):
#     """Soft radial push away from dancer disk within buffer ring."""
#     dx = x - OBST_CX
#     dy = y - OBST_CY
#     r  = math.hypot(dx, dy)
#     if r < SAFE_BUBBLE + buffer_width:
#         if r < 1e-6:
#             return max_force, 0.0
#         strength = max(0.0, (SAFE_BUBBLE + buffer_width - r) / buffer_width)
#         s = max_force * strength
#         return s * (dx / r), s * (dy / r)
#     return 0.0, 0.0

# def is_critical_obstacle(x, y, critical_margin=0.0):
#     dx = x - OBST_CX
#     dy = y - OBST_CY
#     r  = math.hypot(dx, dy)
#     return r < (OBST_RADIUS + critical_margin)

# def try_get_swarm_poses(robot):
#     """
#     Try a few common API names for neighbor poses; return [] if none.
#     Covers both sim and hardware variants.
#     """
#     names = ('get_swarm_poses', 'get_all_poses', 'get_poses', 'swarm_poses')
#     for nm in names:
#         fn = getattr(robot, nm, None)
#         if callable(fn):
#             try:
#                 poses = fn()
#                 if poses:
#                     return poses
#             except:
#                 pass
#     return []

# def detect_sim(robot):
#     """
#     True if simulator-like (has robot.id and no virtual_id callable),
#     else False (assume hardware).
#     """
#     has_id = hasattr(robot, "id")
#     has_virt = hasattr(robot, "virtual_id")
#     if has_id and not callable(getattr(robot, "virtual_id", None)):
#         return True
#     # If both exist, prefer sim if robot.id is callable/usable
#     try:
#         _ = robot.id() if callable(getattr(robot, "id", None)) else robot.id
#         return True
#     except:
#         return False

# def get_robot_id(robot, sim_mode):
#     # sim → robot.id (call or attr); hw → robot.virtual_id()
#     if sim_mode:
#         rid_attr = getattr(robot, "id", None)
#         try:
#             return rid_attr() if callable(rid_attr) else int(rid_attr)
#         except:
#             return -1
#     else:
#         try:
#             return robot.virtual_id()
#         except:
#             return -1

# def randn():
#     # small zero-mean bounded-ish noise using sum of uniforms
#     return (random.random() + random.random() + random.random() - 1.5) / 1.5

# def flow_noise(x, y, t, scale=NOISE_SCALE, speed=NOISE_SPEED):
#     """Lightweight pseudo-Perlin made from sines with drifting phase."""
#     kx = 2.1*scale
#     ky = 1.7*scale
#     ph = speed * t
#     nx = math.sin(kx*x + 0.7*ky*y + 1.2 + ph) + 0.6*math.sin(0.6*kx*x - 1.3*ky*y + ph*0.7)
#     ny = math.sin(0.9*kx*x + 1.4*ky*y + ph*1.3) + 0.6*math.sin(-1.1*kx*x + 0.5*ky*y - 0.8 + ph*0.9)
#     return (0.5*nx, 0.5*ny)  # ~[-1,1]

# def alignment_vec(robot, neighbors, R=0.7):
#     """Return (ax, ay) = unit-average of neighbor headings within radius R."""
#     rx, ry, rth = robot.get_pose()
#     sx = sy = 0.0; n = 0
#     for item in neighbors or []:
#         if isinstance(item, (list, tuple)) and len(item) >= 3:
#             if len(item) == 4: _, nx, ny, nth = item
#             else: nx, ny, nth = item[0], item[1], item[2]
#             dx = nx - rx; dy = ny - ry
#             if dx*dx + dy*dy <= R*R:
#                 sx += math.cos(nth); sy += math.sin(nth); n += 1
#     if n == 0: return (0.0, 0.0)
#     m = math.hypot(sx, sy) or 1.0
#     return (sx/m, sy/m)

# def cohesion_vec(robot, neighbors, R=0.8, target_sep=0.30):
#     """Pull toward local centroid minus a separation bias when far."""
#     rx, ry, _ = robot.get_pose()
#     cx = cy = 0.0; n = 0
#     for item in neighbors or []:
#         if isinstance(item, (list, tuple)) and len(item) >= 3:
#             if len(item) == 4: _, nx, ny, _ = item
#             else: nx, ny = item[0], item[1]
#             dx = nx - rx; dy = ny - ry
#             if dx*dx + dy*dy <= R*R:
#                 cx += nx; cy += ny; n += 1
#     if n == 0: return (0.0, 0.0)
#     cx /= n; cy /= n
#     vx = (cx - rx); vy = (cy - ry)
#     d = math.hypot(vx, vy) or 1.0
#     if d < 1e-6:
#         return (0.0, 0.0)
#     gain = max(0.0, (d - target_sep))  # no pull if already close
#     return (gain * vx/d, gain * vy/d)


# # ----------------- main -----------------

# def usr(robot):
#     robot.delay(3000)

#     sim_mode = detect_sim(robot)
#     rid = get_robot_id(robot, sim_mode)

#     # --- logging setup ---
#     if sim_mode:
#         def logw(s):
#             # simulator expects print(), keep it light
#             print(s)
#         logw("FLOAT(SIM): I am robot %s" % str(rid))

#     # --- logging setup ---
#     if sim_mode:
#         def logw(s):
#             # simulator expects print(), keep it light
#             print(s)
#         logw("FLOAT(SIM): I am robot %s" % str(rid))
#     else:
#         log_main = open("experiment_log.txt", "a")
#         def logw(s):
#             if not s.endswith("\n"):
#                 s += "\n"
#             log_main.write(s)
#             log_main.flush()
#             try:
#                 os.fsync(log_main.fileno())
#             except:
#                 pass
#         logw("FLOAT(HW): I am robot %s" % str(rid))

#     # --- seeded randomness per robot ---
#     try:
#         rnd_seed = int((rid if rid is not None else 0) * 2654435761) & 0xFFFFFFFF
#     except:
#         rnd_seed = 0
#     random.seed(rnd_seed)

#     # --- time base (fallback if get_clock not present) ---
#     has_clock = hasattr(robot, "get_clock")
#     start_time = robot.get_clock() if has_clock else 0.0
#     acc_time = 0.0  # ms-based fallback

#     last_log_sec = -1
#     last_pose = None
#     last_left = 0
#     last_right = 0
#     told_no_swarm_api = False

#     try:
#         while True:
#             # elapsed time
#             now = robot.get_clock() if has_clock else (start_time + acc_time/1000.0)
#             t = (now - start_time) if has_clock else (acc_time/1000.0)
#             if t >= MAX_RUNTIME:
#                 break

#             pose = safe_pose(robot)
#             if pose is None:
#                 robot.set_vel(0, 0)
#                 robot.delay(LOOP_DT_MS)
#                 if not has_clock:
#                     acc_time += LOOP_DT_MS
#                 continue

#             x, y, th = pose
#             last_pose = (x, y)

#             # boundary light + protection
#             bstat = soft_boundary_check(x, y)
#             if bstat == 2:
#                 logw("CRITICAL: Robot %s at boundary [%.3f, %.3f]" % (str(rid), x, y))
#                 robot.set_vel(0, 0)
#                 robot.set_led(255, 0, 0)
#                 break
#             elif bstat == 1:
#                 robot.set_led(180, 220, 255)  # near boundary
#             else:
#                 # breathing cyan
#                 breathe = int(140 + 60 * (0.5 + 0.5*math.sin(0.6*t)))
#                 robot.set_led(0, breathe, breathe)

#             # emergency stop if inside the dancer disk
#             if is_critical_obstacle(x, y, 0.0):
#                 logw("CRITICAL: Robot %s inside obstacle [%.3f, %.3f]" % (str(rid), x, y))
#                 robot.set_vel(0, 0)
#                 robot.set_led(255, 0, 0)
#                 robot.delay(LOOP_DT_MS)
#                 if not has_clock:
#                     acc_time += LOOP_DT_MS
#                 continue

#             # --- base field composition ---
#             bfx, bfy = soft_boundary_force(x, y)
#             ofx, ofy = soft_obstacle_force(x, y)

#             vx = bfx + ofx + LEFT_DRIFT_VX
#             vy = bfy + ofy

#             nx, ny = flow_noise(x, y, t, scale=NOISE_SCALE, speed=NOISE_SPEED)
#             vx += NOISE_GAIN * 0.80 * nx
#             vy += NOISE_GAIN * 0.80 * ny

#             neighbors = try_get_swarm_poses(robot)
#             if neighbors:
#                 for item in neighbors:
#                     if isinstance(item, (list, tuple)) and len(item) >= 3:
#                         if len(item) == 4:
#                             nid, nxp, nyp, nth = item
#                         else:
#                             nxp, nyp, nth = item[0], item[1], item[2]
#                             nid = None
#                         if (nid is not None) and (str(nid) == str(rid)):
#                             continue
#                         dxn = x - nxp
#                         dyn = y - nyp
#                         d2  = dxn*dxn + dyn*dyn
#                         if d2 < 1e-12:
#                             continue
#                         if d2 < (REPULSE_RADIUS*REPULSE_RADIUS):
#                             s = REPULSE_GAIN / d2
#                             vx += s * dxn
#                             vy += s * dyn
#                         d = math.sqrt(d2)
#                         if d < HARD_REP_RADIUS:
#                             s_hard = HARD_REP_GAIN / (d2 * d + 1e-9)
#                             vx += s_hard * dxn
#                             vy += s_hard * dyn
#             else:
#                 if not told_no_swarm_api:
#                     logw("Robot %s: no swarm pose API; proceeding without alignment/cohesion" % str(rid))
#                     told_no_swarm_api = True

#             if neighbors:
#                 ax, ay = alignment_vec(robot, neighbors, R=0.7)
#                 cx, cy = cohesion_vec(robot, neighbors, R=0.8, target_sep=0.30)
#                 vx += ALIGN_GAIN  * ax
#                 vy += ALIGN_GAIN  * ay
#                 vx += COHERE_GAIN * cx
#                 vy += COHERE_GAIN * cy

#             # ---- map (vx, vy) → wheels ----
#             if abs(vx) + abs(vy) < EPS:
#                 vx += 0.03 * (-math.sin(th))
#                 vy += 0.03 * ( math.cos(th))

#             hdg = math.atan2(vy, vx)
#             err = wrap_angle(hdg - th)

#             # smooth long arcs
#             ae = abs(err)
#             if ae < 0.6:
#                 fwd = FWD_FAST * 0.95
#             elif ae < 1.2:
#                 fwd = FWD_FAST * 0.75
#             else:
#                 fwd = FWD_SLOW * 0.65

#             if bstat == 1:
#                 fwd *= 0.75
#             if fwd < FWD_MIN:
#                 fwd = FWD_MIN

#             turn = clamp(TURN_K * err, -1.2, 1.2)

#             left_cmd  = clamp(int(MAX_WHEEL * 0.90 * (fwd - 0.75 * turn)), -MAX_WHEEL,  MAX_WHEEL)
#             right_cmd = clamp(int(MAX_WHEEL * 0.90 * (fwd + 0.75 * turn)), -MAX_WHEEL,  MAX_WHEEL)

#             # EMA smoothing for floaty feel
#             left  = int((1.0 - CMD_SMOOTH) * left_cmd  + CMD_SMOOTH * last_left)
#             right = int((1.0 - CMD_SMOOTH) * right_cmd + CMD_SMOOTH * last_right)
#             last_left, last_right = left, right

#             robot.set_vel(left, right)

#             # periodic status
#             if int(now) != last_log_sec and (t % PRINT_PERIOD) < (LOOP_DT_MS/1000.0 + 0.02):
#                 if sim_mode:
#                     logw("FLOAT(SIM) %s pos [%.3f, %.3f]" % (str(rid), x, y))
#                 else:
#                     logw("FLOAT(HW) %s pos [%.3f, %.3f]" % (str(rid), x, y))
#                 last_log_sec = int(now)

#             robot.delay(LOOP_DT_MS)
#             if not has_clock:
#                 acc_time += LOOP_DT_MS

#     except Exception as e:
#         # error path: stop + red LED, then re-raise
#         try:
#             robot.set_vel(0, 0)
#             robot.set_led(255, 0, 0)
#         except:
#             pass
#         if sim_mode:
#             print("ERROR(FLOAT SIM): %s" % str(e))
#         else:
#             logw("ERROR(FLOAT HW): %s" % str(e))
#         raise
#     finally:
#         final_time = (robot.get_clock() if hasattr(robot, "get_clock") else (start_time + acc_time/1000.0))
#         if last_pose:
#             lx, ly = last_pose
#         else:
#             lx = float('nan'); ly = float('nan')
#         try:
#             robot.set_vel(0, 0)
#         except:
#             pass
#         msg = "FLOAT %s finished at [%.3f, %.3f] after %.1fs" % (str(rid), lx, ly, final_time - (start_time if hasattr(robot, "get_clock") else 0.0))
#         if sim_mode:
#             print(msg)
#         else:
#             logw(msg)
#             try:
#                 log_main.close()
#             except:
#                 pass



# -*- coding: utf-8 -*-
# FLOAT (SIM): light • sustained • indirect
# Smooth, curving motion with low jerk and gentle leftward migration.
import math, struct, random

# --- field & obstacle (meters) ---
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35
FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
OBST_CX, OBST_CY = (-0.1, 0.475)

# --- control/loop ---
MAX_WHEEL   = 35
TURN_K      = 2.2
FWD_BASE    = 0.65      # nominal forward factor
FWD_MIN     = 0.35
DT_MS       = 40
CMD_SMOOTH  = 0.35      # higher smoothing → lower jerk
VEL_SLEW    = 8         # wheel cmd change limit per step (accel cap)

# --- float style gains ---
K_MIG   = 0.08          # gentle leftward drift
K_SEP   = 0.20          # spacing
K_ALI   = 0.18          # match headings (fluid flock)
K_COH   = 0.10          # gentle cohesion
CURVE_NOISE = 0.20      # adds “indirect” softly varying heading

SEP_RADIUS   = 0.26
NEIGH_RADIUS = 0.75

# --- boundary softness ---
SOFT_MARGIN = 0.08
CRIT_MARGIN = 0.02
SOFT_MAX_F  = 0.35

# --- P2P heartbeats ---
HB_FMT   = 'fffffi'     # x,y,th,vx,vy,id
HB_BYTES = struct.calcsize(HB_FMT)
HB_DT    = 0.12
STALE_S  = 0.7

def clamp(v, lo, hi): return lo if v < lo else (hi if v > hi else v)
def wrap(a):
    while a >  math.pi: a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a

def soft_boundary_force(x,y):
    fx=fy=0.0
    if x < X_MIN+SOFT_MARGIN: fx += SOFT_MAX_F*(1-(x-X_MIN)/SOFT_MARGIN)
    elif x > X_MAX-SOFT_MARGIN: fx -= SOFT_MAX_F*(1-(X_MAX-x)/SOFT_MARGIN)
    if y < Y_MIN+SOFT_MARGIN: fy += SOFT_MAX_F*(1-(y-Y_MIN)/SOFT_MARGIN)
    elif y > Y_MAX-SOFT_MARGIN: fy -= SOFT_MAX_F*(1-(Y_MAX-y)/SOFT_MARGIN)
    return fx,fy

def soft_obstacle_force(x, y, maxf=0.55, w=0.12):
    dx,dy = x-OBST_CX, y-OBST_CY; r=math.hypot(dx,dy)
    if r < SAFE_BUBBLE + w:
        if r<1e-6: return maxf,0.0
        s = max(0.0, (SAFE_BUBBLE+w-r)/w)*maxf
        return s*(dx/r), s*(dy/r)
    return 0.0,0.0

def boundary_state(x,y):
    if (x < X_MIN+CRIT_MARGIN or x > X_MAX-CRIT_MARGIN or
        y < Y_MIN+CRIT_MARGIN or y > Y_MAX-CRIT_MARGIN): return 2
    if (x < X_MIN+SOFT_MARGIN or x > X_MAX-SOFT_MARGIN or
        y < Y_MIN+SOFT_MARGIN or y > Y_MAX-SOFT_MARGIN): return 1
    return 0

def safe_pose(robot):
    p = robot.get_pose()
    if p and len(p)>=3: return float(p[0]),float(p[1]),float(p[2])
    return None

def usr(robot):
    robot.delay(400)
    try: vid = int(robot.id())
    except: vid = 0
    random.seed(vid*1103515245 & 0xFFFFFFFF)

    # state
    neighbors, last_seen = {}, {}
    last_hb = -1e9
    lastL = lastR = 0
    drift_phase = random.uniform(-math.pi, math.pi)

    # wake localization
    robot.set_vel(20,20); robot.delay(150)

    while True:
        pose = safe_pose(robot)
        if not pose:
            robot.set_vel(0,0); robot.delay(DT_MS); continue
        x,y,th = pose
        now = robot.get_clock()

        # LEDs
        b = boundary_state(x,y)
        if b==2:
            robot.set_led(100,0,0); robot.set_vel(0,0); robot.delay(DT_MS); continue
        elif b==1: robot.set_led(100,60,0) # amber
        else:      robot.set_led(0,70,80)  # calm teal

        # --- heartbeat send (finite diff for vx,vy) ---
        if now - last_hb >= HB_DT:
            x1,y1,_ = pose; t1 = now
            robot.delay(60)
            p2 = safe_pose(robot)
            if p2:
                x2,y2,th2 = p2; t2 = robot.get_clock()
                dt = max(1e-3, t2-t1)
                vx=(x2-x1)/dt; vy=(y2-y1)/dt
                try: robot.send_msg(struct.pack(HB_FMT, x2,y2,th2,vx,vy,vid))
                except: pass
                last_hb = t2; x,y,th = x2,y2,th2
            else:
                last_hb = now

        # --- receive ---
        for m in (robot.recv_msg() or []):
            try:
                nx,ny,nth,nvx,nvy,nid = struct.unpack(HB_FMT, m[:HB_BYTES])
                if int(nid)!=vid:
                    neighbors[int(nid)] = (nx,ny,nth,nvx,nvy)
                    last_seen[int(nid)] = now
            except: pass
        # prune
        cut = now - STALE_S
        for nid in list(neighbors.keys()):
            if last_seen.get(nid,0) < cut:
                neighbors.pop(nid,None); last_seen.pop(nid,None)

        # --- boidsy float field ---
        # environment
        ex,ey = soft_boundary_force(x,y)
        ox,oy = soft_obstacle_force(x,y)

        # migration to the left
        mx,my = -K_MIG, 0.0

        # neighbor terms
        repx=0.0; repy=0.0; cx=cy=0.0; ax=ay=0.0; n=0
        for _,(nx,ny,nth,nvx,nvy) in neighbors.items():
            dx,dy = x-nx, y-ny
            d2 = dx*dx+dy*dy
            if d2>1e-9:
                d = math.sqrt(d2)
                if d<SEP_RADIUS:
                    s = K_SEP * (SEP_RADIUS - d)/SEP_RADIUS
                    repx += s*(dx/d); repy += s*(dy/d)
                if d <= NEIGH_RADIUS:
                    cx += nx; cy += ny
                    ax += math.cos(nth); ay += math.sin(nth)
                    n+=1
        cohx=cohy=alx=aly=0.0
        if n>0:
            cx/=n; cy/=n
            cohx = K_COH*(cx-x); cohy = K_COH*(cy-y)
            ah = math.atan2(ay,ax)
            alx = K_ALI*math.cos(ah); aly = K_ALI*math.sin(ah)

        # softly varying curvature for “indirect”
        drift_phase += 0.03
        curl = CURVE_NOISE*math.sin(drift_phase)
        curlx = 0.0; curly = curl  # bias toward gentle lateral meander

        vx = ex+ox+mx+repx+cohx+alx+curlx
        vy = ey+oy+my+repy+cohy+aly+curly
        if abs(vx)<1e-6 and abs(vy)<1e-6: vx = 1e-3

        hdg = math.atan2(vy, vx)
        err = wrap(hdg - th)

        # speed: smooth, sustained
        fwd = FWD_BASE
        if b==1: fwd *= 0.8
        fwd = max(FWD_MIN, fwd)

        # map to wheels with *low jerk* (smoothing + slew rate)
        turn = clamp(TURN_K*err, -1.2, 1.2)
        lcmd = clamp(int(MAX_WHEEL*0.9*(fwd - 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
        rcmd = clamp(int(MAX_WHEEL*0.9*(fwd + 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)

        # slew limit
        if lcmd > lastL + VEL_SLEW: lcmd = lastL + VEL_SLEW
        if lcmd < lastL - VEL_SLEW: lcmd = lastL - VEL_SLEW
        if rcmd > lastR + VEL_SLEW: rcmd = lastR + VEL_SLEW
        if rcmd < lastR - VEL_SLEW: rcmd = lastR - VEL_SLEW

        left  = int((1-CMD_SMOOTH)*lcmd + CMD_SMOOTH*lastL)
        right = int((1-CMD_SMOOTH)*rcmd + CMD_SMOOTH*lastR)
        lastL, lastR = left, right
        robot.set_vel(left, right)

        robot.delay(DT_MS)
