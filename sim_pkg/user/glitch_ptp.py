# -*- coding: utf-8 -*-
# Jitter P2P (SIM) — short random darts with peer-aware avoidance + dancer ring safety.
# SIM NOTES:
#   - Uses robot.id() and print() for logging.
#   - To port to physical testbed, change get_id() to robot.virtual_id()
#     and change logw() to use the provided log.write(...).

import math, struct, random

# -------- arena & dancer obstacle (meters) --------
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35
FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
OBST_CX, OBST_CY = (-0.1, 0.475)

# -------- drive / loop --------
MAX_WHEEL   = 35
TURN_K      = 3.0
FWD_JITTER  = 0.95     # forward during dart
FWD_AIM     = 0.35     # forward while aiming
FWD_MIN     = 0.38
CMD_SMOOTH  = 0.20
DT_MS       = 40

# -------- jitter timing (seconds) --------
JITTER_LO   = 0.55
JITTER_HI   = 1.20
AIM_TIME    = 0.20
DART_TIME   = 0.35
COOLDOWN    = 0.25

# -------- collision safety --------
NEIGH_RADIUS = 0.75
SEP_RADIUS   = 0.24
PANIC_RADIUS = 0.18
WALL_MARGIN  = 0.08
WALL_CRIT    = 0.02
MAX_SOFT_F   = 0.50

# -------- heading sampling --------
SAMPLES      = 20
BIAS_STD     = 0.9  # radians; set 0 for uniform

# -------- messaging (heartbeats) --------
HB_FMT   = 'fffffi'    # (x, y, th, vx, vy, vid) 24 bytes
HB_BYTES = struct.calcsize(HB_FMT)
HB_DT    = 0.12        # ~8–9 Hz
STALE_S  = 0.7

# ---------- small helpers ----------
def clamp(v, lo, hi): return lo if v < lo else (hi if v > hi else v)

def wrap_angle(a):
    while a >  math.pi: a -= 2.0*math.pi
    while a <= -math.pi: a += 2.0*math.pi
    return a

def soft_boundary_force(x, y):
    fx = fy = 0.0
    if x < X_MIN + WALL_MARGIN:
        fx += MAX_SOFT_F * (1.0 - (x - X_MIN)/WALL_MARGIN)
    elif x > X_MAX - WALL_MARGIN:
        fx -= MAX_SOFT_F * (1.0 - (X_MAX - x)/WALL_MARGIN)
    if y < Y_MIN + WALL_MARGIN:
        fy += MAX_SOFT_F * (1.0 - (y - Y_MIN)/WALL_MARGIN)
    elif y > Y_MAX - WALL_MARGIN:
        fy -= MAX_SOFT_F * (1.0 - (Y_MAX - y)/WALL_MARGIN)
    return fx, fy

def boundary_state(x, y):
    if (x < X_MIN + WALL_CRIT or x > X_MAX - WALL_CRIT or
        y < Y_MIN + WALL_CRIT or y > Y_MAX - WALL_CRIT):
        return 2
    if (x < X_MIN + WALL_MARGIN or x > X_MAX - WALL_MARGIN or
        y < Y_MIN + WALL_MARGIN or y > Y_MAX - WALL_MARGIN):
        return 1
    return 0

def obstacle_push(x, y, max_force=0.8, buffer_width=0.12):
    dx = x - OBST_CX; dy = y - OBST_CY; r = math.hypot(dx, dy)
    if r < SAFE_BUBBLE + buffer_width:
        if r < 1e-6: return max_force, 0.0
        s = max(0.0, (SAFE_BUBBLE + buffer_width - r) / buffer_width) * max_force
        return s * (dx/r), s * (dy/r)
    return 0.0, 0.0

def safe_pose(robot):
    p = robot.get_pose()
    if p and len(p) >= 3: return float(p[0]), float(p[1]), float(p[2])
    return None

def heading_to_wheels(err, fwd, lastL, lastR):
    turn = clamp(TURN_K * err, -1.5, 1.5)
    lcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL,  MAX_WHEEL)
    rcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL,  MAX_WHEEL)
    left  = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
    right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
    return left, right

# --- portability shims (SIM ⇄ PHYSICAL) ---
def get_id(robot):
    """SIM: robot.id ; PHYSICAL: return robot.virtual_id()"""
    try: return int(robot.id())
    except: return 0

def logw(msg):
    """SIM: print ; PHYSICAL: log.write(msg + '\\n')"""
    print(msg)

# ---------- heading chooser ----------
def choose_safe_heading(x, y, th, neighbors):
    best_h, best_score = th, -1e9
    for _ in range(SAMPLES):
        cand = wrap_angle(th + (random.gauss(0.0, BIAS_STD) if BIAS_STD > 0 else random.uniform(-math.pi, math.pi)))
        step = 0.18
        px = x + step * math.cos(cand)
        py = y + step * math.sin(cand)

        score = 0.0
        for _, (nx, ny, _, _, _) in neighbors.items():
            d = math.hypot(px - nx, py - ny)
            if d < PANIC_RADIUS: score -= 1000.0
            elif d < SEP_RADIUS: score -= 200.0 * (SEP_RADIUS - d)
            else: score += min(d, 0.6)

        od = math.hypot(px - OBST_CX, py - OBST_CY)
        if od < SAFE_BUBBLE: score -= 800.0
        else: score += min(od - SAFE_BUBBLE, 0.5)

        if (px < X_MIN + WALL_MARGIN or px > X_MAX - WALL_MARGIN or
            py < Y_MIN + WALL_MARGIN or py > Y_MAX - WALL_MARGIN):
            score -= 500.0

        if score > best_score:
            best_score, best_h = score, cand
    return best_h

# ---------------- main entry ----------------
def usr(robot):
    robot.delay(400)  # sim settle
    vid = get_id(robot)
    random.seed((vid if vid is not None else 0)*1103515245 & 0xFFFFFFFF)

    neighbors = {}   # vid -> (x,y,th,vx,vy)
    last_seen = {}   # vid -> t
    last_hb   = -1e9

    next_dart_at = 0.0
    aim_until = 0.0
    dart_until = 0.0
    cooldown_until = 0.0
    target_h = 0.0
    evasive_until = 0.0
    lastL = lastR = 0
    last_log = 0.0

    # kick once so pose starts updating
    robot.set_vel(20, 20); robot.delay(120)

    while True:
        pose = safe_pose(robot)
        if pose is None:
            robot.set_vel(0,0); robot.delay(DT_MS); continue
        x, y, th = pose
        now = robot.get_clock()

        # boundary LEDs (sim-safe colors)
        b = boundary_state(x,y)
        if b == 2:
            robot.set_led(100,0,0); robot.set_vel(0,0); robot.delay(DT_MS); continue
        elif b == 1: robot.set_led(120,60,0)
        else:        robot.set_led(40,120,200)

        # --- heartbeat send (finite diff for vx,vy) ---
        if now - last_hb >= HB_DT:
            x1,y1,_ = pose; t1 = now
            robot.delay(50)
            p2 = safe_pose(robot)
            if p2:
                x2,y2,th2 = p2; t2 = robot.get_clock()
                dt = max(1e-3, t2 - t1)
                vx = (x2 - x1)/dt; vy = (y2 - y1)/dt
                try:
                    robot.send_msg(struct.pack(HB_FMT, float(x2), float(y2), float(th2), float(vx), float(vy), int(vid)))
                except: pass
                last_hb = t2
                x,y,th = x2,y2,th2
            else:
                last_hb = now

        # --- receive heartbeats ---
        msgs = robot.recv_msg() or []
        for m in msgs:
            try:
                nx, ny, nth, nvx, nvy, nid = struct.unpack(HB_FMT, m[:HB_BYTES])
                if int(nid) != int(vid):
                    neighbors[int(nid)] = (nx, ny, nth, nvx, nvy)
                    last_seen[int(nid)] = now
            except: pass

        # prune stale neighbors
        cut = now - STALE_S
        for nid in list(neighbors.keys()):
            if last_seen.get(nid, 0.0) < cut:
                neighbors.pop(nid, None); last_seen.pop(nid, None)

        # environmental nudges
        fx, fy = soft_boundary_force(x, y)
        ox, oy = obstacle_push(x, y)
        env_hdg = math.atan2(fy + oy, fx + ox) if (fx*fx + fy*fy + ox*ox + oy*oy) > 1e-9 else th

        # PANIC evade if closest neighbor too near
        nearest_d, nx, ny = 1e9, None, None
        for _, (qx,qy,_,_,_) in neighbors.items():
            d = math.hypot(x-qx, y-qy)
            if d < nearest_d:
                nearest_d, nx, ny = d, qx, qy
        if nearest_d < PANIC_RADIUS:
            evasive_until = now + 0.40
            away = math.atan2(y - ny, x - nx)
            # blend slightly with env to avoid walls/obstacle
            da = wrap_angle(env_hdg - away)
            target_h = wrap_angle(away + 0.25 * da)

        # schedule a new dart if free
        if now >= cooldown_until and now >= evasive_until and now >= next_dart_at and now >= dart_until:
            target_h = choose_safe_heading(x, y, th, neighbors)
            aim_until   = now + AIM_TIME
            dart_until  = aim_until + DART_TIME
            cooldown_until = dart_until + COOLDOWN
            next_dart_at = cooldown_until + random.uniform(JITTER_LO, JITTER_HI)
            robot.set_led(0,180,80)  # mint during dart cycle

        # control mode: evade > dart > aim > idle
        if now < evasive_until:
            err = wrap_angle(target_h - th); fwd = FWD_AIM
        elif now < aim_until:
            err = wrap_angle(target_h - th); fwd = FWD_AIM
        elif now < dart_until:
            err = wrap_angle(target_h - th); fwd = FWD_JITTER
        else:
            err = wrap_angle(env_hdg - th); fwd = max(FWD_MIN, 0.45)

        if b == 1: fwd *= 0.75
        if fwd < FWD_MIN: fwd = FWD_MIN

        left, right = heading_to_wheels(err, fwd, lastL, lastR)
        lastL, lastR = left, right
        robot.set_vel(left, right)

        if now - last_log > 2.0:
            state = ("evade" if now < evasive_until else
                     "dart" if now < dart_until else
                     "aim"  if now < aim_until else
                     "idle")
            logw(f"[SIM jitter_p2p] id={vid} n={len(neighbors)} state={state}")
            last_log = now

        robot.delay(DT_MS)
