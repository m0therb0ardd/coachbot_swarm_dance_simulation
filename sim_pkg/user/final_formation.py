# -*- coding: utf-8 -*-
# DYNAMIC_TWO_RING_FORMATION
#
# Bots start in a random cluster and assemble into two rings
# around the swarm's own center-of-mass (COM), wherever it is
# in the arena. Rings are auto-fit so they stay inside bounds.
#
# - Phase 1: COM discovery + gentle drift toward COM
# - Phase 2: Freeze COM, adjust radii/center to fit arena
# - Phase 3: Assign bots to inner/outer ring based on radius
# - Phase 4: Orbit with collision avoidance
#
# Inner ring  = RED   (heavy inner circle)
# Outer ring  = CYAN  (outer halo)

from __future__ import division
import math, struct, random, os

# ------------------- arena -------------------
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

# ------------------- nominal rings -------------------
R_INNER_NOM = 0.65
R_OUTER_NOM = 0.85

# ------------------- control -------------------
DT_MS      = 40
MAX_WHEEL  = 35
K_TURN     = 2.5

FWD_COM    = 0.45   # speed while drifting to COM
FWD_ORBIT  = 0.55   # base speed while orbiting

# radial + tangential
K_PULL     = 0.60   # pull toward target radius
ORBIT_SPEED = 0.50  # tangential velocity scale

# spacing
SEP_RADIUS = 0.28
K_SEP      = 0.25

# arena soft walls
SOFT_MARGIN = 0.08
CRIT_MARGIN = 0.02
SOFT_MAX_F  = 0.35

# heartbeats (x,y,th,vx,vy,id)
HB_FMT   = 'fffffi'
HB_BYTES = struct.calcsize(HB_FMT)
HB_DT    = 0.12
STALE_T  = 0.7

# ----------------- logging -------------------
LOG = None
LOG_OUT = None

def init_log():
    global LOG, LOG_OUT
    if LOG is None:
        try:
            LOG = open("experiment_log.txt", "a", 1)  # line-buffered
        except:
            LOG = None
    if LOG_OUT is None:
        try:
            LOG_OUT = open("/home/pi/experiment_output", "a", 1)
        except:
            LOG_OUT = None

def logw(msg):
    if not isinstance(msg, str):
        msg = str(msg)
    if not msg.endswith("\n"):
        msg += "\n"
    try:
        if LOG:
            LOG.write(msg)
            LOG.flush()
    except:
        pass
    try:
        if LOG_OUT:
            LOG_OUT.write(msg)
            LOG_OUT.flush()
    except:
        pass

# -------------- helpers -------------------------
def clamp(v, lo, hi):
    if v < lo: return lo
    if v > hi: return hi
    return v

def wrap(a):
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

def boundary_state(x, y):
    # 2 = critical, 1 = warning, 0 = safe
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
        fx += SOFT_MAX_F * (1.0 - (x - X_MIN) / SOFT_MARGIN)
    elif x > X_MAX - SOFT_MARGIN:
        fx -= SOFT_MAX_F * (1.0 - (X_MAX - x) / SOFT_MARGIN)
    if y < Y_MIN + SOFT_MARGIN:
        fy += SOFT_MAX_F * (1.0 - (y - Y_MIN) / SOFT_MARGIN)
    elif y > Y_MAX - SOFT_MARGIN:
        fy -= SOFT_MAX_F * (1.0 - (Y_MAX - y) / SOFT_MARGIN)
    return fx, fy

# -------------- main ---------------------------
def usr(robot):
    init_log()

    try:
        vid = int(robot.virtual_id())
    except:
        vid = 0

    random.seed((vid * 1103515245) & 0xFFFFFFFF)

    neigh = {}       # nid -> (x,y,th,vx,vy)
    last_seen = {}   # nid -> t_last
    last_hb = -1e9
    lastL = lastR = 0

    # wake localization a bit
    robot.set_vel(20, 20)
    robot.delay(150)
    robot.set_vel(0, 0)
    robot.delay(150)

    logw("dynamic_two_rings: START vid=%d" % vid)

    # COM discovery state
    COM_PHASE = True
    t_start = robot.get_clock()
    COM_X = 0.0
    COM_Y = 0.0

    # variables that get frozen after COM phase:
    COM_X0 = 0.0
    COM_Y0 = 0.0
    R_INNER = R_INNER_NOM
    R_OUTER = R_OUTER_NOM
    ring_group = -1  # 0 = inner, 1 = outer

    try:
        while True:
            pose = safe_pose(robot)
            if not pose:
                robot.set_vel(0, 0)
                robot.delay(DT_MS)
                continue

            x, y, th = pose
            now = robot.get_clock()

            # --------------- boundary LEDs & safety ----------------
            b = boundary_state(x, y)
            if b == 2:
                robot.set_led(150, 0, 0)
                robot.set_vel(0, 0)
                robot.delay(DT_MS)
                continue
            elif b == 1:
                robot.set_led(150, 50, 0)  # amber near wall
            else:
                # we'll pick colors below based on phase + ring_group
                pass

            # --------------- heartbeats send -----------------------
            if now - last_hb >= HB_DT:
                x1, y1, t1 = x, y, now
                robot.delay(40)
                p2 = safe_pose(robot)
                if p2:
                    x2, y2, th2 = float(p2[0]), float(p2[1]), float(p2[2])
                    t2 = robot.get_clock()
                    dt = max(1e-3, t2 - t1)
                    vx_hb = (x2 - x1) / dt
                    vy_hb = (y2 - y1) / dt
                    try:
                        robot.send_msg(struct.pack(HB_FMT,
                                                   x2, y2, th2,
                                                   vx_hb, vy_hb,
                                                   vid))
                    except:
                        pass
                    last_hb = t2
                    x, y, th = x2, y2, th2
                else:
                    last_hb = now

            # --------------- heartbeats recv -----------------------
            msgs = robot.recv_msg() or []
            for m in msgs:
                try:
                    nx, ny, nth, nvx, nvy, nid = struct.unpack(HB_FMT,
                                                               m[:HB_BYTES])
                    nid = int(nid)
                    if nid != vid:
                        neigh[nid] = (nx, ny, nth, nvx, nvy)
                        last_seen[nid] = now
                except:
                    pass

            # prune stale neighbors
            for nid in list(neigh.keys()):
                if now - last_seen.get(nid, 0.0) > STALE_T:
                    neigh.pop(nid, None)
                    last_seen.pop(nid, None)

            # --------------- COM compute (always) ------------------
            xs = [x]
            ys = [y]
            for (nx, ny, nth, nvx, nvy) in neigh.values():
                xs.append(nx)
                ys.append(ny)
            COM_X = sum(xs) / len(xs)
            COM_Y = sum(ys) / len(ys)

            # --------------- PHASE 1: drift to COM ----------------
            if COM_PHASE:
                # LED color during COM find: teal-ish
                if b == 0:
                    robot.set_led(0, 170, 190)

                if now - t_start < 4.0:
                    # move toward COM with soft wall push
                    dx = COM_X - x
                    dy = COM_Y - y
                    ang = math.atan2(dy, dx)
                    err = wrap(ang - th)

                    fx, fy = soft_boundary_force(x, y)
                    # simple blend: heading nudged by boundary
                    # (just project env force into heading)
                    if abs(fx) > 1e-6 or abs(fy) > 1e-6:
                        env_angle = math.atan2(dy + fy, dx + fx)
                        err = wrap(env_angle - th)

                    fwd = FWD_COM
                    turn = clamp(K_TURN * err, -1.5, 1.5)

                    L = clamp(int(MAX_WHEEL * (fwd - 0.8 * turn)),
                              -MAX_WHEEL, MAX_WHEEL)
                    R = clamp(int(MAX_WHEEL * (fwd + 0.8 * turn)),
                              -MAX_WHEEL, MAX_WHEEL)

                    robot.set_vel(L, R)
                    robot.delay(DT_MS)
                    continue
                else:
                    # ---- lock COM and adjust ring center/radii to fit arena ----
                    COM_PHASE = False
                    COM_X0 = COM_X
                    COM_Y0 = COM_Y

                    # Start with nominal radii
                    R_INNER = R_INNER_NOM
                    R_OUTER = R_OUTER_NOM

                    # Global max radius that fits arena (with margin)
                    margin = 0.05
                    maxR_x = 0.5 * ((X_MAX - X_MIN) - 2.0 * margin)
                    maxR_y = 0.5 * ((Y_MAX - Y_MIN) - 2.0 * margin)
                    maxR   = maxR_x if maxR_x < maxR_y else maxR_y

                    if R_OUTER > maxR:
                        R_OUTER = maxR
                    if R_INNER > R_OUTER - 0.20:
                        R_INNER = R_OUTER - 0.20
                    if R_INNER < 0.40:
                        R_INNER = 0.40

                    # shift COM inside arena so rings fit
                    min_cx = X_MIN + margin + R_OUTER
                    max_cx = X_MAX - margin - R_OUTER
                    min_cy = Y_MIN + margin + R_OUTER
                    max_cy = Y_MAX - margin - R_OUTER

                    COM_X0 = clamp(COM_X0, min_cx, max_cx)
                    COM_Y0 = clamp(COM_Y0, min_cy, max_cy)

                    # assign ring group based on radius at lock
                    dx0 = x - COM_X0
                    dy0 = y - COM_Y0
                    r0  = math.hypot(dx0, dy0)
                    split_r = 0.5 * (R_INNER + R_OUTER)
                    if r0 <= split_r:
                        ring_group = 0  # inner
                    else:
                        ring_group = 1  # outer

                    logw("dynamic_two_rings: LOCK vid=%d COM=(%.2f,%.2f) "
                         "R_IN=%.2f R_OUT=%.2f ring_group=%d r0=%.2f"
                         % (vid, COM_X0, COM_Y0,
                            R_INNER, R_OUTER, ring_group, r0))

            # --------------- PHASE 2: orbit on two rings -----------
            # LED colors: inner red, outer cyan (unless near wall)
            if b == 0:
                if ring_group == 0:
                    robot.set_led(255, 30, 30)   # inner ring red
                elif ring_group == 1:
                    robot.set_led(40, 180, 255)  # outer ring cyan
                else:
                    robot.set_led(100, 100, 100) # fallback / unknown

            # radius relative to locked COM
            rx = x - COM_X0
            ry = y - COM_Y0
            r  = math.hypot(rx, ry)
            if r < 1e-6:
                ux = 1.0
                uy = 0.0
            else:
                ux = rx / r
                uy = ry / r

            target_r = R_INNER if ring_group == 0 else R_OUTER
            dr = target_r - r

            # radial pull
            vr = K_PULL * dr

            # tangential (choose CCW orbit)
            tx = -uy
            ty = ux

            vx = vr * ux + ORBIT_SPEED * tx
            vy = vr * uy + ORBIT_SPEED * ty

            # soft wall pushback
            bfx, bfy = soft_boundary_force(x, y)
            vx += bfx
            vy += bfy

            # separation
            repx = repy = 0.0
            for (nx, ny, nth, nvx, nvy) in neigh.values():
                dxn = x - nx
                dyn = y - ny
                d2 = dxn*dxn + dyn*dyn
                if d2 < SEP_RADIUS*SEP_RADIUS and d2 > 1e-9:
                    d = math.sqrt(d2)
                    rep = K_SEP * (SEP_RADIUS - d) / SEP_RADIUS
                    repx += rep * (dxn / d)
                    repy += rep * (dyn / d)

            vx += repx
            vy += repy

            if abs(vx) < 1e-6 and abs(vy) < 1e-6:
                vx = 1e-3

            ang = math.atan2(vy, vx)
            err = wrap(ang - th)

            fwd = FWD_ORBIT
            if b == 1:
                fwd *= 0.7

            turn = clamp(K_TURN * err, -1.5, 1.5)

            lcmd = clamp(int(MAX_WHEEL * (fwd - 0.8 * turn)),
                         -MAX_WHEEL, MAX_WHEEL)
            rcmd = clamp(int(MAX_WHEEL * (fwd + 0.8 * turn)),
                         -MAX_WHEEL, MAX_WHEEL)

            # simple smoothing
            if lcmd > lastL + 10: lcmd = lastL + 10
            if lcmd < lastL - 10: lcmd = lastL - 10
            if rcmd > lastR + 10: rcmd = lastR + 10
            if rcmd < lastR - 10: rcmd = lastR - 10

            left  = int(0.82 * lcmd + 0.18 * lastL)
            right = int(0.82 * rcmd + 0.18 * lastR)
            lastL, lastR = left, right

            robot.set_vel(left, right)
            robot.delay(DT_MS)

    except Exception as e:
        logw("dynamic_two_rings: ERROR vid=%d %s" % (vid, repr(e)))
        try:
            robot.set_led(150, 0, 0)
            robot.set_vel(0, 0)
        except:
            pass
        raise
    finally:
        try:
            robot.set_vel(0, 0)
        except:
            pass
