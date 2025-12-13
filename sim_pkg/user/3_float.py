
# -*- coding: utf-8 -*-
# GLIDE (CoachBot): light, sustained, direct-ish
# New version:
# - Computes geometric center of all bots at start
# - Classifies inner vs outer ring based on radius
# - Locks each bot to its *own* initial radius and angle
# - Still uses flocking + sinusoidal wave + upward drift
# - LEDs: blue while moving, red on hard boundary stop

from __future__ import division
import math, struct, random

# --- field (meters) ---
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

# --- control/loop ---
MAX_WHEEL   = 35
TURN_K      = 2.2
FWD_BASE    = 0.65
FWD_MIN     = 0.35
DT_MS       = 40
CMD_SMOOTH  = 0.38
VEL_SLEW    = 7

# --- flocking gains ---
K_MIG   = 0.08
K_SEP   = 0.16
K_ALI   = 0.14
K_COH   = 0.08

SEP_RADIUS   = 0.26
NEIGH_RADIUS = 0.75

# --- sinusoidal wave parameters ---
WAVE_AMP   = 0.18
WAVE_FREQ  = 0.05
PHASE_STEP = 0.6

# --- boundary softness (walls only) ---
SOFT_MARGIN = 0.08
CRIT_MARGIN = 0.02
SOFT_MAX_F  = 0.35

# --- P2P heartbeats ---
HB_FMT   = 'fffffi'  # x,y,th,vx,vy,id
HB_BYTES = struct.calcsize(HB_FMT)
HB_DT    = 0.12
STALE_S  = 0.7

def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def wrap(a):
    while a > math.pi:
        a -= 2*math.pi
    while a <= -math.pi:
        a += 2*math.pi
    return a

def soft_boundary_force(x, y):
    fx = fy = 0.0
    if x < X_MIN + SOFT_MARGIN:
        fx += SOFT_MAX_F * (1 - (x - X_MIN) / SOFT_MARGIN)
    elif x > X_MAX - SOFT_MARGIN:
        fx -= SOFT_MAX_F * (1 - (X_MAX - x) / SOFT_MARGIN)
    if y < Y_MIN + SOFT_MARGIN:
        fy += SOFT_MAX_F * (1 - (y - Y_MIN) / SOFT_MARGIN)
    elif y > Y_MAX - SOFT_MARGIN:
        fy -= SOFT_MAX_F * (1 - (Y_MAX - y) / SOFT_MARGIN)
    return fx, fy

def boundary_state(x, y):
    # 2 = hard wall, 1 = soft margin, 0 = safe
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


def usr(robot):
    robot.delay(400)

    # ID + per-robot random seed
    try:
        vid = int(robot.id)
    except:
        vid = 0
    random.seed(vid * 1103515245 & 0xFFFFFFFF)

    # per-robot phase offset for the sine wave
    phase0 = (vid % 12) * PHASE_STEP

    print('Robot %d starting GLIDE (locked rings, wave + drift)' % vid)

    
    # 1) GOSSIP TO FIND CENTER + RINGS
    GOSSIP_TIME = 2.0
    POSE_FMT = 'ffi'
    POSE_BYTES = struct.calcsize(POSE_FMT)

    poses = {}  # id -> (x, y)
    t0 = robot.get_clock()

    while robot.get_clock() - t0 < GOSSIP_TIME:
        pose = safe_pose(robot)
        if pose:
            xg, yg, thg = pose
            try:
                robot.send_msg(struct.pack(POSE_FMT, xg, yg, vid))
            except:
                pass
            poses[vid] = (xg, yg)

        msgs = robot.recv_msg()
        if msgs:
            if not isinstance(msgs, list):
                msgs = [msgs]
            for m in msgs:
                if not m:
                    continue
                # during gossip, messages should be POSE_FMT
                try:
                    px, py, pid = struct.unpack(POSE_FMT, m[:POSE_BYTES])
                    poses[int(pid)] = (float(px), float(py))
                except:
                    # ignore heartbeats or other messages that might slip in
                    pass

        robot.delay(50)

    if poses:
        xs = [poses[k][0] for k in poses]
        ys = [poses[k][1] for k in poses]
        CX = sum(xs) / float(len(xs))
        CY = sum(ys) / float(len(ys))
    else:
        # fallback use my own pose as center
        myp = safe_pose(robot)
        if myp:
            CX, CY = myp[0], myp[1]
        else:
            CX, CY = 0.0, 0.0

    # compute each robots radius from center
    radii = []
    for pid, (px, py) in poses.items():
        dx = px - CX
        dy = py - CY
        r = math.hypot(dx, dy)
        radii.append((r, int(pid)))
    radii.sort()  # increasing radius

    # choose threshold between inner and outer
    total = len(radii)
    if total >= 2:
        split_idx = total // 2
        # if odd, inner gets smaller half, outer gets larger half
        if split_idx < total:
            r_thresh = 0.5 * (radii[split_idx - 1][0] + radii[split_idx][0])
        else:
            r_thresh = radii[split_idx - 1][0]
    else:
        r_thresh = 0.0  # degenerate case

    # compute initial radius and angle, and ring label
    pose0 = safe_pose(robot)
    if pose0:
        x0, y0, th0 = pose0
    else:
        x0 = y0 = th0 = 0.0

    dx0 = x0 - CX
    dy0 = y0 - CY
    r0 = math.hypot(dx0, dy0)
    ang0 = math.atan2(dy0, dx0) if r0 > 1e-6 else 0.0

    if r0 <= r_thresh:
        my_ring = 0   # inner
    else:
        my_ring = 1   # outer

    # this is the formation we lock to:
    R_TARGET = r0
    target_angle = ang0

    # strong-ish ring lock (we can tune these later)
    KR = 0.9    # radial pull
    KTH = 0.7   # angular pull

    
    # 2) NORMAL GLIDE LOOP
    neighbors = {}
    last_seen = {}
    last_hb   = -1e9
    lastL = 0
    lastR = 0

    # wake localization a bit
    robot.set_vel(20, 20)
    robot.delay(150)

    try:
        while True:
            pose = safe_pose(robot)
            if not pose:
                robot.set_vel(0, 0)
                robot.delay(DT_MS)
                continue

            x, y, th = pose
            now = robot.get_clock()

            # boundary handling
            b = boundary_state(x, y)
            if b == 2:
                # hard boundary -> full stop for this robot
                robot.set_led(255, 0, 0)
                robot.set_vel(0, 0)
                robot.delay(DT_MS)
                continue

            # all good / soft boundary: blue LED
            robot.set_led(0, 0, 255)

            # --- heartbeat send (finite diff vx,vy) ---
            if now - last_hb >= HB_DT:
                x1, y1 = x, y
                t1 = now
                robot.delay(60)
                p2 = safe_pose(robot)
                if p2:
                    x2, y2, th2 = float(p2[0]), float(p2[1]), float(p2[2])
                    t2 = robot.get_clock()
                    dt = max(1e-3, t2 - t1)
                    vx_hb = (x2 - x1) / dt
                    vy_hb = (y2 - y1) / dt
                    try:
                        robot.send_msg(struct.pack(HB_FMT, x2, y2, th2,
                                                   vx_hb, vy_hb, vid))
                    except:
                        pass
                    last_hb = t2
                    x, y, th = x2, y2, th2
                else:
                    last_hb = now

            # --- receive neighbor heartbeats ---
            msgs = robot.recv_msg()
            if msgs:
                if not isinstance(msgs, list):
                    msgs = [msgs]
                for m in msgs:
                    if not m:
                        continue
                    # ignore gossiplike short messages
                    if len(m) < HB_BYTES:
                        continue
                    try:
                        nx, ny, nth, nvx, nvy, nid = struct.unpack(HB_FMT,
                                                                   m[:HB_BYTES])
                        if int(nid) != vid:
                            neighbors[int(nid)] = (nx, ny, nth, nvx, nvy)
                            last_seen[int(nid)] = now
                    except:
                        pass

            # prune stale neighbors
            cut = now - STALE_S
            for nid in list(neighbors.keys()):
                if last_seen.get(nid, 0) < cut:
                    neighbors.pop(nid, None)
                    last_seen.pop(nid, None)

            # --- environment forces (walls only) ---
            ex, ey = soft_boundary_force(x, y)

            # --- sinusoidal wave field (x oscillation) ---
            wave_phase = 2.0 * math.pi * WAVE_FREQ * now + phase0
            vx_wave = WAVE_AMP * math.sin(wave_phase)
            vy_wave = 0.0

            # --- upward migration ---
            mx = 0.0
            my = K_MIG

            # --- neighbor terms (soft flocking) ---
            repx = repy = 0.0
            cx = cy = 0.0
            ax = ay = 0.0
            n = 0

            for nid in neighbors:
                nx, ny, nth, nvx, nvy = neighbors[nid]
                dx = x - nx
                dy = y - ny
                d2 = dx*dx + dy*dy
                if d2 > 1e-9:
                    d = math.sqrt(d2)
                    if d < SEP_RADIUS:
                        s = K_SEP * (SEP_RADIUS - d) / SEP_RADIUS
                        repx += s * (dx / d)
                        repy += s * (dy / d)
                    if d <= NEIGH_RADIUS:
                        cx += nx
                        cy += ny
                        ax += math.cos(nth)
                        ay += math.sin(nth)
                        n += 1

            cohx = cohy = 0.0
            alx = aly = 0.0

            if n > 0:
                cx /= n
                cy /= n
                cohx = K_COH * (cx - x)
                cohy = K_COH * (cy - y)
                ah = math.atan2(ay, ax)
                alx = K_ALI * math.cos(ah)
                aly = K_ALI * math.sin(ah)

            # --- combine fields (pre ring-lock) ---
            vx = ex + vx_wave + mx + repx + cohx + alx
            vy = ey + vy_wave + my + repy + cohy + aly

            
            # RING LOCK (formation)
            dx_c = x - CX
            dy_c = y - CY
            r_now = math.hypot(dx_c, dy_c)

            if r_now > 1e-6:
                urx = dx_c / r_now
                ury = dy_c / r_now
                utx = -ury
                uty = urx
            else:
                urx, ury = 1.0, 0.0
                utx, uty = 0.0, 1.0

            radial_error = R_TARGET - r_now
            ang_now = math.atan2(dy_c, dx_c) if r_now > 1e-6 else target_angle
            ang_error = wrap(target_angle - ang_now)

            # add ring lock
            vx += KR * radial_error * urx + KTH * ang_error * utx
            vy += KR * radial_error * ury + KTH * ang_error * uty

            # convert to wheels
            # 
            if abs(vx) < 1e-6 and abs(vy) < 1e-6:
                vx = 1e-3  # avoid NaN

            hdg = math.atan2(vy, vx)
            err = wrap(hdg - th)

            fwd = FWD_BASE
            if b == 1:
                fwd *= 0.8
            if fwd < FWD_MIN:
                fwd = FWD_MIN

            turn = clamp(TURN_K * err, -1.2, 1.2)
            lcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)),
                         -MAX_WHEEL, MAX_WHEEL)
            rcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)),
                         -MAX_WHEEL, MAX_WHEEL)

            # slew limit
            if lcmd > lastL + VEL_SLEW: lcmd = lastL + VEL_SLEW
            if lcmd < lastL - VEL_SLEW: lcmd = lastL - VEL_SLEW
            if rcmd > lastR + VEL_SLEW: rcmd = lastR + VEL_SLEW
            if rcmd < lastR - VEL_SLEW: rcmd = lastR - VEL_SLEW

            left  = int((1 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
            right = int((1 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
            lastL, lastR = left, right

            robot.set_vel(left, right)
            robot.delay(DT_MS)

    except Exception as e:
        try:
            robot.set_vel(0, 0)
            robot.set_led(255, 0, 0)
        except:
            pass
        print('[GLIDE] ERROR id=%d: %s' % (vid, repr(e)))
        raise
    finally:
        try:
            robot.set_vel(0, 0)
        except:
            pass
from __future__ import division
import math, struct, random

# --- field (meters) ---
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

# --- control/loop ---
MAX_WHEEL   = 35
TURN_K      = 2.2
FWD_BASE    = 0.65
FWD_MIN     = 0.35
DT_MS       = 40
CMD_SMOOTH  = 0.38
VEL_SLEW    = 7

# --- flocking gains ---
K_MIG   = 0.08
K_SEP   = 0.16
K_ALI   = 0.14
K_COH   = 0.08

SEP_RADIUS   = 0.26
NEIGH_RADIUS = 0.75

# --- sinusoidal wave parameters ---
WAVE_AMP   = 0.18
WAVE_FREQ  = 0.05
PHASE_STEP = 0.6

# --- boundary softness (walls only) ---
SOFT_MARGIN = 0.08
CRIT_MARGIN = 0.02
SOFT_MAX_F  = 0.35

# --- P2P heartbeats ---
HB_FMT   = 'fffffi'  # x,y,th,vx,vy,id
HB_BYTES = struct.calcsize(HB_FMT)
HB_DT    = 0.12
STALE_S  = 0.7

def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def wrap(a):
    while a > math.pi:
        a -= 2*math.pi
    while a <= -math.pi:
        a += 2*math.pi
    return a

def soft_boundary_force(x, y):
    fx = fy = 0.0
    if x < X_MIN + SOFT_MARGIN:
        fx += SOFT_MAX_F * (1 - (x - X_MIN) / SOFT_MARGIN)
    elif x > X_MAX - SOFT_MARGIN:
        fx -= SOFT_MAX_F * (1 - (X_MAX - x) / SOFT_MARGIN)
    if y < Y_MIN + SOFT_MARGIN:
        fy += SOFT_MAX_F * (1 - (y - Y_MIN) / SOFT_MARGIN)
    elif y > Y_MAX - SOFT_MARGIN:
        fy -= SOFT_MAX_F * (1 - (Y_MAX - y) / SOFT_MARGIN)
    return fx, fy

def boundary_state(x, y):
    # 2 = hard wall, 1 = soft margin, 0 = safe
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


def usr(robot):
    robot.delay(400)

    # ID + per-robot random seed
    try:
        vid = int(robot.id)
    except:
        vid = 0
    random.seed(vid * 1103515245 & 0xFFFFFFFF)

    # per-robot phase offset for the sine wave
    phase0 = (vid % 12) * PHASE_STEP

    print('Robot %d starting GLIDE (locked rings, wave + drift)' % vid)

    
    # 1) GOSSIP TO FIND CENTER + RINGS
    GOSSIP_TIME = 2.0
    POSE_FMT = 'ffi'
    POSE_BYTES = struct.calcsize(POSE_FMT)

    poses = {}  # id -> (x, y)
    t0 = robot.get_clock()

    while robot.get_clock() - t0 < GOSSIP_TIME:
        pose = safe_pose(robot)
        if pose:
            xg, yg, thg = pose
            try:
                robot.send_msg(struct.pack(POSE_FMT, xg, yg, vid))
            except:
                pass
            poses[vid] = (xg, yg)

        msgs = robot.recv_msg()
        if msgs:
            if not isinstance(msgs, list):
                msgs = [msgs]
            for m in msgs:
                if not m:
                    continue
                # during gossip, messages should be POSE_FMT
                try:
                    px, py, pid = struct.unpack(POSE_FMT, m[:POSE_BYTES])
                    poses[int(pid)] = (float(px), float(py))
                except:
                    # ignore heartbeats or other messages that might slip in
                    pass

        robot.delay(50)

    if poses:
        xs = [poses[k][0] for k in poses]
        ys = [poses[k][1] for k in poses]
        CX = sum(xs) / float(len(xs))
        CY = sum(ys) / float(len(ys))
    else:
        # fallback use my own pose as center
        myp = safe_pose(robot)
        if myp:
            CX, CY = myp[0], myp[1]
        else:
            CX, CY = 0.0, 0.0

    # compute each robots radius from center
    radii = []
    for pid, (px, py) in poses.items():
        dx = px - CX
        dy = py - CY
        r = math.hypot(dx, dy)
        radii.append((r, int(pid)))
    radii.sort()  # increasing radius

    # choose threshold between inner and outer
    total = len(radii)
    if total >= 2:
        split_idx = total // 2
        # if odd, inner gets smaller half, outer gets larger half
        if split_idx < total:
            r_thresh = 0.5 * (radii[split_idx - 1][0] + radii[split_idx][0])
        else:
            r_thresh = radii[split_idx - 1][0]
    else:
        r_thresh = 0.0  # degenerate case

    # compute initial radius and angle, and ring label
    pose0 = safe_pose(robot)
    if pose0:
        x0, y0, th0 = pose0
    else:
        x0 = y0 = th0 = 0.0

    dx0 = x0 - CX
    dy0 = y0 - CY
    r0 = math.hypot(dx0, dy0)
    ang0 = math.atan2(dy0, dx0) if r0 > 1e-6 else 0.0

    if r0 <= r_thresh:
        my_ring = 0   # inner
    else:
        my_ring = 1   # outer

    # this is the formation we lock to:
    R_TARGET = r0
    target_angle = ang0

    # strong-ish ring lock (we can tune these later)
    KR = 0.9    # radial pull
    KTH = 0.7   # angular pull

    
    # 2) NORMAL GLIDE LOOP
    neighbors = {}
    last_seen = {}
    last_hb   = -1e9
    lastL = 0
    lastR = 0

    # wake localization a bit
    robot.set_vel(20, 20)
    robot.delay(150)

    try:
        while True:
            pose = safe_pose(robot)
            if not pose:
                robot.set_vel(0, 0)
                robot.delay(DT_MS)
                continue

            x, y, th = pose
            now = robot.get_clock()

            # boundary handling
            b = boundary_state(x, y)
            if b == 2:
                # hard boundary -> full stop for this robot
                robot.set_led(255, 0, 0)
                robot.set_vel(0, 0)
                robot.delay(DT_MS)
                continue

            # all good / soft boundary: blue LED
            robot.set_led(0, 0, 255)

            # --- heartbeat send (finite diff vx,vy) ---
            if now - last_hb >= HB_DT:
                x1, y1 = x, y
                t1 = now
                robot.delay(60)
                p2 = safe_pose(robot)
                if p2:
                    x2, y2, th2 = float(p2[0]), float(p2[1]), float(p2[2])
                    t2 = robot.get_clock()
                    dt = max(1e-3, t2 - t1)
                    vx_hb = (x2 - x1) / dt
                    vy_hb = (y2 - y1) / dt
                    try:
                        robot.send_msg(struct.pack(HB_FMT, x2, y2, th2,
                                                   vx_hb, vy_hb, vid))
                    except:
                        pass
                    last_hb = t2
                    x, y, th = x2, y2, th2
                else:
                    last_hb = now

            # --- receive neighbor heartbeats ---
            msgs = robot.recv_msg()
            if msgs:
                if not isinstance(msgs, list):
                    msgs = [msgs]
                for m in msgs:
                    if not m:
                        continue
                    # ignore gossiplike short messages
                    if len(m) < HB_BYTES:
                        continue
                    try:
                        nx, ny, nth, nvx, nvy, nid = struct.unpack(HB_FMT,
                                                                   m[:HB_BYTES])
                        if int(nid) != vid:
                            neighbors[int(nid)] = (nx, ny, nth, nvx, nvy)
                            last_seen[int(nid)] = now
                    except:
                        pass

            # prune stale neighbors
            cut = now - STALE_S
            for nid in list(neighbors.keys()):
                if last_seen.get(nid, 0) < cut:
                    neighbors.pop(nid, None)
                    last_seen.pop(nid, None)

            # --- environment forces (walls only) ---
            ex, ey = soft_boundary_force(x, y)

            # --- sinusoidal wave field (x oscillation) ---
            wave_phase = 2.0 * math.pi * WAVE_FREQ * now + phase0
            vx_wave = WAVE_AMP * math.sin(wave_phase)
            vy_wave = 0.0

            # --- upward migration ---
            mx = 0.0
            my = K_MIG

            # --- neighbor terms (soft flocking) ---
            repx = repy = 0.0
            cx = cy = 0.0
            ax = ay = 0.0
            n = 0

            for nid in neighbors:
                nx, ny, nth, nvx, nvy = neighbors[nid]
                dx = x - nx
                dy = y - ny
                d2 = dx*dx + dy*dy
                if d2 > 1e-9:
                    d = math.sqrt(d2)
                    if d < SEP_RADIUS:
                        s = K_SEP * (SEP_RADIUS - d) / SEP_RADIUS
                        repx += s * (dx / d)
                        repy += s * (dy / d)
                    if d <= NEIGH_RADIUS:
                        cx += nx
                        cy += ny
                        ax += math.cos(nth)
                        ay += math.sin(nth)
                        n += 1

            cohx = cohy = 0.0
            alx = aly = 0.0

            if n > 0:
                cx /= n
                cy /= n
                cohx = K_COH * (cx - x)
                cohy = K_COH * (cy - y)
                ah = math.atan2(ay, ax)
                alx = K_ALI * math.cos(ah)
                aly = K_ALI * math.sin(ah)

            # --- combine fields (pre ring-lock) ---
            vx = ex + vx_wave + mx + repx + cohx + alx
            vy = ey + vy_wave + my + repy + cohy + aly

            
            # RING LOCK (formation)
            dx_c = x - CX
            dy_c = y - CY
            r_now = math.hypot(dx_c, dy_c)

            if r_now > 1e-6:
                urx = dx_c / r_now
                ury = dy_c / r_now
                utx = -ury
                uty = urx
            else:
                urx, ury = 1.0, 0.0
                utx, uty = 0.0, 1.0

            radial_error = R_TARGET - r_now
            ang_now = math.atan2(dy_c, dx_c) if r_now > 1e-6 else target_angle
            ang_error = wrap(target_angle - ang_now)

            # add ring lock
            vx += KR * radial_error * urx + KTH * ang_error * utx
            vy += KR * radial_error * ury + KTH * ang_error * uty

            # convert to wheels
            # 
            if abs(vx) < 1e-6 and abs(vy) < 1e-6:
                vx = 1e-3  # avoid NaN

            hdg = math.atan2(vy, vx)
            err = wrap(hdg - th)

            fwd = FWD_BASE
            if b == 1:
                fwd *= 0.8
            if fwd < FWD_MIN:
                fwd = FWD_MIN

            turn = clamp(TURN_K * err, -1.2, 1.2)
            lcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)),
                         -MAX_WHEEL, MAX_WHEEL)
            rcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)),
                         -MAX_WHEEL, MAX_WHEEL)

            # slew limit
            if lcmd > lastL + VEL_SLEW: lcmd = lastL + VEL_SLEW
            if lcmd < lastL - VEL_SLEW: lcmd = lastL - VEL_SLEW
            if rcmd > lastR + VEL_SLEW: rcmd = lastR + VEL_SLEW
            if rcmd < lastR - VEL_SLEW: rcmd = lastR - VEL_SLEW

            left  = int((1 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
            right = int((1 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
            lastL, lastR = left, right

            robot.set_vel(left, right)
            robot.delay(DT_MS)

    except Exception as e:
        try:
            robot.set_vel(0, 0)
            robot.set_led(255, 0, 0)
        except:
            pass
        print('[GLIDE] ERROR id=%d: %s' % (vid, repr(e)))
        raise
    finally:
        try:
            robot.set_vel(0, 0)
        except:
            pass