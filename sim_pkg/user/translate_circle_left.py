# sim_pkg/user/translate_with_center_circle.py
# Rigidly translate a circular formation while honoring a central no-go circle.
# Optional: "beacon" robots (virtual_id 900..999) draw the circle and never move.

import math, struct, random

# ===== Arena (meters) —  your config.json WIDTH/LENGTH =====
ARENA = (-5.0, 5.0, -5.0, 5.0)   # xmin, xmax, ymin, ymax  (10x10)

# ===== No-go circle (centered at origin) =====
FEET = 0.3048
OBST_CX, OBST_CY = 0.0, 0.0
OBST_DIAM_FT = 3.0
OBST_RADIUS   = 0.5 * OBST_DIAM_FT * FEET      # 3 ft diameter => 0.4572 m
OBST_MARGIN   = 0.01                          # extra safety collar (m)

# ===== Formation and motion =====
FORMATION_RADIUS = 0.94    # your ring radius (m)
MOVE_DIR = -1               # -1 = left, +1 = right
SHIFT_PER_TICK = 0.010      # m/tick (~10 mm each control cycle)
STOP_MARGIN    = 0.08       # wall safety

# ===== Spacing & safety between followers =====
COMM_RADIUS  = 0.60
NEIGHBOR_MIN = 0.10
SEPAR_W      = 0.90
ROBOT_RAD    = 0.06
HARD_RADIUS  = 0.09

# ===== Wheel control =====
MAX_WHEEL = 40
TURN_K    = 2.5
FWD_FAST  = 0.8
FWD_SLOW  = 0.35
FWD_MIN_WHEN_THREAT = 0.20
EPS = 1e-3

# ===== Utilities =====
def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def wrap_angle(a):
    while a >  math.pi: a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a
def get_id(robot):
    vid_attr = getattr(robot, "virtual_id", None)
    return vid_attr() if callable(vid_attr) else int(vid_attr)

def separation_push(x, y, nbrs):
    vx = vy = 0.0
    for nx, ny, _ in nbrs:
        dx, dy = x - nx, y - ny
        d = math.hypot(dx, dy) + 1e-6
        if d < NEIGHBOR_MIN:
            s = (NEIGHBOR_MIN - d) / NEIGHBOR_MIN
            vx += SEPAR_W * (dx/d) * s
            vy += SEPAR_W * (dy/d) * s
    return vx, vy

def hard_bubble(x, y, nbrs):
    ax = ay = 0.0; threat = False
    for nx, ny, _ in nbrs:
        dx, dy = x - nx, y - ny
        d = math.hypot(dx, dy) + 1e-6
        if d < HARD_RADIUS:
            threat = True
            push = (HARD_RADIUS - d) / HARD_RADIUS
            ax += (dx/d) * 3.0 * push
            ay += (dy/d) * 3.0 * push
    return ax, ay, threat

def safe_pose(robot):
    p = robot.get_pose()
    if isinstance(p, (list, tuple)) and len(p) >= 3:
        # force numeric types; ignore any extras
        return float(p[0]), float(p[1]), float(p[2])
    return None


def center_repulsion(x, y):
    """Soft push if close to the no-go circle."""
    dx, dy = x - OBST_CX, y - OBST_CY
    r = math.hypot(dx, dy) + 1e-6
    if r < OBST_RADIUS + OBST_MARGIN:
        # push outward proportional to penetration depth
        depth = (OBST_RADIUS + OBST_MARGIN) - r
        k = 2.0
        return k * (dx/r) * depth, k * (dy/r) * depth
    return 0.0, 0.0

# ===== Main =====
def usr(robot):
    my_id = get_id(robot)

    # ----- Beacon robots draw the circle and never move -----
    if 900 <= my_id < 1000:
        robot.set_led(255, 0, 180)  # magenta dot
        while True:
            # (no broadcasts -> followers will ignore these)
            robot.set_vel(0, 0)
            robot.delay(50)

    # Followers (normal robots)
    robot.set_led(0, 180, 180)      # cyan-ish so it’s easy to see

    # Shared-formation bookkeeping
    center0 = None       # estimated initial center of the ring
    rel_off = None       # my initial offset from center0
    s_stop  = None       # max total shift allowed (wall & no-go circle)
    ticks   = 0

    # Temporary buffers to estimate initial center
    lock_ticks = 25       # ~0.5–1.0 s depending on your delay
    acc_x = []; acc_y = []

    while True:
        # my_pose = robot.get_pose()
        # if my_pose is None:
        #     robot.delay()
        #     continue

        # x, y, th = my_pose
        my_pose = safe_pose(robot)
        if my_pose is None:
            # try again next tick (or a short wait helps during startup)
            robot.delay(20)
            continue
        x, y, th = my_pose


        # Broadcast my pose (bytes)
        msg = struct.pack('fffI', x, y, th, my_id)
        robot.send_msg(msg)

        # Let neighbors hear us, then receive
        robot.delay(50)
        msgs = robot.recv_msg() or []

        # Parse neighbor positions (ignore any beacon ids)
        nbrs = []
        for m in msgs:
            try:
                nx, ny, nth, nid = struct.unpack('fffI', m)
                if 900 <= nid < 1000:      # ignore beacons for spacing
                    continue
                dx, dy = nx - x, ny - y
                if dx*dx + dy*dy <= COMM_RADIUS*COMM_RADIUS:
                    nbrs.append((nx, ny, nth))
            except Exception:
                pass

        # --- Phase 1: lock initial center & my offset ---
        if center0 is None:
            acc_x.append(x); acc_y.append(y)
            for nx, ny, _ in nbrs:
                acc_x.append(nx); acc_y.append(ny)
            if ticks >= lock_ticks and len(acc_x) >= 3:
                cx = sum(acc_x)/len(acc_x); cy = sum(acc_y)/len(acc_y)
                center0 = (cx, cy)
                rel_off = (x - cx, y - cy)

                xmin, xmax, _, _ = ARENA

                # wall stop distance (unchanged)
                if MOVE_DIR < 0:
                    s_wall = max(0.0, center0[0] - (xmin + STOP_MARGIN + FORMATION_RADIUS))
                else:
                    s_wall = max(0.0, (xmax - STOP_MARGIN - FORMATION_RADIUS) - center0[0])

                # --- FIXED: center-circle stop distance for rigid translation ---
                # Let R = FORMATION_RADIUS, B = OBST_RADIUS + OBST_MARGIN.
                # As we translate by s, the nearest point on the ring to (0,0) is at distance |R - s|.
                # Keep |R - s| >= B  =>  s <= R - B   (assuming R >= B).
                SAFE_BUBBLE = OBST_RADIUS + OBST_MARGIN
                s_obst = max(0.0, FORMATION_RADIUS - SAFE_BUBBLE)

                # final shared stop
                s_stop = min(s_wall, s_obst)


        # If we haven’t locked the center yet, idle forward a bit
        if center0 is None or rel_off is None or s_stop is None:
            robot.set_vel(10, 10)
            ticks += 1
            robot.delay()
            continue

        # Shared shift amount (same for all robots)
        s = min(ticks * SHIFT_PER_TICK, s_stop)

        # Target keeps the formation rigid while translating
        tx = center0[0] + MOVE_DIR * s + rel_off[0]
        ty = center0[1]                 + rel_off[1]

        # PD toward target + spacing + center-circle repulsion
        ex, ey = tx - x, ty - y
        kp = 1.6
        vx = kp * ex
        vy = kp * ey

        sx, sy = separation_push(x, y, nbrs)
        vx += sx; vy += sy
        ax, ay, threat = hard_bubble(x, y, nbrs)
        vx += ax; vy += ay

        cxp, cyp = center_repulsion(x, y)
        vx += cxp; vy += cyp

        # Stop once we’ve completed the planned shift and are near target
        if abs(s - s_stop) < 1e-6 and (abs(ex) + abs(ey) < 0.01):
            robot.set_vel(0, 0)
            robot.delay()
            continue

        # Map (vx, vy) to differential wheels
        if abs(vx) + abs(vy) < EPS:
            robot.set_vel(0, 0)
        else:
            hdg = math.atan2(vy, vx)
            err = wrap_angle(hdg - th)
            fwd  = FWD_FAST if abs(err) < 0.9 else FWD_SLOW
            if threat: fwd = max(FWD_MIN_WHEN_THREAT, 0.5 * fwd)
            turn = clamp(TURN_K * err, -1.0, 1.0)
            left  = clamp(int(MAX_WHEEL * (fwd - 0.8*turn)), -50, 50)
            right = clamp(int(MAX_WHEEL * (fwd + 0.8*turn)), -50, 50)
            robot.set_vel(left, right)

        ticks += 1
        robot.delay()  # ~20 ms
