import math, struct
# ===== Testbed bounds & center =====
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35
CX, CY = (-0.1, 0.475)

# ===== Dancer no-go circle =====
FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET   # ~0.1524 m
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
OBST_CX, OBST_CY = CX, CY  # obstacle centered on ring center

# ===== Translation plan =====
# MOVE_DIRE -1 means screen up which is viewer left +1 screen down (vertical motion in sim but horizontal irl)
MOVE_DIR    = -1       
SHIFT_RATE  = 0.18      # m/s center velocity
STOP_MARGIN = 0.08      # keep ring this far from wall at stop

# ===== Drive control =====
MAX_WHEEL = 40
TURN_K    = 3.0
FWD_FAST  = 0.8
FWD_SLOW  = 0.30
EPS       = 1e-3

# ===== Rigid-body locking gains =====
KX = 1.2
KY = 2.0
KR = 2.6

# ===== “GUI parity” options =====
WARN_MARGIN_BOUNDARY = 0.05   # yellow warn margin (matches glitch feel)
CRIT_MARGIN_BOUNDARY = 0.02   # hard-stop margin
USE_SOFT_OBST_FORCE  = False  # keep False to preserve your original trajectory
OBST_WARN_BUFFER     = 0.10   # soft obstacle buffer for warning/optional push
OBST_MAX_SOFT_FORCE  = 0.6    # only used if USE_SOFT_OBST_FORCE = True
PRINT_PERIOD         = 2.0    # seconds (like glitch)

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

def wrap_angle(a):
    while a >  math.pi: a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a

def safe_pose(robot):
    p = robot.get_pose()
    if isinstance(p,(list,tuple)) and len(p)>=3:
        return float(p[0]), float(p[1]), float(p[2])
    return None

# def get_id(robot):
#     vid_attr = getattr(robot, "virtual_id", None)
#     try:
#         return vid_attr() if callable(vid_attr) else int(vid_attr or 0)
#     except:
#         return -1

def get_id(robot):
    # Preferred: real robot's integer ID
    if hasattr(robot, "id"):
        try:
            return int(robot.id)
        except:
            pass

    # Simulation / Python test harness: virtual_id()
    if hasattr(robot, "virtual_id"):
        try:
            return robot.virtual_id()
        except:
            pass

    return -1


# ---------- GUI-parity helpers (from “glitch” style) ----------
def soft_boundary_force(x, y, max_force=0.3, boundary_margin=0.08):
    fx, fy = 0.0, 0.0
    if x < X_MIN + boundary_margin:
        fx += max_force * (1.0 - (x - X_MIN) / boundary_margin)
    elif x > X_MAX - boundary_margin:
        fx -= max_force * (1.0 - (X_MAX - x) / boundary_margin)
    if y < Y_MIN + boundary_margin:
        fy += max_force * (1.0 - (y - Y_MIN) / boundary_margin)
    elif y > Y_MAX - boundary_margin:
        fy -= max_force * (1.0 - (Y_MAX - y) / boundary_margin)
    return fx, fy

def is_critical_boundary(x, y, critical_margin=CRIT_MARGIN_BOUNDARY):
    return (x < X_MIN + critical_margin or x > X_MAX - critical_margin or
            y < Y_MIN + critical_margin or y > Y_MAX - critical_margin)

def obstacle_distance(x, y):
    dx, dy = x - OBST_CX, y - OBST_CY
    return math.hypot(dx, dy)

def is_critical_obstacle(x, y, critical_margin=0.0):
    return obstacle_distance(x, y) < (OBST_RADIUS + critical_margin)

def soft_obstacle_force(x, y, max_force=OBST_MAX_SOFT_FORCE, buffer_width=OBST_WARN_BUFFER):
    dx, dy = x - OBST_CX, y - OBST_CY
    r = math.hypot(dx, dy)
    if r < SAFE_BUBBLE + buffer_width:
        if r < 1e-6:
            return max_force, 0.0
        strength = max(0.0, (SAFE_BUBBLE + buffer_width - r) / buffer_width)
        s = max_force * strength
        return s * (dx / r), s * (dy / r)
    return 0.0, 0.0

def usr(robot):
    my_id = get_id(robot)
    robot.set_led(0, 180, 180)  # cyan normal
    print(f"Robot {my_id} starting: Translate with glitch-style GUI")

    # Per-robot state
    rel_off = None
    R_form  = None
    s_stop  = None
    t0      = None
    started = False
    last_print_time = 0.0

    robot.delay(300)  # small settle

    while True:
        pose = safe_pose(robot)
        if pose is None:
            robot.set_vel(0,0)
            robot.delay(20)
            continue
        x, y, th = pose

        # --- CRITICAL STOPS (GUI parity) ---
        if is_critical_obstacle(x, y):
            robot.set_vel(0, 0)
            robot.set_led(255, 50, 0)  # orange/red = critical
            print(f"ROBOT {my_id} OBSTACLE STOP at [{x:.3f}, {y:.3f}]")
            robot.delay(40)
            continue
        if is_critical_boundary(x, y):
            robot.set_vel(0, 0)
            robot.set_led(255, 50, 0)  # orange/red = critical
            print(f"ROBOT {my_id} CRITICAL STOP at [{x:.3f}, {y:.3f}]")
            robot.delay(40)
            continue

        # --- WARNINGS (yellow), otherwise cyan ---
        obst_r = obstacle_distance(x, y)
        near_obstacle = obst_r < (SAFE_BUBBLE + OBST_WARN_BUFFER)
        near_boundary = (x < X_MIN + WARN_MARGIN_BOUNDARY or x > X_MAX - WARN_MARGIN_BOUNDARY or
                         y < Y_MIN + WARN_MARGIN_BOUNDARY or y > Y_MAX - WARN_MARGIN_BOUNDARY)
        if near_obstacle or near_boundary:
            robot.set_led(255, 150, 0)  # yellow warn
        else:
            robot.set_led(0, 180, 180)  # cyan normal

        # Lock initial offset & compute stop distance once
        if rel_off is None:
            rel_off = (x - CX, y - CY)
            R_form  = math.hypot(*rel_off)

            # Wall-limited shift, but now in Y (vertical)
            safety_buffer = 0.05
            if MOVE_DIR < 0:  # moving up (toward Y_MAX)
                s_wall = max(0.0, (Y_MAX - STOP_MARGIN - safety_buffer - R_form) - CY)
            else:  # moving down (toward Y_MIN)
                s_wall = max(0.0, CY - (Y_MIN + STOP_MARGIN + safety_buffer + R_form))


            # Obstacle limit based on bubble vs radius
            s_obst = max(0.0, R_form - SAFE_BUBBLE)
            s_stop = min(s_wall, s_obst)

            # synced start half-second grid
            now = robot.get_clock()
            t0 = math.floor(now * 2.0) / 2.0 + 0.5

            print(f"Robot {my_id} locked: R={R_form:.3f}, s_stop={s_stop:.3f}")

        # Wait for shared start
        t = robot.get_clock()
        if not started:
            if t < t0:
                robot.set_vel(0,0)
                robot.delay(10)
                continue
            started = True
            print(f"Robot {my_id} started moving")

        # Center shift
        s = min(max(0.0, t - t0) * SHIFT_RATE, s_stop)

        # Moving center + rigid target (vertical translation)
        Cx = CX
        Cy = CY - MOVE_DIR * s   # MOVE_DIR=-1 => Cy increases (up)


        # keep center valid
        Cx = max(X_MIN + R_form + 0.08, min(X_MAX - R_form - 0.08, Cx))
        Cy = max(Y_MIN + R_form + 0.08, min(Y_MAX - R_form - 0.08, Cy))

        tx = Cx + rel_off[0]
        ty = Cy + rel_off[1]

        # Position error → base velocity
        ex, ey = tx - x, ty - y
        vx = KX * ex
        vy = KY * ey

        # feed-forward along the translate direction (now vertical)
        vy += -MOVE_DIR * SHIFT_RATE   # MOVE_DIR=-1 => vy += +SHIFT_RATE (up)


        # Radial lock to initial offset
        rx, ry = x - Cx, y - Cy
        vx += KR * (rel_off[0] - rx)
        vy += KR * (rel_off[1] - ry)

        # Soft boundary cushion (visual parity; mild effect)
        bfx, bfy = soft_boundary_force(x, y, max_force=0.3, boundary_margin=0.08)
        vx += bfx; vy += bfy

        # Optional: soft obstacle push to mirror glitch feel (off by default)
        if USE_SOFT_OBST_FORCE:
            ofx, ofy = soft_obstacle_force(x, y)
            vx += ofx; vy += ofy

        # Finished? stop & hold
        if abs(s - s_stop) < 1e-6 and (abs(ex) + abs(ey) < 0.01):
            robot.set_vel(0, 0)
            robot.set_led(0, 255, 0)  # green = completed
            robot.delay(20)
            # keep holding position but continue the GUI loop
            continue

        # Map (vx,vy) → wheels
        if abs(vx) + abs(vy) < EPS:
            robot.set_vel(0, 0)
        else:
            hdg = math.atan2(vy, vx)
            err = wrap_angle(hdg - th)

            abs_err = abs(err)
            if abs_err < 0.5:      fwd = FWD_FAST
            elif abs_err < 1.2:    fwd = FWD_FAST * 0.7
            else:                  fwd = FWD_SLOW

            turn = clamp(TURN_K * err, -1.5, 1.5)

            left  = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
            right = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
            robot.set_vel(left, right)

        # glitch-like periodic console status
        now = robot.get_clock()
        if now - last_print_time > PRINT_PERIOD:
            print(f"Robot {my_id} pos [{x:.3f}, {y:.3f}], s={s:.3f}")
            last_print_time = now

        robot.delay(20)

