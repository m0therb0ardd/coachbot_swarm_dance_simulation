# -*- coding: utf-8 -*-
"""
FLOAT — Sustained + Indirect (Boids + Laban)
- Simulation: uses robot.id and print()
- Hardware: uses robot.virtual_id() and experiment_log.txt
- Behavior: soft boids flock, gentle arcs, meandering attention, low jerk
"""

import math
import os
import random

# =========================
# Arena / obstacle geometry
# =========================
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

FEET = 0.6048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET   # ~0.1524
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
OBST_CX, OBST_CY = (-0.1, 0.475)

# ====================
# Drive / control caps
# ====================
MAX_WHEEL = 35
TURN_K    = 3.0
FWD_FAST  = 0.80
FWD_SLOW  = 0.35
FWD_MIN   = 0.33
EPS       = 1e-3

# Wheel command smoothing (EMA) for floaty feel
CMD_SMOOTH  = 0.30

# =================
# Safety rails
# =================
SOFT_MARGIN     = 0.08
CRIT_MARGIN     = 0.02
SOFT_MAX_FORCE  = 0.35

# Neighbor spacing (repulsion backbone)
REPULSE_RADIUS  = 0.75
REPULSE_GAIN    = 0.10
HARD_REP_RADIUS = 0.18
HARD_REP_GAIN   = 0.26

# ============
# Boids + Laban
# ============
# — Tune these for other modes! —
LABAN = {
    "time_sustained": 0.85,  # 0 sudden … 1 sustained  (low accel/jerk, smoother turns)
    "space_indirect": 0.80,  # 0 direct … 1 indirect   (meander, curved paths)
    "weight_light":   0.80,  # 0 heavy  … 1 light      (lower raw speeds)
    "flow_free":      0.65,  # 0 bound  … 1 free       (more variability)
}

# Base boids gains (scaled below by LABAN where appropriate)
BOIDS = {
    "sep_radius": 0.40,  "sep_gain": 0.60,     # firm close-in separation
    "ali_radius": 0.90,  "ali_gain": 0.35,     # moderate alignment
    "coh_radius": 1.10,  "coh_gain": 0.18,     # light cohesion → loose cloud
}

# Indirectness ingredients
ARC_BIAS_GAIN     = 0.25   # bias to turn slightly perpendicular to velocity
WAYPOINT_GAIN     = 0.15   # gentle pull to a drifting virtual waypoint
WAYPOINT_DRIFT    = 0.015  # (not explicitly used; kept for clarity)

# Sustained feel caps
MAX_TURN_RAD      = 0.65 * (0.6 + 0.4*LABAN["time_sustained"])  # cap on heading step
ACCEL_CAP         = 0.08 * (0.5 + 0.5*LABAN["time_sustained"])  # cap on |Δv|
VEL_EMA           = 0.70 * LABAN["time_sustained"] + 0.20       # stronger EMA if sustained

# Speed scaling from "weight_light"
FLOAT_SPEED_SCALE = 0.65 + 0.25*LABAN["weight_light"]  # ~0.65–0.90 of your FWD scaling

# Keep leftward intention, soft
LEFT_INTENT = -0.06

# Flow field (for indirectness)
NOISE_GAIN  = 0.10    
NOISE_SCALE = 0.35
NOISE_SPEED = 0.05

# ======
# Timing
# ======
PRINT_PERIOD = 2.0
MAX_RUNTIME  = 55.0
LOOP_DT_MS   = 40  # 25 Hz


# =========================
# Helpers
# =========================
def clamp(v, lo, hi):
    if v < lo: return lo
    if v > hi: return hi
    return v

def wrap_angle(a):
    while a >  math.pi: a -= 2.0*math.pi
    while a <= -math.pi: a += 2.0*math.pi
    return a

def safe_pose(robot):
    p = robot.get_pose()
    if p and len(p) >= 3:
        return float(p[0]), float(p[1]), float(p[2])
    return None

def soft_boundary_check(x, y):
    """0=ok, 1=warn, 2=critical."""
    if (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
        y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN):
        return 2
    elif (x < X_MIN + SOFT_MARGIN or x > X_MAX - SOFT_MARGIN or
          y < Y_MIN + SOFT_MARGIN or y > Y_MAX - SOFT_MARGIN):
        return 1
    return 0

def soft_boundary_force(x, y):
    fx = fy = 0.0
    if x < X_MIN + SOFT_MARGIN:
        fx += SOFT_MAX_FORCE * (1.0 - (x - X_MIN)/SOFT_MARGIN)
    elif x > X_MAX - SOFT_MARGIN:
        fx -= SOFT_MAX_FORCE * (1.0 - (X_MAX - x)/SOFT_MARGIN)
    if y < Y_MIN + SOFT_MARGIN:
        fy += SOFT_MAX_FORCE * (1.0 - (y - Y_MIN)/SOFT_MARGIN)
    elif y > Y_MAX - SOFT_MARGIN:
        fy -= SOFT_MAX_FORCE * (1.0 - (Y_MAX - y)/SOFT_MARGIN)
    return fx, fy

def soft_obstacle_force(x, y, max_force=0.55, buffer_width=0.10):
    dx = x - OBST_CX
    dy = y - OBST_CY
    r  = math.hypot(dx, dy)
    if r < SAFE_BUBBLE + buffer_width:
        if r < 1e-6: return max_force, 0.0
        strength = max(0.0, (SAFE_BUBBLE + buffer_width - r) / buffer_width)
        s = max_force * strength
        return s * (dx / r), s * (dy / r)
    return 0.0, 0.0

def is_critical_obstacle(x, y, critical_margin=0.0):
    dx = x - OBST_CX
    dy = y - OBST_CY
    r  = math.hypot(dx, dy)
    return r < (OBST_RADIUS + critical_margin)

def try_get_swarm_poses(robot):
    names = ('get_swarm_poses', 'get_all_poses', 'get_poses', 'swarm_poses')
    for nm in names:
        fn = getattr(robot, nm, None)
        if callable(fn):
            try:
                poses = fn()
                if poses: return poses
            except:
                pass
    return []

def detect_sim(robot):
    has_id = hasattr(robot, "id")
    try:
        _ = robot.id() if callable(getattr(robot, "id", None)) else getattr(robot, "id", None)
        return has_id
    except:
        return False

def get_robot_id(robot, sim_mode):
    if sim_mode:
        rid_attr = getattr(robot, "id", None)
        try:
            return rid_attr() if callable(rid_attr) else int(rid_attr)
        except:
            return -1
    else:
        try:
            return robot.virtual_id()
        except:
            return -1

def unit(x, y):
    n = math.hypot(x, y)
    if n < 1e-9: return 0.0, 0.0
    return x/n, y/n

def limit_vec(x, y, cap):
    n = math.hypot(x, y)
    if n <= cap or n < 1e-9: return x, y
    s = cap / n
    return x*s, y*s

def perlinish(x, y, t, kx, ky, a, b, c):
    return math.sin(kx*x + a*ky*y + b + t) + 0.6*math.sin(0.6*kx*x - c*ky*y + 0.7*t)

def flow2(x, y, t, scale=NOISE_SCALE, speed=NOISE_SPEED):
    """Two gentle, phase-shifted flows."""
    kx = 2.0*scale; ky = 1.8*scale; ph = speed*t
    n1x = perlinish(x, y, ph*1.00, kx, ky, 0.7, 1.2, 1.3)
    n1y = perlinish(x, y, ph*1.10, 0.9*kx, 1.4*ky, 1.1, 0.0, 0.5)
    n2x = perlinish(x+0.8, y-0.4, ph*0.85, 1.3*kx, 0.8*ky, 0.4, 0.6, 1.7)
    n2y = perlinish(x-0.5, y+0.6, ph*0.92, 0.8*kx, 1.1*ky, 1.5, 0.3, 1.1)
    return (0.5*(n1x+n2x), 0.5*(n1y+n2y))

def boids_vectors(robot, neighbors, sep_R, ali_R, coh_R):
    rx, ry, rth = robot.get_pose()

    # Separation
    sx = sy = 0.0
    # Alignment
    ax = ay = 0.0; an = 0
    # Cohesion
    cx = cy = 0.0; cn = 0

    for item in neighbors or []:
        if not (isinstance(item, (list, tuple)) and len(item) >= 3): continue
        if len(item) == 4: nid, nx, ny, nth = item
        else: nx, ny, nth = item[0], item[1], item[2]
        dx = rx - nx; dy = ry - ny; d2 = dx*dx + dy*dy
        if d2 < 1e-12: continue

        # separation
        if d2 < sep_R*sep_R:
            inv = 1.0 / d2
            sx += dx*inv; sy += dy*inv

        # alignment
        if d2 < ali_R*ali_R:
            ax += math.cos(nth); ay += math.sin(nth); an += 1

        # cohesion
        if d2 < coh_R*coh_R:
            cx += nx; cy += ny; cn += 1

    # finalize alignment & cohesion
    if an > 0:
        ax /= an; ay /= an
        ax, ay = unit(ax, ay)
    if cn > 0:
        cx = (cx/cn) - rx; cy = (cy/cn) - ry
        cx, cy = unit(cx, cy)

    # separation direction
    if sx != 0.0 or sy != 0.0:
        sx, sy = unit(sx, sy)

    return (sx, sy), (ax, ay), (cx, cy)


# =========================
# Main entrypoint
# =========================
def usr(robot):
    robot.delay(3000)

    sim_mode = detect_sim(robot)
    rid = get_robot_id(robot, sim_mode)

    # Logging
    if sim_mode:
        def logw(s): print(s)
        logw("FLOAT(SIM): I am robot %s" % str(rid))
    else:
        log_main = open("experiment_log.txt", "a")
        def logw(s):
            if not s.endswith("\n"): s += "\n"
            log_main.write(s); log_main.flush()
            try: os.fsync(log_main.fileno())
            except: pass
        logw("FLOAT(HW): I am robot %s" % str(rid))

    # Randomness per robot
    try:
        rnd_seed = int((rid if rid is not None else 0) * 2654435761) & 0xFFFFFFFF
    except:
        rnd_seed = 0
    random.seed(rnd_seed)

    # Time
    has_clock = hasattr(robot, "get_clock")
    start_time = robot.get_clock() if has_clock else 0.0
    acc_time = 0.0  # ms-based fallback

    last_log_sec = -1
    last_pose = None
    last_left = 0
    last_right = 0
    told_no_swarm_api = False

    # Velocity memory for sustained feel
    last_vx = 0.0
    last_vy = 0.0

    try:
        while True:
            now = robot.get_clock() if has_clock else (start_time + acc_time/1000.0)
            t = (now - start_time) if has_clock else (acc_time/1000.0)
            if t >= MAX_RUNTIME:
                break

            pose = safe_pose(robot)
            if pose is None:
                robot.set_vel(0, 0)
                robot.delay(LOOP_DT_MS)
                if not has_clock: acc_time += LOOP_DT_MS
                continue

            x, y, th = pose
            last_pose = (x, y)

            # Safety lights + checks
            bstat = soft_boundary_check(x, y)
            if bstat == 2:
                logw("CRITICAL: Robot %s at boundary [%.3f, %.3f]" % (str(rid), x, y))
                robot.set_vel(0, 0); robot.set_led(255, 0, 0)
                break
            elif bstat == 1:
                robot.set_led(180, 220, 255)  # near boundary
            else:
                breathe = int(140 + 60 * (0.5 + 0.5*math.sin(0.6*t)))
                robot.set_led(0, breathe, breathe)

            if is_critical_obstacle(x, y, 0.0):
                logw("CRITICAL: Robot %s inside obstacle [%.3f, %.3f]" % (str(rid), x, y))
                robot.set_vel(0, 0); robot.set_led(255, 0, 0)
                robot.delay(LOOP_DT_MS)
                if not has_clock: acc_time += LOOP_DT_MS
                continue

            # ================================
            # Base field composition (Boids+Laban)
            # ================================
            # Walls & obstacle softness
            bfx, bfy = soft_boundary_force(x, y)
            ofx, ofy = soft_obstacle_force(x, y)

            # Two slow flows (indirectness) + soft left intent
            f1x, f1y = flow2(x, y, t, scale=NOISE_SCALE, speed=NOISE_SPEED)
            f2x, f2y = flow2(x+0.3, y-0.2, t+1.7, scale=NOISE_SCALE*0.9, speed=NOISE_SPEED*0.92)
            alpha_indir = 0.35 + 0.5*LABAN["space_indirect"]  # ~0.35–0.85
            mixx = (1.0 - alpha_indir)*f1x + alpha_indir*f2x
            mixy = (1.0 - alpha_indir)*f1y + alpha_indir*f2y

            # Drifting virtual waypoint
            wx = OBST_CX + 0.55*math.sin(0.23*t) + 0.25*math.sin(0.07*t + 1.3)
            wy = OBST_CY + 0.40*math.cos(0.19*t) + 0.20*math.sin(0.05*t + 0.6)
            wpx, wpy = unit(wx - x, wy - y)

            # Neighbors → boids
            neighbors = try_get_swarm_poses(robot)
            if neighbors:
                (sep_x, sep_y), (ali_x, ali_y), (coh_x, coh_y) = boids_vectors(
                    robot, neighbors,
                    BOIDS["sep_radius"], BOIDS["ali_radius"], BOIDS["coh_radius"]
                )
            else:
                sep_x = sep_y = ali_x = ali_y = coh_x = coh_y = 0.0
                if not told_no_swarm_api:
                    logw("Robot %s: no swarm pose API; proceeding without alignment/cohesion" % str(rid))
                    told_no_swarm_api = True

            # Gain scaling with Laban “flow_free”
            sep_g = BOIDS["sep_gain"]                                  # keep firm
            ali_g = BOIDS["ali_gain"] * (0.8 + 0.4*LABAN["flow_free"]) # a touch freer
            coh_g = BOIDS["coh_gain"] * (0.6 + 0.4*LABAN["flow_free"])

            # Compose desired velocity (pre-limits)
            vx = bfx + ofx + LEFT_INTENT
            vy = bfy + ofy

            vx += 0.50 * NOISE_GAIN * mixx
            vy += 0.50 * NOISE_GAIN * mixy

            vx += sep_g * sep_x + ali_g * ali_x + coh_g * coh_x
            vy += sep_g * sep_y + ali_g * ali_y + coh_g * coh_y

            # Curvature bias → gentle arcs
            dirx, diry = math.cos(th), math.sin(th)
            perpx, perpy = -diry, dirx
            arc_gain = ARC_BIAS_GAIN * (0.5 + 0.5*LABAN["space_indirect"])
            vx += arc_gain * perpx
            vy += arc_gain * perpy

            # Soft waypoint pull
            wp_gain = WAYPOINT_GAIN * (0.5 + 0.5*LABAN["space_indirect"])
            vx += wp_gain * wpx
            vy += wp_gain * wpy

            # ================================
            # Sustained feel: accel & turn caps, EMA
            # ================================
            # Accel cap
            dvx = vx - last_vx
            dvy = vy - last_vy
            dvx, dvy = limit_vec(dvx, dvy, ACCEL_CAP)
            vx = last_vx + dvx
            vy = last_vy + dvy

            # EMA smoothing
            vx = VEL_EMA*last_vx + (1.0 - VEL_EMA)*vx
            vy = VEL_EMA*last_vy + (1.0 - VEL_EMA)*vy
            last_vx, last_vy = vx, vy

            # Nudge if near-zero
            if abs(vx) + abs(vy) < EPS:
                vx += 0.02 * (-math.sin(th))
                vy += 0.02 * ( math.cos(th))

            # Heading + capped turn
            hdg = math.atan2(vy, vx)
            err = wrap_angle(hdg - th)
            err = clamp(err, -MAX_TURN_RAD, MAX_TURN_RAD)

            # Forward speed scaled by “weight_light”
            ae = abs(err)
            if ae < 0.6:   fwd = FWD_FAST * FLOAT_SPEED_SCALE
            elif ae < 1.2: fwd = FWD_FAST * 0.80 * FLOAT_SPEED_SCALE
            else:          fwd = FWD_SLOW * 0.70 * FLOAT_SPEED_SCALE
            if bstat == 1:
                fwd *= 0.75
            if fwd < FWD_MIN:
                fwd = FWD_MIN

            turn = clamp(TURN_K * err, -1.1, 1.1)

            left_cmd  = clamp(int(MAX_WHEEL * 0.90 * (fwd - 0.75 * turn)), -MAX_WHEEL, MAX_WHEEL)
            right_cmd = clamp(int(MAX_WHEEL * 0.90 * (fwd + 0.75 * turn)), -MAX_WHEEL, MAX_WHEEL)

            # Wheel EMA (keep floaty)
            left  = int((1.0 - CMD_SMOOTH) * left_cmd  + CMD_SMOOTH * last_left)
            right = int((1.0 - CMD_SMOOTH) * right_cmd + CMD_SMOOTH * last_right)
            last_left, last_right = left, right

            robot.set_vel(left, right)

            # Periodic status
            if int(now) != last_log_sec and (t % PRINT_PERIOD) < (LOOP_DT_MS/1000.0 + 0.02):
                msg = "FLOAT(SIM)" if sim_mode else "FLOAT(HW)"
                logw("%s %s pos [%.3f, %.3f]" % (msg, str(rid), x, y))
                last_log_sec = int(now)

            robot.delay(LOOP_DT_MS)
            if not has_clock: acc_time += LOOP_DT_MS

    except Exception as e:
        try:
            robot.set_vel(0, 0)
            robot.set_led(255, 0, 0)
        except:
            pass
        if sim_mode:
            print("ERROR(FLOAT SIM): %s" % str(e))
        else:
            logw("ERROR(FLOAT HW): %s" % str(e))
        raise
    finally:
        final_time = (robot.get_clock() if hasattr(robot, "get_clock") else (start_time + acc_time/1000.0))
        lx, ly = (last_pose if last_pose else (float('nan'), float('nan')))
        try:
            robot.set_vel(0, 0)
        except:
            pass
        msg = "FLOAT %s finished at [%.3f, %.3f] after %.1fs" % (str(rid), lx, ly, final_time - (start_time if hasattr(robot, "get_clock") else 0.0))
        if sim_mode:
            print(msg)
        else:
            logw(msg)
            try: log_main.close()
            except: pass


