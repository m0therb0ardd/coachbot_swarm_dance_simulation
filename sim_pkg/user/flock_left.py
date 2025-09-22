# sim_pkg/user/flock_left_avoid_center.py
# 30 robots: ID 0 is the center "dancer" (stationary obstacle).
# Others flock left (global -X drift) with boids, avoid the center strongly.

import math, struct, random

# ===== IDs =====
CENTER_ID = 0

# ===== Weights & radii =====
COMM_RADIUS   = 0.60     # must be <= config.json COMM_RANGE
NEIGHBOR_MIN  = 0.12     # soft separation radius between robots
ALIGN_W       = 0.45     # local alignment (small)
COHERE_W      = 0.20     # local cohesion (small)
SEPAR_W       = 1.30     # local separation (strong near contact)

DRIFT_LEFT_W  = 1.20     # push the swarm to the left (-X)
CENTER_AVOID_R = 0.45    # start repelling from center within this radius
CENTER_REP_W   = 2.75    # strength of center avoidance

# Hard collision bubble (robot-robot)
ROBOT_RAD    = 0.06
HARD_RADIUS  = 2.2 * ROBOT_RAD

# Arena (match your config)
ARENA       = (-1.5, 1.5, -1.0, 1.0)  # xmin, xmax, ymin, ymax
WALL_MARGIN = 0.12
WALL_W      = 1.6

# Control
MAX_WHEEL = 40
TURN_K    = 2.5
FWD_FAST  = 0.8
FWD_SLOW  = 0.3

# ===== Utils =====
def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def wrap_angle(a):
    while a >  math.pi: a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a
def get_id(robot):
    vid_attr = getattr(robot, "virtual_id", None)
    return vid_attr() if callable(vid_attr) else int(vid_attr)

# ===== Center (ID 0) behavior =====
def center_tick(robot, my_pose):
    """Broadcast pose & sit still; this is the dancer/obstacle."""
    if my_pose: x, y, th = my_pose
    else:       x, y, th = 0.0, 0.0, 0.0
    msg = struct.pack('fffI', x, y, th, CENTER_ID)
    robot.send_msg(msg)
    robot.set_vel(0, 0)         # don't move the center
    robot.delay(50)             # let others receive

# ===== Field terms =====
def boids_local(x, y, th, nbrs):
    vx = vy = 0.0
    if not nbrs: return vx, vy
    # alignment (weak)
    avg_th = math.atan2(sum(math.sin(n) for *_, n in nbrs),
                        sum(math.cos(n) for *_, n in nbrs))
    vx += ALIGN_W * math.cos(avg_th); vy += ALIGN_W * math.sin(avg_th)
    # cohesion (weak)
    cx = sum(nx for nx,_,_ in nbrs)/len(nbrs)
    cy = sum(ny for _,ny,_ in nbrs)/len(nbrs)
    vx += COHERE_W * (cx - x);        vy += COHERE_W * (cy - y)
    # separation
    for nx, ny, _ in nbrs:
        dx, dy = x - nx, y - ny
        d = math.hypot(dx, dy) + 1e-6
        if d < NEIGHBOR_MIN:
            s = (NEIGHBOR_MIN - d) / NEIGHBOR_MIN
            vx += SEPAR_W * (dx/d) * s; vy += SEPAR_W * (dy/d) * s
    return vx, vy

def drift_left():
    # constant global drift toward -X
    return -DRIFT_LEFT_W, 0.0

def avoid_center(x, y, center_pose):
    if not center_pose: return 0.0, 0.0
    cx, cy, _ = center_pose
    rx, ry = x - cx, y - cy
    d = math.hypot(rx, ry) + 1e-6
    if d >= CENTER_AVOID_R: return 0.0, 0.0
    # linear ramp inside CENTER_AVOID_R, scaled by inverse distance a bit
    s = (CENTER_AVOID_R - d) / CENTER_AVOID_R
    ux, uy = rx / d, ry / d
    return CENTER_REP_W * s * ux, CENTER_REP_W * s * uy

def wall_repulsion(x, y):
    vx = vy = 0.0
    xmin, xmax, ymin, ymax = ARENA
    left, right = x - xmin, xmax - x
    bottom, top = y - ymin, ymax - y
    if left   < WALL_MARGIN: vx += WALL_W * (WALL_MARGIN - left)   / WALL_MARGIN
    if right  < WALL_MARGIN: vx -= WALL_W * (WALL_MARGIN - right)  / WALL_MARGIN
    if bottom < WALL_MARGIN: vy += WALL_W * (WALL_MARGIN - bottom) / WALL_MARGIN
    if top    < WALL_MARGIN: vy -= WALL_W * (WALL_MARGIN - top)    / WALL_MARGIN
    return vx, vy

def hard_bubble(x, y, nbrs):
    ax = ay = 0.0; threat = False
    for nx, ny, _ in nbrs:
        dx, dy = x - nx, y - ny
        d = math.hypot(dx, dy) + 1e-6
        if d < HARD_RADIUS:
            threat = True
            push = (HARD_RADIUS - d) / HARD_RADIUS
            ax += (dx/d) * 3.0 * push; ay += (dy/d) * 3.0 * push
    return ax, ay, threat

# ===== Main =====
def usr(robot):
    my_id = get_id(robot)

    # LED: center magenta; followers teal-ish
    if my_id == CENTER_ID: robot.set_led(255, 0, 180)
    else:                  robot.set_led(0, 150, 150)

    while True:
        my_pose = robot.get_pose()  # [x,y,th] or None

        # center robot = dancer/obstacle
        if my_id == CENTER_ID:
            center_tick(robot, my_pose)
            continue

        # followers: broadcast pose
        if my_pose:
            x, y, th = my_pose
            robot.send_msg(struct.pack('fffI', x, y, th, my_id))

        # send -> short delay -> receive
        robot.delay(50)
        msgs = robot.recv_msg() or []

        # parse neighbors & center
        center_pose = None; nbrs = []
        if my_pose:
            x, y, th = my_pose
            for m in msgs:
                try:
                    nx, ny, nth, nid = struct.unpack('fffI', m)
                    if nid == CENTER_ID:
                        center_pose = (nx, ny, nth)
                        continue  # don't treat center as a neighbor
                    dx, dy = nx - x, ny - y
                    d2 = dx*dx + dy*dy
                    if 1e-6 < d2 <= COMM_RADIUS*COMM_RADIUS:
                        nbrs.append((nx, ny, nth))
                except Exception:
                    pass

        # build desired velocity
        vx = vy = 0.0
        if my_pose:
            # global drift to the left
            dx, dy = drift_left()
            vx += dx; vy += dy
            # local boids
            bx, by = boids_local(x, y, th, nbrs)
            vx += bx;  vy += by
            # avoid the center dancer
            cx, cy = avoid_center(x, y, center_pose)
            vx += cx;  vy += cy
            # safety + walls
            ax, ay, threat = hard_bubble(x, y, nbrs)
            vx += ax;  vy += ay
            wx, wy = wall_repulsion(x, y)
            vx += wx;  vy += wy

            # map to wheels
            hdg = math.atan2(vy, vx) if (abs(vx)+abs(vy)) > 1e-6 else th
            err = wrap_angle(hdg - th)
            fwd  = FWD_FAST if abs(err) < 0.9 else FWD_SLOW
            turn = clamp(TURN_K * err, -1.0, 1.0)
            if threat: fwd = 0.0
            left  = clamp(int(MAX_WHEEL * (fwd - 0.8*turn)), -50, 50)
            right = clamp(int(MAX_WHEEL * (fwd + 0.8*turn)), -50, 50)
            robot.set_vel(left, right)

        robot.delay()  # ~20 ms
