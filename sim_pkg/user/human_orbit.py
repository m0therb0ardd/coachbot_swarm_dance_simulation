# sim_pkg/user/leader_flock_follow.py
# All robots flock, but the leader (ID=0) sets the group's position + heading target.

import math, struct, random

# ==== Tunables ====
LEADER_ID = 0

# follow-the-leader weights
FOLLOW_POS_W   = 1.30   # pull toward leader position
FOLLOW_ALIGN_W = 1.10   # align heading to leader heading
FOLLOW_DIST    = 0.60   # desired leader distance (pack radius)

# neighborhood (boids) terms
COMM_RADIUS   = 0.60
NEIGHBOR_MIN  = 0.12
ALIGN_W       = 0.40    # small local alignment to keep formation tidy
COHERE_W      = 0.20    # small local cohesion (we mostly follow leader)
SEPAR_W       = 1.30

# safety + walls
ROBOT_RAD   = 0.06
HARD_RADIUS = 2.2 * ROBOT_RAD
ARENA       = (-1.5, 1.5, -1.0, 1.0)  # xmin,xmax,ymin,ymax  <- match config.json
WALL_MARGIN = 0.12
WALL_W      = 1.6

# control
MAX_WHEEL = 40
TURN_K    = 2.5
FWD_FAST  = 0.8
FWD_SLOW  = 0.3

# ==== utils ====
def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def wrap_angle(a):
    while a >  math.pi: a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a
def get_id(robot):
    vid_attr = getattr(robot, "virtual_id", None)
    return vid_attr() if callable(vid_attr) else int(vid_attr)

# ==== leader tick ====
def leader_tick(robot, my_pose):
    # broadcast actual pose so followers can lock on
    if my_pose: x, y, th = my_pose
    else:       x, y, th = 0.0, 0.0, 0.0
    msg = struct.pack('fffI', x, y, th, get_id(robot))
    robot.send_msg(msg)
    # optional gentle drift so there is motion
    robot.set_vel(15, 15)
    robot.delay(50)

# ==== forces ====
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
    # separation (strong near contact)
    for nx, ny, _ in nbrs:
        dx, dy = x - nx, y - ny
        d = math.hypot(dx, dy) + 1e-6
        if d < NEIGHBOR_MIN:
            s = (NEIGHBOR_MIN - d) / NEIGHBOR_MIN
            vx += SEPAR_W * (dx/d) * s; vy += SEPAR_W * (dy/d) * s
    return vx, vy

def follow_leader(x, y, th, leader):
    lx, ly, lth = leader
    # position spring toward a ring around leader at FOLLOW_DIST
    rx, ry = lx - x, ly - y
    r = math.hypot(rx, ry) + 1e-6
    urx, ury = rx / r, ry / r
    # push toward/away so distance ~ FOLLOW_DIST
    vpx = FOLLOW_POS_W * (FOLLOW_DIST - r) * urx
    vpy = FOLLOW_POS_W * (FOLLOW_DIST - r) * ury
    # align to leader heading
    vax = FOLLOW_ALIGN_W * math.cos(lth)
    vay = FOLLOW_ALIGN_W * math.sin(lth)
    return vpx + vax, vpy + vay

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

# ==== user entry ====
def usr(robot):
    my_id = get_id(robot)
    # LEDs: red leader, bluish followers
    if my_id == LEADER_ID: robot.set_led(255, 0, 0)
    else:                  robot.set_led(0, 80 + int(80*random.random()), 140)

    while True:
        my_pose = robot.get_pose()  # [x,y,th] or None

        # leader loop
        if my_id == LEADER_ID:
            leader_tick(robot, my_pose)
            continue

        # followers broadcast their pose
        if my_pose:
            x, y, th = my_pose
            robot.send_msg(struct.pack('fffI', x, y, th, my_id))

        # send -> short delay -> receive
        robot.delay(50)
        msgs = robot.recv_msg() or []

        # parse neighbors & leader
        leader = None; nbrs = []
        if my_pose:
            x, y, th = my_pose
            for m in msgs:
                try:
                    nx, ny, nth, nid = struct.unpack('fffI', m)
                    if nid == LEADER_ID: leader = (nx, ny, nth)
                    dx, dy = nx - x, ny - y
                    d2 = dx*dx + dy*dy
                    if 1e-6 < d2 <= COMM_RADIUS*COMM_RADIUS:
                        nbrs.append((nx, ny, nth))
                except Exception:
                    pass

        # build desired velocity
        vx = vy = 0.0
        if my_pose:
            # local flocking (light touch)
            bx, by = boids_local(x, y, th, nbrs)
            vx += bx; vy += by
            # strong leader-follow
            if leader:
                fx, fy = follow_leader(x, y, th, leader)
                vx += fx; vy += fy
            # safety + walls
            ax, ay, threat = hard_bubble(x, y, nbrs)
            vx += ax; vy += ay
            wx, wy = wall_repulsion(x, y)
            vx += wx; vy += wy

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
