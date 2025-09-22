# sim_pkg/user/flocking.py
import math, struct, random

MAX_WHEEL = 40
COMM_RADIUS = 0.6
NEIGHBOR_MIN = 0.12
ALIGN_W = 0.9
COHERE_W = 0.6
SEPAR_W = 1.3

ARENA = (-1.5, 1.5, -1.0, 1.0)  # xmin, xmax, ymin, ymax  ← set to your arena
WALL_MARGIN = 0.12
WALL_W = 1.6

# hard collision bubble (robot–robot)
ROBOT_RAD = 0.06          # ~6 cm (tweak to match sim bot radius)
HARD_RADIUS = 2.2 * ROBOT_RAD

def wrap_angle(a):
    while a >  math.pi: a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a

def usr(robot):
    # randomize LED so you can see they’re active
    robot.set_led(0, 30 + int(70*random.random()), 100)

    # while True:
    while True:
        # 1) broadcast my pose
        my_pose = robot.get_pose()
        if my_pose:
            x, y, th = my_pose
            vid_attr = getattr(robot, "virtual_id", None)
            virt_id = vid_attr() if callable(vid_attr) else int(vid_attr)
            msg = struct.pack('fffI', x, y, th, virt_id)
            robot.send_msg(msg)

        # give neighbors a tick to hear you, then read
        robot.delay(50)                  # ~50 ms helps a lot
        msgs = robot.recv_msg() or []


        # 2) read neighbor poses
        msgs = robot.recv_msg() or []
        nbrs = []
        if my_pose:
            x, y, th = my_pose
            for m in msgs:
                try:
                    nx, ny, nth, nid = struct.unpack('fffI', m)
                    dx, dy = nx - x, ny - y
                    d2 = dx*dx + dy*dy
                    if 1e-6 < d2 <= COMM_RADIUS*COMM_RADIUS:
                        nbrs.append((nx, ny, nth))
                except Exception:
                    pass

        vx = vy = 0.0
        if nbrs:
            # alignment
            avg_th = math.atan2(sum(math.sin(nth) for _,_,nth in nbrs),
                                sum(math.cos(nth) for _,_,nth in nbrs))
            vx += ALIGN_W * math.cos(avg_th)
            vy += ALIGN_W * math.sin(avg_th)

            # cohesion
            cx = sum(nx for nx,_,_ in nbrs) / len(nbrs)
            cy = sum(ny for _,ny,_ in nbrs) / len(nbrs)
            vx += COHERE_W * (cx - x)
            vy += COHERE_W * (cy - y)

            # soft separation
            for nx, ny, _ in nbrs:
                dx, dy = x - nx, y - ny
                dist = math.hypot(dx, dy) + 1e-6
                if dist < NEIGHBOR_MIN:
                    scale = (NEIGHBOR_MIN - dist) / NEIGHBOR_MIN
                    vx += SEPAR_W * (dx/dist) * scale
                    vy += SEPAR_W * (dy/dist) * scale

        # --- hard safety bubble ---
        hard_threat = False
        if my_pose and nbrs:
            ax = ay = 0.0
            for nx, ny, _ in nbrs:
                dx, dy = x - nx, y - ny
                d = math.hypot(dx, dy) + 1e-6
                if d < HARD_RADIUS:
                    hard_threat = True
                    push = (HARD_RADIUS - d) / HARD_RADIUS
                    ax += (dx / d) * 3.0 * push
                    ay += (dy / d) * 3.0 * push
            vx += ax; vy += ay

        # --- wall repulsion ---
        if my_pose:
            xmin, xmax, ymin, ymax = ARENA
            left   = x - xmin
            right  = xmax - x
            bottom = y - ymin
            top    = ymax - y
            if left < WALL_MARGIN:   vx += WALL_W * (WALL_MARGIN - left) / WALL_MARGIN
            if right < WALL_MARGIN:  vx -= WALL_W * (WALL_MARGIN - right) / WALL_MARGIN
            if bottom < WALL_MARGIN: vy += WALL_W * (WALL_MARGIN - bottom) / WALL_MARGIN
            if top < WALL_MARGIN:    vy -= WALL_W * (WALL_MARGIN - top) / WALL_MARGIN

        
        # 4) convert desired (vx,vy) into differential wheels
        # 4) convert desired (vx,vy) into differential wheels
        if my_pose:
            hdg = math.atan2(vy, vx) if (abs(vx)+abs(vy)) > 1e-6 else th
            err = wrap_angle(hdg - th)

            # base forward + turn
            fwd  = 0.8 if abs(err) < 0.9 else 0.3
            turn = max(-1.0, min(1.0, 2.5 * err))

            # if in hard-threat bubble, stop forward motion
            if hard_threat:
                fwd = 0.0

            left  = int(MAX_WHEEL * (fwd - 0.8*turn))
            right = int(MAX_WHEEL * (fwd + 0.8*turn))
            left  = max(-50, min(50, left))
            right = max(-50, min(50, right))
            robot.set_vel(left, right)

        # 5) pace the loop
        robot.delay()  # ~20 ms
