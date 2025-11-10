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


# # -*- coding: utf-8 -*-
# """
# Boids-like FLOAT for the simulator
# - Migration toward screen-left (goal_x)
# - Separation from neighbors (if sim exposes poses)
# - Dancer circle avoidance (SAFE_BUBBLE)
# - Soft wall avoidance
# - Accel and speed caps for smooth, sustained motion
# """

# from dataclasses import dataclass
# from typing import List, Tuple
# import math
# import os
# import random

# # ============ geometry (match your arena) ============
# X_MIN, X_MAX = -1.2, 1.0
# Y_MIN, Y_MAX = -1.4, 2.35

# FEET = 0.6048
# OBST_DIAM_FT = 1.0
# OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET   # ~0.1524
# OBST_MARGIN  = 0.03
# SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
# OBST_CX, OBST_CY = (-0.1, 0.475)

# # ============ drive mapping (wheel caps etc.) ============
# MAX_WHEEL = 35
# TURN_K    = 3.0
# FWD_FAST  = 0.80
# FWD_SLOW  = 0.35
# FWD_MIN   = 0.33
# CMD_SMOOTH = 0.30   # wheel EMA
# EPS = 1e-3

# # ============ dataclasses (your API) ============
# Vec2 = Tuple[float, float]

# @dataclass
# class Agent:
#     x: float
#     y: float
#     vx: float
#     vy: float
#     id: int = 0  # optional

# @dataclass
# class Params:
#     # gains
#     k_mig: float = 0.6
#     k_sep: float = 1.4
#     k_obs: float = 1.2
#     k_wall: float = 0.8
#     # radii / cutoffs
#     r_sep: float = 0.25
#     r_obs: float = 0.28
#     r_wall: float = 0.25
#     # speeds/limits
#     v_max: float = 0.35     # m/s in world frame (we'll map to wheels)
#     a_max: float = 1.2      # m/s^2 accel cap per step
#     # migration target (screen-left)
#     goal_x: float = -1.0
#     # numerics
#     eps: float = 1e-6

# @dataclass
# class Bounds:
#     x_min: float; x_max: float; y_min: float; y_max: float

# @dataclass
# class Circle:
#     cx: float; cy: float; r: float  # dancer SAFE_BUBBLE

# # ============ helpers ============
# def _limit_vec(x: float, y: float, max_mag: float) -> Vec2:
#     m = math.hypot(x, y)
#     if m <= max_mag or m == 0.0:
#         return (x, y)
#     s = max_mag / m
#     return (x*s, y*s)

# def _unit(x: float, y: float, eps: float = 1e-9) -> Vec2:
#     m = math.hypot(x, y)
#     if m < eps: return (0.0, 0.0)
#     return (x/m, y/m)

# def _try_get_swarm_poses(robot):
#     """Return list of neighbor poses if the sim exposes them, else []."""
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

# def _detect_sim_id(robot):
#     rid_attr = getattr(robot, "id", None)
#     try:
#         return rid_attr() if callable(rid_attr) else int(rid_attr)
#     except:
#         return -1

# # ============ boids step (single agent against neighbor list) ============
# def boids_step_single(ax: Agent, dt: float, params: Params, bounds: Bounds, dancer: Circle,
#                       neighbors_xy: List[Tuple[float, float]]) -> Tuple[float, float]:
#     """
#     Compute next (vx, vy) for one agent given neighbor (x,y) positions.
#     """
#     # 1) Migration: push left toward goal_x
#     mig = (params.goal_x - ax.x, 0.0)  # horizontal attractive vector
#     mig_mag = max(params.eps, math.hypot(*mig))
#     mig_vec = (params.k_mig * mig[0] / mig_mag, params.k_mig * mig[1] / mig_mag)

#     # 2) Separation: repel from neighbors within r_sep
#     sep_x = 0.0; sep_y = 0.0
#     rsep2 = params.r_sep * params.r_sep
#     for (nx, ny) in neighbors_xy:
#         dx = ax.x - nx; dy = ax.y - ny
#         d2 = dx*dx + dy*dy
#         if 0.0 < d2 < rsep2:
#             d = math.sqrt(d2)
#             scale = params.k_sep / (d2 + params.eps)  # 1/d^2
#             sep_x += scale * (dx / (d + params.eps))
#             sep_y += scale * (dy / (d + params.eps))

#     # 3) Dancer obstacle: repel from SAFE_BUBBLE + margin
#     dxo = ax.x - dancer.cx; dyo = ax.y - dancer.cy
#     do = math.hypot(dxo, dyo)
#     clearance = do - dancer.r
#     obs_x = 0.0; obs_y = 0.0
#     if clearance < params.r_obs:
#         nxo = dxo / max(params.eps, do)
#         nyo = dyo / max(params.eps, do)
#         strength = params.k_obs / max(params.eps, clearance * clearance)
#         obs_x = strength * nxo
#         obs_y = strength * nyo

#     # 4) Soft walls: inverse-square push inside r_wall
#     wall_x = 0.0; wall_y = 0.0
#     dl = ax.x - bounds.x_min
#     if dl < params.r_wall:
#         wall_x += params.k_wall / max(params.eps, dl * dl)
#     dr = bounds.x_max - ax.x
#     if dr < params.r_wall:
#         wall_x -= params.k_wall / max(params.eps, dr * dr)
#     db = ax.y - bounds.y_min
#     if db < params.r_wall:
#         wall_y += params.k_wall / max(params.eps, db * db)
#     dtp = bounds.y_max - ax.y
#     if dtp < params.r_wall:
#         wall_y -= params.k_wall / max(params.eps, dtp * dtp)

#     # Combine as an acceleration-like vector, cap accel
#     ax_des = mig_vec[0] + sep_x + obs_x + wall_x
#     ay_des = mig_vec[1] + sep_y + obs_y + wall_y
#     ax_des, ay_des = _limit_vec(ax_des, ay_des, params.a_max)

#     # Semi-implicit Euler velocity update, cap speed
#     vx_new = ax.vx + ax_des * dt
#     vy_new = ax.vy + ay_des * dt
#     vx_new, vy_new = _limit_vec(vx_new, vy_new, params.v_max)

#     return vx_new, vy_new

# # ============ main sim entry ============

# def usr(robot):
#     """
#     Simulator entrypoint. Each robot runs this script independently.
#     We reconstruct local 'Agent' state per robot and pull neighbor (x,y) via sim API if available.
#     """
#     robot.delay(3000)

#     rid = _detect_sim_id(robot)

#     # logging
#     def logw(s): print(s)
#     logw(f"FLOAT-BOIDS(SIM): I am robot {rid}")

#     # seeded randomness per robot (if you add any stochasticity later)
#     random.seed((rid if rid is not None else 0) * 2654435761 & 0xFFFFFFFF)

#     # sim timebase
#     has_clock = hasattr(robot, "get_clock")
#     start_time = robot.get_clock() if has_clock else 0.0
#     acc_time = 0.0
#     LOOP_DT_MS = 40
#     MAX_RUNTIME = 55.0
#     last_log_sec = -1

#     # constants struct
#     P = Params()
#     B = Bounds(X_MIN, X_MAX, Y_MIN, Y_MAX)
#     D = Circle(OBST_CX, OBST_CY, SAFE_BUBBLE)

#     # wheel EMA
#     last_left = 0
#     last_right = 0

#     # local agent state (init from pose; world-frame velocity starts at 0)
#     pose = robot.get_pose() or (0.0, 0.0, 0.0)
#     ax = Agent(x=float(pose[0]), y=float(pose[1]), vx=0.0, vy=0.0, id=rid)

#     try:
#         while True:
#             now = robot.get_clock() if has_clock else (start_time + acc_time/1000.0)
#             t = (now - start_time) if has_clock else (acc_time/1000.0)
#             if t >= MAX_RUNTIME:
#                 break

#             p = robot.get_pose()
#             if not p or len(p) < 3:
#                 robot.set_vel(0, 0)
#                 robot.delay(LOOP_DT_MS)
#                 if not has_clock: acc_time += LOOP_DT_MS
#                 continue

#             x, y, th = float(p[0]), float(p[1]), float(p[2])
#             ax.x, ax.y = x, y  # keep agent’s position synced to sim

#             # gather neighbors (x,y) if sim exposes them
#             neighbors_xy: List[Tuple[float, float]] = []
#             poses = _try_get_swarm_poses(robot)
#             if poses:
#                 for item in poses:
#                     # accept (id,x,y,theta) or (x,y,theta)
#                     if isinstance(item, (list, tuple)) and len(item) >= 3:
#                         if len(item) == 4:
#                             nid, nx, ny, _nth = item
#                             # ignore self if sim includes it
#                             try:
#                                 if int(nid) == int(rid): continue
#                             except:
#                                 pass
#                         else:
#                             nx, ny = float(item[0]), float(item[1])
#                         neighbors_xy.append((float(nx), float(ny)))

#             # perform boids step (world-frame velocity)
#             vx, vy = boids_step_single(ax, dt=LOOP_DT_MS/1000.0, params=P, bounds=B, dancer=D,
#                                        neighbors_xy=neighbors_xy)
#             ax.vx, ax.vy = vx, vy

#             # --- map desired world velocity → wheel commands ---
#             # If near-zero, add a tiny nudge to avoid stall
#             if abs(vx) + abs(vy) < EPS:
#                 vx += 0.02 * (-math.sin(th))
#                 vy += 0.02 * ( math.cos(th))

#             # desired heading & turn error
#             hdg = math.atan2(vy, vx)
#             err = hdg - th
#             while err >  math.pi: err -= 2.0*math.pi
#             while err <= -math.pi: err += 2.0*math.pi

#             # forward bias based on heading error (prefer smooth arcs)
#             ae = abs(err)
#             if ae < 0.6:   fwd = FWD_FAST
#             elif ae < 1.2: fwd = FWD_FAST * 0.80
#             else:          fwd = FWD_SLOW
#             if fwd < FWD_MIN: fwd = FWD_MIN

#             turn = max(-1.1, min(1.1, TURN_K * err))

#             # wheel command (with EMA smoothing)
#             left_cmd  = int(MAX_WHEEL * 0.90 * (fwd - 0.75 * turn))
#             right_cmd = int(MAX_WHEEL * 0.90 * (fwd + 0.75 * turn))
#             left_cmd  = max(-MAX_WHEEL, min(MAX_WHEEL, left_cmd))
#             right_cmd = max(-MAX_WHEEL, min(MAX_WHEEL, right_cmd))

#             left  = int((1.0 - CMD_SMOOTH) * left_cmd  + CMD_SMOOTH * last_left)
#             right = int((1.0 - CMD_SMOOTH) * right_cmd + CMD_SMOOTH * last_right)
#             last_left, last_right = left, right

#             robot.set_vel(left, right)

#             # periodic log
#             if int(now) != last_log_sec and (t % 2.0) < (LOOP_DT_MS/1000.0 + 0.02):
#                 logw(f"FLOAT-BOIDS(SIM) {rid} pos [{x:.3f}, {y:.3f}] nbh={len(neighbors_xy)}")
#                 last_log_sec = int(now)

#             robot.delay(LOOP_DT_MS)
#             if not has_clock: acc_time += LOOP_DT_MS

#     except Exception as e:
#         try:
#             robot.set_vel(0, 0)
#             robot.set_led(255, 0, 0)
#         except:
#             pass
#         print(f"ERROR(FLOAT-BOIDS SIM): {e}")
#         raise
#     finally:
#         final_time = (robot.get_clock() if hasattr(robot, "get_clock") else (start_time + acc_time/1000.0))
#         print(f"FLOAT-BOIDS {rid} finished at [{ax.x:.3f}, {ax.y:.3f}] after {final_time - (start_time if has_clock else 0.0):.1f}s")
