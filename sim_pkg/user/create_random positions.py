# #!/usr/bin/env python3
# import math, random, csv

# # ===== Arena bounds =====
# X_MIN, X_MAX = -1.2, 1.0
# Y_MIN, Y_MAX = -1.4, 2.35

# # ===== Dancer no-go zone =====
# FEET = 0.3048
# OBST_DIAM_FT = 1.0
# OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET   # ≈ 0.1524 m
# OBST_MARGIN  = 0.03
# SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
# OBST_CX, OBST_CY = (-0.1, 0.475)

# # ===== Generation parameters =====
# N_BOTS = 11
# MIN_SEP = 0.12        # min spacing between bots
# SAFE_MARGIN = 0.08    # margin from boundaries

# SAVE_TO_FILE = True
# FILENAME = "init_pose_random.csv"

# def random_pose():
#     """Return a random (x, y, theta) inside bounds and outside SAFE_BUBBLE."""
#     while True:
#         x = random.uniform(X_MIN + SAFE_MARGIN, X_MAX - SAFE_MARGIN)
#         y = random.uniform(Y_MIN + SAFE_MARGIN, Y_MAX - SAFE_MARGIN)
#         dx, dy = x - OBST_CX, y - OBST_CY
#         r = math.hypot(dx, dy)
#         if r > SAFE_BUBBLE + 0.08:  # outside the no-go zone
#             theta = random.uniform(-math.pi, math.pi)
#             return x, y, theta

# def too_close(p1, p2, min_sep=MIN_SEP):
#     dx, dy = p1[0]-p2[0], p1[1]-p2[1]
#     return (dx*dx + dy*dy) < (min_sep*min_sep)

# def main():
#     poses = []
#     for i in range(N_BOTS):
#         while True:
#             p = random_pose()
#             if all(not too_close(p, q) for q in poses):
#                 poses.append(p)
#                 break

#     # Print to console
#     for i, (x, y, th) in enumerate(poses):
#         print(f"{i},{x:.6f},{y:.6f},{th:.6f}")

#     # Optionally write CSV
#     if SAVE_TO_FILE:
#         with open(FILENAME, "w", newline="") as f:
#             writer = csv.writer(f)
#             for i, (x, y, th) in enumerate(poses):
#                 writer.writerow([i, x, y, th])
#         print(f"\nSaved {len(poses)} poses to {FILENAME}")

# if __name__ == "__main__":
#     main()

#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import math, random, csv, sys

# --- arena bounds (meters) ---
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35
BOUND_MARGIN  = 0.02   # tiny wall margin to avoid edge cases

# --- dancer no-go circle ---
FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
BUBBLE_EXTRA = 0.00    # additional clearance; set 0.04 if you want more
CX, CY = (-0.1, 0.475)

# --- generation params ---
N_BOTS    = 11
MIN_SEP   = 0.02        # 2 cm minimum pairwise spacing
SEED      = 123         # None for nondeterministic
ORIENT    = "random"    # "random", "tangent", or "radial"
MAX_TRIES = 200000      # total attempts budget

SAVE_TO_FILE = True
FILENAME     = "init_pose_random.csv"

# --- helpers ---
def in_bounds(x, y):
    return (X_MIN + BOUND_MARGIN <= x <= X_MAX - BOUND_MARGIN and
            Y_MIN + BOUND_MARGIN <= y <= Y_MAX - BOUND_MARGIN)

def outside_bubble(x, y, r_bub):
    dx = x - CX
    dy = y - CY
    return (dx*dx + dy*dy) >= (r_bub * r_bub)

def too_close_xy(x1, y1, x2, y2, min_sep):
    dx = x1 - x2
    dy = y1 - y2
    return (dx*dx + dy*dy) < (min_sep * min_sep)

def heading_for(x, y, mode):
    if mode == "tangent":
        dx, dy = x - CX, y - CY
        r = math.hypot(dx, dy)
        if r < 1e-9:
            return random.uniform(-math.pi, math.pi)
        urx, ury = dx / r, dy / r
        tx, ty = -ury, urx   # CCW tangent
        return math.atan2(ty, tx)
    elif mode == "radial":
        return math.atan2(y - CY, x - CX)
    else:
        return random.uniform(-math.pi, math.pi)

def main():
    if SEED is not None:
        random.seed(SEED)

    poses = []  # list of (x, y, th)
    tries = 0
    r_bub = SAFE_BUBBLE + BUBBLE_EXTRA

    while len(poses) < N_BOTS and tries < MAX_TRIES:
        tries += 1
        x = random.uniform(X_MIN + BOUND_MARGIN, X_MAX - BOUND_MARGIN)
        y = random.uniform(Y_MIN + BOUND_MARGIN, Y_MAX - BOUND_MARGIN)

        if not in_bounds(x, y):
            continue
        if not outside_bubble(x, y, r_bub):
            continue

        clash = False
        for (px, py, _pth) in poses:
            if too_close_xy(x, y, px, py, MIN_SEP):
                clash = True
                break
        if clash:
            continue

        th = heading_for(x, y, ORIENT)
        poses.append((x, y, th))

    if len(poses) < N_BOTS:
        print("Placed %d / %d after %d tries. Consider lowering MIN_SEP or margins."
              % (len(poses), N_BOTS, tries))
    else:
        print("Placed %d bots after %d tries" % (len(poses), tries))

    for i, (x, y, th) in enumerate(poses):
        print("%d,%.6f,%.6f,%.6f" % (i, x, y, th))

    if SAVE_TO_FILE:
        py3 = (sys.version_info[0] >= 3)
        mode = "w" if py3 else "wb"
        f = open(FILENAME, mode)
        try:
            w = csv.writer(f)
            for i, (x, y, th) in enumerate(poses):
                w.writerow([i, x, y, th])
        finally:
            f.close()
        print("\nSaved %d poses to %s" % (len(poses), FILENAME))

if __name__ == "__main__":
    main()
