#!/usr/bin/env python3
import math, random, csv

# ===== Arena bounds =====
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

# ===== Dancer no-go zone =====
FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET   # ≈ 0.1524 m
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
OBST_CX, OBST_CY = (-0.1, 0.475)

# ===== Generation parameters =====
N_BOTS = 11
MIN_SEP = 0.12        # min spacing between bots
SAFE_MARGIN = 0.08    # margin from boundaries

SAVE_TO_FILE = True
FILENAME = "init_pose_random.csv"

def random_pose():
    """Return a random (x, y, theta) inside bounds and outside SAFE_BUBBLE."""
    while True:
        x = random.uniform(X_MIN + SAFE_MARGIN, X_MAX - SAFE_MARGIN)
        y = random.uniform(Y_MIN + SAFE_MARGIN, Y_MAX - SAFE_MARGIN)
        dx, dy = x - OBST_CX, y - OBST_CY
        r = math.hypot(dx, dy)
        if r > SAFE_BUBBLE + 0.08:  # outside the no-go zone
            theta = random.uniform(-math.pi, math.pi)
            return x, y, theta

def too_close(p1, p2, min_sep=MIN_SEP):
    dx, dy = p1[0]-p2[0], p1[1]-p2[1]
    return (dx*dx + dy*dy) < (min_sep*min_sep)

def main():
    poses = []
    for i in range(N_BOTS):
        while True:
            p = random_pose()
            if all(not too_close(p, q) for q in poses):
                poses.append(p)
                break

    # Print to console
    for i, (x, y, th) in enumerate(poses):
        print(f"{i},{x:.6f},{y:.6f},{th:.6f}")

    # Optionally write CSV
    if SAVE_TO_FILE:
        with open(FILENAME, "w", newline="") as f:
            writer = csv.writer(f)
            for i, (x, y, th) in enumerate(poses):
                writer.writerow([i, x, y, th])
        print(f"\nSaved {len(poses)} poses to {FILENAME}")

if __name__ == "__main__":
    main()
