# import math, csv, os

# N_TOTAL = 10              # 1 center + 29 on ring
# RING_RADIUS = 1.0        # meters (within COMM range so they see center)
# OUT = "sim_pkg/user/init_pose_circle.csv"

# os.makedirs(os.path.dirname(OUT), exist_ok=True)
# with open(OUT, "w", newline="") as f:
#     w = csv.writer(f)
#     # ID 0 = center "dancer"
#     w.writerow([0, 0.0, 0.0, 0.0])
#     # IDs 1..29 = ring, tangent heading (CCW)
#     for i in range(1, N_TOTAL):
#         k = i - 1
#         ang = 2.0 * math.pi * k / (N_TOTAL - 1)
#         x = RING_RADIUS * math.cos(ang)
#         y = RING_RADIUS * math.sin(ang)
#         th = ang + math.pi/2  # tangent CCW; for CW use ang - math.pi/2
#         w.writerow([i, x, y, th])

# print(f"Wrote {OUT}")

# sim_pkg/user/make_init_circle_csv.py


# sim_pkg/user/make_init_circle_csv.py
import math, csv, os

# ===== user knobs =====
N_BOTS = 10                                  # number of coachbots on the ring (IDs 0..N_BOTS-1)
OUT = "sim_pkg/user/init_pose_circle.csv"    # run from repo root, or adjust

# No-go (dancer) circle
OBST_DIAM_FT = 3.0
FEET = 0.3048
OBST_RADIUS = 0.5 * OBST_DIAM_FT * FEET      # 3 ft diameter -> 0.4572 m radius

# We want the robots' ring to be TWICE the boundary circle (by radius):
RING_SCALE = 2.0                              # ring_radius = RING_SCALE * OBST_RADIUS
MARGIN_M = 0.08                               # small extra collar

# Headings on the ring
TANGENT_CCW = True                            # tangent CCW (flip for CW)
ANGLE_OFFSET = 0.0                            # rotate the whole ring if desired (radians)

# Optional: stationary beacons to "draw" the boundary (IDs 900..)
ADD_BEACONS = True
NUM_BEACONS = 24

# ===== compute ring radius =====
ring_radius = max(RING_SCALE * OBST_RADIUS, OBST_RADIUS + MARGIN_M)

# ===== write csv =====
os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "w", newline="") as f:
    w = csv.writer(f)

    # ring robots: IDs 0..N_BOTS-1
    for i in range(N_BOTS):
        ang = ANGLE_OFFSET + 2.0 * math.pi * i / N_BOTS
        x = ring_radius * math.cos(ang)
        y = ring_radius * math.sin(ang)
        th = ang + (math.pi/2 if TANGENT_CCW else -math.pi/2)
        w.writerow([i, x, y, th])

    # optional beacons to "draw" the no-go circle
    if ADD_BEACONS:
        for k in range(NUM_BEACONS):
            a = 2.0 * math.pi * k / NUM_BEACONS
            bx = OBST_RADIUS * math.cos(a)
            by = OBST_RADIUS * math.sin(a)
            w.writerow([900 + k, bx, by, 0.0])

print(f"Wrote {OUT}")
print(f"No-go radius = {OBST_RADIUS:.4f} m; ring radius = {ring_radius:.4f} m")
