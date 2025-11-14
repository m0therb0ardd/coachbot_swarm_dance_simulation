# sim_pkg/user/make_circle_position_csv.py
import math, csv, os, sys

# ----------------- Testbed bounds (meters) -----------------
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35
CX = (X_MIN + X_MAX) / 2.0    # -0.1
CY = (Y_MIN + Y_MAX) / 2.0    #  0.475

# ----------------- Dancer no-go circle (1-foot DIAMETER) -----------------
# FEET = 0.3048
FEET = 1.2
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET   # 0.1524 m
OBST_MARGIN  = 0.03                        # small extra collar (tweakable)
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN   # ~0.1824 m

# ----------------- Ring + spacing -----------------
N_BOTS       = 11                         # IDs 0..4
MIN_SPACING  = 0.25                        # per testbed rule (m)
WALL_MARGIN  = 0.08                        # stay off walls by this much
TANGENT_CCW  = True
ANGLE_OFFSET = 0.0
OUT          = "sim_pkg/user/init_pose.csv"

# ---- compute feasible radius range, centered at (CX,CY) ----
R_wall = min(X_MAX - CX, CX - X_MIN, Y_MAX - CY, CY - Y_MIN) - WALL_MARGIN
if R_wall <= 0:
    print("Field too small after WALL_MARGIN; reduce WALL_MARGIN.", file=sys.stderr)
    sys.exit(1)

# spacing-limited minimum radius (adjacent chord >= MIN_SPACING)
if N_BOTS >= 2:
    R_spacing = MIN_SPACING / (2.0 * math.sin(math.pi / N_BOTS))
else:
    R_spacing = 0.0

# choose a radius safely between bubble and wall: midpoint works well
R_mid = 0.5 * (SAFE_BUBBLE + R_wall)

# final radius honors spacing min and wall max, prefers midpoint
R = min(R_wall, max(R_mid, R_spacing))
if R < SAFE_BUBBLE:
    print(
        "Impossible geometry:\n"
        f"  SAFE_BUBBLE={SAFE_BUBBLE:.3f}  R_wall={R_wall:.3f}  R_spacing={R_spacing:.3f}",
        file=sys.stderr
    )
    sys.exit(2)

# ---- write CSV ----
os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "w", newline="") as f:
    w = csv.writer(f)
    for i in range(N_BOTS):
        ang = ANGLE_OFFSET + 2.0 * math.pi * i / max(1, N_BOTS)
        x = CX + R * math.cos(ang)
        y = CY + R * math.sin(ang)
        th = ang + (math.pi/2 if TANGENT_CCW else -math.pi/2)
        assert X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX, "Pose out of bounds; adjust WALL_MARGIN."
        w.writerow([i, x, y, th])

# ---- report ----
adj = 2.0 * R * math.sin(math.pi / N_BOTS) if N_BOTS >= 2 else float('inf')
s_obst = max(0.0, R - SAFE_BUBBLE)
s_wall_left  = max(0.0, CX - (X_MIN + WALL_MARGIN + R))
s_wall_right = max(0.0, (X_MAX - WALL_MARGIN - R) - CX)

print(f"Wrote {OUT}")
print(f"Center=({CX:.3f},{CY:.3f})  R_used={R:.3f} m  SAFE_BUBBLE={SAFE_BUBBLE:.3f} m")
print(f"Adjacent spacing={adj:.3f} m (>= {MIN_SPACING} m rule)")
print(f"Shift budgets (max translation): bubble={s_obst:.3f} m, wall_left={s_wall_left:.3f} m, wall_right={s_wall_right:.3f} m")
