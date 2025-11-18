# # sim_pkg/user/make_circle_position_csv.py
# import math, csv, os, sys

# # ----------------- Testbed bounds (meters) -----------------
# X_MIN, X_MAX = -1.2, 1.0
# Y_MIN, Y_MAX = -1.4, 2.35
# CX = (X_MIN + X_MAX) / 2.0    # -0.1
# CY = (Y_MIN + Y_MAX) / 2.0    #  0.475

# # ----------------- Dancer no-go circle (1-foot DIAMETER) -----------------
# # FEET = 0.3048
# FEET = 1.2
# OBST_DIAM_FT = 1.0
# OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET   # 0.1524 m
# OBST_MARGIN  = 0.03                        # small extra collar (tweakable)
# SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN   # ~0.1824 m

# # ----------------- Ring + spacing -----------------
# N_BOTS       = 11                         # IDs 0..4
# MIN_SPACING  = 0.25                        # per testbed rule (m)
# WALL_MARGIN  = 0.08                        # stay off walls by this much
# TANGENT_CCW  = True
# ANGLE_OFFSET = 0.0
# OUT          = "sim_pkg/user/init_pose.csv"

# # ---- compute feasible radius range, centered at (CX,CY) ----
# R_wall = min(X_MAX - CX, CX - X_MIN, Y_MAX - CY, CY - Y_MIN) - WALL_MARGIN
# if R_wall <= 0:
#     print("Field too small after WALL_MARGIN; reduce WALL_MARGIN.", file=sys.stderr)
#     sys.exit(1)

# # spacing-limited minimum radius (adjacent chord >= MIN_SPACING)
# if N_BOTS >= 2:
#     R_spacing = MIN_SPACING / (2.0 * math.sin(math.pi / N_BOTS))
# else:
#     R_spacing = 0.0

# # choose a radius safely between bubble and wall: midpoint works well
# R_mid = 0.5 * (SAFE_BUBBLE + R_wall)

# # final radius honors spacing min and wall max, prefers midpoint
# R = min(R_wall, max(R_mid, R_spacing))
# if R < SAFE_BUBBLE:
#     print(
#         "Impossible geometry:\n"
#         f"  SAFE_BUBBLE={SAFE_BUBBLE:.3f}  R_wall={R_wall:.3f}  R_spacing={R_spacing:.3f}",
#         file=sys.stderr
#     )
#     sys.exit(2)

# # ---- write CSV ----
# os.makedirs(os.path.dirname(OUT), exist_ok=True)
# with open(OUT, "w", newline="") as f:
#     w = csv.writer(f)
#     for i in range(N_BOTS):
#         ang = ANGLE_OFFSET + 2.0 * math.pi * i / max(1, N_BOTS)
#         x = CX + R * math.cos(ang)
#         y = CY + R * math.sin(ang)
#         th = ang + (math.pi/2 if TANGENT_CCW else -math.pi/2)
#         assert X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX, "Pose out of bounds; adjust WALL_MARGIN."
#         w.writerow([i, x, y, th])

# # ---- report ----
# adj = 2.0 * R * math.sin(math.pi / N_BOTS) if N_BOTS >= 2 else float('inf')
# s_obst = max(0.0, R - SAFE_BUBBLE)
# s_wall_left  = max(0.0, CX - (X_MIN + WALL_MARGIN + R))
# s_wall_right = max(0.0, (X_MAX - WALL_MARGIN - R) - CX)

# print(f"Wrote {OUT}")
# print(f"Center=({CX:.3f},{CY:.3f})  R_used={R:.3f} m  SAFE_BUBBLE={SAFE_BUBBLE:.3f} m")
# print(f"Adjacent spacing={adj:.3f} m (>= {MIN_SPACING} m rule)")
# print(f"Shift budgets (max translation): bubble={s_obst:.3f} m, wall_left={s_wall_left:.3f} m, wall_right={s_wall_right:.3f} m")



# sim_pkg/user/make_two_ring_position_csv.py
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
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN   # inner radius must stay outside this

# ----------------- Ring + spacing -----------------
N_INNER      = 8                          # number of bots on inner ring
N_OUTER      = 12                           # number of bots on outer ring
MIN_SPACING  = 0.25                        # chord length between adjacent bots (per ring)
WALL_MARGIN  = 0.08                        # stay off walls by this much
RING_GAP     = 0.18                        # minimum radial gap between inner & outer rings

TANGENT_CCW      = True
ANGLE_OFFSET_IN  = 0.0                     # global rotation for inner ring
ANGLE_OFFSET_OUT = math.pi / max(1, N_OUTER)  # offset outer ring by half-step

OUT          = "sim_pkg/user/init_pose_two_rings.csv"

# ---- helper: spacing-limited minimum radius for a ring ----
def spacing_min_radius(n_bots, min_spacing):
    if n_bots >= 2:
        # chord length c = 2 R sin(pi / n) >= min_spacing
        return min_spacing / (2.0 * math.sin(math.pi / n_bots))
    else:
        return 0.0

# ---- overall wall-limited maximum radius (same center for both rings) ----
R_wall = min(X_MAX - CX, CX - X_MIN, Y_MAX - CY, CY - Y_MIN) - WALL_MARGIN
if R_wall <= 0:
    print("Field too small after WALL_MARGIN; reduce WALL_MARGIN.", file=sys.stderr)
    sys.exit(1)

# ---- outer ring radius ----
R_outer_spacing = spacing_min_radius(N_OUTER, MIN_SPACING)
# a "nice" middle radius if we only had one ring
R_mid_single = 0.5 * (SAFE_BUBBLE + R_wall)

# outer ring must be at least SAFE_BUBBLE + RING_GAP
R_outer_min = max(SAFE_BUBBLE + RING_GAP, R_outer_spacing)
# prefer something like the original midpoint, but obey min / wall
R_outer = min(R_wall, max(R_mid_single, R_outer_min))

if R_outer < R_outer_min - 1e-6:
    print(
        "Cannot place outer ring:\n"
        f"  R_outer_min={R_outer_min:.3f}, R_wall={R_wall:.3f}",
        file=sys.stderr
    )
    sys.exit(2)

# ---- inner ring radius ----
R_inner_spacing = spacing_min_radius(N_INNER, MIN_SPACING)

# inner must be outside SAFE_BUBBLE and inside (R_outer - RING_GAP)
R_inner_max = R_outer - RING_GAP
R_inner_min = max(SAFE_BUBBLE, R_inner_spacing)

if R_inner_max <= R_inner_min:
    print(
        "Cannot place inner ring:\n"
        f"  R_inner_min={R_inner_min:.3f}, R_inner_max={R_inner_max:.3f}",
        file=sys.stderr
    )
    sys.exit(3)

# choose a nice radius in the available band
R_inner = 0.5 * (R_inner_min + R_inner_max)

# sanity-check both radii
if not (SAFE_BUBBLE <= R_inner < R_outer <= R_wall + 1e-6):
    print(
        "Bad radii after selection:\n"
        f"  R_inner={R_inner:.3f}, R_outer={R_outer:.3f}, "
        f"SAFE_BUBBLE={SAFE_BUBBLE:.3f}, R_wall={R_wall:.3f}",
        file=sys.stderr
    )
    sys.exit(4)

# ---- write CSV ----
os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "w", newline="") as f:
    w = csv.writer(f)

    # inner ring: IDs 0 .. N_INNER-1
    for i in range(N_INNER):
        ang = ANGLE_OFFSET_IN + 2.0 * math.pi * i / max(1, N_INNER)
        x = CX + R_inner * math.cos(ang)
        y = CY + R_inner * math.sin(ang)
        th = ang + (math.pi/2 if TANGENT_CCW else -math.pi/2)
        assert X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX, "Inner pose out of bounds; adjust WALL_MARGIN or RING_GAP."
        w.writerow([i, x, y, th])

    # outer ring: IDs N_INNER .. N_INNER+N_OUTER-1
    for j in range(N_OUTER):
        ang = ANGLE_OFFSET_OUT + 2.0 * math.pi * j / max(1, N_OUTER)
        x = CX + R_outer * math.cos(ang)
        y = CY + R_outer * math.sin(ang)
        th = ang + (math.pi/2 if TANGENT_CCW else -math.pi/2)
        assert X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX, "Outer pose out of bounds; adjust WALL_MARGIN or RING_GAP."
        w.writerow([N_INNER + j, x, y, th])

# ---- report ----
def adj_spacing(R, n):
    if n >= 2:
        return 2.0 * R * math.sin(math.pi / n)
    else:
        return float('inf')

adj_in  = adj_spacing(R_inner, N_INNER)
adj_out = adj_spacing(R_outer, N_OUTER)

print(f"Wrote {OUT}")
print(f"Center=({CX:.3f},{CY:.3f})")
print(f"Inner ring:  N={N_INNER}, R_inner={R_inner:.3f} m, spacing={adj_in:.3f} m (>= {MIN_SPACING} m rule)")
print(f"Outer ring:  N={N_OUTER}, R_outer={R_outer:.3f} m, spacing={adj_out:.3f} m (>= {MIN_SPACING} m rule)")
print(f"Radial gap between rings ≈ {R_outer - R_inner:.3f} m (target >= {RING_GAP} m)")
print(f"SAFE_BUBBLE={SAFE_BUBBLE:.3f} m, R_wall={R_wall:.3f} m")
