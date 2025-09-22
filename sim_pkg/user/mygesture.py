# def usr(robot):
#     # Each robot turns left for 2 seconds, then right for 2 seconds, then stops
#     id = robot.id

#     # Start turning left
#     robot.set_vel(-1.0, 1.0)
#     robot.delay(6000)  # delay is in milliseconds

#     # Now turn right
#     robot.set_vel(1.0, -1.0)
#     robot.delay(6000)

#     # Stop
#     robot.set_vel(0, 0)

#     # Optionally blink LED green to show finished
#     for _ in range(3):
#         robot.set_led(0, 100, 0)
#         robot.delay(200)
#         robot.set_led(0, 0, 0)
#         robot.delay(200)

#     # Stop forever
#     while True:
#         robot.delay(1000)


# # user/mygesture.py
# import math, ast

# def _parse_pose(p):
#     """Return (ok, x, y, theta) where ok=False if p is invalid."""
#     if p is None or p is False:
#         return False, None, None, None
#     if isinstance(p, str):
#         try:
#             p = ast.literal_eval(p)
#         except Exception:
#             return False, None, None, None
#     if isinstance(p, (list, tuple)) and len(p) >= 3:
#         try:
#             x = float(p[0]); y = float(p[1]); theta = float(p[2])
#             return True, x, y, theta
#         except Exception:
#             return False, None, None, None
#     return False, None, None, None

# def usr(robot):
#     target = math.pi        # facing left (west)
#     tol = 0.12              # tolerance (radians). tweak as needed.
#     turn_l, turn_r = -10, 10  # turning left in place

#     # optional: show we're starting
#     robot.set_led(0, 0, 255)

#     while True:
#         pose_raw = robot.get_pose()
#         ok, x, y, theta = _parse_pose(pose_raw)

#         if not ok:
#             # Pose not available—log and try again shortly
#             robot.log(["POSE_INVALID", pose_raw])
#             robot.set_vel(0, 0)   # safe stop while waiting for good pose
#             robot.delay(100)
#             continue

#         # normalize angle error to [-pi, pi]
#         err = (target - theta + math.pi) % (2 * math.pi) - math.pi

#         # Stop if close enough to facing left
#         if abs(err) < tol or abs(abs(theta) - math.pi) < tol:
#             robot.set_vel(0, 0)
#             # blink green to confirm success
#             for _ in range(2):
#                 robot.set_led(0, 255, 0)
#                 robot.delay(150)
#                 robot.set_led(0, 0, 0)
#                 robot.delay(150)
#             break

#         # Keep turning left
#         robot.set_vel(turn_l, turn_r)
#         robot.delay(100)  # re-check at 10 Hz

#     # idle after success
#     while True:
#         robot.delay(1000)


# user/mygesture.py
import math, ast

X_LEFT_LINE = 0.50      # stop line: bots move until x <= this
MARGIN      = 0.05      # tolerance so we don't jitter on the line
SPIN_SLOW   = (-5, 5)   # slow turn left
ARC_SLOW    = (8, 10)   # gentle forward-left arc
HEADING_TOL = 0.12      # ~7°

def _parse_pose(p):
    if p is None or p is False: return False, None, None, None
    if isinstance(p, str):
        try: p = ast.literal_eval(p)
        except Exception: return False, None, None, None
    if isinstance(p, (list, tuple)) and len(p) >= 3:
        try: return True, float(p[0]), float(p[1]), float(p[2])
        except Exception: return False, None, None, None
    return False, None, None, None

def _turn_left_until_facing_left(robot):
    target = math.pi
    while True:
        ok, x, y, th = _parse_pose(robot.get_pose())
        if not ok: robot.set_vel(0,0); robot.delay(80); continue
        err = (target - th + math.pi) % (2*math.pi) - math.pi
        if abs(err) < HEADING_TOL or abs(abs(th)-math.pi) < HEADING_TOL:
            robot.set_vel(0,0); return
        robot.set_vel(*SPIN_SLOW)
        robot.delay(100)

def usr(robot):
    # small sync so phases feel simultaneous
    start_at = robot.get_clock() + 1.0
    while robot.get_clock() < start_at: robot.delay(20)

    # get pose
    ok, x, y, th = _parse_pose(robot.get_pose())
    while not ok:
        robot.delay(80)
        ok, x, y, th = _parse_pose(robot.get_pose())

    if x > X_LEFT_LINE + MARGIN:
        # RIGHT of the line → rotate left, then advance left until on/left of line
        _turn_left_until_facing_left(robot)
        safety_ms = 20_000
        elapsed = 0
        while True:
            ok, x, y, th = _parse_pose(robot.get_pose())
            if ok and x <= X_LEFT_LINE + MARGIN:
                break
            robot.set_vel(*ARC_SLOW)  # gentle forward-left
            robot.delay(100)
            elapsed += 100
            if elapsed >= safety_ms: break
        robot.set_vel(0, 0)

    else:
        # ALREADY at/left of the line → do NOT move further left
        # optional: subtle idle animation (tiny up/down sway)
        idle_ms = 3000
        elapsed = 0
        while elapsed < idle_ms:
            robot.set_vel(10, 10)   # small forward creep
            robot.delay(200)
            robot.set_vel(0, 0)
            robot.delay(200)
            elapsed += 400

    # done: blink green
    for _ in range(2):
        robot.set_led(0,255,0); robot.delay(150)
        robot.set_led(0,0,0);   robot.delay(150)

    while True: robot.delay(1000)
