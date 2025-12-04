# -*- coding: utf-8 -*-
# Rigid body horizontal translation for two-ring formation
# Movement is along the X axis only
# MOVE_DIR = +1 moves right
# MOVE_DIR = -1 moves left
# All robots maintain their initial offset relative to the ring center
# Robots move together as one rigid body and then correct at the end

import math
import os
import struct

# Arena bounds
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

# Ring center reference point
CX, CY = (-0.1, 0.475)

# Dancer no-go circle
FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS = 0.5 * OBST_DIAM_FT * FEET
OBST_MARGIN = 0.03
SAFE_BUBBLE = OBST_RADIUS + OBST_MARGIN

# Movement direction
# +1 means move right
# -1 means move left
MOVE_DIR = +1

# Translation rate and stop conditions
BASE_SHIFT_RATE = 0.18
STOP_MARGIN = 0.08

# Wheel commands
MAX_WHEEL = 35
TURN_K = 3.0
FWD_FAST = 0.8
FWD_SLOW = 0.3
EPS = 1e-3

# PD gains
KX = 1.2
KY = 2.0
KR = 2.6

# Time settings
MAX_MOVE_TIME = 20.0
DT_MS = 20

# Heartbeat format: x, y, theta, id, s_stop
HB_FMT = "fffif"
HB_BYTES = struct.calcsize(HB_FMT)
HB_DT = 0.20
STALE_S = 1.0


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def safe_pose(robot):
    p = robot.get_pose()
    if p and len(p) >= 3:
        return float(p[0]), float(p[1]), float(p[2])
    return None


def usr(robot):

    robot.delay(3000)

    try:
        vid = robot.virtual_id()
    except:
        vid = -1

    log_main = open("experiment_log.txt", "a")
    def logw(s):
        if not s.endswith("\n"):
            s = s + "\n"
        log_main.write(s)
        log_main.flush()
        try:
            os.fsync(log_main.fileno())
        except:
            pass

    try:
        logw("Robot %s online" % str(vid))

        def boundary_state(x, y):
            warn = 0.08
            crit = 0.02
            if (x < X_MIN + crit or x > X_MAX - crit or
                y < Y_MIN + crit or y > Y_MAX - crit):
                return 2
            if (x < X_MIN + warn or x > X_MAX - warn or
                y < Y_MIN + warn or y > Y_MAX - warn):
                return 1
            return 0

        rel_off = None
        R_form = None
        local_s_stop = None
        global_s_stop = None
        started = False
        t0 = None

        nbr_limits = {}
        last_hb = -1e9

        start_time = robot.get_clock()
        motion_start = None
        forced_time_stop = False

        last_log = -1.0
        last_pose = None

        while (robot.get_clock() - start_time) < 55.0:

            pose = safe_pose(robot)
            if pose is None:
                robot.set_vel(0, 0)
                robot.delay(DT_MS)
                continue

            x, y, th = pose
            last_pose = (x, y)
            now = robot.get_clock()

            b = boundary_state(x, y)
            if b == 2:
                robot.set_vel(0, 0)
                robot.set_led(255, 0, 0)
                logw("Robot %s hit boundary" % str(vid))
                break
            elif b == 1:
                robot.set_led(255, 150, 0)
            else:
                robot.set_led(0, 180, 180)

            if rel_off is None:
                rel_off = (x - CX, y - CY)
                R_form = math.hypot(rel_off[0], rel_off[1])

                safety_buffer = 0.05
                if MOVE_DIR > 0:
                    s_wall = max(0.0, (X_MAX - STOP_MARGIN - safety_buffer - R_form) - CX)
                else:
                    s_wall = max(0.0, CX - (X_MIN + STOP_MARGIN + safety_buffer + R_form))

                s_obst = max(0.0, R_form - SAFE_BUBBLE)
                local_s_stop = min(s_wall, s_obst)

                t0 = robot.get_clock() + 1.0
                logw("Robot %s R_form %.3f local_s_stop %.3f" % (str(vid), R_form, local_s_stop))
                robot.set_led(255, 200, 0)

            if local_s_stop is not None and (now - last_hb) >= HB_DT:
                try:
                    msg = struct.pack(HB_FMT, float(x), float(y), float(th),
                                      int(vid), float(local_s_stop))
                    robot.send_msg(msg)
                except:
                    pass
                last_hb = now

            msgs = robot.recv_msg()
            if msgs:
                for m in msgs:
                    try:
                        if len(m) >= HB_BYTES:
                            nx, ny, nth, nid, n_s_stop = struct.unpack(HB_FMT, m[:HB_BYTES])
                            n_s_stop = float(n_s_stop)
                            nid = int(nid)
                            if nid != vid:
                                nbr_limits[nid] = (n_s_stop, now)
                    except:
                        pass

            cutoff = now - STALE_S
            for nid in list(nbr_limits.keys()):
                if nbr_limits[nid][1] < cutoff:
                    nbr_limits.pop(nid, None)

            if local_s_stop is not None:
                vals = [local_s_stop]
                for nid in nbr_limits:
                    vals.append(nbr_limits[nid][0])
                global_s_stop = min(vals)
            else:
                global_s_stop = None

            if not started:
                if now < t0:
                    robot.set_vel(0, 0)
                    robot.delay(DT_MS)
                    continue
                started = True
                motion_start = now
                robot.set_led(0, 200, 0)
                logw("Robot %s start motion" % str(vid))

            if motion_start is not None:
                if (now - motion_start) >= MAX_MOVE_TIME:
                    forced_time_stop = True

            if global_s_stop is not None:
                s_raw = max(0.0, (now - t0) * BASE_SHIFT_RATE)
                s = min(s_raw, global_s_stop)
            else:
                s = 0.0

            Cx = CX + MOVE_DIR * s
            Cy = CY

            Cx = max(X_MIN + R_form + 0.08, min(X_MAX - R_form - 0.08, Cx))

            tx = Cx + rel_off[0]
            ty = Cy + rel_off[1]

            ex = tx - x
            ey = ty - y
            pos_err = math.hypot(ex, ey)

            reached_global = False
            if global_s_stop is not None:
                if abs(global_s_stop - s) < 1e-3 and pos_err < 0.03:
                    reached_global = True

            if reached_global or forced_time_stop:
                robot.set_vel(0, 0)
                robot.set_led(0, 80, 255)
                logw("Robot %s reached final" % str(vid))
                break

            vx = KX * ex + KR * (rel_off[0] - (x - Cx))
            vy = KY * ey + KR * (rel_off[1] - (y - Cy))

            if abs(vx) + abs(vy) > EPS:
                hdg = math.atan2(vy, vx)
                err = wrap_angle(hdg - th)

                abs_err = abs(err)
                if abs_err < 0.5:
                    fwd = FWD_FAST
                elif abs_err < 1.2:
                    fwd = FWD_FAST * 0.7
                else:
                    fwd = FWD_SLOW

                if b == 1:
                    fwd *= 0.7

                turn = clamp(TURN_K * err, -1.5, 1.5)
                left = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
                right = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)), -MAX_WHEEL, MAX_WHEEL)
                robot.set_vel(left, right)
            else:
                robot.set_vel(0, 0)

            if now - last_log > 2.0 and global_s_stop is not None:
                logw("Robot %s s %.3f global %.3f pos_err %.3f" %
                     (str(vid), s, global_s_stop, pos_err))
                last_log = now

            robot.delay(DT_MS)

    except Exception as e:
        logw("ERROR %s" % str(e))
        try:
            robot.set_vel(0, 0)
            robot.set_led(255, 0, 0)
        except:
            pass
        raise

    finally:
        ftime = robot.get_clock()
        if last_pose:
            lx, ly = last_pose
        else:
            lx = ly = float('nan')
        try:
            robot.set_vel(0, 0)
        except:
            pass
        logw("Robot %s finished at %.3f %.3f time %.1f" %
             (str(vid), lx, ly, ftime - start_time))
        log_main.close()
