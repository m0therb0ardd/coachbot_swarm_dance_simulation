
# -*- coding: utf-8 -*-
# GLIDE CONTINUOUS (rigid ring + medium wave) UNTIL WALL
# GLOBAL STOP:
# - If ANY bot hits soft boundary: that bot turns YELLOW, broadcasts STOP_NOW several times,
#   then freezes.
# - If ANY bot hits critical boundary: turns RED, broadcasts STOP_NOW several times, then freezes.
# - All bots that receive STOP_NOW turn WHITE and freeze immediately.

from __future__ import division
import math
import struct
import os

# ----------------- Arena -----------------
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

# ----------------- Wave motion -----------------
UPWARD_SPEED = 0.035   # m/s upward drift
WAVE_AMP     = 0.13    # base lateral amplitude
WAVE_FREQ    = 0.05    # Hz

# Option 1: BIG lateral swing of the COM
BIG_WAVE_MULT = 2.0    # try 3.0, can raise to 4.0 5.0 if you want even more

DT_MS     = 40
MAX_WHEEL = 35
TURN_K    = 3.0
K_P_POS   = 1.8        # target correction gain

# ----------------- Boundary logic -----------------
SOFT_MARGIN = 0.08
CRIT_MARGIN = 0.02
SOFT_MAX_F  = 0.35

# ----------------- COM P2P (for ring center) -----------------
HB_FMT   = 'ffi'   # x, y, id
HB_BYTES = struct.calcsize(HB_FMT)
HB_DT    = 0.15
STALE_S  = 1.0

# ----------------- STOP message (structured) -----------------
STOP_NOW   = 99
STOP_FMT   = 'i'        # 4-byte int
STOP_BYTES = struct.calcsize(STOP_FMT)

# ----------------- Logging -----------------
LOG     = None
LOG_OUT = None

def init_log():
    global LOG, LOG_OUT
    try:
        LOG = open("experiment_log.txt", "a", 1)
    except:
        LOG = None
    try:
        LOG_OUT = open("/home/pi/experiment_output", "a", 1)
    except:
        LOG_OUT = None

def logw(msg):
    if not isinstance(msg, str):
        msg = str(msg)
    if not msg.endswith("\n"):
        msg += "\n"
    try:
        if LOG:
            LOG.write(msg)
            LOG.flush()
    except:
        pass
    try:
        if LOG_OUT:
            LOG_OUT.write(msg)
            LOG_OUT.flush()
    except:
        pass

# ----------------- Helpers -----------------
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

def boundary_state(x, y):
    if (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
        y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN):
        return 2
    if (x < X_MIN + SOFT_MARGIN or x > X_MAX - SOFT_MARGIN or
        y < Y_MIN + SOFT_MARGIN or y > Y_MAX - SOFT_MARGIN):
        return 1
    return 0

def soft_boundary_force(x, y):
    fx = fy = 0.0
    if x < X_MIN + SOFT_MARGIN:
        fx += SOFT_MAX_F * (1.0 - (x - X_MIN) / SOFT_MARGIN)
    elif x > X_MAX - SOFT_MARGIN:
        fx -= SOFT_MAX_F * (1.0 - (X_MAX - x) / SOFT_MARGIN)
    if y < Y_MIN + SOFT_MARGIN:
        fy += SOFT_MAX_F * (1.0 - (y - Y_MIN) / SOFT_MARGIN)
    elif y > Y_MAX - SOFT_MARGIN:
        fy -= SOFT_MAX_F * (1.0 - (Y_MAX - y) / SOFT_MARGIN)
    return fx, fy

def broadcast_stop(robot, vid, tag):
    """Send STOP_NOW several times to make sure it is heard."""
    pkt = struct.pack(STOP_FMT, STOP_NOW)
    for _ in range(4):
        try:
            robot.send_msg(pkt)
        except:
            pass
        robot.delay(30)
    logw("%s: vid=%d BROADCAST STOP_NOW" % (tag, vid))

def check_for_stop(msgs, robot, vid, phase_tag):
    """Check incoming messages for STOP_NOW. If found, freeze and return True."""
    if not msgs:
        return False
    for m in msgs:
        try:
            if len(m) >= STOP_BYTES:
                code = struct.unpack(STOP_FMT, m[:STOP_BYTES])[0]
                if code == STOP_NOW:
                    robot.set_led(255, 255, 255)
                    robot.set_vel(0, 0)
                    logw("%s: vid=%d RECEIVED STOP_NOW" % (phase_tag, vid))
                    return True
        except:
            # ignore garbage / malformed
            pass
    return False

# ----------------- MAIN -----------------
def usr(robot):
    init_log()

    try:
        vid = int(robot.id)
    except:
        vid = 0

    logw("glide_until_wall_global: START vid=%d" % vid)

    neighbors = {}
    last_hb = -1e9

    lastL = lastR = 0

    # wake localization
    robot.set_vel(20, 20)
    robot.delay(150)
    robot.set_vel(0, 0)
    robot.delay(150)

    # ---- Phase 1: COM estimation ----
    t0 = robot.get_clock()
    COM_PHASE_T = 3.0
    C0x = C0y = None
    rel_off = None

    try:
        while True:
            # Check for STOP messages first
            msgs = robot.recv_msg() or []
            if check_for_stop(msgs, robot, vid, "COM_PHASE"):
                return

            pose = safe_pose(robot)
            if not pose:
                robot.set_vel(0, 0)
                robot.delay(DT_MS)
                continue

            x, y, th = pose
            now = robot.get_clock()

            b = boundary_state(x, y)
            if b == 2:
                # Critical boundary during COM -> broadcast STOP and freeze
                robot.set_led(255, 0, 0)
                broadcast_stop(robot, vid, "COM_CRIT")
                robot.set_vel(0, 0)
                return
            elif b == 1:
                robot.set_led(255, 200, 0)
                # Even hitting soft margin during COM is enough to stop everything
                broadcast_stop(robot, vid, "COM_SOFT")
                robot.set_vel(0, 0)
                return
            else:
                robot.set_led(0, 0, 200)

            # COM heartbeats
            if now - last_hb >= HB_DT:
                try:
                    robot.send_msg(struct.pack(HB_FMT, x, y, vid))
                except:
                    pass
                last_hb = now

            # collect neighbor positions
            if msgs:
                for m in msgs:
                    if len(m) >= HB_BYTES:
                        try:
                            nx, ny, nid = struct.unpack(HB_FMT, m[:HB_BYTES])
                            nid = int(nid)
                            if nid != vid:
                                neighbors[nid] = (nx, ny, now)
                        except:
                            pass

            cut = now - STALE_S
            for nid in list(neighbors.keys()):
                if neighbors[nid][2] < cut:
                    neighbors.pop(nid, None)

            # Compute COM
            xs = [x]
            ys = [y]
            for (nx, ny, tlast) in neighbors.values():
                xs.append(nx)
                ys.append(ny)
            COMx = sum(xs) / float(len(xs))
            COMy = sum(ys) / float(len(ys))

            if now - t0 >= COM_PHASE_T:
                C0x = COMx
                C0y = COMy
                rel_off = (x - C0x, y - C0y)
                break

            robot.set_vel(0, 0)
            robot.delay(DT_MS)

    except Exception as e:
        logw("glide_until_wall_global: ERROR in COM vid=%d %s" % (vid, repr(e)))
        try:
            robot.set_led(255, 0, 0)
            robot.set_vel(0, 0)
        except:
            pass
        raise

    if rel_off is None:
        robot.set_led(255, 0, 0)
        logw("glide_until_wall_global: vid=%d missing rel_off, abort" % vid)
        return

    # ---- Phase 2: Continuous GLIDE until STOP or wall ----
    t_wave0 = robot.get_clock()

    try:
        while True:
            # Check for STOP messages first
            msgs = robot.recv_msg() or []
            if check_for_stop(msgs, robot, vid, "GLIDE_PHASE"):
                return

            pose = safe_pose(robot)
            if not pose:
                robot.set_vel(0, 0)
                robot.delay(DT_MS)
                continue

            x, y, th = pose
            now = robot.get_clock()

            b = boundary_state(x, y)

            if b == 2:
                # Critical boundary: red, broadcast STOP, freeze
                robot.set_led(255, 0, 0)
                broadcast_stop(robot, vid, "GLIDE_CRIT")
                robot.set_vel(0, 0)
                return
            elif b == 1:
                # Soft boundary: yellow, broadcast STOP, freeze
                robot.set_led(255, 200, 0)
                broadcast_stop(robot, vid, "GLIDE_SOFT")
                robot.set_vel(0, 0)
                return
            else:
                robot.set_led(0, 0, 255)

            # (Optional) still send COM-style heartbeats if you want
            if now - last_hb >= HB_DT:
                try:
                    robot.send_msg(struct.pack(HB_FMT, x, y, vid))
                except:
                    pass
                last_hb = now

            # Moving wave center with BIG lateral swing
            t_wave = now - t_wave0

            # Option 1: amplify COM lateral oscillation
            Cx = C0x + (WAVE_AMP * BIG_WAVE_MULT) * math.sin(2.0 * math.pi * WAVE_FREQ * t_wave)
            Cy = C0y + UPWARD_SPEED * t_wave

            # This bot's rigid target
            tx = Cx + rel_off[0]
            ty = Cy + rel_off[1]

            bfx, bfy = soft_boundary_force(x, y)
            ex = tx - x
            ey = ty - y

            vx = K_P_POS * ex + bfx
            vy = K_P_POS * ey + bfy

            if abs(vx) < 1e-6 and abs(vy) < 1e-6:
                vx = 1e-3

            hdg = math.atan2(vy, vx)
            err = wrap_angle(hdg - th)

            fwd = 0.60
            turn = clamp(TURN_K * err, -1.5, 1.5)

            lcmd = clamp(int(MAX_WHEEL * (fwd - 0.8 * turn)),
                         -MAX_WHEEL, MAX_WHEEL)
            rcmd = clamp(int(MAX_WHEEL * (fwd + 0.8 * turn)),
                         -MAX_WHEEL, MAX_WHEEL)

            # simple smoothing
            slew = 10
            if lcmd > lastL + slew:
                lcmd = lastL + slew
            if lcmd < lastL - slew:
                lcmd = lastL - slew
            if rcmd > lastR + slew:
                rcmd = lastR + slew
            if rcmd < lastR - slew:
                rcmd = lastR - slew

            left  = int(0.82 * lcmd + 0.18 * lastL)
            right = int(0.82 * rcmd + 0.18 * lastR)
            lastL, lastR = left, right

            robot.set_vel(left, right)
            robot.delay(DT_MS)

    except Exception as e:
        logw("glide_until_wall_global: ERROR in GLIDE vid=%d %s" % (vid, repr(e)))
        try:
            robot.set_led(255, 0, 0)
            robot.set_vel(0, 0)
        except:
            pass
        raise
    finally:
        try:
            robot.set_vel(0, 0)
        except:
            pass
        logw("glide_until_wall_global: FINISH vid=%d" % vid)