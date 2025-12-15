
from __future__ import division
import math
import struct
import os

# ----------------- Arena -----------------
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

# ----------------- Motion parameters -----------------

MOVE_DIR      = 1          
MOVE_DIST     = 0.35        
MOVE_DURATION = 8.0       

# Lockin correction phase
CORR_MAX_TIME = 6.0         # max seconds for final correction
CORR_POS_TOL  = 0.015       # 1.5 cm position tolerance
CORR_VEL_TOL  = 0.01        # m/s-ish velocity tolerance

# ----------------- Control -----------------
DT_MS      = 40
MAX_WHEEL  = 35
TURN_K     = 3.0

FWD_TRANSLATE = 0.65   # speed during main translation
FWD_CORR_MAX  = 0.45   # max speed during correction
FWD_CORR_MIN  = 0.10   # min forward to avoid stall

# simple gains for vector control
K_P_POS  = 1.8   # position  velocity gain (translation)
K_P_CORR = 2.4   # tighter during correction

# ----------------- Soft walls -----------------
SOFT_MARGIN = 0.08
CRIT_MARGIN = 0.02
SOFT_MAX_F  = 0.35

# ----------------- COM P2P -----------------
# Just share (x, y, id) for COM estimate.
HB_FMT   = 'ffi'
HB_BYTES = struct.calcsize(HB_FMT)
HB_DT    = 0.15
STALE_S  = 1.0

# ----------------- Logging -----------------
LOG = None
LOG_OUT = None

def init_log():
    global LOG, LOG_OUT
    if LOG is None:
        try:
            LOG = open("experiment_log.txt", "a", 1)  # line-buffered
        except:
            LOG = None
    if LOG_OUT is None:
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
    if v < lo: return lo
    if v > hi: return hi
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


# ----------------- Main -----------------
def usr(robot):
    init_log()

    try:
        vid = int(robot.id)
    except:
        vid = 0

    logw("rigid_sideways: START vid=%d MOVE_DIR=%d MOVE_DIST=%.3f" %
         (vid, MOVE_DIR, MOVE_DIST))

    neighbors = {}
    last_hb   = -1e9

    lastL = lastR = 0

    # For correction velocity estimate
    last_pose_for_vel = None  # (x, y, t)

    # --- wake localization ---
    robot.set_vel(20, 20)
    robot.delay(150)
    robot.set_vel(0, 0)
    robot.delay(150)

    # ------------- State machine -------------
    STATE_COM       = 0  # estimate C0 (formation center)
    STATE_TRANSLATE = 1  # move center from C0 to C_final
    STATE_CORRECT   = 2  # lock-in to final positions

    state = STATE_COM

    t_start      = robot.get_clock()
    COM_PHASE_T  = 3.0   # seconds to collect COM

    # Shared center for this script run:
    C0x = C0y = None
    Cfx = Cfy = None     # final center after translation

    rel_off = None       # (dx, dy)

    # Translation timing
    t_translate_start = None

    # Correction timing
    t_corr_start = None

    start_time = t_start
    max_runtime = 60.0   # global hard cap

    try:
        while (robot.get_clock() - start_time) < max_runtime:
            pose = safe_pose(robot)
            if not pose:
                robot.set_vel(0, 0)
                robot.delay(DT_MS)
                continue

            x, y, th = pose
            now = robot.get_clock()

            # ---------------- Boundary LEDs + safety ----------------
            b = boundary_state(x, y)
            if b == 2:
                robot.set_led(255, 0, 0)
                robot.set_vel(0, 0)
                logw("rigid_sideways: vid=%d CRITICAL boundary at (%.3f, %.3f)" %
                     (vid, x, y))
                break
            elif b == 1:
                # amber near walls
                robot.set_led(255, 150, 0)
            else:
                # color depends on state (set below)
                pass

            # ---------------- Heartbeats for COM ----------------
            if now - last_hb >= HB_DT:
                try:
                    msg = struct.pack(HB_FMT, float(x), float(y), int(vid))
                    robot.send_msg(msg)
                except:
                    pass
                last_hb = now

            msgs = robot.recv_msg() or []
            for m in msgs:
                try:
                    if len(m) >= HB_BYTES:
                        nx, ny, nid = struct.unpack(HB_FMT, m[:HB_BYTES])
                        nid = int(nid)
                        if nid != vid:
                            neighbors[nid] = (float(nx), float(ny), now)
                except:
                    pass

            # prune stale neighbors
            cutoff = now - STALE_S
            for nid in list(neighbors.keys()):
                if neighbors[nid][2] < cutoff:
                    neighbors.pop(nid, None)

            # ---------------- State: COM estimation ----------------
            if state == STATE_COM:
                # teal-ish during COM phase
                if b == 0:
                    robot.set_led(0, 180, 180)

                # compute COM of self + neighbors
                xs = [x]
                ys = [y]
                for (nx, ny, t_last) in neighbors.values():
                    xs.append(nx)
                    ys.append(ny)
                COMx = sum(xs) / len(xs)
                COMy = sum(ys) / len(ys)

                # after COM_PHASE_T seconds, freeze C0 and rel_off
                if now - t_start >= COM_PHASE_T:
                    C0x = COMx
                    C0y = COMy
                    rel_off = (x - C0x, y - C0y)

                    # compute final center along X
                    # we trust user to pick MOVE_DIST small enough to stay in bounds.
                    # Cfx = C0x + MOVE_DIR * MOVE_DIST
                    # Cfy = C0y
                    # compute final center along Y instead of X
                    Cfx = C0x
                    Cfy = C0y + MOVE_DIR * MOVE_DIST

                    # (optional) clip final center to arena with a small margin
                    margin = 0.05
                    Cfx = clamp(Cfx, X_MIN + margin, X_MAX - margin)
                    Cfy = clamp(Cfy, Y_MIN + margin, Y_MAX - margin)

                    t_translate_start = now
                    state = STATE_TRANSLATE

                    logw("rigid_sideways: vid=%d C0=(%.3f,%.3f) rel_off=(%.3f,%.3f) Cfinal=(%.3f,%.3f)" %
                         (vid, C0x, C0y, rel_off[0], rel_off[1], Cfx, Cfy))

                robot.set_vel(0, 0)
                robot.delay(DT_MS)
                continue

            # --------------- After COM we must have C0 and rel_off ---------------
            if C0x is None or rel_off is None:
                # defensive: shouldn't happen, but just stop
                robot.set_vel(0, 0)
                robot.set_led(255, 0, 0)
                logw("rigid_sideways: vid=%d missing C0 / rel_off, aborting" % vid)
                break

            # --------------- State: main translation ---------------
            if state == STATE_TRANSLATE:
                if b == 0:
                    # purpleish during translation
                    robot.set_led(120, 40, 180)

                # timenormalized interpolation 0..1
                t_rel = (now - t_translate_start) / max(1e-3, MOVE_DURATION)
                if t_rel > 1.0:
                    t_rel = 1.0

                Ctx = C0x + t_rel * (Cfx - C0x)
                Cty = C0y + t_rel * (Cfy - C0y)

                tx = Ctx + rel_off[0]
                ty = Cty + rel_off[1]

                # add soft wall force to keep things tidy
                bfx, bfy = soft_boundary_force(x, y)

                ex = tx - x
                ey = ty - y

                # simple Pcontrol to target plus wall push
                vx = K_P_POS * ex + bfx
                vy = K_P_POS * ey + bfy

                if abs(vx) < 1e-6 and abs(vy) < 1e-6:
                    vx = 1e-3

                hdg = math.atan2(vy, vx)
                err = wrap_angle(hdg - th)

                fwd = FWD_TRANSLATE
                if b == 1:
                    fwd *= 0.7

                turn = clamp(TURN_K * err, -1.5, 1.5)

                lcmd = clamp(int(MAX_WHEEL * (fwd - 0.8 * turn)),
                             -MAX_WHEEL, MAX_WHEEL)
                rcmd = clamp(int(MAX_WHEEL * (fwd + 0.8 * turn)),
                             -MAX_WHEEL, MAX_WHEEL)

                # simple smoothing
                slew = 10
                if lcmd > lastL + slew: lcmd = lastL + slew
                if lcmd < lastL - slew: lcmd = lastL - slew
                if rcmd > lastR + slew: rcmd = lastR + slew
                if rcmd < lastR - slew: rcmd = lastR - slew

                left  = int(0.82 * lcmd + 0.18 * lastL)
                right = int(0.82 * rcmd + 0.18 * lastR)
                lastL, lastR = left, right

                robot.set_vel(left, right)

                if t_rel >= 1.0 or (now - t_translate_start) > (MOVE_DURATION + 3.0):
                    state = STATE_CORRECT
                    t_corr_start = now
                    last_pose_for_vel = (x, y, now)
                    logw("rigid_sideways: vid=%d entering CORRECTION phase" % vid)

                robot.delay(DT_MS)
                continue

            # --------------- State: lock-in correction ---------------
            if state == STATE_CORRECT:
                if b == 0:
                    # blush when locking into final position
                    robot.set_led(0, 90, 255)

                # fixed final center
                tx = Cfx + rel_off[0]
                ty = Cfy + rel_off[1]

                ex = tx - x
                ey = ty - y
                pos_err = math.hypot(ex, ey)

                # estimate approximate speed from last pose
                v_est = 0.0
                if last_pose_for_vel is not None:
                    lx, ly, lt = last_pose_for_vel
                    dtv = max(1e-3, now - lt)
                    dist = math.hypot(x - lx, y - ly)
                    v_est = dist / dtv
                last_pose_for_vel = (x, y, now)

                # stop conditions
                done_geom = (pos_err < CORR_POS_TOL and v_est < CORR_VEL_TOL)
                done_time = (now - t_corr_start) > CORR_MAX_TIME

                if done_geom or done_time:
                    robot.set_vel(0, 0)
                    robot.set_led(0, 200, 80)  
                    logw("rigid_sideways: vid=%d LOCKED pos_err=%.3f v_est=%.3f (time_done=%s)" %
                         (vid, pos_err, v_est, str(done_time)))
                    break

                # otherwise keep correcting
                bfx, bfy = soft_boundary_force(x, y)

                vx = K_P_CORR * ex + bfx
                vy = K_P_CORR * ey + bfy

                if abs(vx) < 1e-6 and abs(vy) < 1e-6:
                    vx = 1e-3

                hdg = math.atan2(vy, vx)
                err = wrap_angle(hdg - th)

                fwd = FWD_CORR_MAX * max(0.25, min(1.0, pos_err / 0.12))
                if b == 1:
                    fwd *= 0.7
                if fwd < FWD_CORR_MIN:
                    fwd = FWD_CORR_MIN

                turn = clamp(TURN_K * err, -1.5, 1.5)

                lcmd = clamp(int(MAX_WHEEL * (fwd - 0.8 * turn)),
                             -MAX_WHEEL, MAX_WHEEL)
                rcmd = clamp(int(MAX_WHEEL * (fwd + 0.8 * turn)),
                             -MAX_WHEEL, MAX_WHEEL)

                slew = 10
                if lcmd > lastL + slew: lcmd = lastL + slew
                if lcmd < lastL - slew: lcmd = lastL - slew
                if rcmd > lastR + slew: rcmd = lastR + slew
                if rcmd < lastR - slew: rcmd = lastR - slew

                left  = int(0.82 * lcmd + 0.18 * lastL)
                right = int(0.82 * rcmd + 0.18 * lastR)
                lastL, lastR = left, right

                robot.set_vel(left, right)
                robot.delay(DT_MS)
                continue

        # end main while

    except Exception as e:
        logw("rigid_sideways: ERROR vid=%d %s" % (vid, repr(e)))
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
        logw("rigid_sideways: FINISH vid=%d" % vid)
