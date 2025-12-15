

from __future__ import division
import math, struct, random

X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

MAX_WHEEL = 35
TURN_K    = 3.0
FWD_FAST  = 0.95
FWD_SLOW  = 0.10
FWD_MIN   = 0.45
CMD_SMOOTH= 0.18
LOOP_DT_MS= 40

WAVE_PERIOD    = 4.0
DASH_TIME      = 1.2
AIM_TIME       = 0.35

PUNCH_HEADING  = -0.5 * math.pi
ANGLE_JITTER   = 0.22
COOLDOWN       = 5.0

SOFT_MARGIN     = 0.08
CRIT_MARGIN     = 0.0
SOFT_MAX_FORCE  = 0.35

TYPE_PULSE = 2
STOP_NOW   = 99                   
P_FMT = 'iiii'
P_BYTES = struct.calcsize(P_FMT)

def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def wrap_angle(a):
    while a >  math.pi:  a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a

def safe_pose(robot):
    p = robot.get_pose()
    if p and len(p) >= 3:
        return float(p[0]), float(p[1]), float(p[2])
    return None

def soft_boundary_check(x, y):
    if (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
        y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN):
        return 2
    if (x < X_MIN + SOFT_MARGIN or x > X_MAX - SOFT_MARGIN or
        y < Y_MIN + SOFT_MARGIN or y > Y_MAX - SOFT_MARGIN):
        return 1
    return 0

def usr(robot):
    robot.delay(400)

    try:
        vid = int(robot.id)
    except:
        vid = -1

    random.seed((vid if vid is not None else 0)*1103515245 & 0xFFFFFFFF)
    print('Robot %d starting PUNCH (down)' % vid)

    my_vid = vid
    lowest_seen = my_vid
    last_pulse_emit = -1e9
    seq = 0
    seen_seqs = set()

    dash_active = False
    dash_until  = 0.0
    aim_until   = 0.0
    aim_heading = PUNCH_HEADING
    cool_until  = 0.0

    last_left = 0
    last_right = 0

    global_stop = False           
    sentinel_id = None            

    robot.set_vel(20,20); robot.delay(120)

    try:
        while True:

            if global_stop:       
                robot.set_led(255,255,255)  
                robot.set_vel(0,0)
                return

            pose = safe_pose(robot)
            if pose is None:
                robot.set_vel(0,0); robot.delay(LOOP_DT_MS); continue
            x,y,th = pose
            now = robot.get_clock()

            msgs = robot.recv_msg()
            if msgs:
                for m in msgs:
                    try:
                        tpe, lvid, s, _ = struct.unpack(P_FMT, m[:P_BYTES])

                        if tpe == STOP_NOW:
                            global_stop = True
                            continue

                        if tpe == TYPE_PULSE:
                            lowest_seen = min(lowest_seen, int(lvid))
                            if s not in seen_seqs:
                                seen_seqs.add(s)
                                robot.send_msg(struct.pack(P_FMT, TYPE_PULSE, int(lvid), int(s), 0))
                                robot.delay(40)
                                if now >= cool_until:
                                    aim_heading = PUNCH_HEADING + random.uniform(-ANGLE_JITTER,ANGLE_JITTER)
                                    aim_until = now + AIM_TIME
                                    dash_until = aim_until + DASH_TIME
                                    dash_active = True
                                    cool_until = dash_until + COOLDOWN
                                    robot.set_led(0,100,47)
                    except:
                        pass

            bstat = soft_boundary_check(x,y)

            if bstat == 2:  
                if not global_stop:
                    sentinel_id = vid
                    robot.set_led(255,0,0)
                    robot.send_msg(struct.pack(P_FMT, STOP_NOW, vid, 0, 0))  
                global_stop = True
                continue

            elif bstat == 1:  
                if not global_stop:
                    # sentinel_id = vid
                    robot.set_led(255,200,0)   
                #     robot.send_msg(struct.pack(P_FMT, STOP_NOW, vid, 0, 0))
                # global_stop = True
                # # continue

            else:
                robot.set_led(0,70,70)

            if my_vid == lowest_seen and (now - last_pulse_emit) >= WAVE_PERIOD:
                seq += 1
                robot.send_msg(struct.pack(P_FMT, TYPE_PULSE, my_vid, seq, 0))
                last_pulse_emit = now
                seen_seqs.add(seq)
                robot.set_led(0,100,47)
                if now >= cool_until:
                    aim_heading = PUNCH_HEADING + random.uniform(-ANGLE_JITTER,ANGLE_JITTER)
                    aim_until = now + AIM_TIME
                    dash_until = aim_until + DASH_TIME
                    dash_active = True
                    cool_until = dash_until + COOLDOWN

            vx = 0.0
            vy = -0.05   

            if dash_active:

                if now < aim_until:
                    robot.set_led(0,70,70)

                    target = aim_heading
                    err = wrap_angle(target - th)
                    fwd = FWD_SLOW * 0.7
                    if not global_stop and bstat==0:
                        robot.set_led(0, 0, 255)
                    if abs(err) > 1.3:  
                        fwd = max(fwd, 0.22)

                else:
                    if not global_stop and bstat==0:
                        robot.set_led(255,0,0)
                    if now >= dash_until:
                        dash_active = False
                        robot.set_led(0,70,70)
                        fwd = FWD_SLOW
                        err = wrap_angle(aim_heading - th)
                    else:
                        target = aim_heading
                        err = wrap_angle(target - th)
                        fwd = FWD_FAST
                        if abs(err) > 1.3:
                            fwd = max(fwd, 0.22)

                turn = clamp(TURN_K * err, -1.5, 1.5)
                lcmd = clamp(int(MAX_WHEEL*(fwd - 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
                rcmd = clamp(int(MAX_WHEEL*(fwd + 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)

            else:
                err = wrap_angle(math.atan2(vy, vx) - th)

                if abs(err) < 0.5:   fwd = FWD_SLOW
                elif abs(err) < 1.2: fwd = FWD_SLOW * 0.8
                else:                fwd = FWD_SLOW * 0.6
                if fwd < FWD_MIN: fwd = FWD_MIN
                if abs(err) > 1.3:
                    fwd = max(fwd, 0.22)

                turn = clamp(TURN_K * err, -1.5, 1.5)
                lcmd = clamp(int(MAX_WHEEL*(fwd - 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
                rcmd = clamp(int(MAX_WHEEL*(fwd + 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)

            left  = int((1.0 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * last_left)
            right = int((1.0 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * last_right)
            last_left = left
            last_right = right
            robot.set_vel(left,right)

            robot.delay(LOOP_DT_MS)

    except Exception as e:
        robot.set_led(255,0,0)
        robot.set_vel(0,0)
        print('[punch] ERROR id=%d: %s' % (vid,repr(e)))
        raise

    finally:
        robot.set_vel(0,0)

