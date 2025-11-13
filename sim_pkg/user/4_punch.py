import math, struct, random

# --- field & obstacle (meters) ---
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35
FEET = 0.3048
OBST_DIAM_FT = 1.0
OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
OBST_MARGIN  = 0.03
SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
OBST_CX, OBST_CY = (-0.1, 0.475)

# --- control/loop ---
MAX_WHEEL   = 35
TURN_K      = 3.4
DT_MS       = 40
CMD_SMOOTH  = 0.08        # snappy
VEL_SLEW    = 24          # allow big jumps

# --- headings / feel ---
RIGHT_HEADING = -math.pi/2 #this is down in world coords
FWD_CHARGE    = 1.00      # full send
FWD_FEINT     = 0.35
FWD_RECOIL    = 0.55
FWD_DRIFT     = 0.38

# --- timing (seconds) ---
WAVE_PERIOD = 3.0         # pulse interval (leader)
AIM_TIME    = 0.18
FEINT_TIME  = 0.12        # quick left jab if blocked
CHARGE_TIME = 0.55
IMPACT_HOLD = 0.10        # heavy stop on impact
DOUBLE_TAP  = 0.08        # second mini-burst after hold
RECOIL_TIME = 0.30
COOLDOWN    = 0.75

# --- safety / interaction ---
SOFT_MARGIN = 0.08
CRIT_MARGIN = 0.02
SOFT_MAX_F  = 0.55
PANIC_R     = 0.18
SEP_R       = 0.28
NEIGH_R     = 0.75
K_SEP       = 0.48
K_ALI       = 0.10

# --- P2P pulse (sync the hit) ---
TYPE_PULSE  = 2
P_FMT       = 'iiii'        # (type, leader_vid, seq, 0)
P_BYTES     = struct.calcsize(P_FMT)

# --- Heartbeats (for spacing & feint checks) ---
HB_FMT   = 'fffffi'         # x,y,th,vx,vy,id
HB_BYTES = struct.calcsize(HB_FMT)
HB_DT    = 0.12
STALE_S  = 0.7

def clamp(v, lo, hi): return lo if v < lo else (hi if v > hi else v)
def wrap(a):
    while a >  math.pi: a -= 2*math.pi
    while a <= -math.pi: a += 2*math.pi
    return a

def soft_boundary_force(x,y):
    fx=fy=0.0
    if x < X_MIN+SOFT_MARGIN: fx += SOFT_MAX_F*(1-(x-X_MIN)/SOFT_MARGIN)
    elif x > X_MAX-SOFT_MARGIN: fx -= SOFT_MAX_F*(1-(X_MAX-x)/SOFT_MARGIN)
    if y < Y_MIN+SOFT_MARGIN: fy += SOFT_MAX_F*(1-(y-Y_MIN)/SOFT_MARGIN)
    elif y > Y_MAX-SOFT_MARGIN: fy -= SOFT_MAX_F*(1-(Y_MAX-y)/SOFT_MARGIN)
    return fx,fy

def soft_obstacle_force(x, y, maxf=0.85, w=0.12):
    dx,dy = x-OBST_CX, y-OBST_CY; r=math.hypot(dx,dy)
    if r < SAFE_BUBBLE + w:
        if r<1e-6: return maxf,0.0
        s = max(0.0, (SAFE_BUBBLE+w-r)/w)*maxf
        return s*(dx/r), s*(dy/r)
    return 0.0,0.0

def boundary_state(x,y):
    if (x < X_MIN+CRIT_MARGIN or x > X_MAX-CRIT_MARGIN or
        y < Y_MIN+CRIT_MARGIN or y > Y_MAX-CRIT_MARGIN): return 2
    if (x < X_MIN+SOFT_MARGIN or x > X_MAX-SOFT_MARGIN or
        y < Y_MIN+SOFT_MARGIN or y > Y_MAX-SOFT_MARGIN): return 1
    return 0

def safe_pose(robot):
    p = robot.get_pose()
    if p and len(p)>=3: return float(p[0]),float(p[1]),float(p[2])
    return None

def heading_to_wheels(err, fwd, lastL, lastR):
    turn = clamp(TURN_K*err, -2.0, 2.0)
    lcmd = clamp(int(MAX_WHEEL*0.95*(fwd - 0.9*turn)), -MAX_WHEEL, MAX_WHEEL)
    rcmd = clamp(int(MAX_WHEEL*0.95*(fwd + 0.9*turn)), -MAX_WHEEL, MAX_WHEEL)
    # slew + snappy smoothing
    if lcmd > lastL + VEL_SLEW: lcmd = lastL + VEL_SLEW
    if lcmd < lastL - VEL_SLEW: lcmd = lastL - VEL_SLEW
    if rcmd > lastR + VEL_SLEW: rcmd = lastR + VEL_SLEW
    if rcmd < lastR - VEL_SLEW: rcmd = lastR - VEL_SLEW
    left  = int((1-CMD_SMOOTH)*lcmd + CMD_SMOOTH*lastL)
    right = int((1-CMD_SMOOTH)*rcmd + CMD_SMOOTH*lastR)
    return left, right

# ---------------- main entry ----------------
def usr(robot):
    robot.delay(400)
    try: vid = int(robot.id())
    except: vid = 0
    random.seed(vid*2654435761 & 0xFFFFFFFF)

    # state
    neighbors, last_seen = {}, {}
    last_hb = -1e9

    # pulse sync
    my_vid = vid
    lowest_seen = my_vid
    last_pulse_emit = -1e9
    seq = 0
    seen_seqs = set()

    # choreography state machine
    state = "reset"
    aim_until = feint_until = charge_until = hold_until = tap_until = recoil_until = 0.0
    target_h = RIGHT_HEADING
    next_ready = 0.0

    # wheel memory
    lastL = lastR = 0

    # wake localization
    robot.set_vel(20,20); robot.delay(120)

    while True:
        pose = safe_pose(robot)
        if not pose:
            robot.set_vel(0,0); robot.delay(DT_MS); continue
        x,y,th = pose
        now = robot.get_clock()

        # LEDs
        b = boundary_state(x,y)
        if b==2:
            robot.set_led(100,0,0); robot.set_vel(0,0); robot.delay(DT_MS); continue
        elif b==1: robot.set_led(100,60,0)
        else:
            if   state=="aim":    robot.set_led(100,100,0)  # tense yellow
            elif state=="feint":  robot.set_led(60,60,100)  # steel blue
            elif state=="charge": robot.set_led(100,0,70)   # magenta hit
            elif state=="hold":   robot.set_led(80,0,0)     # heavy stop
            elif state=="tap":    robot.set_led(100,0,100)  # hot pink mini-hit
            elif state=="recoil": robot.set_led(0,100,70)   # mint recoil
            else:                 robot.set_led(0,60,60)    # teal idle

        # ---- heartbeats (for spacing) ----
        if now - last_hb >= HB_DT:
            x1,y1,_ = pose; t1 = now
            robot.delay(50)
            p2 = safe_pose(robot)
            if p2:
                x2,y2,th2 = p2; t2 = robot.get_clock()
                dt = max(1e-3, t2-t1)
                vx=(x2-x1)/dt; vy=(y2-y1)/dt
                try: robot.send_msg(struct.pack(HB_FMT, x2,y2,th2,vx,vy,vid))
                except: pass
                last_hb=t2; x,y,th = x2,y2,th2
            else:
                last_hb = now

        for m in (robot.recv_msg() or []):
            # pulses AND heartbeats can be interleaved; try pulse first (tiny)
            try:
                tpe, lvid, s, sent = struct.unpack(P_FMT, m[:P_BYTES])
                if tpe == TYPE_PULSE:
                    lowest_seen = min(lowest_seen, int(lvid))
                    if s not in seen_seqs:
                        seen_seqs.add(s)
                        # relay
                        robot.send_msg(struct.pack(P_FMT, TYPE_PULSE, int(lvid), int(s), 0))
                        # schedule local attack window with tiny offset
                        if now >= next_ready:
                            jitter = 0.02 * (vid % 5)   # subtle stagger
                            state = "aim"
                            aim_until   = now + AIM_TIME + jitter
                            # decide if we need a feint (someone in our lane?)
                            feint_needed = False
                            ahead_r = 0.30
                            for _,(nx,ny,nth,_,_) in neighbors.items():
                                # project neighbor into my forward-right lane
                                relx = (nx - x); rely = (ny - y)
                                # within a narrow rightward corridor?
                                if relx > -0.05 and abs(rely) < 0.18 and math.hypot(relx,rely) < ahead_r:
                                    feint_needed = True; break
                            if feint_needed:
                                state = "feint"
                                feint_until = aim_until + FEINT_TIME
                                # feint a tiny LEFT tap before charging right
                                target_h = wrap(RIGHT_HEADING + math.radians(180))  # left blip
                            else:
                                target_h = RIGHT_HEADING
            except Exception:
                # not a pulse; try heartbeat
                try:
                    nx,ny,nth,nvx,nvy,nid = struct.unpack(HB_FMT, m[:HB_BYTES])
                    if int(nid)!=vid:
                        neighbors[int(nid)]=(nx,ny,nth,nvx,nvy)
                        last_seen[int(nid)]=now
                except: pass

        # prune neighbors
        cut = now - STALE_S
        for nid in list(neighbors.keys()):
            if last_seen.get(nid,0)<cut:
                neighbors.pop(nid,None); last_seen.pop(nid,None)

        # leader emits pulses
        if my_vid == lowest_seen and (now - last_pulse_emit) >= WAVE_PERIOD:
            seq += 1
            robot.send_msg(struct.pack(P_FMT, TYPE_PULSE, my_vid, seq, 0))
            seen_seqs.add(seq)
            last_pulse_emit = now

        # safety fields (used in charge/idle)
        ex,ey = soft_boundary_force(x,y)
        ox,oy = soft_obstacle_force(x,y)

        # strong separation
        repx=repy=0.0; ax=ay=0.0; n=0; nearest=1e9; nn=(None,None)
        for _,(nx,ny,nth,_,_) in neighbors.items():
            dx,dy = x-nx, y-ny
            d = math.hypot(dx,dy)
            if d < nearest: nearest, nn = d, (nx,ny)
            if d>1e-6 and d<SEP_R:
                s = K_SEP * (SEP_R - d)/SEP_R
                repx += s*(dx/d); repy += s*(dy/d)
            if d<=NEIGH_R:
                ax += math.cos(nth); ay += math.sin(nth); n+=1
        alx=aly=0.0
        if n>0:
            ah = math.atan2(ay,ax)
            alx = K_ALI*math.cos(ah); aly = K_ALI*math.sin(ah)

        # emergency evade (rare; during charge we still respect this)
        evade_h = None
        if nearest < PANIC_R and nn[0] is not None:
            nx,ny = nn
            evade_h = math.atan2(y-ny, x-nx)

        # --- state machine ---
        if state == "reset":
            # idle until a pulse arrives
            err = 0.0; fwd = 0.0
        elif state == "aim":
            if now >= aim_until:
                state = "charge"; charge_until = now + CHARGE_TIME
                target_h = RIGHT_HEADING
            err = wrap(target_h - th); fwd = FWD_FEINT
        elif state == "feint":
            if now >= feint_until:
                state = "charge"; charge_until = now + CHARGE_TIME
                target_h = RIGHT_HEADING
            err = wrap(target_h - th); fwd = FWD_FEINT
        elif state == "charge":
            if now >= charge_until:
                state = "hold"; hold_until = now + IMPACT_HOLD
                robot.set_vel(0,0)  # slam stop
                robot.delay(DT_MS); lastL=lastR=0
                continue
            base_vx = math.cos(RIGHT_HEADING)
            base_vy = math.sin(RIGHT_HEADING)

            vx = base_vx + repx + 0.4*(ex+ox) + 0.2*alx
            vy = base_vy + repy + 0.4*(ey+oy) + 0.2*aly

            if evade_h is not None:
                vx += 0.2*math.cos(evade_h); vy += 0.2*math.sin(evade_h)

            hdg = math.atan2(vy, vx)
            err = wrap(hdg - th); fwd = FWD_CHARGE
            
        elif state == "hold":
            if now >= hold_until:
                state = "tap"; tap_until = now + DOUBLE_TAP
            robot.set_vel(0,0); robot.delay(DT_MS); lastL=lastR=0; continue
        elif state == "tap":
            if now >= tap_until:
                state = "recoil"; recoil_until = now + RECOIL_TIME
            # quick mini-burst in RIGHT_HEADING
            err = wrap(RIGHT_HEADING - th); fwd = 0.70
        elif state == "recoil":
            if now >= recoil_until:
                state = "reset"; next_ready = now + COOLDOWN
            # *** CHANGED: recoil opposite-ish to attack (mostly upward) ***
            rc = RIGHT_HEADING + math.pi - 0.35   # flip attack dir, slight angle
            vx = math.cos(rc) + ex + ox
            vy = math.sin(rc) + ey + oy
            hdg = math.atan2(vy, vx)
            err = wrap(hdg - th); fwd = FWD_RECOIL
        else:
            err = 0.0; fwd = FWD_DRIFT

        # dial down near boundary except for the stop/hold
        if b==1 and state not in ("hold","tap"): fwd *= 0.85

        left,right = heading_to_wheels(err, fwd, lastL, lastR)
        lastL,lastR = left,right
        robot.set_vel(left,right)

        # heartbeat of the loop
        robot.delay(DT_MS)
        # light print every ~2s
        # (sim engine often ignores too-frequent prints)
        # (optional) — left as minimal to avoid spam
