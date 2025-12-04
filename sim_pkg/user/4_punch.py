# # import math, struct, random, os 

# # # ----------------- Logging (sim + hardware) -----------------
# # LOG = None   # global log handle

# # def init_log():
# #     """
# #     Try to open experiment_log.txt in a hardware-like way.
# #     If it fails (e.g., sim with no FS), we just fall back to logw only.
# #     """
# #     global LOG
# #     if LOG is not None:
# #         return
# #     try:
# #         # line-buffered like your FLOAT HW script
# #         LOG = open("experiment_log.txt", "a", 1)
# #     except Exception:
# #         LOG = None

# # def logw(msg):
# #     """
# #     Write to log file (if available) AND logw to stdout.
# #     Safe in both sim and hardware.
# #     """
# #     if not isinstance(msg, str):
# #         msg = str(msg)
# #     line = msg if msg.endswith("\n") else msg + "\n"

# #     # Log file (hardware) if available
# #     if LOG is not None:
# #         try:
# #             LOG.write(line)
# #             LOG.flush()
# #             os.fsync(LOG.fileno())
# #         except Exception:
# #             pass

# #     # Always also logw (sim / console)
# #     print(line.rstrip("\n"))
    

# # # --- field & obstacle (meters) ---
# # X_MIN, X_MAX = -1.2, 1.0
# # Y_MIN, Y_MAX = -1.4, 2.35
# # FEET = 0.3048
# # OBST_DIAM_FT = 1.0
# # OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
# # OBST_MARGIN  = 0.03
# # SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
# # OBST_CX, OBST_CY = (-0.1, 0.475)

# # # --- control/loop ---
# # MAX_WHEEL   = 35
# # TURN_K      = 3.4
# # DT_MS       = 40
# # CMD_SMOOTH  = 0.08        # snappy
# # VEL_SLEW    = 24          # allow big jumps

# # # --- headings / feel ---
# # RIGHT_HEADING = -math.pi/2 #this is down in world coords
# # FWD_CHARGE    = 1.00      # full send
# # FWD_FEINT     = 0.35
# # FWD_RECOIL    = 0.55
# # FWD_DRIFT     = 0.38

# # # --- timing (seconds) ---
# # WAVE_PERIOD = 3.0         # pulse interval (leader)
# # AIM_TIME    = 0.18
# # FEINT_TIME  = 0.12        # quick left jab if blocked
# # CHARGE_TIME = 0.55
# # IMPACT_HOLD = 0.10        # heavy stop on impact
# # DOUBLE_TAP  = 0.08        # second mini-burst after hold
# # RECOIL_TIME = 0.30
# # COOLDOWN    = 0.75

# # # --- safety / interaction ---
# # SOFT_MARGIN = 0.08
# # CRIT_MARGIN = 0.02
# # SOFT_MAX_F  = 0.55
# # PANIC_R     = 0.18
# # SEP_R       = 0.28
# # NEIGH_R     = 0.75
# # K_SEP       = 0.48
# # K_ALI       = 0.10

# # # --- P2P pulse (sync the hit) ---
# # TYPE_PULSE  = 2
# # P_FMT       = 'iiii'        # (type, leader_vid, seq, 0)
# # P_BYTES     = struct.calcsize(P_FMT)

# # # --- Heartbeats (for spacing & feint checks) ---
# # HB_FMT   = 'fffffi'         # x,y,th,vx,vy,id
# # HB_BYTES = struct.calcsize(HB_FMT)
# # HB_DT    = 0.12
# # STALE_S  = 0.7

# # def clamp(v, lo, hi): return lo if v < lo else (hi if v > hi else v)
# # def wrap(a):
# #     while a >  math.pi: a -= 2*math.pi
# #     while a <= -math.pi: a += 2*math.pi
# #     return a

# # def soft_boundary_force(x,y):
# #     fx=fy=0.0
# #     if x < X_MIN+SOFT_MARGIN: fx += SOFT_MAX_F*(1-(x-X_MIN)/SOFT_MARGIN)
# #     elif x > X_MAX-SOFT_MARGIN: fx -= SOFT_MAX_F*(1-(X_MAX-x)/SOFT_MARGIN)
# #     if y < Y_MIN+SOFT_MARGIN: fy += SOFT_MAX_F*(1-(y-Y_MIN)/SOFT_MARGIN)
# #     elif y > Y_MAX-SOFT_MARGIN: fy -= SOFT_MAX_F*(1-(Y_MAX-y)/SOFT_MARGIN)
# #     return fx,fy

# # def soft_obstacle_force(x, y, maxf=0.85, w=0.12):
# #     dx,dy = x-OBST_CX, y-OBST_CY; r=math.hypot(dx,dy)
# #     if r < SAFE_BUBBLE + w:
# #         if r<1e-6: return maxf,0.0
# #         s = max(0.0, (SAFE_BUBBLE+w-r)/w)*maxf
# #         return s*(dx/r), s*(dy/r)
# #     return 0.0,0.0

# # def boundary_state(x,y):
# #     if (x < X_MIN+CRIT_MARGIN or x > X_MAX-CRIT_MARGIN or
# #         y < Y_MIN+CRIT_MARGIN or y > Y_MAX-CRIT_MARGIN): return 2
# #     if (x < X_MIN+SOFT_MARGIN or x > X_MAX-SOFT_MARGIN or
# #         y < Y_MIN+SOFT_MARGIN or y > Y_MAX-SOFT_MARGIN): return 1
# #     return 0

# # def safe_pose(robot):
# #     p = robot.get_pose()
# #     if p and len(p)>=3: return float(p[0]),float(p[1]),float(p[2])
# #     return None

# # def heading_to_wheels(err, fwd, lastL, lastR):
# #     turn = clamp(TURN_K*err, -2.0, 2.0)
# #     lcmd = clamp(int(MAX_WHEEL*0.95*(fwd - 0.9*turn)), -MAX_WHEEL, MAX_WHEEL)
# #     rcmd = clamp(int(MAX_WHEEL*0.95*(fwd + 0.9*turn)), -MAX_WHEEL, MAX_WHEEL)
# #     # slew + snappy smoothing
# #     if lcmd > lastL + VEL_SLEW: lcmd = lastL + VEL_SLEW
# #     if lcmd < lastL - VEL_SLEW: lcmd = lastL - VEL_SLEW
# #     if rcmd > lastR + VEL_SLEW: rcmd = lastR + VEL_SLEW
# #     if rcmd < lastR - VEL_SLEW: rcmd = lastR - VEL_SLEW
# #     left  = int((1-CMD_SMOOTH)*lcmd + CMD_SMOOTH*lastL)
# #     right = int((1-CMD_SMOOTH)*rcmd + CMD_SMOOTH*lastR)
# #     return left, right

# # # ---------------- main entry ----------------
# # def usr(robot):
# #     init_log()
# #     robot.delay(400)
# #     try: vid = int(robot.id())
# #     except: vid = 0
# #     random.seed(vid*2654435761 & 0xFFFFFFFF)

# #     # state
# #     neighbors, last_seen = {}, {}
# #     last_hb = -1e9

# #     # pulse sync
# #     my_vid = vid
# #     lowest_seen = my_vid
# #     last_pulse_emit = -1e9
# #     seq = 0
# #     seen_seqs = set()

# #     # choreography state machine
# #     state = "reset"
# #     aim_until = feint_until = charge_until = hold_until = tap_until = recoil_until = 0.0
# #     target_h = RIGHT_HEADING
# #     next_ready = 0.0

# #     # wheel memory
# #     lastL = lastR = 0

# #     # wake localization
# #     robot.set_vel(20,20); robot.delay(120)

# #     while True:
# #         pose = safe_pose(robot)
# #         if not pose:
# #             robot.set_vel(0,0); robot.delay(DT_MS); continue
# #         x,y,th = pose
# #         now = robot.get_clock()

# #         # LEDs
# #         b = boundary_state(x,y)
# #         if b==2:
# #             robot.set_led(100,0,0); robot.set_vel(0,0); robot.delay(DT_MS); continue
# #         elif b==1: robot.set_led(100,60,0)
# #         else:
# #             if   state=="aim":    robot.set_led(100,100,0)  # tense yellow
# #             elif state=="feint":  robot.set_led(60,60,100)  # steel blue
# #             elif state=="charge": robot.set_led(100,0,70)   # magenta hit
# #             elif state=="hold":   robot.set_led(80,0,0)     # heavy stop
# #             elif state=="tap":    robot.set_led(100,0,100)  # hot pink mini-hit
# #             elif state=="recoil": robot.set_led(0,100,70)   # mint recoil
# #             else:                 robot.set_led(0,60,60)    # teal idle

# #         # ---- heartbeats (for spacing) ----
# #         if now - last_hb >= HB_DT:
# #             x1,y1,_ = pose; t1 = now
# #             robot.delay(50)
# #             p2 = safe_pose(robot)
# #             if p2:
# #                 x2,y2,th2 = p2; t2 = robot.get_clock()
# #                 dt = max(1e-3, t2-t1)
# #                 vx=(x2-x1)/dt; vy=(y2-y1)/dt
# #                 try: robot.send_msg(struct.pack(HB_FMT, x2,y2,th2,vx,vy,vid))
# #                 except: pass
# #                 last_hb=t2; x,y,th = x2,y2,th2
# #             else:
# #                 last_hb = now

# #         for m in (robot.recv_msg() or []):
# #             # pulses AND heartbeats can be interleaved; try pulse first (tiny)
# #             try:
# #                 tpe, lvid, s, sent = struct.unpack(P_FMT, m[:P_BYTES])
# #                 if tpe == TYPE_PULSE:
# #                     lowest_seen = min(lowest_seen, int(lvid))
# #                     if s not in seen_seqs:
# #                         seen_seqs.add(s)
# #                         # relay
# #                         robot.send_msg(struct.pack(P_FMT, TYPE_PULSE, int(lvid), int(s), 0))
# #                         # schedule local attack window with tiny offset
# #                         if now >= next_ready:
# #                             jitter = 0.02 * (vid % 5)   # subtle stagger
# #                             state = "aim"
# #                             aim_until   = now + AIM_TIME + jitter
# #                             # decide if we need a feint (someone in our lane?)
# #                             feint_needed = False
# #                             ahead_r = 0.30
# #                             for _,(nx,ny,nth,_,_) in neighbors.items():
# #                                 # project neighbor into my forward-right lane
# #                                 relx = (nx - x); rely = (ny - y)
# #                                 # within a narrow rightward corridor?
# #                                 if relx > -0.05 and abs(rely) < 0.18 and math.hypot(relx,rely) < ahead_r:
# #                                     feint_needed = True; break
# #                             if feint_needed:
# #                                 state = "feint"
# #                                 feint_until = aim_until + FEINT_TIME
# #                                 # feint a tiny LEFT tap before charging right
# #                                 target_h = wrap(RIGHT_HEADING + math.radians(180))  # left blip
# #                             else:
# #                                 target_h = RIGHT_HEADING
# #             except Exception:
# #                 # not a pulse; try heartbeat
# #                 try:
# #                     nx,ny,nth,nvx,nvy,nid = struct.unpack(HB_FMT, m[:HB_BYTES])
# #                     if int(nid)!=vid:
# #                         neighbors[int(nid)]=(nx,ny,nth,nvx,nvy)
# #                         last_seen[int(nid)]=now
# #                 except: pass

# #         # prune neighbors
# #         cut = now - STALE_S
# #         for nid in list(neighbors.keys()):
# #             if last_seen.get(nid,0)<cut:
# #                 neighbors.pop(nid,None); last_seen.pop(nid,None)

# #         # leader emits pulses
# #         if my_vid == lowest_seen and (now - last_pulse_emit) >= WAVE_PERIOD:
# #             seq += 1
# #             robot.send_msg(struct.pack(P_FMT, TYPE_PULSE, my_vid, seq, 0))
# #             seen_seqs.add(seq)
# #             last_pulse_emit = now

# #         # safety fields (used in charge/idle)
# #         ex,ey = soft_boundary_force(x,y)
# #         ox,oy = soft_obstacle_force(x,y)

# #         # strong separation
# #         repx=repy=0.0; ax=ay=0.0; n=0; nearest=1e9; nn=(None,None)
# #         for _,(nx,ny,nth,_,_) in neighbors.items():
# #             dx,dy = x-nx, y-ny
# #             d = math.hypot(dx,dy)
# #             if d < nearest: nearest, nn = d, (nx,ny)
# #             if d>1e-6 and d<SEP_R:
# #                 s = K_SEP * (SEP_R - d)/SEP_R
# #                 repx += s*(dx/d); repy += s*(dy/d)
# #             if d<=NEIGH_R:
# #                 ax += math.cos(nth); ay += math.sin(nth); n+=1
# #         alx=aly=0.0
# #         if n>0:
# #             ah = math.atan2(ay,ax)
# #             alx = K_ALI*math.cos(ah); aly = K_ALI*math.sin(ah)

# #         # emergency evade (rare; during charge we still respect this)
# #         evade_h = None
# #         if nearest < PANIC_R and nn[0] is not None:
# #             nx,ny = nn
# #             evade_h = math.atan2(y-ny, x-nx)

# #         # --- state machine ---
# #         if state == "reset":
# #             # idle until a pulse arrives
# #             err = 0.0; fwd = 0.0
# #         elif state == "aim":
# #             if now >= aim_until:
# #                 state = "charge"; charge_until = now + CHARGE_TIME
# #                 target_h = RIGHT_HEADING
# #             err = wrap(target_h - th); fwd = FWD_FEINT
# #         elif state == "feint":
# #             if now >= feint_until:
# #                 state = "charge"; charge_until = now + CHARGE_TIME
# #                 target_h = RIGHT_HEADING
# #             err = wrap(target_h - th); fwd = FWD_FEINT
# #         elif state == "charge":
# #             if now >= charge_until:
# #                 state = "hold"; hold_until = now + IMPACT_HOLD
# #                 robot.set_vel(0,0)  # slam stop
# #                 robot.delay(DT_MS); lastL=lastR=0
# #                 continue
# #             base_vx = math.cos(RIGHT_HEADING)
# #             base_vy = math.sin(RIGHT_HEADING)

# #             vx = base_vx + repx + 0.4*(ex+ox) + 0.2*alx
# #             vy = base_vy + repy + 0.4*(ey+oy) + 0.2*aly

# #             if evade_h is not None:
# #                 vx += 0.2*math.cos(evade_h); vy += 0.2*math.sin(evade_h)

# #             hdg = math.atan2(vy, vx)
# #             err = wrap(hdg - th); fwd = FWD_CHARGE
            
# #         elif state == "hold":
# #             if now >= hold_until:
# #                 state = "tap"; tap_until = now + DOUBLE_TAP
# #             robot.set_vel(0,0); robot.delay(DT_MS); lastL=lastR=0; continue
# #         elif state == "tap":
# #             if now >= tap_until:
# #                 state = "recoil"; recoil_until = now + RECOIL_TIME
# #             # quick mini-burst in RIGHT_HEADING
# #             err = wrap(RIGHT_HEADING - th); fwd = 0.70
# #         elif state == "recoil":
# #             if now >= recoil_until:
# #                 state = "reset"; next_ready = now + COOLDOWN
# #             # *** CHANGED: recoil opposite-ish to attack (mostly upward) ***
# #             rc = RIGHT_HEADING + math.pi - 0.35   # flip attack dir, slight angle
# #             vx = math.cos(rc) + ex + ox
# #             vy = math.sin(rc) + ey + oy
# #             hdg = math.atan2(vy, vx)
# #             err = wrap(hdg - th); fwd = FWD_RECOIL
# #         else:
# #             err = 0.0; fwd = FWD_DRIFT

# #         # dial down near boundary except for the stop/hold
# #         if b==1 and state not in ("hold","tap"): fwd *= 0.85

# #         left,right = heading_to_wheels(err, fwd, lastL, lastR)
# #         lastL,lastR = left,right
# #         robot.set_vel(left,right)

# #         # heartbeat of the loop
# #         robot.delay(DT_MS)
# #         # light logw every ~2s
# #         # (sim engine often ignores too-frequent logws)
# #         # (optional) — left as minimal to avoid spam







# # # # -*- coding: utf-8 -*-
# # # # PUNCH (CoachBot): sharp downward zig-zag bursts
# # # # - Overall motion: downward
# # # # - Each pulse: turn to a diagonal, then dash along it
# # # # - Diagonals alternate: down-right, down-left, down-right, ...
# # # # - Uses leader pulses (decentralized, P2P)
# # # # - No dancer obstacle; walls only
# # # from __future__ import division
# # # import math
# # # import struct
# # # import random

# # # # -------- arena bounds (meters) --------
# # # X_MIN, X_MAX = -1.2, 1.0
# # # Y_MIN, Y_MAX = -1.4, 2.35

# # # # -------- drive & loop --------
# # # MAX_WHEEL       = 35
# # # FWD_FAST        = 0.95     # dash speed
# # # FWD_SLOW        = 0.10     # idle creep
# # # FWD_MIN         = 0.45
# # # TURN_K_DASH     = 6.0      # strong turning in dash/aim
# # # TURN_K_IDLE     = 3.0      # gentler when idle
# # # CMD_SMOOTH_DASH = 0.08     # less smoothing during dash (sharper)
# # # CMD_SMOOTH_IDLE = 0.18     # more smoothing in idle
# # # LOOP_DT_MS      = 40

# # # # -------- waves / dash timing --------
# # # WAVE_PERIOD = 3.0    # leader emits pulse every 3s (feel free to tune)
# # # AIM_TIME    = 0.35   # aim turn time before dash
# # # DASH_TIME   = 1.3    # duration of the punch dash
# # # COOLDOWN    = 3.0    # rest after a dash

# # # # -------- zig-zag geometry --------
# # # # Base DOWN heading: -pi/2 (negative y)
# # # PUNCH_HEADING = -0.5 * math.pi

# # # # Diagonal offset around DOWN
# # # ZIG_ANGLE = 0.35 * math.pi   # ~63° off down (very diagonal)

# # # # -------- boundary softness (walls only) --------
# # # SOFT_MARGIN    = 0.08
# # # CRIT_MARGIN    = 0.02
# # # SOFT_MAX_FORCE = 0.35

# # # def soft_boundary_force(x, y):
# # #     fx = 0.0
# # #     fy = 0.0
# # #     if x < X_MIN + SOFT_MARGIN:
# # #         fx += SOFT_MAX_FORCE * (1.0 - (x - X_MIN)/SOFT_MARGIN)
# # #     elif x > X_MAX - SOFT_MARGIN:
# # #         fx -= SOFT_MAX_FORCE * (1.0 - (X_MAX - x)/SOFT_MARGIN)
# # #     if y < Y_MIN + SOFT_MARGIN:
# # #         fy += SOFT_MAX_FORCE * (1.0 - (y - Y_MIN)/SOFT_MARGIN)
# # #     elif y > Y_MAX - SOFT_MARGIN:
# # #         fy -= SOFT_MAX_FORCE * (1.0 - (Y_MAX - y)/SOFT_MARGIN)
# # #     return fx, fy

# # # def soft_boundary_check(x, y):
# # #     if (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
# # #         y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN):
# # #         return 2
# # #     if (x < X_MIN + SOFT_MARGIN or x > X_MAX - SOFT_MARGIN or
# # #         y < Y_MIN + SOFT_MARGIN or y > Y_MAX - SOFT_MARGIN):
# # #         return 1
# # #     return 0

# # # # -------- messaging --------
# # # TYPE_PULSE = 2
# # # P_FMT      = 'iiii'  # (type, leader_vid, seq, sentinel)
# # # P_BYTES    = struct.calcsize(P_FMT)

# # # def clamp(v, lo, hi):
# # #     return lo if v < lo else (hi if v > hi else v)

# # # def wrap_angle(a):
# # #     while a >  math.pi:
# # #         a -= 2.0*math.pi
# # #     while a <= -math.pi:
# # #         a += 2.0*math.pi
# # #     return a

# # # def safe_pose(robot):
# # #     p = robot.get_pose()
# # #     if p and len(p) >= 3:
# # #         return float(p[0]), float(p[1]), float(p[2])
# # #     return None

# # # def usr(robot):
# # #     # --- identity (HARDWARE) ---
# # #     try:
# # #         vid = int(robot.virtual_id())
# # #     except:
# # #         vid = -1

# # #     random.seed((vid if vid is not None else 0)*1103515245 & 0xFFFFFFFF)
# # #     print('Robot %d starting PUNCH zig-leg choreography' % vid)

# # #     my_vid       = int(vid)
# # #     lowest_seen  = my_vid
# # #     last_pulse_emit = -1e9
# # #     seq          = 0

# # #     seen_seqs      = set()
# # #     dash_active    = False
# # #     dash_until     = 0.0
# # #     aim_until      = 0.0
# # #     dash_start_time= 0.0

# # #     # zig sign: +1 = down-right, -1 = down-left
# # #     # we’ll flip this each time we start a dash
# # #     # seed based on vid so half start right, half left
# # #     zig_sign = 1 if (my_vid % 2 == 0) else -1
# # #     zig_heading = PUNCH_HEADING + zig_sign * ZIG_ANGLE

# # #     last_left  = 0
# # #     last_right = 0
# # #     last_print = 0.0
# # #     cool_until = 0.0

# # #     # wake localization
# # #     robot.set_vel(20, 20)
# # #     robot.delay(120)

# # #     try:
# # #         while True:
# # #             pose = safe_pose(robot)
# # #             if pose is None:
# # #                 robot.set_vel(0,0)
# # #                 robot.delay(LOOP_DT_MS)
# # #                 continue
           
# # #             x, y, th = pose
# # #             now = robot.get_clock()

# # #             # boundary / LEDs
# # #             bstat = soft_boundary_check(x, y)
# # #             if bstat == 2:
# # #                 robot.set_led(100,0,0)
# # #                 robot.set_vel(0,0)
# # #                 print('[punch_zigleg] CRITICAL id=%d boundary [%.3f,%.3f]' % (vid, x, y))
# # #                 robot.delay(LOOP_DT_MS)
# # #                 continue
# # #             elif bstat == 1:
# # #                 robot.set_led(100,59,0)   # amber-ish
# # #             else:
# # #                 robot.set_led(0,70,70)    # teal

# # #             # --- receive pulses ---
# # #             msgs = robot.recv_msg()
# # #             if msgs:
# # #                 for m in msgs:
# # #                     try:
# # #                         tpe, lvid, s, _sent = struct.unpack(P_FMT, m[:P_BYTES])
# # #                         if tpe == TYPE_PULSE:
# # #                             lowest_seen = min(lowest_seen, int(lvid))
# # #                             if s not in seen_seqs:
# # #                                 seen_seqs.add(s)
# # #                                 # relay once
# # #                                 robot.send_msg(struct.pack(P_FMT,
# # #                                                            TYPE_PULSE,
# # #                                                            int(lvid),
# # #                                                            int(s),
# # #                                                            0))
# # #                                 robot.delay(50)
# # #                                 # trigger own leg if cooled
# # #                                 if now >= cool_until:
# # #                                     # flip zig side
# # #                                     zig_sign   *= -1
# # #                                     zig_heading = PUNCH_HEADING + zig_sign * ZIG_ANGLE

# # #                                     aim_until       = now + AIM_TIME
# # #                                     dash_until      = aim_until + DASH_TIME
# # #                                     dash_start_time = aim_until
# # #                                     dash_active     = True
# # #                                     cool_until      = dash_until + COOLDOWN
# # #                                     robot.set_led(0,100,47)  # mint
# # #                                     print('[punch_zigleg] id=%d DASH start (pulse %d, sign=%+d)' %
# # #                                           (vid, s, zig_sign))
# # #                     except:
# # #                         pass

# # #             # --- leader pulses ---
# # #             if my_vid == lowest_seen and (now - last_pulse_emit) >= WAVE_PERIOD:
# # #                 seq += 1
# # #                 pkt = struct.pack(P_FMT, TYPE_PULSE, my_vid, seq, 0)
# # #                 robot.send_msg(pkt)
# # #                 last_pulse_emit = now
# # #                 seen_seqs.add(seq)
# # #                 print('[punch_zigleg] leader=%d pulse seq=%d' % (my_vid, seq))
# # #                 if now >= cool_until:
# # #                     zig_sign   *= -1
# # #                     zig_heading = PUNCH_HEADING + zig_sign * ZIG_ANGLE

# # #                     aim_until       = now + AIM_TIME
# # #                     dash_until      = aim_until + DASH_TIME
# # #                     dash_start_time = aim_until
# # #                     dash_active     = True
# # #                     cool_until      = dash_until + COOLDOWN
# # #                     robot.set_led(0,100,47)

# # #             # --- base wall force (used in both modes) ---
# # #             bx, by = soft_boundary_force(x, y)

# # #             if dash_active:
# # #                 # ----- AIM + DASH -----
# # #                 if now < aim_until:
# # #                     # Turn toward fixed zig_heading BEFORE dash
# # #                     target_h = zig_heading
# # #                     err = wrap_angle(target_h - th)
# # #                     fwd = FWD_SLOW * 0.6
# # #                     K_turn = TURN_K_DASH
# # #                     smooth = CMD_SMOOTH_DASH
# # #                 else:
# # #                     if now >= dash_until:
# # #                         # dash finished
# # #                         dash_active = False
# # #                         robot.set_led(0,70,70)  # teal idle
# # #                         fwd = FWD_SLOW
# # #                         err = wrap_angle(PUNCH_HEADING - th)
# # #                         K_turn = TURN_K_IDLE
# # #                         smooth = CMD_SMOOTH_IDLE
# # #                         print('[punch_zigleg] id=%d DASH end' % vid)
# # #                     else:
# # #                         # DASH: go fast along zig_heading (fixed)
# # #                         target_h = zig_heading
# # #                         err = wrap_angle(target_h - th)
# # #                         fwd = FWD_FAST
# # #                         K_turn = TURN_K_DASH
# # #                         smooth = CMD_SMOOTH_DASH

# # #                 turn = clamp(K_turn * err, -2.6, 2.6)
# # #                 lcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)),
# # #                              -MAX_WHEEL, MAX_WHEEL)
# # #                 rcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)),
# # #                              -MAX_WHEEL, MAX_WHEEL)

# # #                 left  = int((1.0 - smooth) * lcmd + smooth * last_left)
# # #                 right = int((1.0 - smooth) * rcmd + smooth * last_right)

# # #             else:
# # #                 # ----- IDLE drift: small downward + walls -----
# # #                 vx_idle = bx
# # #                 vy_idle = by - 0.06  # small downward bias

# # #                 if abs(vx_idle) < 1e-6 and abs(vy_idle) < 1e-6:
# # #                     vy_idle = -1e-3

# # #                 hdg_idle = math.atan2(vy_idle, vx_idle)
# # #                 err = wrap_angle(hdg_idle - th)
# # #                 ae = abs(err)
# # #                 if ae < 0.5:
# # #                     fwd = FWD_SLOW
# # #                 elif ae < 1.2:
# # #                     fwd = FWD_SLOW * 0.8
# # #                 else:
# # #                     fwd = FWD_SLOW * 0.6
# # #                 if bstat == 1:
# # #                     fwd = fwd * 0.7
# # #                 if fwd < FWD_MIN:
# # #                     fwd = FWD_MIN

# # #                 turn = clamp(TURN_K_IDLE * err, -2.0, 2.0)
# # #                 lcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)),
# # #                              -MAX_WHEEL, MAX_WHEEL)
# # #                 rcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)),
# # #                              -MAX_WHEEL, MAX_WHEEL)

# # #                 left  = int((1.0 - CMD_SMOOTH_IDLE) * lcmd + CMD_SMOOTH_IDLE * last_left)
# # #                 right = int((1.0 - CMD_SMOOTH_IDLE) * rcmd + CMD_SMOOTH_IDLE * last_right)

# # #             last_left  = left
# # #             last_right = right
# # #             robot.set_vel(left, right)

# # #             if now - last_print > 2.0:
# # #                 state = "dash" if dash_active else "idle"
# # #                 print('[punch_zigleg] id=%d leader=%d state=%s pos[%.3f,%.3f]' %
# # #                       (vid, lowest_seen, state, x, y))
# # #                 last_print = now

# # #             robot.delay(LOOP_DT_MS)

# # #     except Exception as e:
# # #         try:
# # #             robot.set_vel(0,0)
# # #             robot.set_led(100,0,0)
# # #         except:
# # #             pass
# # #         print('[punch_zigleg] ERROR id=%d: %s' % (vid, repr(e)))
# # #         raise
# # #     finally:
# # #         try:
# # #             robot.set_vel(0,0)
# # #         except:
# # #             pass



# # # # -*- coding: utf-8 -*-
# # # # PUNCH_ZIGZAG (CoachBot): sharp, bursty, downward zig-zag
# # # # Flocking structure copied from FLOAT, but:
# # # #  - migration is downward instead of up
# # # #  - path is piecewise diagonal (zig-zag)
# # # #  - speed alternates between bursts and slower drift
# # # from __future__ import division
# # # import math
# # # import struct
# # # import random

# # # # --- field (meters) ---
# # # X_MIN, X_MAX = -1.2, 1.0
# # # Y_MIN, Y_MAX = -1.4, 2.35

# # # # --- dancer / ring center (used for ring preservation) ---
# # # FEET = 0.3048
# # # OBST_DIAM_FT = 1.0
# # # OBST_RADIUS  = 0.5 * OBST_DIAM_FT * FEET
# # # OBST_MARGIN  = 0.03
# # # SAFE_BUBBLE  = OBST_RADIUS + OBST_MARGIN
# # # OBST_CX, OBST_CY = (-0.1, 0.475)

# # # # --- control/loop ---
# # # MAX_WHEEL   = 35
# # # TURN_K      = 3.0
# # # FWD_FAST    = 0.95   # strong dash
# # # FWD_SLOW    = 0.05   # very gentle drift between bursts
# # # FWD_MIN     = 0.00   # allow almost stopping
# # # DT_MS       = 40
# # # CMD_SMOOTH  = 0.18   # less smoothing = punchier
# # # VEL_SLEW    = 10     # allow snappier changes

# # # # --- flocking gains ---
# # # K_MIG   = 0.11      # migration strength (downward diagonals)
# # # K_SEP   = 0.20      # separation
# # # K_ALI   = 0.16      # alignment
# # # K_COH   = 0.10      # cohesion

# # # SEP_RADIUS   = 0.26
# # # NEIGH_RADIUS = 0.75

# # # # --- ring preservation ---
# # # K_RING = 0.35       # radial spring to keep each bot on its original ring

# # # # --- zig-zag wave parameters ---
# # # ZIG_FREQ   = 0.20   # Hz (how often they flip diagonals)
# # # PHASE_STEP = 0.6    # phase offset per virtual_id
# # # ZIG_X_MAG  = 1.0    # x component of diagonal
# # # ZIG_Y_MAG  = 1.0    # y component of diagonal (downwards)

# # # # fraction of the cycle where we "dash"
# # # DASH_DUTY  = 0.45   # 45% of the period is a burst

# # # # --- boundary softness (walls only) ---
# # # SOFT_MARGIN = 0.08
# # # CRIT_MARGIN = 0.02
# # # SOFT_MAX_F  = 0.35

# # # # --- P2P heartbeats (same as FLOAT) ---
# # # HB_FMT   = 'fffffi'  # x,y,th,vx,vy,id
# # # HB_BYTES = struct.calcsize(HB_FMT)
# # # HB_DT    = 0.12
# # # STALE_S  = 0.7

# # # def clamp(v, lo, hi):
# # #     return lo if v < lo else (hi if v > hi else v)

# # # def wrap(a):
# # #     while a > math.pi:
# # #         a -= 2*math.pi
# # #     while a <= -math.pi:
# # #         a += 2*math.pi
# # #     return a

# # # def soft_boundary_force(x, y):
# # #     fx = fy = 0.0
# # #     if x < X_MIN + SOFT_MARGIN:
# # #         fx += SOFT_MAX_F * (1 - (x - X_MIN) / SOFT_MARGIN)
# # #     elif x > X_MAX - SOFT_MARGIN:
# # #         fx -= SOFT_MAX_F * (1 - (X_MAX - x) / SOFT_MARGIN)
# # #     if y < Y_MIN + SOFT_MARGIN:
# # #         fy += SOFT_MAX_F * (1 - (y - Y_MIN) / SOFT_MARGIN)
# # #     elif y > Y_MAX - SOFT_MARGIN:
# # #         fy -= SOFT_MAX_F * (1 - (Y_MAX - y) / SOFT_MARGIN)
# # #     return fx, fy

# # # def boundary_state(x, y):
# # #     if (x < X_MIN + CRIT_MARGIN or x > X_MAX - CRIT_MARGIN or
# # #         y < Y_MIN + CRIT_MARGIN or y > Y_MAX - CRIT_MARGIN):
# # #         return 2
# # #     if (x < X_MIN + SOFT_MARGIN or x > X_MAX - SOFT_MARGIN or
# # #         y < Y_MIN + SOFT_MARGIN or y > Y_MAX - SOFT_MARGIN):
# # #         return 1
# # #     return 0

# # # def safe_pose(robot):
# # #     p = robot.get_pose()
# # #     if p and len(p) >= 3:
# # #         return float(p[0]), float(p[1]), float(p[2])
# # #     return None

# # # def usr(robot):
# # #     robot.delay(400)

# # #     try:
# # #         vid = int(robot.virtual_id())
# # #     except:
# # #         vid = 0

# # #     random.seed(vid*1103515245 & 0xFFFFFFFF)

# # #     # per-robot phase offset for the zig-zag timing
# # #     phase0 = (vid % 12) * PHASE_STEP

# # #     print('Robot %d starting PUNCH_ZIGZAG choreography (downward, bursty)' % vid)

# # #     neighbors = {}
# # #     last_seen = {}
# # #     last_hb = -1e9
# # #     lastL = 0
# # #     lastR = 0

# # #     # ring radius for this robot (initialized on first valid pose)
# # #     ring_R0 = None

# # #     # wake localization
# # #     robot.set_vel(20, 20)
# # #     robot.delay(150)

# # #     try:
# # #         while True:
# # #             pose = safe_pose(robot)
# # #             if not pose:
# # #                 robot.set_vel(0, 0)
# # #                 robot.delay(DT_MS)
# # #                 continue

# # #             x, y, th = pose
# # #             now = robot.get_clock()

# # #             # initialize ring radius once
# # #             if ring_R0 is None:
# # #                 dx0 = x - OBST_CX
# # #                 dy0 = y - OBST_CY
# # #                 ring_R0 = math.sqrt(dx0*dx0 + dy0*dy0)

# # #             # LEDs (only wall proximity)
# # #             b = boundary_state(x, y)
# # #             if b == 2:
# # #                 robot.set_led(100, 0, 0)
# # #                 robot.set_vel(0, 0)
# # #                 robot.delay(DT_MS)
# # #                 continue
# # #             elif b == 1:
# # #                 robot.set_led(100, 60, 0)  # amber
# # #             else:
# # #                 robot.set_led(80, 10, 10)  # punchy red-ish

# # #             # --- heartbeat send (finite diff for vx,vy) ---
# # #             if now - last_hb >= HB_DT:
# # #                 x1 = x
# # #                 y1 = y
# # #                 t1 = now
# # #                 robot.delay(60)
# # #                 p2 = safe_pose(robot)
# # #                 if p2:
# # #                     x2, y2, th2 = float(p2[0]), float(p2[1]), float(p2[2])
# # #                     t2 = robot.get_clock()
# # #                     dt = max(1e-3, t2 - t1)
# # #                     vx_hb = (x2 - x1) / dt
# # #                     vy_hb = (y2 - y1) / dt
# # #                     try:
# # #                         robot.send_msg(struct.pack(HB_FMT, x2, y2, th2,
# # #                                                    vx_hb, vy_hb, vid))
# # #                     except:
# # #                         pass
# # #                     last_hb = t2
# # #                     x, y, th = x2, y2, th2
# # #                 else:
# # #                     last_hb = now

# # #             # --- receive neighbor heartbeats ---
# # #             msgs = robot.recv_msg()
# # #             if msgs:
# # #                 for m in msgs:
# # #                     try:
# # #                         nx, ny, nth, nvx, nvy, nid = struct.unpack(HB_FMT,
# # #                                                                    m[:HB_BYTES])
# # #                         if int(nid) != vid:
# # #                             neighbors[int(nid)] = (nx, ny, nth, nvx, nvy)
# # #                             last_seen[int(nid)] = now
# # #                     except:
# # #                         pass

# # #             # prune stale neighbors
# # #             cut = now - STALE_S
# # #             for nid in list(neighbors.keys()):
# # #                 if last_seen.get(nid, 0) < cut:
# # #                     neighbors.pop(nid, None)
# # #                     last_seen.pop(nid, None)

# # #             # --- environment forces (walls only) ---
# # #             ex, ey = soft_boundary_force(x, y)

# # #             # --- zig-zag timing & direction ---
# # #             wave_phase = 2.0 * math.pi * ZIG_FREQ * now + phase0
# # #             s = math.sin(wave_phase)

# # #             # direction: diagonals (down-right or down-left)
# # #             if s >= 0.0:
# # #                 zig_x = ZIG_X_MAG
# # #             else:
# # #                 zig_x = -ZIG_X_MAG
# # #             zig_y = -ZIG_Y_MAG  # downward

# # #             # normalize diagonal vector
# # #             nrm = math.sqrt(zig_x*zig_x + zig_y*zig_y)
# # #             if nrm < 1e-6:
# # #                 nrm = 1.0
# # #             zig_x /= nrm
# # #             zig_y /= nrm

# # #             # --- bursty forward speed based on phase ---
# # #             phase_01 = (wave_phase / (2.0*math.pi)) % 1.0
# # #             if phase_01 < DASH_DUTY:
# # #                 fwd = FWD_FAST
# # #                 mig_scale = 1.0      # strong diagonal push during dash
# # #             else:
# # #                 fwd = FWD_SLOW
# # #                 mig_scale = 0.35     # weaker diagonal in drift segment

# # #             # slow slightly in soft boundary zone
# # #             if b == 1:
# # #                 fwd *= 0.7

# # #             # never drop below minimum
# # #             if fwd < FWD_MIN:
# # #                 fwd = FWD_MIN

# # #             # migration along diagonal (scaled by burst/drift state)
# # #             mx = K_MIG * zig_x * mig_scale
# # #             my = K_MIG * zig_y * mig_scale

# # #             # --- neighbor terms (flocking) ---
# # #             repx = repy = 0.0
# # #             cx = cy = 0.0
# # #             ax = ay = 0.0
# # #             n = 0

# # #             for nid in neighbors:
# # #                 nx, ny, nth, nvx, nvy = neighbors[nid]
# # #                 dx = x - nx
# # #                 dy = y - ny
# # #                 d2 = dx*dx + dy*dy
# # #                 if d2 > 1e-9:
# # #                     d = math.sqrt(d2)
# # #                     if d < SEP_RADIUS:
# # #                         ssep = K_SEP * (SEP_RADIUS - d) / SEP_RADIUS
# # #                         repx += ssep * (dx / d)
# # #                         repy += ssep * (dy / d)
# # #                     if d <= NEIGH_RADIUS:
# # #                         cx += nx
# # #                         cy += ny
# # #                         ax += math.cos(nth)
# # #                         ay += math.sin(nth)
# # #                         n += 1

# # #             cohx = cohy = 0.0
# # #             alx = aly = 0.0

# # #             if n > 0:
# # #                 cx /= n
# # #                 cy /= n
# # #                 cohx = K_COH * (cx - x)
# # #                 cohy = K_COH * (cy - y)
# # #                 ah = math.atan2(ay, ax)
# # #                 alx = K_ALI * math.cos(ah)
# # #                 aly = K_ALI * math.sin(ah)

# # #             # --- ring-preserving radial force ---
# # #             ringx = 0.0
# # #             ringy = 0.0
# # #             if ring_R0 is not None:
# # #                 dxC = x - OBST_CX
# # #                 dyC = y - OBST_CY
# # #                 rC2 = dxC*dxC + dyC*dyC
# # #                 if rC2 > 1e-6:
# # #                     rC = math.sqrt(rC2)
# # #                     radial_err = rC - ring_R0   # >0 = too far out, <0 = too far in
# # #                     ux = dxC / rC
# # #                     uy = dyC / rC
# # #                     ringx = -K_RING * radial_err * ux
# # #                     ringy = -K_RING * radial_err * uy

# # #             # --- combine fields ---
# # #             vx = ex + mx + repx + cohx + alx + ringx
# # #             vy = ey + my + repy + cohy + aly + ringy

# # #             if abs(vx) < 1e-6 and abs(vy) < 1e-6:
# # #                 vx = 1e-3

# # #             hdg = math.atan2(vy, vx)
# # #             err = wrap(hdg - th)

# # #             # map to wheels
# # #             turn = clamp(TURN_K * err, -1.5, 1.5)
# # #             lcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8 * turn)),
# # #                          -MAX_WHEEL, MAX_WHEEL)
# # #             rcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8 * turn)),
# # #                          -MAX_WHEEL, MAX_WHEEL)

# # #             # slew limit
# # #             if lcmd > lastL + VEL_SLEW: lcmd = lastL + VEL_SLEW
# # #             if lcmd < lastL - VEL_SLEW: lcmd = lastL - VEL_SLEW
# # #             if rcmd > lastR + VEL_SLEW: rcmd = lastR + VEL_SLEW
# # #             if rcmd < lastR - VEL_SLEW: rcmd = lastR - VEL_SLEW

# # #             left  = int((1 - CMD_SMOOTH) * lcmd + CMD_SMOOTH * lastL)
# # #             right = int((1 - CMD_SMOOTH) * rcmd + CMD_SMOOTH * lastR)
# # #             lastL, lastR = left, right

# # #             robot.set_vel(left, right)
# # #             robot.delay(DT_MS)

# # #     except Exception as e:
# # #         try:
# # #             robot.set_vel(0,0)
# # #             robot.set_led(100,0,0)
# # #         except:
# # #             pass
# # #         print('[punch_zigzag] ERROR id=%d: %s' % (vid, repr(e)))
# # #         raise
# # #     finally:
# # #         try:
# # #             robot.set_vel(0,0)
# # #         except:
# # #             pass


# # -*- coding: utf-8 -*-
# # PUNCH (CoachBot): sharp, bursty, rightward zig-zag
# # ORIGINAL VERSION BEFORE DIRECTION WAS FLIPPED DOWNWARD

# from __future__ import division
# import math
# import struct
# import random

# # --- field (meters) ---
# X_MIN, X_MAX = -1.2, 1.0
# Y_MIN, Y_MAX = -1.4, 2.35

# # --- control parameters ---
# MAX_WHEEL   = 35
# TURN_K      = 3.0
# FWD_FAST    = 0.92     # burst speed
# FWD_SLOW    = 0.20     # drift speed
# FWD_MIN     = 0.08
# DT_MS       = 40
# CMD_SMOOTH  = 0.18
# VEL_SLEW    = 10

# # --- flocking gains ---
# K_MIG   = 0.14      # migration strength
# K_SEP   = 0.22
# K_ALI   = 0.15
# K_COH   = 0.08

# SEP_RADIUS   = 0.26
# NEIGH_RADIUS = 0.75

# # --- zig-zag wave parameters (RIGHTWARD) ---
# ZIG_FREQ   = 0.22      # switch rate
# PHASE_STEP = 0.6
# ZIG_X_MAG  = 1.0        # rightward bias
# ZIG_Y_MAG  = 0.6        # up/down diagonals

# # portion of cycle in FAST mode
# DASH_DUTY  = 0.42

# # --- boundaries ---
# SOFT_MARGIN    = 0.08
# CRIT_MARGIN    = 0.02
# SOFT_MAX_FORCE = 0.35

# # --- heartbeat ---
# HB_FMT   = 'fffffi'
# HB_BYTES = struct.calcsize(HB_FMT)
# HB_DT    = 0.12
# STALE_S  = 0.7

# def clamp(v, lo, hi): return lo if v < lo else (hi if v > hi else v)

# def wrap(a):
#     while a > math.pi: a -= 2*math.pi
#     while a <= -math.pi: a += 2*math.pi
#     return a

# def soft_boundary_force(x, y):
#     fx = fy = 0.0
#     if x < X_MIN + SOFT_MARGIN:
#         fx += SOFT_MAX_FORCE * (1 - (x - X_MIN) / SOFT_MARGIN)
#     elif x > X_MAX - SOFT_MARGIN:
#         fx -= SOFT_MAX_FORCE * (1 - (X_MAX - x) / SOFT_MARGIN)
#     if y < Y_MIN + SOFT_MARGIN:
#         fy += SOFT_MAX_FORCE * (1 - (y - Y_MIN) / SOFT_MARGIN)
#     elif y > Y_MAX - SOFT_MARGIN:
#         fy -= SOFT_MAX_FORCE * (1 - (Y_MAX - y) / SOFT_MARGIN)
#     return fx, fy

# def safe_pose(robot):
#     p = robot.get_pose()
#     if p and len(p) >= 3:
#         return float(p[0]), float(p[1]), float(p[2])
#     return None


# def usr(robot):
#     robot.delay(400)

#     try:
#         vid = int(robot.virtual_id())
#     except:
#         vid = 0

#     random.seed(vid * 1103515245)

#     phase0 = (vid % 12) * PHASE_STEP

#     neighbors = {}
#     last_seen = {}
#     last_hb   = -1e9
#     lastL = lastR = 0

#     # wake localization
#     robot.set_vel(20, 20)
#     robot.delay(150)

#     while True:
#         pose = safe_pose(robot)
#         if not pose:
#             robot.set_vel(0,0)
#             robot.delay(DT_MS)
#             continue

#         x, y, th = pose
#         now = robot.get_clock()

#         # --- boundary LEDs ---
#         fx_b, fy_b = soft_boundary_force(x, y)
#         near_wall = (abs(fx_b) + abs(fy_b)) > 1e-6

#         if near_wall:
#             robot.set_led(100, 50, 0)
#         else:
#             robot.set_led(100, 0, 0)     # punchy red

#         # --- heartbeat send ---
#         if now - last_hb >= HB_DT:
#             try:
#                 robot.send_msg(struct.pack(HB_FMT, x, y, th, 0.0, 0.0, vid))
#             except:
#                 pass
#             last_hb = now

#         # --- heartbeat receive ---
#         msgs = robot.recv_msg() or []
#         if not isinstance(msgs, list):
#             msgs = [msgs]

#         for m in msgs:
#             try:
#                 nx, ny, nth, nvx, nvy, nid = struct.unpack(HB_FMT, m[:HB_BYTES])
#                 if nid != vid:
#                     neighbors[nid] = (nx, ny, nth, nvx, nvy)
#                     last_seen[nid] = now
#             except:
#                 pass

#         # prune stale
#         for nid in list(neighbors.keys()):
#             if now - last_seen.get(nid, 0) > STALE_S:
#                 neighbors.pop(nid, None)

#         # ------------------------
#         # ZIGZAG RIGHTWARD FIELD
#         # ------------------------
#         wave_phase = 2 * math.pi * ZIG_FREQ * now + phase0
#         s = math.sin(wave_phase)

#         # rightward + up/down diagonal
#         zig_x = ZIG_X_MAG
#         zig_y = +ZIG_Y_MAG if s >= 0 else -ZIG_Y_MAG

#         # normalize
#         L = math.sqrt(zig_x**2 + zig_y**2)
#         zig_x /= L; zig_y /= L

#         # migration force
#         mx = K_MIG * zig_x
#         my = K_MIG * zig_y

#         # ------------------------
#         # BOIDS: repulsion, cohesion, alignment
#         # ------------------------
#         repx = repy = 0.0
#         cx = cy = 0.0
#         ax = ay = 0.0
#         n = 0

#         for nid in neighbors:
#             nx, ny, nth, nvx, nvy = neighbors[nid]
#             dx = x - nx
#             dy = y - ny
#             d2 = dx*dx + dy*dy
#             if d2 > 1e-9:
#                 d = math.sqrt(d2)
#             7    if d < SEP_RADIUS:
#                     rep = K_SEP * (SEP_RADIUS - d) / SEP_RADIUS
#                     repx += rep * (dx / d)
#                     repy += rep * (dy / d)
#                 if d <= NEIGH_RADIUS:
#                     cx += nx
#                     cy += ny
#                     ax += math.cos(nth)
#                     ay += math.sin(nth)
#                     n += 1

#         cohx = cohy = 0
#         alx = aly = 0
#         if n > 0:
#             cx /= n; cy /= n
#             cohx = K_COH * (cx - x)
#             cohy = K_COH * (cy - y)
#             ah = math.atan2(ay, ax)
#             alx = K_ALI * math.cos(ah)
#             aly = K_ALI * math.sin(ah)

#         # combine fields
#         vx = fx_b + mx + repx + cohx + alx
#         vy = fy_b + my + repy + cohy + aly

#         if abs(vx) + abs(vy) < 1e-6:
#             vx = 1e-3

#         # convert to target heading
#         hdg = math.atan2(vy, vx)
#         err = wrap(hdg - th)

#         # ------------------------
#         # BURST MODE
#         # ------------------------
#         phase01 = (wave_phase / (2 * math.pi)) % 1.0
#         if phase01 < DASH_DUTY:
#             fwd = FWD_FAST
#         else:
#             fwd = FWD_SLOW

#         if near_wall:
#             fwd *= 0.7

#         fwd = max(fwd, FWD_MIN)

#         turn = clamp(TURN_K * err, -1.5, 1.5)
#         lcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd - 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)
#         rcmd = clamp(int(MAX_WHEEL * 0.9 * (fwd + 0.8*turn)), -MAX_WHEEL, MAX_WHEEL)

#         # smoothing
#         left  = int((1 - CMD_SMOOTH)*lcmd + CMD_SMOOTH*lastL)
#         right = int((1 - CMD_SMOOTH)*rcmd + CMD_SMOOTH*lastR)
#         lastL, lastR = left, right

#         robot.set_vel(left, right)
#         robot.delay(DT_MS)


# -*- coding: utf-8 -*-
# PUNCH (downward, no dancer avoidance) - COACHBOT VERSION
# Coordinated wave-like dashes triggered by leader pulses
# Using ring-lock + COM logic + anti-spin correction + GLOBAL STOP
# Python 2.7 compatible

from __future__ import division
import math, struct, random

# -------- arena bounds (meters) --------
X_MIN, X_MAX = -1.2, 1.0
Y_MIN, Y_MAX = -1.4, 2.35

# -------- drive & loop --------
MAX_WHEEL = 35
TURN_K    = 3.0
FWD_FAST  = 0.95
FWD_SLOW  = 0.10
FWD_MIN   = 0.45
CMD_SMOOTH= 0.18
LOOP_DT_MS= 40

# -------- waves --------
WAVE_PERIOD    = 4.0
DASH_TIME      = 1.2
AIM_TIME       = 0.35

# DOWNWARD punch
PUNCH_HEADING  = -0.5 * math.pi
ANGLE_JITTER   = 0.22
COOLDOWN       = 5.0

# -------- boundary softness --------
SOFT_MARGIN     = 0.08
CRIT_MARGIN     = 0.02
SOFT_MAX_FORCE  = 0.35

# -------- messaging --------
TYPE_PULSE = 2
STOP_NOW   = 99                    # === GLOBAL STOP ADDITION ===

# pulse + stop messages share this format
P_FMT = 'iiii'
P_BYTES = struct.calcsize(P_FMT)

# -------- utils --------
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

    # --- GLOBAL STOP FLAG ---
    global_stop = False           # === GLOBAL STOP ADDITION ===
    sentinel_id = None            # the bot that hits yellow

    # wake localization
    robot.set_vel(20,20); robot.delay(120)

    try:
        while True:

            # --- early-out if global stop triggered ---
            if global_stop:       # === GLOBAL STOP ADDITION ===
                robot.set_led(255,255,255)  # white
                robot.set_vel(0,0)
                return

            pose = safe_pose(robot)
            if pose is None:
                robot.set_vel(0,0); robot.delay(LOOP_DT_MS); continue
            x,y,th = pose
            now = robot.get_clock()

            # --- pulse + STOP message listening ---
            msgs = robot.recv_msg()
            if msgs:
                for m in msgs:
                    try:
                        tpe, lvid, s, _ = struct.unpack(P_FMT, m[:P_BYTES])

                        # === GLOBAL STOP ADDITION ===
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

            # --- boundary check ---
            bstat = soft_boundary_check(x,y)

            if bstat == 2:  # critical -> sentinel triggers global stop
                if not global_stop:
                    sentinel_id = vid
                    robot.set_led(255,0,0)
                    robot.send_msg(struct.pack(P_FMT, STOP_NOW, vid, 0, 0))  # STOP NOW
                global_stop = True
                continue

            elif bstat == 1:  # soft boundary: sentinel triggers global stop
                if not global_stop:
                    sentinel_id = vid
                    robot.set_led(255,200,0)   # sentinel yellow
                    robot.send_msg(struct.pack(P_FMT, STOP_NOW, vid, 0, 0))
                global_stop = True
                continue

            else:
                robot.set_led(0,70,70)

            # --- leader emits pulses ---
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

            # ===== MOTION =====
            vx = 0.0
            vy = -0.05   # downward drift

            if dash_active:

                if now < aim_until:
                    target = aim_heading
                    err = wrap_angle(target - th)
                    fwd = FWD_SLOW * 0.7
                    if abs(err) > 1.3:   # anti-spin
                        fwd = max(fwd, 0.22)

                else:
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

            # smoothing motors
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


