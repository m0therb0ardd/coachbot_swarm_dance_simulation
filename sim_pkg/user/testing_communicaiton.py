
# test_comms_simple.py
# Minimal script to test messaging in the simulator.
# Each robot:
#   - gets its ID
#   - periodically broadcasts (time, id)
#   - listens for messages and prints what it receives
#   - flashes LED so you can see it's alive

import struct

def get_id(robot):
    """Try to get a stable integer ID for this robot."""
    if hasattr(robot, "id"):
        try:
            return int(robot.id)
        except Exception:
            pass
    if hasattr(robot, "virtual_id"):
        try:
            return int(robot.virtual_id())
        except Exception:
            pass
    return -1

def usr(robot):
    my_id = get_id(robot)

    MSG_FMT  = "fi"                      # float time, int id
    MSG_SIZE = struct.calcsize(MSG_FMT)  # 8 bytes

    print(f"[Robot {my_id}] test_comms_simple.py starting")

    robot.set_led(0, 0, 80)
    robot.delay(300)

    last_broadcast = 0.0

    while True:
        now = robot.get_clock()

        # --- 1) Broadcast my (time, id) every 0.5s ---
        if now - last_broadcast > 0.5:
            msg = struct.pack(MSG_FMT, float(now), int(my_id))
            # In this sim, send_msg may always return False even when it works,
            # so we don't bother checking the return value.
            robot.send_msg(msg)
            print(f"[Robot {my_id}] SENT msg at t={now:.2f}")
            robot.set_led(0, 0, 100)  # brief blue pop
            last_broadcast = now

        # --- 2) Check for incoming messages ---
        msgs = robot.recv_msg()
        if msgs:
            for raw in msgs:
                if not raw:
                    continue
                try:
                    t_sender, id_sender = struct.unpack(MSG_FMT, raw[:MSG_SIZE])
                    print(f"[Robot {my_id}] RECEIVED from {id_sender} at sim t={t_sender:.2f}")
                    robot.set_led(0, 100, 0)  # green on receive
                except Exception as e:
                    print(f"[Robot {my_id}] failed to unpack msg: {e}")

        robot.delay(50)
