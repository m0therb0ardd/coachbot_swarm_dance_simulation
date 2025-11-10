import math
import struct


def usr(robot):
    robot.delay(3000)
    # log = open("experiment_log.txt", "w")

    log.write("Starting: face left and move left 5s\n"); log.flush()
    write_str = "I am robot " + str(robot.virtual_id()) + "\n"
    log.write(write_str); log.flush()

    while True:
        for i in range(1):  # run once (pattern like your example)
            robot.delay()

            # --- params you can tune ---
            SPIN_SPEED = 25     # wheel pwm for in-place spin
            SPIN_MS    = 1000   # ~90 deg CCW; tune on your platform
            DRIVE      = 25     # straight drive pwm
            DRIVE_MS   = 5000   # 5 seconds

            # 1) turn CCW to face left (west)
            robot.set_led(0, 100, 0)          # green = turning
            robot.set_vel(-SPIN_SPEED, SPIN_SPEED)
            robot.delay(SPIN_MS)

            # 2) drive left (forward after reorientation) for 5s
            robot.set_led(0, 0, 100)          # blue = moving
            robot.set_vel(DRIVE, DRIVE)
            robot.delay(DRIVE_MS)

            # 3) stop and mark done
            robot.set_vel(0, 0)
            robot.set_led(100, 0, 0)          # red = done
            robot.delay(500)

            log.write("Finished running the simple example program\n")
            log.flush()

        log.write("experiment complete\n")
        log.flush()
        log.close()
        return
