import cv2
import numpy as np

VIDEO_PATH = '/home/catherine-maglione//coachbot_simulation/swarm-simulator/sim_pkg/user/usr_recording.mp4'

cap = cv2.VideoCapture(VIDEO_PATH)

paused = False
frame_hsv = None
h_samples = []
s_samples = []
v_samples = []

def click_event(event, x, y, flags, param):
    global frame_hsv, h_samples, s_samples, v_samples
    if event == cv2.EVENT_LBUTTONDOWN and frame_hsv is not None:
        hsv = frame_hsv[y, x]
        h, s, v = int(hsv[0]), int(hsv[1]), int(hsv[2])
        print(f"Clicked HSV = {h, s, v}")
        h_samples.append(h)
        s_samples.append(s)
        v_samples.append(v)
        print(f"Collected {len(h_samples)} samples.")

cv2.namedWindow("Video")
cv2.setMouseCallback("Video", click_event)

while True:
    if not paused:
        ret, frame = cap.read()
        if not ret:
            print("End of video.")
            break
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    disp = frame.copy()
    cv2.putText(disp, "[SPACE]=pause/play, a=prev, d=next, s=save HSV", 
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

    cv2.imshow("Video", disp)
    key = cv2.waitKey(0 if paused else 1) & 0xFF

    # --- Controls ---
    if key == ord(' '):   # pause/play
        paused = not paused

    elif key == ord('d'): # next frame
        paused = True
        ret, frame = cap.read()
        if ret:
            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    elif key == ord('a'): # previous frame
        paused = True
        pos = cap.get(cv2.CAP_PROP_POS_FRAMES)
        cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, pos-2))

        ret, frame = cap.read()
        if ret:
            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    elif key == ord('s'): # save ranges
        if len(h_samples) > 0:
            h_min, h_max = min(h_samples), max(h_samples)
            s_min, s_max = min(s_samples), max(s_samples)
            v_min, v_max = min(v_samples), max(v_samples)

            print("\n================ HSV RANGE ================")
            print(f"lower = np.array([{h_min}, {s_min}, {v_min}])")
            print(f"upper = np.array([{h_max}, {s_max}, {v_max}])")
            print("===========================================\n")

        else:
            print("No samples collected yet.")

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
