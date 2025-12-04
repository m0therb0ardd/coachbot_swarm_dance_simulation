import cv2
import numpy as np

# -----------------------------
# USER SETTINGS
# -----------------------------

# You can add ANY number of colors here.
# Give each one a name + HSV bounds.
COLOR_RANGES = {
    "color1": (np.array([72, 50, 200]), np.array([82, 255, 255])),
    "color2": (np.array([84, 5, 180]),  np.array([94, 200, 255])),
    "color3": (np.array([30, 5, 200]),  np.array([40, 150, 255])),
    "color4": (np.array([16, 40, 120]), np.array([26, 255, 255])),
}

VIDEO_PATH = '/home/catherine-maglione//coachbot_simulation/swarm-simulator/sim_pkg/user/usr_recording.mp4'

OUTPUT_PATH = '/home/catherine-maglione//coachbot_simulation/swarm-simulator/sim_pkg/user/light_trails_redo.mp4'


GLOW_DECAY = 0.95     # 1.0 = no fade, 0.8 = fade quickly
DILATE_KERNEL = 5      # smooth/thicker trail (px), or set 0 for none
ALPHA = 0.4            # blend weight between original frame & trails

# -----------------------------
# SETUP
# -----------------------------
cap = cv2.VideoCapture(VIDEO_PATH)

if not cap.isOpened():
    print("❌ Could not open video.")
    exit()

frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps     = int(cap.get(cv2.CAP_PROP_FPS))

out = cv2.VideoWriter(
    OUTPUT_PATH,
    cv2.VideoWriter_fourcc(*'mp4v'),
    fps,
    (frame_w, frame_h)
)

# Accumulation buffer for all light trails
light_canvas = np.zeros((frame_h, frame_w, 3), dtype=np.float32)

kernel = np.ones((DILATE_KERNEL, DILATE_KERNEL), np.uint8) if DILATE_KERNEL > 0 else None

frame_idx = 0

# -----------------------------
# MAIN LOOP
# -----------------------------
while True:
    ret, frame = cap.read()
    if not ret:
        print("🎉 Finished processing all frames")
        break
    
    frame_idx += 1
    print(f"Frame {frame_idx}")

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    color_mask_total = np.zeros((frame_h, frame_w), dtype=np.uint8)

    # Track each color independently
    for color_name, (lower, upper) in COLOR_RANGES.items():
        mask = cv2.inRange(hsv, lower, upper)

        if kernel is not None:
            mask = cv2.dilate(mask, kernel, iterations=1)

        color_mask_total = cv2.bitwise_or(color_mask_total, mask)

        # Extract only that color's bright pixels
        detected = cv2.bitwise_and(frame, frame, mask=mask)

        # Add to canvas (accumulate!)
        light_canvas += detected.astype(np.float32)

    # Optional fading so trails slowly disappear instead of building to white
    light_canvas *= GLOW_DECAY

    # Blend original + accumulated trails
    output_frame = cv2.addWeighted(
        frame.astype(np.float32), ALPHA,
        light_canvas, 1 - ALPHA,
        0
    ).astype(np.uint8)

    out.write(output_frame)

    # Debug windows
    cv2.imshow("Mask", color_mask_total)
    cv2.imshow("Light Trails", output_frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cap.release()
out.release()
cv2.destroyAllWindows()

print(f"✨ Saved light trail video: {OUTPUT_PATH}")
