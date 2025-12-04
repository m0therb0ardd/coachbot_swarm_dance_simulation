# # #!/usr/bin/env python3
# # import cv2
# # import numpy as np

# # GRID_IMAGE_PATH = "red_black_grid.jpg"
# # WIDTH  = 1280
# # HEIGHT = 720

# # SIGMA     = 150
# # STRENGTH  = 100

# # grid_orig = cv2.imread(GRID_IMAGE_PATH)
# # if grid_orig is None:
# #     raise RuntimeError("Failed to load image")
# # grid_orig = cv2.resize(grid_orig, (WIDTH, HEIGHT))

# # yy, xx = np.mgrid[0:HEIGHT, 0:WIDTH].astype(np.float32)

# # mouse_pos = (WIDTH//2, HEIGHT//2)

# # def on_mouse(event, x, y, flags, param):
# #     global mouse_pos
# #     if event == cv2.EVENT_MOUSEMOVE:
# #         mouse_pos = (x, y)

# # cv2.namedWindow("DistortGrid", cv2.WINDOW_NORMAL)
# # cv2.setWindowProperty("DistortGrid", cv2.WND_PROP_FULLSCREEN,
# #                      cv2.WINDOW_FULLSCREEN)
# # cv2.setMouseCallback("DistortGrid", on_mouse)

# # while True:
# #     px, py = mouse_pos
# #     img = grid_orig.copy()

# #     # compute warp
# #     dist = np.sqrt((xx - px)**2 + (yy - py)**2)
# #     influence = np.exp(-(dist**2)/(2*SIGMA**2))
# #     dx = (xx - px)/ (dist + 1e-6) * influence * STRENGTH
# #     dy = (yy - py)/ (dist + 1e-6) * influence * STRENGTH

# #     map_x = (xx + dx).astype(np.float32)
# #     map_y = (yy + dy).astype(np.float32)
# #     warped = cv2.remap(img, map_x, map_y,
# #                        interpolation=cv2.INTER_LINEAR,
# #                        borderMode=cv2.BORDER_REFLECT)

# #     cv2.imshow("DistortGrid", warped)
# #     key = cv2.waitKey(1) & 0xFF
# #     if key == ord('q'):
# #         break

# # cv2.destroyAllWindows()


#!/usr/bin/env python3
import cv2
import numpy as np
import math

# --- Configuration ---
GRID_IMAGE_PATH = "red_black_grid.jpg"  # use your grid image path
WIDTH  = 1280
HEIGHT = 720

SIGMA     = 150     # radius of influence
STRENGTH  = 120     # how much outward bulge

# Load image
img0 = cv2.imread(GRID_IMAGE_PATH)
if img0 is None:
    raise RuntimeError("Failed to load image: %s" % GRID_IMAGE_PATH)
img0 = cv2.resize(img0, (WIDTH, HEIGHT))
grid = img0.copy()

yy, xx = np.mgrid[0:HEIGHT, 0:WIDTH].astype(np.float32)

mouse_pos = (WIDTH//2, HEIGHT//2)

def on_mouse(event, x, y, flags, param):
    global mouse_pos
    if event == cv2.EVENT_MOUSEMOVE:
        mouse_pos = (x, y)

cv2.namedWindow("OutwardBulge", cv2.WINDOW_NORMAL)
cv2.setWindowProperty("OutwardBulge", cv2.WND_PROP_FULLSCREEN,
                     cv2.WINDOW_FULLSCREEN)
cv2.setMouseCallback("OutwardBulge", on_mouse)

while True:
    px, py = mouse_pos

    # compute influence field
    dx = np.zeros_like(xx)
    dy = np.zeros_like(yy)

    dist = np.sqrt((xx - px)**2 + (yy - py)**2)
    influence = np.exp(-(dist**2) / (2 * SIGMA**2))

    # outward displacement
    dx += (xx - px) / (dist + 1e-6) * influence * STRENGTH
    dy += (yy - py) / (dist + 1e-6) * influence * STRENGTH

    map_x = (xx + dx).astype(np.float32)
    map_y = (yy + dy).astype(np.float32)

    warped = cv2.remap(grid, map_x, map_y,
                       interpolation=cv2.INTER_LINEAR,
                       borderMode=cv2.BORDER_REFLECT)

    cv2.imshow("OutwardBulge", warped)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()


# #!/usr/bin/env python3
# import cv2
# import numpy as np
# import time

# # --- Configuration ---
# GRID_IMAGE_PATH = "red_black_grid.jpg"   #
# WIDTH  = 1280
# HEIGHT = 720

# MAX_BOTS     = 10
# SIGMA        = 120          # smaller radius for sharper ripple
# STRENGTH     = 80           # how far the ripple pushes
# WAVE_SPEED   = 40           # speed of the ripple outward
# WAVE_FALLOFF = 0.95         # decay each frame

# # Load grid image
# img0 = cv2.imread(GRID_IMAGE_PATH)
# if img0 is None:
#     raise RuntimeError("Failed to load image: %s" % GRID_IMAGE_PATH)
# img0 = cv2.resize(img0, (WIDTH, HEIGHT))
# grid = img0.copy()

# yy, xx = np.mgrid[0:HEIGHT, 0:WIDTH].astype(np.float32)

# # Track ripple events: each event = (px, py, start_time)
# ripples = []

# def add_ripple(px, py):
#     ripples.append({"px":px, "py":py, "t0":time.time(), "radius":0.0})

# # Mouse callback
# def on_mouse(event, x, y, flags, param):
#     if event == cv2.EVENT_MOUSEMOVE:
#         add_ripple(x, y)

# cv2.namedWindow("WaterRippleGrid", cv2.WINDOW_NORMAL)
# cv2.setWindowProperty("WaterRippleGrid", cv2.WND_PROP_FULLSCREEN,
#                      cv2.WINDOW_FULLSCREEN)
# cv2.setMouseCallback("WaterRippleGrid", on_mouse)

# while True:
#     # clean up old ripples
#     now = time.time()
#     ripples = [r for r in ripples if now - r["t0"] < 3.0]  # keep 3 seconds

#     # Start with base grid
#     img = grid.copy().astype(np.float32)

#     # Prepare maps
#     dx = np.zeros_like(xx)
#     dy = np.zeros_like(yy)

#     for r in ripples:
#         px = r["px"]
#         py = r["py"]
#         age = now - r["t0"]

#         # update radius of ripple
#         r["radius"] = age * WAVE_SPEED

#         # compute distance map
#         dist = np.sqrt((xx - px)**2 + (yy - py)**2)
#         # influence is high where distance ≈ radius
#         influence = np.exp(-((dist - r["radius"])**2) / (2 * (SIGMA**2)))
#         # radial outward displacement
#         dx += (xx - px) / (dist + 1e-6) * influence * STRENGTH
#         dy += (yy - py) / (dist + 1e-6) * influence * STRENGTH

#     map_x = (xx + dx).astype(np.float32)
#     map_y = (yy + dy).astype(np.float32)
#     warped = cv2.remap(img, map_x, map_y,
#                        interpolation=cv2.INTER_LINEAR,
#                        borderMode=cv2.BORDER_REFLECT)
#     warped = np.clip(warped, 0, 255).astype(np.uint8)

#     cv2.imshow("WaterRippleGrid", warped)
#     key = cv2.waitKey(1) & 0xFF
#     if key == ord('q'):
#         break

# cv2.destroyAllWindows()


