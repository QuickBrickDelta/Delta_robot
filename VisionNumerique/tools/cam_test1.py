# live_square_gstreamer_5fps.py
import cv2
import numpy as np
import time

# ---------- Réglages ----------
WINDOW_SIZE = 900   # taille de la fenêtre carrée
FPS = 5             # 5 Hz comme demandé
# Choix du mode carré : "letterbox" (FOV max, bandes) ou "crop" (zoom centre)
SQUARE_MODE = "letterbox"

# Pipeline 1 : FOV maximal (plein capteur 16:9) -> downscale pour OpenCV
# Si c'est lourd chez toi, commente Pipeline 1 et décommente Pipeline 2.
pipeline = (
    "libcamerasrc ! "
    "video/x-raw,format=NV12,width=4608,height=2592,framerate=5/1 ! "
    "videoscale ! video/x-raw,width=1920,height=1080 ! "
    "queue max-size-buffers=1 leaky=downstream ! "
    "videoconvert ! video/x-raw,format=BGR ! "
    "appsink drop=true max-buffers=1 sync=false"
)

# Pipeline 2 (alternative) : 4:3 (plus de vertical) directement en capteur
# pipeline = (
#     "libcamerasrc ! "
#     "video/x-raw,format=NV12,width=2028,height=1520,framerate=5/1 ! "
#     "queue max-size-buffers=1 leaky=downstream ! "
#     "videoconvert ! video/x-raw,format=BGR ! "
#     "appsink drop=true max-buffers=1 sync=false"
# )

def to_square_letterbox(bgr, size):
    h, w = bgr.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = max(1, int(w*scale)), max(1, int(h*scale))
    resized = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((size, size, 3), dtype=np.uint8)
    x0, y0 = (size - nw)//2, (size - nh)//2
    canvas[y0:y0+nh, x0:x0+nw] = resized
    return canvas

def to_square_crop(bgr, size):
    h, w = bgr.shape[:2]
    side = min(h, w)
    x0, y0 = (w - side)//2, (h - side)//2
    crop = bgr[y0:y0+side, x0:x0+side]
    return cv2.resize(crop, (size, size), interpolation=cv2.INTER_AREA)

def open_cap():
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    return cap

cap = open_cap()
if not cap.isOpened():
    raise SystemExit("Erreur : impossible d'ouvrir la caméra via GStreamer.")

win = "LIVE carre"
cv2.namedWindow(win, cv2.WINDOW_NORMAL)
cv2.resizeWindow(win, WINDOW_SIZE, WINDOW_SIZE)
print("Commandes : q = quitter | c = basculer letterbox/crop")

last = time.time()
frames = 0
fps_est = 0.0

while True:
    ok, frame = cap.read()
    if not ok or frame is None:
        # tentative de reconnexion simple
        cap.release()
        time.sleep(0.1)
        cap = open_cap()
        continue

    # carré
    if SQUARE_MODE == "letterbox":
        square = to_square_letterbox(frame, WINDOW_SIZE)
        mode_text = "LETTERBOX (FOV max)"
    else:
        square = to_square_crop(frame, WINDOW_SIZE)
        mode_text = "CROP (centre)"

    # fps estimé & debug
    frames += 1
    now = time.time()
    if now - last >= 1.0:
        fps_est = frames / (now - last)
        frames = 0
        last = now

    txt = f"{mode_text} | src {frame.shape[1]}x{frame.shape[0]} | ~{fps_est:.1f} fps"
    cv2.putText(square, txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2, cv2.LINE_AA)
    cv2.imshow(win, square)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('c'):
        SQUARE_MODE = "crop" if SQUARE_MODE == "letterbox" else "letterbox"

cap.release()
cv2.destroyAllWindows()