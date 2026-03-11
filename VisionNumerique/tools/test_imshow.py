# live_square_only_minimal.py
import time
import cv2
import numpy as np
from picamera2 import Picamera2

LIVE_W, LIVE_H = 1280, 720   # léger pour imshow
FPS = 5                      # 5 Hz comme demandé
WIN = 900                    # fenêtre carrée
SQUARE_MODE = "letterbox"    # "letterbox" (FOV max) ou "crop"

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

picam2 = Picamera2()
cfg = picam2.create_preview_configuration(
    main={"size": (LIVE_W, LIVE_H), "format": "RGB888"},
    buffer_count=4,
)
cfg["controls"] = {"FrameDurationLimits": (int(1e9//FPS), int(1e9//FPS))}
picam2.configure(cfg)
picam2.start()
time.sleep(0.2)

win = "LIVE carre"
cv2.namedWindow(win, cv2.WINDOW_NORMAL)
cv2.resizeWindow(win, WIN, WIN)

print("Commandes : c = basculer letterbox/crop | q = quitter")
print("⚠️ Clique une fois dans la fenêtre pour le focus si tu veux utiliser le clavier.")

n, t0 = 0, time.time()
fps_est = 0.0

while True:
    # 1) Capture (RGB) -> BGR
    rgb = picam2.capture_array()
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    meanv = float(bgr.mean())

    # 2) Carré
    if SQUARE_MODE == "letterbox":
        square = to_square_letterbox(bgr, WIN)
        mode_text = "LETTERBOX (FOV max)"
    else:
        square = to_square_crop(bgr, WIN)
        mode_text = "CROP (zoom centre)"

    # 3) Overlay debug
    n += 1
    if time.time() - t0 >= 1.0:
        fps_est = n / (time.time() - t0)
        n, t0 = 0, time.time()

    info = f"{mode_text} | src {bgr.shape[1]}x{bgr.shape[0]} | mean {meanv:.1f} | ~{fps_est:.1f} fps"
    cv2.putText(square, info, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2, cv2.LINE_AA)

    # 4) Affichage
    cv2.imshow(win, square)

    # 5) Clavier
    key = cv2.waitKey(20) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('c'):
        SQUARE_MODE = "crop" if SQUARE_MODE == "letterbox" else "letterbox"

cv2.destroyAllWindows()
picam2.stop()