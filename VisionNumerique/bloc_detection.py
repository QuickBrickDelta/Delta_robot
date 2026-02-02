#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
python_live_square.py
Affichage live d'un flux GStreamer à 5 FPS, en carré (letterbox/crop) + overlay FPS.

Exécution (dans ton venv):
    source ~/venvs/cam_311/bin/activate
    python python_live_square.py

Dépendances:
    pip install opencv-python numpy
"""

import os
import time
import signal
from typing import Tuple

import cv2
import numpy as np

# ---------- Réglages ----------
WINDOW_SIZE = 900          # taille de la fenêtre carrée
FPS = 5                    # 5 Hz
SQUARE_MODE = "letterbox"  # "letterbox" (FOV max) ou "crop" (zoom centre)

# Pipeline 1 : FOV maximal (plein capteur 16:9) -> downscale pour OpenCV
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

# ---------- Fonctions utilitaires d'affichage carré ----------
def to_square_letterbox(bgr: np.ndarray, size: int) -> Tuple[np.ndarray, float, int, int]:
    """Retourne (img_carrée, scale, x0, y0)."""
    h, w = bgr.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
    resized = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((size, size, 3), dtype=np.uint8)
    x0, y0 = (size - nw) // 2, (size - nh) // 2
    canvas[y0:y0 + nh, x0:x0 + nw] = resized
    return canvas, scale, x0, y0

def to_square_crop(bgr: np.ndarray, size: int) -> Tuple[np.ndarray, int, int, int]:
    """Retourne (img_carrée, side, x_off, y_off)."""
    h, w = bgr.shape[:2]
    side = min(h, w)
    x0, y0 = (w - side) // 2, (h - side) // 2
    crop = bgr[y0:y0 + side, x0:x0 + side]
    resized = cv2.resize(crop, (size, size), interpolation=cv2.INTER_AREA)
    return resized, side, x0, y0

def open_cap():
    return cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

def main():
    global SQUARE_MODE

    # gestion Ctrl+C propre
    stop_flag = {"stop": False}
    def _sigint(_sig, _frm):
        stop_flag["stop"] = True
    signal.signal(signal.SIGINT, _sigint)

    cap = open_cap()
    if not cap.isOpened():
        raise SystemExit("Erreur : impossible d'ouvrir la caméra via GStreamer.")

    win = "LIVE carre"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, WINDOW_SIZE, WINDOW_SIZE)
    print("Commandes : q = quitter | c = letterbox/crop")

    last = time.time()
    frames = 0
    fps_est = 0.0

    target_dt = 1.0 / float(FPS)

    while not stop_flag["stop"]:
        t0 = time.time()

        ok, frame = cap.read()
        if not ok or frame is None:
            # reconnexion simple si la caméra "rame"
            cap.release()
            time.sleep(0.1)
            cap = open_cap()
            continue

        h, w = frame.shape[:2]

        # Image carrée
        if SQUARE_MODE == "letterbox":
            square, *_ = to_square_letterbox(frame, WINDOW_SIZE)
            mode_text = "LETTERBOX (FOV max)"
        else:
            square, *_ = to_square_crop(frame, WINDOW_SIZE)
            mode_text = "CROP (centre)"

        # FPS estimé
        frames += 1
        now = time.time()
        if now - last >= 1.0:
            fps_est = frames / (now - last)
            frames = 0
            last = now

        txt = f"{mode_text} | src {w}x{h} | ~{fps_est:.1f} fps"
        cv2.putText(square, txt, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow(win, square)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('c'):
            SQUARE_MODE = "crop" if SQUARE_MODE == "letterbox" else "letterbox"

        # timing ~5 Hz (optionnel, utile si le pipeline envoie plus)
        dt = time.time() - t0
        if dt < target_dt:
            time.sleep(target_dt - dt)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    os.environ.setdefault("GST_DEBUG", "0")
    try:
        main()
    except KeyboardInterrupt:
        pass
