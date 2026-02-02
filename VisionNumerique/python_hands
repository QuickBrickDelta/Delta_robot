#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
python_hands.py
Détection de mains (MediaPipe Hands) sur un flux GStreamer à 5 FPS,
avec affichage carré (letterbox/crop) et overlay FPS.

À exécuter dans un venv Python 3.11 où mediapipe est installé :
    source ~/venvs/cam_311/bin/activate
    python python_hands.py

Dépendances :
    pip install mediapipe opencv-python numpy
"""

import os
import time
import signal
import sys
from typing import List, Tuple

import cv2
import numpy as np
import mediapipe as mp

# ---------- Réglages ----------
WINDOW_SIZE = 900          # taille de la fenêtre carrée
FPS = 5                    # 5 Hz
SQUARE_MODE = "letterbox"  # "letterbox" (FOV max) ou "crop" (zoom centre)
SHOW_BBOX = True           # affiche les boîtes englobantes
MODEL_COMPLEXITY = 0       # 0 = léger ; 1/2 = plus précis mais plus lourd

# Pipeline 1 : FOV maximal (plein capteur 16:9) -> downscale pour OpenCV
# Si c'est lourd sur ta machine, commente Pipeline 1 et décommente Pipeline 2.
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
def to_square_letterbox(bgr: np.ndarray, size: int):
    """Retourne (img_carrée, scale, x0, y0) pour la projection des points."""
    h, w = bgr.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
    resized = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((size, size, 3), dtype=np.uint8)
    x0, y0 = (size - nw) // 2, (size - nh) // 2
    canvas[y0:y0 + nh, x0:x0 + nw] = resized
    return canvas, scale, x0, y0

def to_square_crop(bgr: np.ndarray, size: int):
    """Retourne (img_carrée, side, x_off, y_off) pour la projection des points."""
    h, w = bgr.shape[:2]
    side = min(h, w)
    x0, y0 = (w - side) // 2, (h - side) // 2
    crop = bgr[y0:y0 + side, x0:x0 + side]
    resized = cv2.resize(crop, (size, size), interpolation=cv2.INTER_AREA)
    return resized, side, x0, y0

def project_points_to_square_letterbox(points_xy: List[Tuple[float, float]],
                                       scale: float, x0: int, y0: int):
    """Projette des points source (pixels) dans l'image carrée letterbox."""
    out = []
    for x, y in points_xy:
        out.append((int(x * scale + x0), int(y * scale + y0)))
    return out

def project_points_to_square_crop(points_xy: List[Tuple[float, float]],
                                  size: int, side: int, x_off: int, y_off: int):
    """Projette des points source (pixels) dans l'image carrée crop-centre."""
    scale = size / side
    out = []
    for x, y in points_xy:
        out.append((int((x - x_off) * scale), int((y - y_off) * scale)))
    return out

def bbox_from_points(pts: List[Tuple[int, int]], pad: int = 6,
                     clamp_w: int = WINDOW_SIZE, clamp_h: int = WINDOW_SIZE):
    """Boîte englobante avec marge et clamp aux bords."""
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    x1, y1 = max(0, min(xs) - pad), max(0, min(ys) - pad)
    x2, y2 = min(clamp_w - 1, max(xs) + pad), min(clamp_h - 1, max(ys) + pad)
    return x1, y1, x2, y2

def open_cap():
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    return cap

# ---------- MediaPipe Hands ----------
mp_hands = mp.solutions.hands

def make_hands(model_complexity: int):
    return mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=2,
        model_complexity=model_complexity,   # 0/1/2
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )

def main():
    global SQUARE_MODE, SHOW_BBOX, MODEL_COMPLEXITY

    # gestion Ctrl+C propre
    stop_flag = {"stop": False}
    def _sigint(_sig, _frm):
        stop_flag["stop"] = True
    signal.signal(signal.SIGINT, _sigint)

    hands = make_hands(MODEL_COMPLEXITY)

    cap = open_cap()
    if not cap.isOpened():
        raise SystemExit("Erreur : impossible d'ouvrir la caméra via GStreamer.")

    win = "LIVE carre - MediaPipe Hands"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, WINDOW_SIZE, WINDOW_SIZE)
    print("Commandes : q = quitter | c = letterbox/crop | m = complexité 0/1/2 | b = bbox on/off")

    last = time.time()
    frames = 0
    fps_est = 0.0

    while not stop_flag["stop"]:
        ok, frame = cap.read()
        if not ok or frame is None:
            # reconnexion simple si la caméra "rame"
            cap.release()
            time.sleep(0.1)
            cap = open_cap()
            continue

        h, w = frame.shape[:2]

        # Détection MediaPipe sur frame source (RGB)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(rgb)

        # Préparer l'image carrée et les paramètres de projection
        if SQUARE_MODE == "letterbox":
            square, scale, x0, y0 = to_square_letterbox(frame, WINDOW_SIZE)
            mode_text = "LETTERBOX (FOV max)"
        else:
            square, side, x_off, y_off = to_square_crop(frame, WINDOW_SIZE)
            mode_text = "CROP (centre)"

        # Dessiner les mains si présentes
        n_hands = 0
        if result.multi_hand_landmarks:
            n_hands = len(result.multi_hand_landmarks)
            # Par sécurité : handedness peut être None dans de rares cas
            handed_list = result.multi_handedness or [None] * n_hands

            for idx, hand_landmarks in enumerate(result.multi_hand_landmarks):
                handedness = handed_list[idx] if idx < len(handed_list) else None

                # Landmarks -> pixels dans l'image source
                pts_src = []
                for lm in hand_landmarks.landmark:
                    x_px = lm.x * w
                    y_px = lm.y * h
                    pts_src.append((x_px, y_px))

                # Projection vers l'image carrée
                if SQUARE_MODE == "letterbox":
                    pts_sq = project_points_to_square_letterbox(pts_src, scale, x0, y0)
                else:
                    pts_sq = project_points_to_square_crop(pts_src, WINDOW_SIZE, side, x_off, y_off)

                # Connexions standards (21 points)
                connections = mp_hands.HAND_CONNECTIONS

                # Segments
                for a, b in connections:
                    xa, ya = pts_sq[a]
                    xb, yb = pts_sq[b]
                    cv2.line(square, (xa, ya), (xb, yb), (0, 255, 255), 2, cv2.LINE_AA)

                # Points
                for x_s, y_s in pts_sq:
                    cv2.circle(square, (x_s, y_s), 3, (0, 140, 255), -1, lineType=cv2.LINE_AA)

                # Etiquette gauche/droite près du poignet (id=0)
                label = None
                if handedness and handedness.classification:
                    label = handedness.classification[0].label  # 'Left' ou 'Right'
                if label:
                    lx, ly = pts_sq[0]
                    cv2.putText(square, label, (lx + 8, ly - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2, cv2.LINE_AA)

                # Boîte englobante (optionnelle)
                if SHOW_BBOX:
                    x1, y1, x2, y2 = bbox_from_points(pts_sq, pad=8,
                                                      clamp_w=WINDOW_SIZE, clamp_h=WINDOW_SIZE)
                    cv2.rectangle(square, (x1, y1), (x2, y2), (90, 220, 90), 2)

        # FPS estimé & overlay
        frames += 1
        now = time.time()
        if now - last >= 1.0:
            fps_est = frames / (now - last)
            frames = 0
            last = now

        txt = f"{mode_text} | src {w}x{h} | ~{fps_est:.1f} fps | hands: {n_hands} | model={MODEL_COMPLEXITY} | bbox={'on' if SHOW_BBOX else 'off'}"
        cv2.putText(square, txt, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow(win, square)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('c'):
            SQUARE_MODE = "crop" if SQUARE_MODE == "letterbox" else "letterbox"
        elif key == ord('b'):
            SHOW_BBOX = not SHOW_BBOX
        elif key == ord('m'):
            # alterner 0 -> 1 -> 2 -> 0
            MODEL_COMPLEXITY = (MODEL_COMPLEXITY + 1) % 3
            hands.close()
            hands = make_hands(MODEL_COMPLEXITY)

    # nettoyage
    hands.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # Quelques variables d'environnement utiles pour GStreamer (optionnel)
    os.environ.setdefault("GST_DEBUG", "0")  # augmente (ex: "2") si besoin de debug
    try:
        main()
    except KeyboardInterrupt:
        pass