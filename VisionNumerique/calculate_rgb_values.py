#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
capture_and_probe_rgb.py
- Capture une image depuis la caméra (libcamerasrc/GStreamer) et l'affiche
- Clic gauche: affiche les valeurs RGB (et HSV/LAB) du pixel
- Glisser-déposer rectangle (clic gauche maintenu) : calcule la moyenne RGB/HSV/LAB de la zone
- 's' pour sauvegarder l'image capturée en PNG
- 'c' pour exporter un CSV (log des points/ROI mesurés)
- 'q' ou ESC pour quitter

Dépendances:
    pip install opencv-python numpy
"""

import os
import csv
import time
import cv2
import numpy as np
from datetime import datetime

# ---------- Réglages caméra ----------
# Résolution raisonnable (1080p) pour un calibrage confortable
WIDTH = 1920
HEIGHT = 1080
FPS = 5

# Pipeline GStreamer (adapte lens-position / af-mode si nécessaire)
PIPELINE = (
    'libcamerasrc af-mode=manual lens-position=3.4 ! '
    f'video/x-raw,format=NV12,width={WIDTH},height={HEIGHT},framerate={FPS}/1 ! '
    'videoconvert ! video/x-raw,format=BGR ! '
    'appsink drop=true max-buffers=1 sync=false'
)

# ---------- Variables globales UI ----------
win_name = "Capture & Probe RGB (clic pour lire pixel, drag rect pour moyenne)"
dragging = False
pt_start = None
pt_end = None
last_frame = None
snapshot = None
measure_log = []  # list of dicts: type 'pixel' ou 'roi', valeurs, positions, timestamp


def open_camera():
    cap = cv2.VideoCapture(PIPELINE, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise SystemExit("Erreur : impossible d'ouvrir la caméra via GStreamer. "
                         "Assure-toi que PipeWire/WirePlumber ne bloquent pas /dev/video*, "
                         "ou teste d'abord avec 'libcamera-hello'.")
    return cap


def safe_get_pixel(img, x, y):
    h, w = img.shape[:2]
    x = int(np.clip(x, 0, w - 1))
    y = int(np.clip(y, 0, h - 1))
    b, g, r = img[y, x].astype(int).tolist()
    # conversions
    hsv = cv2.cvtColor(np.uint8([[[b, g, r]]]), cv2.COLOR_BGR2HSV)[0, 0]
    lab = cv2.cvtColor(np.uint8([[[b, g, r]]]), cv2.COLOR_BGR2LAB)[0, 0]
    H, S, V = [int(v) for v in hsv]
    L, A, B = [int(v) for v in lab]
    return (r, g, b), (H, S, V), (L, A, B)


def avg_in_rect(img, x0, y0, x1, y1):
    x0, x1 = sorted([int(x0), int(x1)])
    y0, y1 = sorted([int(y0), int(y1)])
    h, w = img.shape[:2]
    x0 = int(np.clip(x0, 0, w - 1))
    x1 = int(np.clip(x1, 0, w - 1))
    y0 = int(np.clip(y0, 0, h - 1))
    y1 = int(np.clip(y1, 0, h - 1))
    if x1 <= x0 or y1 <= y0:
        return None

    roi = img[y0:y1, x0:x1]
    if roi.size == 0:
        return None

    mean_bgr = roi.mean(axis=(0, 1))  # BGR float
    mean_bgr = [float(v) for v in mean_bgr]
    # Conversions sur patch 1x1
    bgr_1x1 = np.uint8([[mean_bgr]])
    hsv_1x1 = cv2.cvtColor(bgr_1x1, cv2.COLOR_BGR2HSV)[0, 0]
    lab_1x1 = cv2.cvtColor(bgr_1x1, cv2.COLOR_BGR2LAB)[0, 0]
    mean_rgb = [mean_bgr[2], mean_bgr[1], mean_bgr[0]]
    mean_hsv = [float(hsv_1x1[0]), float(hsv_1x1[1]), float(hsv_1x1[2])]
    mean_lab = [float(lab_1x1[0]), float(lab_1x1[1]), float(lab_1x1[2])]
    return {
        "rgb": mean_rgb,
        "hsv": mean_hsv,
        "lab": mean_lab,
        "rect": (x0, y0, x1, y1)
    }


def on_mouse(event, x, y, flags, param):
    global dragging, pt_start, pt_end, last_frame, snapshot, measure_log

    if last_frame is None:
        return

    if event == cv2.EVENT_LBUTTONDOWN:
        dragging = True
        pt_start = (x, y)
        pt_end = (x, y)

    elif event == cv2.EVENT_MOUSEMOVE and dragging:
        pt_end = (x, y)

    elif event == cv2.EVENT_LBUTTONUP:
        dragging = False
        pt_end = (x, y)
        # Si déplacement minimal -> clic simple (pixel)
        if pt_start is not None:
            dx = abs(pt_end[0] - pt_start[0])
            dy = abs(pt_end[1] - pt_start[1])
            if dx < 5 and dy < 5:
                # Lecture pixel
                rgb, hsv, lab = safe_get_pixel(last_frame, x, y)
                print(f"[PIXEL] (x={x}, y={y})  RGB={rgb}  HSV={hsv}  LAB={lab}")
                measure_log.append({
                    "type": "pixel",
                    "x": int(x),
                    "y": int(y),
                    "r": int(rgb[0]),
                    "g": int(rgb[1]),
                    "b": int(rgb[2]),
                    "H": int(hsv[0]),
                    "S": int(hsv[1]),
                    "V": int(hsv[2]),
                    "L": int(lab[0]),
                    "A": int(lab[1]),
                    "B": int(lab[2]),
                    "timestamp": time.time()
                })
            else:
                # ROI moyenne
                stats = avg_in_rect(last_frame, pt_start[0], pt_start[1], pt_end[0], pt_end[1])
                if stats is not None:
                    (x0, y0, x1, y1) = stats["rect"]
                    rgb = [round(v, 1) for v in stats["rgb"]]
                    hsv = [round(v, 1) for v in stats["hsv"]]
                    lab = [round(v, 1) for v in stats["lab"]]
                    print(f"[ROI] ({x0},{y0})-({x1},{y1})  RGB(avg)={rgb}  HSV(avg)={hsv}  LAB(avg)={lab}")
                    measure_log.append({
                        "type": "roi",
                        "x0": int(x0), "y0": int(y0), "x1": int(x1), "y1": int(y1),
                        "r": float(rgb[0]), "g": float(rgb[1]), "b": float(rgb[2]),
                        "H": float(hsv[0]), "S": float(hsv[1]), "V": float(hsv[2]),
                        "L": float(lab[0]), "A": float(lab[1]), "B": float(lab[2]),
                        "timestamp": time.time()
                    })
        pt_start = None
        pt_end = None


def draw_overlay(img):
    """Dessine le rectangle de sélection en cours."""
    overlay = img.copy()
    if dragging and pt_start and pt_end:
        x0, y0 = pt_start
        x1, y1 = pt_end
        cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 255, 255), 2)
    # Texte aide
    cv2.putText(overlay, "Clic: pixel RGB/HSV/LAB | Drag: ROI moyenne | s: save PNG | c: export CSV | q: quitter",
                (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 255, 20), 2, cv2.LINE_AA)
    return overlay


def save_png(img):
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = f"capture_{ts}.png"
    cv2.imwrite(name, img)
    print(f"Image sauvegardee: {name}")
    return name


def export_csv(log, image_name):
    if not log:
        print("Aucune mesure a exporter.")
        return None
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = f"measures_{ts}.csv"
    with open(name, "w", newline="") as f:
        writer = csv.writer(f)
        # En-tête
        writer.writerow([
            "type", "x", "y", "x0", "y0", "x1", "y1",
            "R", "G", "B", "H", "S", "V", "L", "A", "B_lab",
            "image", "timestamp"
        ])
        for m in log:
            if m["type"] == "pixel":
                writer.writerow([
                    "pixel", m["x"], m["y"], "", "", "", "",
                    m["r"], m["g"], m["b"], m["H"], m["S"], m["V"], m["L"], m["A"], m["B"],
                    image_name, m["timestamp"]
                ])
            else:
                writer.writerow([
                    "roi", "", "", m["x0"], m["y0"], m["x1"], m["y1"],
                    m["r"], m["g"], m["b"], m["H"], m["S"], m["V"], m["L"], m["A"], m["B"],
                    image_name, m["timestamp"]
                ])
    print(f"CSV exporte: {name}")
    return name


def main():
    global last_frame, snapshot
    os.environ.setdefault("GST_DEBUG", "0")

    # 1) Ouvrir caméra
    cap = open_camera()

    # 2) Capturer 1 frame (laisser passer quelques frames pour expos stable)
    warmup = 10
    for _ in range(warmup):
        cap.read()

    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        raise SystemExit("Capture echouee (frame vide).")

    snapshot = frame.copy()
    last_frame = frame

    # 3) UI
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 1280, int(1280 * (frame.shape[0] / frame.shape[1])))
    cv2.setMouseCallback(win_name, on_mouse)

    while True:
        disp = draw_overlay(last_frame)
        # Feedback en direct sous la souris
        x, y = tuple(int(v) for v in cv2.getWindowImageRect(win_name)[:2])
        cv2.imshow(win_name, disp)
        k = cv2.waitKey(20) & 0xFF
        if k in (27, ord('q')):  # ESC ou q
            break
        elif k == ord('s'):
            save_png(snapshot)
        elif k == ord('c'):
            # Exporter les mesures en CSV
            export_csv(measure_log, image_name="(last_saved_png_or_ram)")
        # 'r' pour réinitialiser les mesures
        elif k == ord('r'):
            measure_log.clear()
            print("Mesures effacees.")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass