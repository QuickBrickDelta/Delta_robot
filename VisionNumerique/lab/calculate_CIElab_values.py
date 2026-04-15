#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
capture_and_calibrate_lab.py
- Flux caméra (GStreamer) et UI de mesure pour calibrer les couleurs en CIELAB (float32).
- Clic: lit pixel (RGB/HSV/Lab) | Drag: ROI -> moyenne/std sur L*, a*, b*.
- Appuie sur 1..9 pour assigner la DERNIÈRE ROI mesurée à un nom de couleur.
- 'e' : exporte lab_colors.json (centres chromatiques + thr_ab proposés + L_range).
- 's' : sauvegarde PNG | 'r' : reset mesures | 'q'/ESC : quitter.

Dépendances:
    pip install opencv-python numpy
"""

import os
import json
import time
import cv2
import numpy as np
from datetime import datetime
from pathlib import Path

# ---------- Caméra ----------
WIDTH = 1920
HEIGHT = 1080
FPS = 5
PIPELINE = (
    'libcamerasrc af-mode=manual lens-position=3.4 ! '
    f'video/x-raw,format=NV12,width={WIDTH},height={HEIGHT},framerate={FPS}/1 ! '
    'videoconvert ! video/x-raw,format=BGR ! '
    'appsink drop=true max-buffers=1 sync=false'
)

# ---------- UI ----------
WIN = "Calibration CIELAB (clic/drag) | 1..9: assigne couleur | e: export JSON"
dragging = False
pt0 = None
pt1 = None
last_frame = None
last_roi_stats = None  # dict avec moy/std L*,a*,b*
assigned = {}          # nom_couleur -> dict spec
measure_log = []

# Associer des touches aux noms (adapte librement)
KEY_TO_COLOR = {
    ord('1'): "red",
    ord('2'): "green",
    ord('3'): "blue",
    ord('4'): "yellow",
    ord('5'): "orange",
    ord('6'): "purple",
    ord('7'): "white",
    ord('8'): "black",
    ord('9'): "custom",
}

# Paramètres d'estimation
K_THR = 2.5  # thr_ab = K_THR * sqrt(std_a^2 + std_b^2)
L_RANGE_SIGMA = 2.0
OUT_DIR = Path(__file__).resolve().parent

def open_camera():
    cap = cv2.VideoCapture(PIPELINE, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise SystemExit("Erreur : impossible d'ouvrir la caméra via GStreamer.")
    return cap

def roi_stats_lab(bgr, x0, y0, x1, y1):
    x0, x1 = sorted([int(x0), int(x1)])
    y0, y1 = sorted([int(y0), int(y1)])
    h, w = bgr.shape[:2]
    x0 = int(np.clip(x0, 0, w - 1))
    x1 = int(np.clip(x1, 0, w - 1))
    y0 = int(np.clip(y0, 0, h - 1))
    y1 = int(np.clip(y1, 0, h - 1))
    if x1 <= x0 or y1 <= y0:
        return None

    roi_bgr = bgr[y0:y1, x0:x1]
    if roi_bgr.size == 0:
        return None

    # Lab float32 (L*[0..100], a*,b*[-127..127])
    roi_lab = cv2.cvtColor(roi_bgr.astype(np.float32)/255.0, cv2.COLOR_BGR2LAB)
    L = roi_lab[:, :, 0].astype(np.float32)
    A = roi_lab[:, :, 1].astype(np.float32)
    B = roi_lab[:, :, 2].astype(np.float32)

    stats = {
        "L_mean": float(np.mean(L)), "L_std": float(np.std(L)),
        "a_mean": float(np.mean(A)), "a_std": float(np.std(A)),
        "b_mean": float(np.mean(B)), "b_std": float(np.std(B)),
        "rect": (x0, y0, x1, y1),
    }
    return stats

def on_mouse(event, x, y, flags, param):
    global dragging, pt0, pt1, last_roi_stats, last_frame, measure_log
    if last_frame is None:
        return

    if event == cv2.EVENT_LBUTTONDOWN:
        dragging = True
        pt0 = (x, y)
        pt1 = (x, y)

    elif event == cv2.EVENT_MOUSEMOVE and dragging:
        pt1 = (x, y)

    elif event == cv2.EVENT_LBUTTONUP:
        dragging = False
        pt1 = (x, y)
        dx, dy = abs(pt1[0]-pt0[0]), abs(pt1[1]-pt0[1])
        if dx < 5 and dy < 5:
            # lecture pixel
            b, g, r = last_frame[y, x].astype(int).tolist()
            hsv = cv2.cvtColor(np.uint8([[[b, g, r]]]), cv2.COLOR_BGR2HSV)[0, 0]
            lab8 = cv2.cvtColor(np.uint8([[[b, g, r]]]), cv2.COLOR_BGR2LAB)[0, 0]
            lab32 = cv2.cvtColor(np.float32([[[b, g, r]]])/255.0, cv2.COLOR_BGR2LAB)[0, 0]
            print(f"[PIXEL] (x={x}, y={y})  RGB=({r},{g},{b})  HSV={tuple(int(v) for v in hsv)}  "
                  f"LAB8={tuple(int(v) for v in lab8)}  LAB32=({lab32[0]:.2f},{lab32[1]:.2f},{lab32[2]:.2f})")
            measure_log.append({"type":"pixel","x":x,"y":y,"rgb":(r,g,b),
                                "lab32":(float(lab32[0]),float(lab32[1]),float(lab32[2])),
                                "t":time.time()})
        else:
            # ROI
            stats = roi_stats_lab(last_frame, pt0[0], pt0[1], pt1[0], pt1[1])
            if stats is not None:
                last_roi_stats = stats
                Lm, Ls = stats["L_mean"], stats["L_std"]
                am, as_ = stats["a_mean"], stats["a_std"]
                bm, bs_ = stats["b_mean"], stats["b_std"]
                print(f"[ROI] L*={Lm:.2f}±{Ls:.2f}  a*={am:.2f}±{as_:.2f}  b*={bm:.2f}±{bs_:.2f} "
                      f"rect={stats['rect']}")
                measure_log.append({"type":"roi","stats":stats,"t":time.time()})
        pt0 = pt1 = None

def draw_overlay(img):
    overlay = img.copy()
    if dragging and pt0 and pt1:
        cv2.rectangle(overlay, pt0, pt1, (0,255,255), 2)
    cv2.putText(overlay, "Drag: ROI Lab | 1..9: assigne couleur | e: export JSON | s: PNG | r: reset | q: quit",
                (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20,255,20), 2, cv2.LINE_AA)
    if last_roi_stats is not None:
        Lm, Ls = last_roi_stats["L_mean"], last_roi_stats["L_std"]
        am, as_ = last_roi_stats["a_mean"], last_roi_stats["a_std"]
        bm, bs_ = last_roi_stats["b_mean"], last_roi_stats["b_std"]
        txt = f"L*={Lm:.1f}±{Ls:.1f}  a*={am:.1f}±{as_:.1f}  b*={bm:.1f}±{bs_:.1f}"
        cv2.putText(overlay, txt, (15, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2, cv2.LINE_AA)
        cv2.putText(overlay, txt, (15, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 1, cv2.LINE_AA)
    return overlay

def suggest_thr_ab(a_std, b_std, k=K_THR):
    # ΔE_ab local ~ bruit -> seuil = k * std_chroma
    return float(k * np.sqrt(a_std*a_std + b_std*b_std))

def suggest_L_range(L_mean, L_std, nsig=L_RANGE_SIGMA):
    lo = float(max(0.0, L_mean - nsig*L_std))
    hi = float(min(100.0, L_mean + nsig*L_std))
    if hi < lo:  # garde sûre
        lo, hi = 0.0, 100.0
    return (lo, hi)

def main():
    global last_frame, last_roi_stats, assigned
    os.environ.setdefault("GST_DEBUG", "0")

    cap = open_camera()
    # petit warmup
    for _ in range(8):
        cap.read()
    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        raise SystemExit("Capture échouée.")

    last_frame = frame.copy()
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1280, int(1280 * (frame.shape[0]/frame.shape[1])))
    cv2.setMouseCallback(WIN, on_mouse)

    print("Commandes :")
    print("  - Drag: dessine une ROI -> calcule moyenne & std en Lab (float32)")
    print("  - 1..9: assigne la dernière ROI au nom de couleur correspondant")
    print("  - e   : exporte lab_colors.json")
    print("  - s   : save PNG | r : reset | q/ESC : quitter")

    png_name = None

    while True:
        disp = draw_overlay(last_frame)
        cv2.imshow(WIN, disp)
        k = cv2.waitKey(20) & 0xFF

        if k in (27, ord('q')):  # ESC ou q
            break

        if k == ord('s'):
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            png_path = OUT_DIR / f"calib_{ts}.png"
            cv2.imwrite(str(png_path), last_frame)
            print("PNG sauvegardé:", png_path)

        elif k == ord('r'):
            last_roi_stats = None
            assigned.clear()
            print("Mesures et affectations effacées.")

        elif k == ord('e'):
            if not assigned:
                print("Rien à exporter (assigne d'abord au moins une couleur avec 1..9).")
                continue
            out = {}
            for cname, s in assigned.items():
                Lm, Ls = s["L_mean"], s["L_std"]
                am, as_ = s["a_mean"], s["a_std"]
                bm, bs_ = s["b_mean"], s["b_std"]
                thr = suggest_thr_ab(as_, bs_)
                Lrng = suggest_L_range(Lm, Ls)
                out[cname] = {
                    "lab": [Lm, am, bm],
                    "thr_ab": thr,
                    "L_range": [float(Lrng[0]), float(Lrng[1])]
                }
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            json_name = OUT_DIR / f"lab_colors_{ts}.json"
            lab_colors_path = OUT_DIR / "lab_colors.json"
            legacy_lab_colors_path = OUT_DIR.parent / "lab_colors.json"
            with open(lab_colors_path, "w", encoding="utf-8") as f:
                json.dump(out, f, indent=2)
            with open(legacy_lab_colors_path, "w", encoding="utf-8") as f:
                json.dump(out, f, indent=2)
            with open(json_name, "w", encoding="utf-8") as f:
                json.dump(out, f, indent=2)
            print("Export JSON ->", lab_colors_path, "et", legacy_lab_colors_path, "(archive:", json_name, ")")
            for kcolor, spec in out.items():
                print(f"  {kcolor}: lab={tuple(round(v,2) for v in spec['lab'])}  thr_ab={spec['thr_ab']:.2f}  L*={spec['L_range']}")

        elif k in KEY_TO_COLOR and last_roi_stats is not None:
            cname = KEY_TO_COLOR[k]
            assigned[cname] = dict(last_roi_stats)
            print(f"Couleur '{cname}' affectée à la dernière ROI.")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass