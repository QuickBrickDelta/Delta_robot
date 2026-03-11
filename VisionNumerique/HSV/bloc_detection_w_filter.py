#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bloc_detection_w_filter.py
Détection de blocs avec soustraction de lumière ambiante (Flash Filter).
Touches :
  - 'f' : Activer/Désactiver le filtre LED
  - 'q' : Quitter
  - 'c' : Changer mode carré (letterbox/crop)
  - '1 à 6' : Ajuster les seuils en direct
"""

import os
import time
import signal
import json
from pathlib import Path
from typing import Tuple, Dict, List

import cv2
import numpy as np

# ---------- Réglages ----------
WINDOW_SIZE = 900
FPS = 5
SQUARE_MODE = "letterbox"
SHOW_VIS = True

# Seuils par défaut (Relaxés pour le vert)
MIN_AREA_PX = 2000
COLOR_STD_THRESH = 40.0
DOMINANT_FRAC_THRESH = 0.15
RECT_ANGLE_TOL_DEG = 20.0
RECT_AREA_RATIO_MIN = 0.70
ASPECT_RATIO_RANGE = None

# Dossiers
_THIS_DIR = Path(__file__).resolve().parent
FLASH_DATA_DIR = _THIS_DIR / "flash_data"
HSV_JSON_PATH = _THIS_DIR / "hsv_ranges.json"
H_PATH = _THIS_DIR.parent / "calibration" / "homography_plane.npz"

# Pipeline GStreamer
pipeline = (
    'libcamerasrc af-mode=manual lens-position=3.4 ! '
    'video/x-raw,format=NV12,width=4608,height=2592,framerate=5/1 ! '
    'videoscale ! video/x-raw,width=1920,height=1080 ! '
    'queue max-size-buffers=1 leaky=downstream ! '
    'videoconvert ! video/x-raw,format=BGR ! '
    'appsink drop=true max-buffers=1 sync=false'
)

# ---------- Logique du Filtre Flash ----------

def load_flash_filter(data_dir: Path):
    params_path = data_dir / "flash_params.npz"
    if not params_path.exists():
        print(f"[WARN] Paramètres de filtre introuvables dans {data_dir}")
        return None
    try:
        data = np.load(str(params_path))
        return {
            "ambient_ref": cv2.imread(str(data_dir / "ambient_ref.png")).astype(np.float32),
            "led_contrib": cv2.imread(str(data_dir / "led_contrib.png")).astype(np.float32),
            "led_mask": cv2.imread(str(data_dir / "led_mask.png"), cv2.IMREAD_GRAYSCALE),
            "alpha": float(data["alpha"]),
            "beta": float(data["beta"])
        }
    except Exception as e:
        print(f"Erreur chargement filtre: {e}")
        return None

def apply_flash_filter(frame: np.ndarray, f_data: dict) -> np.ndarray:
    img_f = frame.astype(np.float32)
    res = img_f - (f_data["alpha"] * f_data["ambient_ref"])
    res = (res / (f_data["led_contrib"] + 1e-6)) * 255.0
    res = np.clip(res, 0, 255).astype(np.uint8)
    if f_data["led_mask"] is not None:
        res = cv2.bitwise_and(res, res, mask=f_data["led_mask"])
    return res

# ---------- Fonctions de Détection (Issues de ton bloc_detection.py) ----------

def load_color_ranges(json_path: Path):
    if not json_path.exists():
        raise FileNotFoundError(f"Fichier {json_path} manquant.")
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    ranges_out = {}
    for color_name, info in data.get("colors", {}).items():
        color_list = []
        for r in info.get("ranges", []):
            color_list.append((np.array(r[0], dtype=np.uint8), np.array(r[1], dtype=np.uint8)))
        ranges_out[color_name] = color_list
    return ranges_out

def _is_rectangle_approx(cnt, angle_tol_deg, area_ratio_min):
    if len(cnt) < 4: return False
    hull = cv2.convexHull(cnt)
    peri = cv2.arcLength(hull, True)
    approx = cv2.approxPolyDP(hull, 0.03 * peri, True)
    if len(approx) != 4 or not cv2.isContourConvex(approx): return False
    rect = cv2.minAreaRect(approx)
    rect_area = max(rect[1][0] * rect[1][1], 1e-6)
    if cv2.contourArea(approx) / rect_area < area_ratio_min: return False
    return True

def _region_uniformity_lab(bgr, cnt):
    h, w = bgr.shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(mask, [cnt], -1, 255, thickness=cv2.FILLED)
    x, y, bw, bh = cv2.boundingRect(cnt)
    roi = bgr[y:y+bh, x:x+bw]
    mask_roi = mask[y:y+bh, x:x+bw]
    if roi.size == 0 or np.count_nonzero(mask_roi) < 50: return 1e9, 0.0
    lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
    mean, std = cv2.meanStdDev(lab, mask=mask_roi)
    color_std_ab = (float(std[1][0])**2 + float(std[2][0])**2) ** 0.5
    return color_std_ab, float(np.count_nonzero(mask_roi)) / (bw * bh + 1e-6)

def _dominant_color_fraction_hsv(bgr, cnt, ranges):
    h, w = bgr.shape[:2]
    mask_cnt = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(mask_cnt, [cnt], -1, 255, thickness=cv2.FILLED)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    total = np.count_nonzero(mask_cnt)
    if total == 0: return 0.0
    merged = None
    for lo, hi in ranges:
        mm = cv2.inRange(hsv, lo, hi)
        merged = mm if merged is None else cv2.bitwise_or(merged, mm)
    inside = cv2.bitwise_and(merged, merged, mask=mask_cnt)
    return float(np.count_nonzero(inside)) / float(total)

def detect_colored_blocks(bgr, color_ranges, min_area_px, color_std_thresh, dom_frac_thresh, rect_angle_tol, rect_area_ratio, aspect_ratio):
    out = []
    h, w = bgr.shape[:2]
    blurred = cv2.GaussianBlur(bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    kernel_open = np.ones((3, 3), np.uint8)

    for color_name, ranges in color_ranges.items():
        mask = np.zeros((h, w), dtype=np.uint8)
        for lo, hi in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for c in cnts:
            area = cv2.contourArea(c)
            if area < min_area_px: continue
            if not _is_rectangle_approx(c, rect_angle_tol, rect_area_ratio): continue
            std_ab, _ = _region_uniformity_lab(bgr, c)
            if std_ab > color_std_thresh: continue
            frac = _dominant_color_fraction_hsv(bgr, c, ranges)
            if frac < dom_frac_thresh: continue
            
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            out.append({"color": color_name, "center": rect[0], "angle_deg": rect[2], "box": box, "area": area})
    return out

# ---------- Homographie & Affichage ----------

def load_homography(path):
    if not Path(path).exists(): return None
    data = np.load(str(path), allow_pickle=True)
    return data["H"], data["cam_center_world"]

def to_square_letterbox(bgr, size):
    h, w = bgr.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = int(w * scale), int(h * scale)
    resized = cv2.resize(bgr, (nw, nh))
    canvas = np.zeros((size, size, 3), dtype=np.uint8)
    x0, y0 = (size - nw) // 2, (size - nh) // 2
    canvas[y0:y0+nh, x0:x0+nw] = resized
    return canvas, scale, x0, y0

# ---------- Main ----------

def main():
    global SQUARE_MODE, SHOW_VIS, MIN_AREA_PX, COLOR_STD_THRESH, DOMINANT_FRAC_THRESH, RECT_ANGLE_TOL_DEG, RECT_AREA_RATIO_MIN

    COLOR_RANGES = load_color_ranges(HSV_JSON_PATH)
    flash_filter_data = load_flash_filter(FLASH_DATA_DIR)
    use_filter = False
    
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    win = "Bloc Detection w/ Filter"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    while True:
        ok, frame = cap.read()
        if not ok: break

        # Étape Filtre
        work_frame = apply_flash_filter(frame, flash_filter_data) if (use_filter and flash_filter_data) else frame
        
        # Détection
        detections = detect_colored_blocks(work_frame, COLOR_RANGES, MIN_AREA_PX, COLOR_STD_THRESH, DOMINANT_FRAC_THRESH, RECT_ANGLE_TOL_DEG, RECT_AREA_RATIO_MIN, ASPECT_RATIO_RANGE)

        # Affichage
        square, scale, x0, y0 = to_square_letterbox(work_frame, WINDOW_SIZE)
        status = "FILTRE: ON (LED ONLY)" if use_filter else "FILTRE: OFF (BRUT)"
        cv2.putText(square, f"{status} | {len(detections)} blocs", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        for det in detections:
            box_disp = (det["box"] * scale + [x0, y0]).astype(np.int32)
            cv2.drawContours(square, [box_disp], 0, (255, 255, 255), 2)
            cv2.putText(square, det["color"], (box_disp[0][0], box_disp[0][1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cv2.imshow(win, square)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): break
        elif key == ord('f'): use_filter = not use_filter
        elif key == ord('c'): SQUARE_MODE = "crop" if SQUARE_MODE == "letterbox" else "letterbox"
        elif key == ord('2'): COLOR_STD_THRESH += 2
        elif key == ord('"'): COLOR_STD_THRESH -= 2

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()