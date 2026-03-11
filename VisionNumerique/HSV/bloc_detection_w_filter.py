#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

# Filtres (tes paramètres relaxés pour le vert)
MIN_AREA_PX = 2000
COLOR_STD_THRESH = 40.0
DOMINANT_FRAC_THRESH = 0.15
RECT_ANGLE_TOL_DEG = 20.0
RECT_AREA_RATIO_MIN = 0.70
ASPECT_RATIO_RANGE = None

_THIS_DIR = Path(__file__).resolve().parent
HSV_JSON_PATH = _THIS_DIR / "hsv_ranges.json"
H_PATH = _THIS_DIR.parent / "calibration" / "homography_plane.npz"

pipeline = (
    'libcamerasrc af-mode=manual lens-position=3.4 ! '
    'video/x-raw,format=NV12,width=4608,height=2592,framerate=5/1 ! '
    'videoscale ! video/x-raw,width=1920,height=1080 ! '
    'videoconvert ! video/x-raw,format=BGR ! '
    'appsink drop=true max-buffers=1 sync=false'
)

# ---------- Calibration & Homographie ----------

def load_homography(path=H_PATH):
    if not Path(path).exists(): return None
    data = np.load(str(path), allow_pickle=True)
    return data["H"], data["cam_center_world"].reshape(2)

def pix_to_world_cm(pt_uv, H):
    u, v = float(pt_uv[0]), float(pt_uv[1])
    ph = H @ np.array([u, v, 1.0], dtype=np.float64)
    if abs(ph[2]) < 1e-9: return None
    return (ph[0] / ph[2], ph[1] / ph[2])

# ---------- Logique Filtre Flash ----------

def compute_filter(img_off, img_on):
    off_f = img_off.astype(np.float32)
    on_f = img_on.astype(np.float32)
    # Contribution réelle des LEDs
    led_contrib = on_f - off_f
    diff_gray = cv2.cvtColor(np.abs(led_contrib).astype(np.uint8), cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(diff_gray, 8, 255, cv2.THRESH_BINARY)
    return {
        "ambient_ref": off_f,
        "led_contrib": led_contrib,
        "led_mask": mask,
        "alpha": 0.95  # Facteur de soustraction d'ambiance
    }

def apply_flash_filter(frame, filt):
    img_f = frame.astype(np.float32)
    # Soustraction de l'ambiance
    res = img_f - (filt["alpha"] * filt["ambient_ref"])
    # Normalisation par la puissance LED
    res = (res / (filt["led_contrib"] + 1e-6)) * 255.0
    res = np.clip(res, 0, 255).astype(np.uint8)
    return cv2.bitwise_and(res, res, mask=filt["led_mask"])

# ---------- Fonctions de Détection Stricte ----------

def _is_rectangle_approx(cnt, angle_tol, area_ratio):
    if len(cnt) < 4: return False
    hull = cv2.convexHull(cnt)
    peri = cv2.arcLength(hull, True)
    approx = cv2.approxPolyDP(hull, 0.03 * peri, True)
    if len(approx) != 4 or not cv2.isContourConvex(approx): return False
    rect = cv2.minAreaRect(approx)
    rect_area = max(rect[1][0] * rect[1][1], 1e-6)
    return (cv2.contourArea(approx) / rect_area) >= area_ratio

def _region_uniformity_lab(bgr, cnt):
    h, w = bgr.shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(mask, [cnt], -1, 255, -1)
    x, y, bw, bh = cv2.boundingRect(cnt)
    roi, m_roi = bgr[y:y+bh, x:x+bw], mask[y:y+bh, x:x+bw]
    if roi.size == 0: return 1e9
    lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
    _, std = cv2.meanStdDev(lab, mask=m_roi)
    return (float(std[1][0])**2 + float(std[2][0])**2) ** 0.5

def _dominant_fraction(bgr, cnt, ranges):
    h, w = bgr.shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(mask, [cnt], -1, 255, -1)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    total = np.count_nonzero(mask)
    if total == 0: return 0.0
    color_mask = None
    for lo, hi in ranges:
        m = cv2.inRange(hsv, lo, hi)
        color_mask = m if color_mask is None else cv2.bitwise_or(color_mask, m)
    inside = cv2.bitwise_and(color_mask, mask)
    return np.count_nonzero(inside) / total

def detect_blocks(bgr, color_ranges):
    out = []
    h, w = bgr.shape[:2]
    blurred = cv2.GaussianBlur(bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    for name, ranges in color_ranges.items():
        mask = np.zeros((h, w), dtype=np.uint8)
        for lo, hi in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
        
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for c in cnts:
            area = cv2.contourArea(c)
            if area < MIN_AREA_PX: continue
            if not _is_rectangle_approx(c, RECT_ANGLE_TOL_DEG, RECT_AREA_RATIO_MIN): continue
            if _region_uniformity_lab(bgr, c) > COLOR_STD_THRESH: continue
            if _dominant_fraction(bgr, c, ranges) < DOMINANT_FRAC_THRESH: continue
            
            rect = cv2.minAreaRect(c)
            out.append({"color": name, "box": cv2.boxPoints(rect), "center": rect[0], "angle": rect[2]})
    return out

# ---------- Affichage Carré ----------

def to_square(bgr, size):
    h, w = bgr.shape[:2]
    scale = size / max(h, w)
    nw, nh = int(w * scale), int(h * scale)
    res = cv2.resize(bgr, (nw, nh))
    canvas = np.zeros((size, size, 3), dtype=np.uint8)
    x0, y0 = (size - nw) // 2, (size - nh) // 2
    canvas[y0:y0+nh, x0:x0+nw] = res
    return canvas, scale, x0, y0

# ---------- Main ----------

def main():
    with open(HSV_JSON_PATH, 'r') as f:
        data = json.load(f)
    COLOR_RANGES = {n: [(np.array(r[0]), np.array(r[1])) for r in i["ranges"]] for n, i in data["colors"].items()}
    
    H_DATA = load_homography()
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    img_off, img_on, filt = None, None, None
    show_filtered = False

    print("Commandes: [o] Capture LEDs OFF | [l] Capture LEDs ON | [c] Créer Filtre | [f] On/Off Filtre")

    while True:
        ok, frame = cap.read()
        if not ok: break

        proc_frame = apply_flash_filter(frame, filt) if (show_filtered and filt) else frame
        detections = detect_blocks(proc_frame, COLOR_RANGES)
        
        sq, scale, x0, y0 = to_square(proc_frame, WINDOW_SIZE)
        
        # Dessin
        color_map = {"red":(0,0,255), "green":(0,255,0), "blue":(255,0,0), "yellow":(0,255,255), "orange":(0,165,255)}
        for det in detections:
            c_name = det["color"]
            bgr = color_map.get(c_name, (255,255,255))
            pts = (det["box"] * scale + [x0, y0]).astype(np.int32)
            cv2.drawContours(sq, [pts], 0, bgr, 3)
            
            # World coordinates if homography exists
            label = c_name
            if H_DATA:
                xw, yw = pix_to_world_cm(det["center"], H_DATA[0])
                label += f" ({xw-H_DATA[1][0]:.1f}, {yw-H_DATA[1][1]:.1f})cm"
            
            cv2.putText(sq, label, (pts[0][0], pts[0][1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 2)

        # HUD
        info = f"Filtre: {'ACTIF' if show_filtered else 'OFF'} | {len(detections)} blocs"
        cv2.putText(sq, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
        if img_off is not None: cv2.circle(sq, (15, 50), 6, (0,0,255), -1)
        if img_on is not None: cv2.circle(sq, (35, 50), 6, (0,255,0), -1)

        cv2.imshow("Detection w/ Filter", sq)
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'): break
        elif k == ord('o'): img_off = frame.copy(); print("Capture OFF OK")
        elif k == ord('l'): img_on = frame.copy(); print("Capture ON OK")
        elif k == ord('c') and img_off is not None and img_on is not None:
            filt = compute_filter(img_off, img_on); print("Filtre généré")
        elif k == ord('f'): show_filtered = not show_filtered

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()