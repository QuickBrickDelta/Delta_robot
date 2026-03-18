#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bloc_detection_w_filter.py
Pipeline intégral : 
1. Capture Manuelle (OFF/ON) 
2. Filtre Ambiant dosable (+/-)
3. Détection Stricte (LAB + HSV + GÉO)
4. Mapping Monde (cm)
"""


#"Commandes: [o] Capture LEDs OFF | [l] Capture LEDs ON | [c] Créer Filtre | [f] On/Off Filtre")
import os
import time
import json
from pathlib import Path
from typing import Tuple, Dict, List
import cv2
import numpy as np

# ---------- Réglages par défaut ----------
WINDOW_SIZE = 900
FPS = 5
SQUARE_MODE = "letterbox"

# Paramètres de détection (tes valeurs relaxées pour le vert)
MIN_AREA_PX = 2000
COLOR_STD_THRESH = 50.0
DOMINANT_FRAC_THRESH = 0.15
RECT_ANGLE_TOL_DEG = 20.0
RECT_AREA_RATIO_MIN = 0.70

_THIS_DIR = Path(__file__).resolve().parent
HSV_JSON_PATH = _THIS_DIR / "hsv_ranges.json"
H_PATH = _THIS_DIR.parent / "calibration" / "homography_plane.npz"

pipeline = (
    'libcamerasrc af-mode=manual lens-position=3.4 ! '
    'video/x-raw,format=NV12,width=4608,height=2592,framerate=5/1 ! '
    'videoscale ! video/x-raw,width=1920,height=1080 ! '
    'videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false'
)

# ---------- 1. Homographie & Mapping ----------

def load_homography(path=H_PATH):
    if not Path(path).exists(): return None
    data = np.load(str(path), allow_pickle=True)
    return data["H"], data["cam_center_world"].reshape(2)

def pix_to_world_cm(pt_uv, H):
    u, v = float(pt_uv[0]), float(pt_uv[1])
    ph = H @ np.array([u, v, 1.0], dtype=np.float64)
    if abs(ph[2]) < 1e-9: return None
    return (ph[0] / ph[2], ph[1] / ph[2])

# ---------- 2. Logique Filtre Flash ----------

def compute_filter(img_off, img_on):
    off_f = img_off.astype(np.float32)
    on_f = img_on.astype(np.float32)
    led_contrib = on_f - off_f
    diff_gray = cv2.cvtColor(np.abs(led_contrib).astype(np.uint8), cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(diff_gray, 8, 255, cv2.THRESH_BINARY)
    return {
        "ambient_ref": off_f,
        "led_contrib": led_contrib,
        "led_mask": mask,
        "alpha": 0.95
    }

def apply_flash_filter(frame, filt, strength=0.5):
    if filt is None: return frame
    img_f = frame.astype(np.float32)
    subtracted = img_f - (filt.get("alpha", 0.95) * filt["ambient_ref"])
    normalized = (subtracted / (filt["led_contrib"] + 1e-6)) * 255.0
    pure_filter = np.clip(normalized, 0, 255).astype(np.uint8)
    pure_filter = cv2.bitwise_and(pure_filter, pure_filter, mask=filt["led_mask"])
    return cv2.addWeighted(frame, 1.0 - strength, pure_filter, strength, 0)

# ---------- 3. Fonctions de Détection Stricte ----------

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
    roi = bgr[y:y+bh, x:x+bw]
    m_roi = mask[y:y+bh, x:x+bw]
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

def detect_blocks(bgr, color_ranges, h_data=None):
    """
    Détecte les blocs avec :
    1. Filtrage couleur HSV (multi-plages)
    2. Nettoyage morphologique (Open + Close pour stabilité)
    3. Filtrage géométrique (Rectangle approx)
    4. Filtrage Uniformité (LAB)
    5. Filtrage Dimensions Réelles (cm) si h_data est fourni
    """
    out = []
    h, w = bgr.shape[:2]
    
    # Réduction du bruit avant détection
    blurred = cv2.GaussianBlur(bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    # Noyaux pour la morphologie
    kernel_open = np.ones((3, 3), np.uint8)
    kernel_close = np.ones((7, 7), np.uint8) # Plus grand pour stabiliser le clignotage

    for name, ranges in color_ranges.items():
        # 1. Création du masque pour la couleur
        mask = np.zeros((h, w), dtype=np.uint8)
        for lo, hi in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lo, hi))
        
        # 2. Nettoyage : supprimer bruit (Open) puis boucher les trous (Close)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)
        
        # 3. Recherche de contours
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for c in cnts:
            area = cv2.contourArea(c)
            if area < MIN_AREA_PX:
                continue
            
            # 4. Vérification forme géométrique (approx rectangle)
            if not _is_rectangle_approx(c, RECT_ANGLE_TOL_DEG, RECT_AREA_RATIO_MIN):
                continue
                
            # 5. Vérification uniformité de la couleur (LAB)
            if _region_uniformity_lab(bgr, c) > COLOR_STD_THRESH:
                continue
            
            # 6. Vérification fraction de couleur dominante
            if _dominant_fraction(bgr, c, ranges) < DOMINANT_FRAC_THRESH:
                continue
            
            # 7. --- NOUVEAU : Filtrage par dimensions réelles (cm) ---
            rect = cv2.minAreaRect(c)
            (cx, cy), (w_px, h_px), angle = rect
            
            if h_data is not None:
                H_matrix = h_data[0]
                # Calcul de l'échelle locale (ratio pixels/cm)
                p_center = pix_to_world_cm((cx, cy), H_matrix)
                p_offset = pix_to_world_cm((cx + 100, cy), H_matrix)
                
                if p_center and p_offset:
                    # Distance en cm pour 100 pixels
                    dist_cm = np.sqrt((p_center[0]-p_offset[0])**2 + (p_center[1]-p_offset[1])**2)
                    px_to_cm = dist_cm / 100.0
                    
                    dim1 = w_px * px_to_cm
                    dim2 = h_px * px_to_cm
                    
                    # TRIER pour comparer [petit, grand] peu importe l'angle
                    dims_cm = sorted([dim1, dim2])
                    
                    # EXEMPLE : On cherche des blocs de 3.2cm x 6.4cm (LEGO 2x4 approx)
                    # Tu peux ajuster ou commenter ces lignes si tu veux tout accepter
                    # TARGET_W, TARGET_H = 3.2, 6.4
                    # TOLERANCE = 0.8
                    # if abs(dims_cm[0] - TARGET_W) > TOLERANCE or abs(dims_cm[1] - TARGET_H) > TOLERANCE:
                    #     continue

            # Si toutes les conditions sont remplies, on ajoute le bloc
            out.append({
                "color": name,
                "box": cv2.boxPoints(rect),
                "center": (cx, cy),
                "angle": angle,
                "dims_cm": dims_cm if h_data else None
            })
            
    return out

# ---------- 4. Affichage & Main ----------

def to_square(bgr, size):
    h, w = bgr.shape[:2]
    scale = size / max(h, w)
    nw, nh = int(w * scale), int(h * scale)
    canvas = np.zeros((size, size, 3), dtype=np.uint8)
    x0, y0 = (size - nw) // 2, (size - nh) // 2
    canvas[y0:y0+nh, x0:x0+nw] = cv2.resize(bgr, (nw, nh))
    return canvas, scale, x0, y0

def main():
    global MIN_AREA_PX, COLOR_STD_THRESH, DOMINANT_FRAC_THRESH, RECT_AREA_RATIO_MIN

    # Chargement
    with open(HSV_JSON_PATH, 'r') as f:
        data = json.load(f)
    COLOR_RANGES = {n: [(np.array(r[0]), np.array(r[1])) for r in i["ranges"]] for n, i in data["colors"].items()}
    H_DATA = load_homography()
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    
    img_off, img_on, filt = None, None, None
    show_filtered = False
    strength = 0.15 

    print("--- COMMANDES ---")
    print("[o] Capture OFF | [l] Capture ON | [c] Calcul Filtre")
    print("[f] Toggle Filtre | [+/-] Force | [1/!] Area | [2/\"] Std | [3/#] Frac")

    while True:
        ok, frame = cap.read()
        if not ok: break

        proc_frame = apply_flash_filter(frame, filt, strength) if (show_filtered and filt) else frame
        detections = detect_blocks(proc_frame, COLOR_RANGES, H_DATA)
        
        sq, scale, x0, y0 = to_square(proc_frame, WINDOW_SIZE)
        cmap = {
            "red": (0, 0, 255),
            "green_dark": (0, 100, 0),    # Vert foncé
            "green_light": (144, 238, 144), # Vert clair
            "brown": (19, 69, 139),       # Marron (SaddleBrown)
            "orange": (0, 165, 255),      # Orange
            "black": (30, 30, 30),        # Gris très foncé/Noir
            "white": (240, 240, 240),     # Blanc cassé
            "yellow": (0, 255, 255),
            "blue": (255, 0, 0)
        }

        for det in detections:
            bgr = cmap.get(det["color"], (255,255,255))
            pts = (det["box"] * scale + [x0, y0]).astype(np.int32)
            cv2.drawContours(sq, [pts], 0, bgr, 3)
            
            label = det["color"]
            if H_DATA:
                xw, yw = pix_to_world_cm(det["center"], H_DATA[0])
                label += f" ({xw-H_DATA[1][0]:.1f}, {yw-H_DATA[1][1]:.1f})cm"
            cv2.putText(sq, label, (pts[0][0], pts[0][1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 2)

        # HUD Détaillé
        hud = [
            f"FILTRE: {strength*100:.0f}% ({'ON' if show_filtered else 'OFF'})",
            f"MIN_AREA: {MIN_AREA_PX}",
            f"STD_THRESH: {COLOR_STD_THRESH:.1f}",
            f"DOM_FRAC: {DOMINANT_FRAC_THRESH:.2f}",
            f"RECT_RATIO: {RECT_AREA_RATIO_MIN:.2f}"
        ]
        for i, text in enumerate(hud):
            cv2.putText(sq, text, (10, 30 + i*25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        # Indicateurs de capture
        if img_off is not None: cv2.circle(sq, (WINDOW_SIZE-40, 30), 10, (0,0,255), -1)
        if img_on is not None: cv2.circle(sq, (WINDOW_SIZE-20, 30), 10, (0,255,0), -1)

        cv2.imshow("Detection w/ Filter", sq)
        
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'): break
        elif k == ord('o'): img_off = frame.copy(); print("OFF OK")
        elif k == ord('l'): img_on = frame.copy(); print("ON OK")
        elif k == ord('c') and img_off is not None and img_on is not None:
            filt = compute_filter(img_off, img_on); print("Filtre OK")
        elif k == ord('f'): show_filtered = not show_filtered
        elif k == ord('+'): strength = min(1.0, strength + 0.05)
        elif k == ord('-'): strength = max(0.0, strength - 0.05)
        
        # Tuning touches
        elif k == ord('1'): MIN_AREA_PX += 200
        elif k == ord('!'): MIN_AREA_PX = max(0, MIN_AREA_PX - 200)
        elif k == ord('2'): COLOR_STD_THRESH += 2
        elif k == ord('"'): COLOR_STD_THRESH = max(1, COLOR_STD_THRESH - 2)
        elif k == ord('3'): DOMINANT_FRAC_THRESH = min(1.0, DOMINANT_FRAC_THRESH + 0.05)
        elif k == ord('#'): DOMINANT_FRAC_THRESH = max(0.0, DOMINANT_FRAC_THRESH - 0.05)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()