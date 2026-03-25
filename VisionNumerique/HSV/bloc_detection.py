#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# HSV mean=(113.36,159.97,181.22) std=(4.08,26.00,39.57) rect=(892, 810, 995, 861) n=5253
"""
python_live_square_detect.py
Affichage live d'un flux GStreamer à 5 FPS en carré + détection stricte de blocs colorés (vue de dessous).
- Seules les régions de couleur uniforme ET rectangulaires sont retenues
- Couleur (par HSV), position (x, y) en pixels du frame source, angle (° CCW)
- Superposition adaptée à l'affichage carré (letterbox/crop)

Exécution (dans ton venv):
    source ~/venvs/cam_311/bin/activate
    python python_live_square_detect.py

Dépendances:
    pip install opencv-python numpy
"""

import os
import time
import signal
from pathlib import Path
from typing import Tuple, Dict, List
import json

import cv2
import numpy as np

# ---------- Réglages ----------
WINDOW_SIZE = 900          # taille de la fenêtre carrée
FPS = 2                    # 2 Hz
SQUARE_MODE = "letterbox"  # "letterbox" (FOV max) ou "crop" (zoom centre)
SHOW_VIS = True            # affichage des contours / boxes

# Pipeline 1 : FOV maximal (plein capteur 16:9) -> downscale pour OpenCV
pipeline = (
    'libcamerasrc af-mode=manual lens-position=3.4 ! '
    'video/x-raw,format=NV12,width=4608,height=2592,framerate=2/1 ! '
    'videoscale ! video/x-raw,width=1920,height=1080 ! '
    'queue max-size-buffers=1 leaky=downstream ! '
    'videoconvert ! video/x-raw,format=BGR ! '
    'appsink drop=true max-buffers=1 sync=false'
)

# Calibration positions
_THIS_DIR = Path(__file__).resolve().parent
H_PATH = _THIS_DIR.parent / "calibration" / "homography_plane.npz"

def load_homography(path=H_PATH):
    path = Path(path)
    if not path.exists():
        return None
    data = np.load(str(path), allow_pickle=True)
    H = data["H"]
    imgW = int(data["imgW"])
    imgH = int(data["imgH"])
    cam_center_world = data["cam_center_world"]  # shape (2,)
    return H, imgW, imgH, cam_center_world

def pix_to_world_cm(pt_uv, H):
    """pt_uv = (u,v) pixel → (X,Y) world in cm using planar homography."""
    u, v = float(pt_uv[0]), float(pt_uv[1])
    ph = H @ np.array([u, v, 1.0], dtype=np.float64)
    if abs(ph[2]) < 1e-9:
        return None
    xw = ph[0] / ph[2]
    yw = ph[1] / ph[2]
    return (float(xw), float(yw))

# ---------- Chargement des paramètres couleur (HSV) ----------

_THIS_DIR = Path(__file__).resolve().parent
HSV_JSON_PATH = _THIS_DIR / "hsv_ranges.json"

if not HSV_JSON_PATH.exists():
    raise FileNotFoundError(f"ERREUR CRITIQUE : Le fichier de calibration '{HSV_JSON_PATH}' est introuvable. "
                            "Génère-le d'abord avec calibrate_hsv_ranges.py")

def load_color_ranges(json_path: Path):
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    ranges_out = {}
    for color_name, info in data.get("colors", {}).items():
        color_list = []
        for r in info.get("ranges", []):
            # Conversion des listes JSON [H, S, V] en tableaux numpy pour OpenCV
            lo = np.array(r[0], dtype=np.uint8)
            hi = np.array(r[1], dtype=np.uint8)
            color_list.append((lo, hi))
        ranges_out[color_name] = color_list
    return ranges_out

# Chargement effectif
COLOR_RANGES = load_color_ranges(HSV_JSON_PATH)
print(f"Calibration chargée avec succès : {list(COLOR_RANGES.keys())}")


# Filtres de taille / forme (alignés avec `bloc_detection_w_filter.py`)
MIN_AREA_PX = 2000            # px min pour accepter un contour
COLOR_STD_THRESH = 50.0      # tolérance d'uniformité LAB (plus tolérant que 40)
DOMINANT_FRAC_THRESH = 0.15  # fraction dominante HSV
RECT_ANGLE_TOL_DEG = 20.0    # tolérance de “rectangularité” via l'approche actuelle
RECT_AREA_RATIO_MIN = 0.70   # ratio aire_contour / aire_rect minimum

# Optionnel: contrainte de ratio L/H (désactivée par défaut)
ASPECT_RATIO_RANGE = None    # ex: (0.7, 4.0) pour types de briques

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

def draw_hud(square_img,
             min_area_px,
             color_std_thresh,
             dom_frac_thresh,
             rect_ang_tol,
             rect_area_ratio_min,
             aspect_ratio_range):
    hud_lines = [
        f"min_area={min_area_px}px",
        f"std_ab<={color_std_thresh:.1f}",
        f"dom_frac>={dom_frac_thresh:.2f}",
        f"ang_tol={rect_ang_tol:.1f}°",
        f"area_ratio>={rect_area_ratio_min:.2f}",
        f"aspect={'OFF' if aspect_ratio_range is None else f'{aspect_ratio_range[0]:.2f}-{aspect_ratio_range[1]:.2f}'}",
    ]
    x, y = 10, 54  # under the existing top line
    for i, line in enumerate(hud_lines):
        yy = y + i * 20
        cv2.putText(square_img, line, (x, yy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(square_img, line, (x, yy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)

def map_point_to_square_letterbox(pt, scale, x0, y0):
    x, y = pt
    return int(x * scale + x0), int(y * scale + y0)

def map_point_to_square_crop(pt, size, side, x_off, y_off):
    x, y = pt
    scale = size / float(side)
    return int((x - x_off) * scale), int((y - y_off) * scale)

def map_poly_to_square(poly, mode_params, mode):
    if mode == "letterbox":
        scale, x0, y0 = mode_params
        return np.array([map_point_to_square_letterbox((p[0], p[1]), scale, x0, y0) for p in poly], dtype=np.int32)
    else:
        size, side, x_off, y_off = mode_params
        return np.array([map_point_to_square_crop((p[0], p[1]), size, side, x_off, y_off) for p in poly], dtype=np.int32)

# ---------- Détection blocs : helpers ----------
def angle_via_pca(cnt: np.ndarray) -> float:
    """
    Angle du premier axe principal en degrés, 0° = +X (droite), CCW positif.
    """
    data = cnt.reshape(-1, 2).astype(np.float32)
    mean, eigenvectors = cv2.PCACompute(data, mean=None)
    v = eigenvectors[0]  # vecteur principal
    ang = np.degrees(np.arctan2(v[1], v[0]))  # [-180, 180)
    return ang

def _angle_between(v1, v2):
    """Angle in degrees between vectors v1 and v2."""
    v1 = v1 / (np.linalg.norm(v1) + 1e-8)
    v2 = v2 / (np.linalg.norm(v2) + 1e-8)
    cosang = np.clip(np.dot(v1, v2), -1.0, 1.0)
    return np.degrees(np.arccos(cosang))

def _is_rectangle_approx(cnt: np.ndarray,
                         angle_tol_deg: float = 20.0,
                         area_ratio_min: float = 0.70) -> bool:
    if len(cnt) < 4: return False
    
    # Calcul de la solidité (Aire / Aire du Convex Hull)
    area = cv2.contourArea(cnt)
    hull = cv2.convexHull(cnt)
    hull_area = cv2.contourArea(hull)
    solidity = area / float(hull_area + 1e-6)
    
    # Calcul du ratio d'aire avec le rectangle orienté
    rect = cv2.minAreaRect(cnt)
    rect_area = max(rect[1][0] * rect[1][1], 1e-6)
    rect_ratio = area / rect_area
    
    # Pour un bloc, on veut une solidité > 0.9 et un bon ratio d'aire
    return solidity > 0.90 and rect_ratio > area_ratio_min

def _region_uniformity_lab(bgr: np.ndarray, cnt: np.ndarray) -> Tuple[float, float]:
    """
    Retourne (color_std_ab, coverage_ratio) dans la région du contour.
    color_std_ab = sqrt(std(a)^2 + std(b)^2) en CIELAB (robuste à l'éclairage).
    coverage_ratio = aire du masque / aire du rect englobant -> rejette artefacts minces.
    """
    h, w = bgr.shape[:2]
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(mask, [cnt], -1, 255, thickness=cv2.FILLED)

    x, y, bw, bh = cv2.boundingRect(cnt)
    roi = bgr[y:y+bh, x:x+bw]
    mask_roi = mask[y:y+bh, x:x+bw]

    if roi.size == 0 or np.count_nonzero(mask_roi) < 50:
        return 1e9, 0.0

    lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
    mean, std = cv2.meanStdDev(lab, mask=mask_roi)
    std_a, std_b = float(std[1][0]), float(std[2][0])
    color_std_ab = (std_a**2 + std_b**2) ** 0.5

    coverage_ratio = float(np.count_nonzero(mask_roi)) / float(bw * bh + 1e-6)
    return color_std_ab, coverage_ratio

def _dominant_color_fraction_hsv(bgr: np.ndarray, cnt: np.ndarray,
                                 color_ranges: List[Tuple[np.ndarray, np.ndarray]]) -> float:
    """
    Fraction de pixels à l'intérieur du contour appartenant aux plages HSV configurées.
    """
    h, w = bgr.shape[:2]
    mask_cnt = np.zeros((h, w), dtype=np.uint8)
    cv2.drawContours(mask_cnt, [cnt], -1, 255, thickness=cv2.FILLED)

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    total = np.count_nonzero(mask_cnt)
    if total == 0:
        return 0.0

    merged = None
    for lo, hi in color_ranges:
        mm = cv2.inRange(hsv, lo, hi)
        merged = mm if merged is None else cv2.bitwise_or(merged, mm)

    inside = cv2.bitwise_and(merged, merged, mask=mask_cnt)
    frac = float(np.count_nonzero(inside)) / float(total)
    return frac

def detect_blocks(bgr, color_ranges, h_data=None):
    out = []
    h, w = bgr.shape[:2]
    
    # 1. Image de travail
    blurred = cv2.GaussianBlur(bgr, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
    # 2. MASQUE GLOBAL d'exclusion (Pixels déjà assignés)
    already_assigned_mask = np.zeros((h, w), dtype=np.uint8)
    
    kernel_open = np.ones((3, 3), np.uint8)
    kernel_close = np.ones((11, 11), np.uint8)

    # Note: L'ordre dans color_ranges.items() devient important. 
    # Les premières couleurs ont la priorité.
    for name, ranges in color_ranges.items():
        # Création du masque pour la couleur actuelle
        color_mask = np.zeros((h, w), dtype=np.uint8)
        for lo, hi in ranges:
            color_mask = cv2.bitwise_or(color_mask, cv2.inRange(hsv, lo, hi))
        
        # --- ÉTAPE CLÉ : On soustrait les pixels déjà pris par une autre couleur ---
        color_mask = cv2.bitwise_and(color_mask, cv2.bitwise_not(already_assigned_mask))
        
        # Nettoyage habituel
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel_open)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel_close)
        
        # Recherche de contours
        cnts, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for c in cnts:
            area = cv2.contourArea(c)
            if area < MIN_AREA_PX:
                continue
            
            if not _is_rectangle_approx(c, RECT_ANGLE_TOL_DEG, RECT_AREA_RATIO_MIN):
                continue
                
            if _region_uniformity_lab(bgr, c) > COLOR_STD_THRESH:
                continue
            
            if _dominant_fraction(bgr, c, ranges) < DOMINANT_FRAC_THRESH:
                continue
            
            rect = cv2.minAreaRect(c)
            (cx, cy), (w_px, h_px), angle = rect
            
            # (Optionnel) Calcul des dimensions réelles comme tu l'as déjà
            # --- CALCUL DES DIMENSIONS RÉELLES (cm) ---
            dims_cm = None
            if h_data is not None:
                H_matrix = h_data[0]
                
                # On utilise le centre du bloc pour calculer l'échelle locale (ratio pixels/cm)
                # On compare le centre avec un point décalé de 100 pixels
                p_center = pix_to_world_cm((cx, cy), H_matrix)
                p_offset = pix_to_world_cm((cx + 100, cy), H_matrix)
                
                if p_center and p_offset:
                    # Calcul de la distance réelle en cm pour ces 100 pixels
                    dist_cm = np.sqrt((p_center[0]-p_offset[0])**2 + (p_center[1]-p_offset[1])**2)
                    px_to_cm = dist_cm / 100.0
                    
                    # Conversion des dimensions pixels en cm
                    dim1 = w_px * px_to_cm
                    dim2 = h_px * px_to_cm
                    
                    # On trie pour avoir toujours [petit_coté, grand_coté]
                    dims_cm = sorted([dim1, dim2])
                    
                    #--- FILTRE LEGO (Optionnel) ---
                    #Si tu veux filtrer les briques 2x4 (environ 1.6 x 3.2 cm)
                    TARGET_W, TARGET_H = 1.6, 3.2
                    TOL = 0.8
                    if abs(dims_cm[0] - TARGET_W) > TOL or abs(dims_cm[1] - TARGET_H) > TOL:
                        continue

            # SI LE BLOC EST VALIDE :
            # On l'ajoute à la liste
            out.append({
                "color": name,
                "box": cv2.boxPoints(rect),
                "center": (cx, cy),
                "angle": angle,
                "dims_cm": dims_cm
            })
            
            # ON MARQUE LE BLOC comme "assigné" dans le masque global
            # pour que les couleurs suivantes ignorent cette zone
            cv2.drawContours(already_assigned_mask, [c], -1, 255, -1)
            
    return out

# ---------- IO caméra ----------
def open_cap():
    return cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

# ---------- Main (inchangé pour l'exécution) ----------
def main():
    global SQUARE_MODE, SHOW_VIS
    global MIN_AREA_PX, COLOR_STD_THRESH, DOMINANT_FRAC_THRESH
    global RECT_ANGLE_TOL_DEG, RECT_AREA_RATIO_MIN, ASPECT_RATIO_RANGE


    # gestion Ctrl+C propre
    stop_flag = {"stop": False}
    def _sigint(_sig, _frm):
        stop_flag["stop"] = True
    signal.signal(signal.SIGINT, _sigint)

    cap = open_cap()
    if not cap.isOpened():
        raise SystemExit("Erreur : impossible d'ouvrir la caméra via GStreamer.")

    loaded = load_homography(H_PATH)
    if loaded is None:
        raise SystemExit(
            "Homography absente. Lance d'abord: python3 calibrate_homography.py"
        )

    H_cm, IMGW, IMGH, CAM_CENTER_WORLD = loaded
    CAM_CENTER_WORLD = CAM_CENTER_WORLD.reshape(2)
    print("Homography loaded. Camera-center world offset:", CAM_CENTER_WORLD)

    win = "LIVE carre (detect strict)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, WINDOW_SIZE, WINDOW_SIZE)
    print("Commandes : q = quitter | c = letterbox/crop | v = toggle overlay")

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

        # --- Détection sur le frame source (w x h) ---
        detections = detect_blocks(
            frame,
            COLOR_RANGES,
            min_area_px=MIN_AREA_PX,
            color_std_thresh=COLOR_STD_THRESH,
            dominant_frac_thresh=DOMINANT_FRAC_THRESH,
            rect_angle_tol_deg=RECT_ANGLE_TOL_DEG,
            rect_area_ratio_min=RECT_AREA_RATIO_MIN,
            aspect_ratio_range=ASPECT_RATIO_RANGE
        )

        # --- Image carrée + paramètres de mapping ---
        if SQUARE_MODE == "letterbox":
            square, scale, x0, y0 = to_square_letterbox(frame, WINDOW_SIZE)
            mode_text = "LETTERBOX (FOV max)"
            mode_params = (scale, x0, y0)
        else:
            square, side, x_off, y_off = to_square_crop(frame, WINDOW_SIZE)
            mode_text = "CROP (centre)"
            mode_params = (WINDOW_SIZE, side, x_off, y_off)

        # --- Overlay ---
        if SHOW_VIS:
            for det in detections:
                cx, cy = det["center"]
                box = det["box"]
                ang = det["angle_deg"]
                col = det["color"]

                # mapping des points dans l'image carrée
                if SQUARE_MODE == "letterbox":
                    px, py = map_point_to_square_letterbox((cx, cy), *mode_params)
                    box_disp = map_poly_to_square(box, mode_params, "letterbox")
                else:
                    px, py = map_point_to_square_crop((cx, cy), *mode_params)
                    box_disp = map_poly_to_square(box, mode_params, "crop")

                # couleur d'affichage par nom
                color_bgr = {
                    "red": (0, 0, 255),
                    "green": (0, 255, 0),
                    "blue": (255, 0, 0),
                    "yellow": (0, 255, 255),
                    "orange": (0, 165, 255),
                    "purple": (255, 0, 255),
                    "white": (220, 220, 220),
                }.get(col, (255, 255, 255))

                # --- NEW: Pixel -> World (cm) ---
                xy_world = pix_to_world_cm((cx, cy), H_cm)
                if xy_world is None:
                    Xcm, Ycm = float('nan'), float('nan')
                else:
                    Xcm = xy_world[0] - CAM_CENTER_WORLD[0]
                    Ycm = xy_world[1] - CAM_CENTER_WORLD[1]

                # Draw the box + point
                cv2.drawContours(square, [box_disp], 0, color_bgr, 2)
                cv2.circle(square, (px, py), 5, color_bgr, -1)

                # Label in centimeters
                label = f"{col} | X={Xcm:+.1f}cm Y={Ycm:+.1f}cm | {ang:+.1f}°"
                cv2.putText(square, label, (px + 8, py - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_bgr, 2, cv2.LINE_AA)

        # FPS estimé
        frames += 1
        now = time.time()
        if now - last >= 1.0:
            fps_est = frames / (now - last)
            frames = 0
            last = now

        txt = f"{mode_text} | src {w}x{h} | ~{fps_est:.1f} fps | {len(detections)} bloc(s)"
        cv2.putText(square, txt, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        # Draw tuning HUD
        draw_hud(square,
                MIN_AREA_PX,
                COLOR_STD_THRESH,
                DOMINANT_FRAC_THRESH,
                RECT_ANGLE_TOL_DEG,
                RECT_AREA_RATIO_MIN,
                ASPECT_RATIO_RANGE)

        cv2.imshow(win, square)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('c'):
            SQUARE_MODE = "crop" if SQUARE_MODE == "letterbox" else "letterbox"
        elif key == ord('v'):
            SHOW_VIS = not SHOW_VIS

        # --- Live tuning ---
        elif key == ord('1'):  # MIN_AREA_PX +
            MIN_AREA_PX = int(min(20000, MIN_AREA_PX + 200))
        elif key == ord('!'):  # MIN_AREA_PX -
            MIN_AREA_PX = int(max(0, MIN_AREA_PX - 200))

        elif key == ord('2'):  # COLOR_STD_THRESH +
            COLOR_STD_THRESH = float(min(50.0, COLOR_STD_THRESH + 1.0))
        elif key == ord('"'):  # COLOR_STD_THRESH -
            COLOR_STD_THRESH = float(max(0.5, COLOR_STD_THRESH - 1.0))

        elif key == ord('3'):  # DOMINANT_FRAC_THRESH +
            DOMINANT_FRAC_THRESH = float(min(0.98, DOMINANT_FRAC_THRESH + 0.05))
        elif key == ord('#'):  # DOMINANT_FRAC_THRESH -
            DOMINANT_FRAC_THRESH = float(max(0.10, DOMINANT_FRAC_THRESH - 0.05))

        elif key == ord('4'):  # RECT_ANGLE_TOL_DEG +
            RECT_ANGLE_TOL_DEG = float(min(45.0, RECT_ANGLE_TOL_DEG + 2.5))
        elif key == ord('$'):  # RECT_ANGLE_TOL_DEG -
            RECT_ANGLE_TOL_DEG = float(max(2.5, RECT_ANGLE_TOL_DEG - 2.5))

        elif key == ord('5'):  # RECT_AREA_RATIO_MIN +
            RECT_AREA_RATIO_MIN = float(min(0.98, RECT_AREA_RATIO_MIN + 0.02))
        elif key == ord('%'):  # RECT_AREA_RATIO_MIN -
            RECT_AREA_RATIO_MIN = float(max(0.50, RECT_AREA_RATIO_MIN - 0.02))

        elif key == ord('6'):  # Toggle aspect ratio constraint
            if ASPECT_RATIO_RANGE is None:
                ASPECT_RATIO_RANGE = (0.70, 4.00)  # example window for LEGO-like rectangles
            else:
                ASPECT_RATIO_RANGE = None

        # timing ~5 Hz
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