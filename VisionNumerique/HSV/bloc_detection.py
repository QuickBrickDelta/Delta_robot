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
FPS = 5                    # 5 Hz
SQUARE_MODE = "letterbox"  # "letterbox" (FOV max) ou "crop" (zoom centre)
SHOW_VIS = True            # affichage des contours / boxes

# Pipeline 1 : FOV maximal (plein capteur 16:9) -> downscale pour OpenCV
pipeline = (
    'libcamerasrc af-mode=manual lens-position=3.4 ! '
    'video/x-raw,format=NV12,width=4608,height=2592,framerate=5/1 ! '
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


# Filtres de taille / forme (relaxés)
MIN_AREA_PX = 2000            # ↓ de 1200 -> 800
COLOR_STD_THRESH = 40.0      # ↑ de 6.0 -> 10.0 (plus tolérant)
DOMINANT_FRAC_THRESH = 0.15  # ↓ de 0.65 -> 0.55
RECT_ANGLE_TOL_DEG = 20.0    # ↑ de 15.0 -> 22.5
RECT_AREA_RATIO_MIN = 0.70   # ↓ de 0.86 -> 0.80

# Optionnel: contrainte de ratio L/H (désactivée par défaut)
ASPECT_RATIO_RANGE = None    # ex: (0.7, 4.0) pour types de briques

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

# ---------- Détection stricte : uniforme + rectangulaire ----------
def detect_colored_blocks(bgr: np.ndarray,
                          color_ranges: Dict[str, List[Tuple[np.ndarray, np.ndarray]]],
                          min_area_px: int = MIN_AREA_PX,
                          color_std_thresh: float = COLOR_STD_THRESH,
                          dominant_frac_thresh: float = DOMINANT_FRAC_THRESH,
                          rect_angle_tol_deg: float = RECT_ANGLE_TOL_DEG,
                          rect_area_ratio_min: float = RECT_AREA_RATIO_MIN,
                          aspect_ratio_range=None) -> List[dict]:
    """
    Détecteur strict:
      - Préfiltre couleur HSV
      - Pour chaque contour:
          * area >= min_area_px
          * convexe & approx 4 sommets & angles ~90° (tolérance rect_angle_tol_deg)
          * area(contour)/area(minAreaRect) >= rect_area_ratio_min
          * uniformité LAB: std_ab <= color_std_thresh
          * fraction dominante HSV >= dominant_frac_thresh
          * (optionnel) ratio L/H dans aspect_ratio_range
      - Retour: liste de dicts (color, center, angle_deg, box, area, contour, ...).
    """
    out = []

    blurred = cv2.GaussianBlur(bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    h, w = bgr.shape[:2]

    kernel = np.ones((5, 5), np.uint8)
    kernel_open = np.ones((3, 3), np.uint8)

    for color_name, ranges in COLOR_RANGES.items():
        # Création d'un masque vide
        mask = np.zeros((h, w), dtype=np.uint8)
        
        # On combine toutes les plages définies pour cette couleur (ex: les 2 plages du rouge)
        for (lower, upper) in ranges:
            term_mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.bitwise_or(mask, term_mask)
        
        # Appliquer le reste des filtres (morphologie) sur le masque combiné
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in cnts:
            area = cv2.contourArea(c)
            if area < min_area_px:
                continue

            # Géométrie rectangulaire stricte
            if not _is_rectangle_approx(c,
                                        angle_tol_deg=rect_angle_tol_deg,
                                        area_ratio_min=rect_area_ratio_min):
                continue

            # Aspect ratio optionnel
            if aspect_ratio_range is not None:
                rect = cv2.minAreaRect(c)
                (rw, rh) = rect[1]
                if rw == 0 or rh == 0:
                    continue
                ratio = max(rw, rh) / max(1e-6, min(rw, rh))
                rmin, rmax = aspect_ratio_range
                if not (rmin <= ratio <= rmax):
                    continue

            # Uniformité de couleur (LAB)
            color_std_ab, coverage = _region_uniformity_lab(bgr, c)
            if color_std_ab > color_std_thresh:
                continue

            # Adhérence à la couleur cible
            frac = _dominant_color_fraction_hsv(bgr, c, ranges)
            if frac < dominant_frac_thresh:
                continue

            # OK -> propriétés
            rect = cv2.minAreaRect(c)  # ((cx,cy),(w,h),angle)
            (cx, cy), (rw, rh), _ = rect
            box = cv2.boxPoints(rect)
            angle = angle_via_pca(c)

            out.append({
                "color": color_name,
                "center": (float(cx), float(cy)),
                "angle_deg": float(angle),
                "box": box,
                "area": float(area),
                "contour": c,
                # debug facultatif
                "uniform_std_ab": float(color_std_ab),
                "dominant_frac": float(frac),
                "coverage": float(coverage),
            })

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
        detections = detect_colored_blocks(
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