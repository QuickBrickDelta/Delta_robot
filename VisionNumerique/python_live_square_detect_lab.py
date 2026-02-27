#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
python_live_square_detect_lab.py

Affichage live d'un flux GStreamer à 5 FPS en carré + détection stricte de
blocs colorés (vue de dessous) en CIE L*a*b* avec pré-filtre HSV optionnel.

Caractéristiques:
- Pré-filtre HSV large par couleur (activable)
- Sélection chromatique Lab (ΔE_ab sur (a*,b*)) + L* optionnel
- Seuil de chroma Lab (Cmin) pour éviter de "manger" le fond
- Auto-frein si le masque couvre trop de l'image (réduit thr_ab)
- Anti "plein écran": rejette les contours géants
- Morphologie douce (3x3, CLOSE=1)
- Mode debug masque (m/g) + HUD complet

Touches (en plus de la version précédente):
  p             : toggle pré-filtre HSV
  m / g         : debug masque ON/OFF et couleur suivante
  l             : activer/désactiver la contrainte sur L*
  [ et ]        : thr_ab global -/+
  { et }        : Lmin -/+
  ( et )        : Lmax -/+
  1 / !         : MIN_AREA_PX +/-
  4 / $         : RECT_ANGLE_TOL_DEG +/-
  5 / %         : RECT_AREA_RATIO_MIN +/-
  6             : toggle contrainte aspect ratio
  c / v / q     : comme avant

Exécution:
    source ~/venvs/cam_311/bin/activate
    python python_live_square_detect_lab.py

Dépendances:
    pip install opencv-python numpy
"""
import os
import sys
import time
import signal
from typing import Tuple, Dict, List

import cv2
import numpy as np
import json

# -- S'assurer que les fichiers voisins (lab_colors.json, .npz, etc.) sont trouvés
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
os.chdir(SCRIPT_DIR)

LAB_CONFIG_PATH = "lab_colors.json"

def load_lab_config(path=LAB_CONFIG_PATH):
    if not os.path.exists(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    out = {}
    for name, spec in data.items():
        L0, a0, b0 = spec["lab"]
        thr = float(spec.get("thr_ab", 22.0))
        Lrng = spec.get("L_range", [5.0, 100.0])
        out[name] = {
            "lab": (float(L0), float(a0), float(b0)),
            "thr_ab": thr,
            "L_range": (float(Lrng[0]), float(Lrng[1]))
        }
    return out

# ---------- Réglages ----------
WINDOW_SIZE = 900          # taille de la fenêtre carrée
FPS = 5                    # 5 Hz
SQUARE_MODE = "letterbox"  # "letterbox" (FOV max) ou "crop" (zoom centre)
SHOW_VIS = True            # affichage des contours / boxes

# Pré-filtre HSV (ON par défaut)
PREHSV_ENABLED = True

# Débogage masque
DEBUG_MASK = False
DEBUG_COLOR = None  # "green" | "blue" | "yellow" | "red" | None

# Anti "plein écran"
MAX_AREA_FRAC = 0.45  # rejette tout contour > 45% de la surface image

# Auto-frein si masque déborde trop
MAX_COLOR_COVERAGE = 0.25  # 25% de l'image

# Seuil minimal de chroma C* = sqrt(a^2+b^2) par couleur (à ajuster)
C_MIN_BY_COLOR = {
    "green":  8.0,
    "blue":   10.0,
    "yellow": 15.0,
    "red":    10.0,
}

# Pipeline : plein capteur 16:9 -> downscale pour OpenCV
pipeline = (
    'libcamerasrc af-mode=manual lens-position=3.4 ! '
    'video/x-raw,format=NV12,width=4608,height=2592,framerate=5/1 ! '
    'videoscale ! video/x-raw,width=1920,height=1080 ! '
    'queue max-size-buffers=1 leaky=downstream ! '
    'videoconvert ! video/x-raw,format=BGR ! '
    'appsink drop=true max-buffers=1 sync=false'
)

# Calibration positions
H_PATH = "homography_plane.npz"

def load_homography(path=H_PATH):
    if not os.path.exists(path):
        return None
    data = np.load(path, allow_pickle=True)
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

# ---------- Paramètres couleur (CIELAB + HSV) ----------
# Références sRGB (BGR OpenCV) -> converties en Lab (float32) à l'initialisation, si aucun JSON.
COLOR_TARGETS_SRGB_BGR: Dict[str, Tuple[int, int, int]] = {
    "green":  (0, 255, 0),
    "blue":   (255, 0, 0),
    "yellow": (0, 255, 255),
    "red":    (0, 0, 255),
}

# Pré-filtre HSV large (plages à ajuster selon ton éclairage)
# OpenCV: H:[0..179], S:[0..255], V:[0..255]
HSV_RANGES: Dict[str, List[Tuple[np.ndarray, np.ndarray]]] = {
    "green":  [ (np.array([ 40,  60,  50]), np.array([ 85, 255, 255])) ],   # ~ 80° +- marge
    "blue":   [ (np.array([ 90,  70,  40]), np.array([135, 255, 255])) ],
    "yellow": [ (np.array([ 15, 120,  80]), np.array([ 40, 255, 255])) ],
    "red": [
        (np.array([  0, 110,  60]), np.array([ 10, 255, 255])),              # bas
        (np.array([170, 110,  60]), np.array([179, 255, 255]))               # haut
    ],
}

THR_AB_DEFAULT = 22.0
L_RANGE_DEFAULT = (5.0, 100.0)
USE_L_CONSTRAINT = True

# Sera rempli avec {name: {"lab":(L*,a*,b*), "thr_ab":..., "L_range":(...)}}
COLOR_TARGETS_LAB: Dict[str, dict] = {}

# ---------- Filtres taille/forme ----------
MIN_AREA_PX = 3600
COLOR_STD_THRESH = 15.0
DOMINANT_FRAC_THRESH = 0.80   # (non utilisé en Lab pour filtrer, conservé pour debug)
RECT_ANGLE_TOL_DEG = 12.5     # plus tolérant que 5°
RECT_AREA_RATIO_MIN = 0.90
ASPECT_RATIO_RANGE = None

# ---------- Affichage carré ----------
def to_square_letterbox(bgr: np.ndarray, size: int) -> Tuple[np.ndarray, float, int, int]:
    h, w = bgr.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
    resized = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((size, size, 3), dtype=np.uint8)
    x0, y0 = (size - nw) // 2, (size - nh) // 2
    canvas[y0:y0 + nh, x0:x0 + nw] = resized
    return canvas, scale, x0, y0

def to_square_crop(bgr: np.ndarray, size: int) -> Tuple[np.ndarray, int, int, int]:
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
             aspect_ratio_range,
             thr_ab,
             L_range,
             use_L,
             prehsv_enabled):
    hud_lines = [
        f"min_area={min_area_px}px",
        f"std_ab<={color_std_thresh:.1f}",
        f"ΔE_ab<={thr_ab:.1f}",
        f"L*={'OFF' if not use_L or L_range is None else f'{L_range[0]:.1f}-{L_range[1]:.1f}'}",
        f"HSV prefilter={'ON' if prehsv_enabled else 'OFF'}",
        f"ang_tol={rect_ang_tol:.1f}°",
        f"area_ratio>={rect_area_ratio_min:.2f}",
        f"aspect={'OFF' if aspect_ratio_range is None else f'{aspect_ratio_range[0]:.2f}-{aspect_ratio_range[1]:.2f}'}",
    ]
    x, y = 10, 54
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
    data = cnt.reshape(-1, 2).astype(np.float32)
    mean, eigenvectors = cv2.PCACompute(data, mean=None)
    v = eigenvectors[0]
    ang = np.degrees(np.arctan2(v[1], v[0]))
    return ang

def _is_rectangle_approx(cnt: np.ndarray,
                         angle_tol_deg: float = 20.0,
                         area_ratio_min: float = 0.80) -> bool:
    if len(cnt) < 4:
        return False

    hull = cv2.convexHull(cnt)
    if len(hull) < 4:
        return False

    peri = cv2.arcLength(hull, True)
    approx = cv2.approxPolyDP(hull, 0.03 * peri, True)
    if len(approx) != 4:
        return False

    if not cv2.isContourConvex(approx):
        return False

    # Angles proches de 90°
    pts = approx.reshape(-1, 2).astype(np.float32)
    for i in range(4):
        p0 = pts[i]
        p1 = pts[(i + 1) % 4]
        p2 = pts[(i + 2) % 4]
        v1 = p0 - p1
        v2 = p2 - p1
        v1 /= (np.linalg.norm(v1) + 1e-8)
        v2 /= (np.linalg.norm(v2) + 1e-8)
        ang = np.degrees(np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0)))
        if abs(ang - 90.0) > angle_tol_deg:
            return False

    rect = cv2.minAreaRect(approx)
    rect_area = max(rect[1][0] * rect[1][1], 1e-6)
    approx_area = cv2.contourArea(approx)
    if approx_area / rect_area < area_ratio_min:
        return False

    return True

def _region_uniformity_lab(bgr: np.ndarray, cnt: np.ndarray) -> Tuple[float, float]:
    """
    Retourne (color_std_ab, coverage_ratio) dans la région du contour.
    Calcul en Lab uint8 (proche des seuils historiques).
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

# ---------- Pré-filtre HSV ----------
def _hsv_prefilter(bgr: np.ndarray,
                   ranges: List[Tuple[np.ndarray, np.ndarray]]) -> np.ndarray:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = None
    for lo, hi in ranges:
        mm = cv2.inRange(hsv, lo, hi)
        mask = mm if mask is None else cv2.bitwise_or(mask, mm)
    return mask if mask is not None else np.zeros(bgr.shape[:2], dtype=np.uint8)

# ---------- Détection stricte en CIELAB ----------
def _prepare_lab_targets():
    global COLOR_TARGETS_LAB
    COLOR_TARGETS_LAB = {}
    for name, bgr in COLOR_TARGETS_SRGB_BGR.items():
        patch = np.uint8([[[bgr[0], bgr[1], bgr[2]]]])
        patch_f = patch.astype(np.float32) / 255.0
        lab = cv2.cvtColor(patch_f, cv2.COLOR_BGR2LAB)[0, 0]
        L0, a0, b0 = float(lab[0]), float(lab[1]), float(lab[2])
        COLOR_TARGETS_LAB[name] = {
            "lab": (L0, a0, b0),
            "thr_ab": THR_AB_DEFAULT,
            "L_range": L_RANGE_DEFAULT,
        }

def _lab_frame_from_bgr(bgr: np.ndarray) -> np.ndarray:
    """Retourne Lab float32: L*∈[0..100], a*,b*∈[-127..127]"""
    bgr32 = bgr.astype(np.float32) / 255.0
    lab32 = cv2.cvtColor(bgr32, cv2.COLOR_BGR2LAB)
    return lab32

def _lab_color_mask(lab_img: np.ndarray, target_lab: Tuple[float, float, float],
                    thr_ab: float, L_range: Tuple[float, float] | None,
                    use_L: bool, Cmin: float | None) -> np.ndarray:
    L = lab_img[:, :, 0]
    A = lab_img[:, :, 1]
    B = lab_img[:, :, 2]
    L0, a0, b0 = target_lab
    dA = A - a0
    dB = B - b0
    delta_ab = cv2.magnitude(dA, dB)  # sqrt(dA^2 + dB^2)
    mask = (delta_ab <= thr_ab)
    if use_L and (L_range is not None):
        Lmin, Lmax = L_range
        mask &= (L >= Lmin) & (L <= Lmax)
    if Cmin is not None:
        C = cv2.magnitude(A, B)       # chroma
        mask &= (C >= Cmin)
    return (mask.astype(np.uint8) * 255)

def detect_colored_blocks_lab(bgr: np.ndarray,
                              color_targets_lab: Dict[str, dict],
                              min_area_px: int = MIN_AREA_PX,
                              color_std_thresh: float = COLOR_STD_THRESH,
                              rect_angle_tol_deg: float = RECT_ANGLE_TOL_DEG,
                              rect_area_ratio_min: float = RECT_AREA_RATIO_MIN,
                              aspect_ratio_range=None,
                              use_L: bool = USE_L_CONSTRAINT,
                              prehsv_enabled: bool = PREHSV_ENABLED) -> List[dict]:
    out = []

    blurred = cv2.GaussianBlur(bgr, (5, 5), 0)
    lab32 = _lab_frame_from_bgr(blurred)

    # Morphologie douce
    kernel = np.ones((3, 3), np.uint8)

    # HSV image (une fois)
    hsv_img = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV) if prehsv_enabled else None

    for color_name, spec in color_targets_lab.items():
        target_lab = spec["lab"]
        thr_ab = spec["thr_ab"]
        L_range = spec["L_range"]
        Cmin = C_MIN_BY_COLOR.get(color_name, None)

        # 1) Pré-filtre HSV large (optionnel)
        if prehsv_enabled and (color_name in HSV_RANGES):
            mask_hsv = None
            for lo, hi in HSV_RANGES[color_name]:
                mm = cv2.inRange(hsv_img, lo, hi)
                mask_hsv = mm if mask_hsv is None else cv2.bitwise_or(mask_hsv, mm)
        else:
            mask_hsv = None

        # 2) Masque Lab (ΔE_ab + L* + Cmin)
        mask_lab = _lab_color_mask(lab32, target_lab, thr_ab, L_range, use_L, Cmin=Cmin)

        # 3) Combinaison (HSV ∧ Lab) si HSV actif; sinon Lab seul
        mask = cv2.bitwise_and(mask_hsv, mask_lab) if (mask_hsv is not None) else mask_lab

        # Morphologie
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        # Debug masque (affichage par couleur)
        if DEBUG_MASK and (DEBUG_COLOR is None or DEBUG_COLOR == color_name):
            cv2.imshow(f"mask_{color_name}", mask)

        # Auto-frein si le masque déborde trop
        coverage = float(cv2.countNonZero(mask)) / float(mask.shape[0] * mask.shape[1])
        if coverage > MAX_COLOR_COVERAGE:
            spec["thr_ab"] = max(2.0, spec["thr_ab"] - 1.0)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        H_img, W_img = bgr.shape[:2]

        for c in cnts:
            area = cv2.contourArea(c)
            if area < min_area_px:
                continue

            # Anti "plein écran"
            if area > MAX_AREA_FRAC * (H_img * W_img):
                continue

            if not _is_rectangle_approx(c, angle_tol_deg=rect_angle_tol_deg,
                                         area_ratio_min=rect_area_ratio_min):
                continue

            if aspect_ratio_range is not None:
                rect = cv2.minAreaRect(c)
                (rw, rh) = rect[1]
                if rw == 0 or rh == 0:
                    continue
                ratio = max(rw, rh) / max(1e-6, min(rw, rh))
                rmin, rmax = aspect_ratio_range
                if not (rmin <= ratio <= rmax):
                    continue

            color_std_ab, coverage_roi = _region_uniformity_lab(bgr, c)
            if color_std_ab > color_std_thresh:
                continue

            rect = cv2.minAreaRect(c)  # ((cx,cy),(w,h),angle)
            (cx, cy), (rw, rh), _ = rect
            box = cv2.boxPoints(rect)
            box = np.rint(box).astype(np.int32)  # arrondi + int32, compatible OpenCV/NumPy 2.x
            angle = angle_via_pca(c)

            out.append({
                "color": color_name,
                "center": (float(cx), float(cy)),
                "angle_deg": float(angle),
                "box": box,
                "area": float(area),
                "contour": c,
                "uniform_std_ab": float(color_std_ab),
                "coverage": float(coverage_roi),
                "thr_ab": float(thr_ab),
                "L_range": L_range,
            })

    return out

# ---------- IO caméra ----------
def open_cap():
    return cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

# ---------- Main ----------
def main():
    global SQUARE_MODE, SHOW_VIS
    global MIN_AREA_PX, COLOR_STD_THRESH
    global RECT_ANGLE_TOL_DEG, RECT_AREA_RATIO_MIN, ASPECT_RATIO_RANGE
    global USE_L_CONSTRAINT
    global COLOR_TARGETS_LAB
    global PREHSV_ENABLED, DEBUG_MASK, DEBUG_COLOR

    print("[DBG] __file__ =", os.path.abspath(__file__))
    print("[DBG] CWD      =", os.getcwd())
    print("[DBG] LAB_CONFIG_PATH =", os.path.abspath(LAB_CONFIG_PATH))

    _prepare_lab_targets()

    prof = load_lab_config()
    if prof:
        COLOR_TARGETS_LAB = prof
        print("[LAB] Profil chargé:", list(COLOR_TARGETS_LAB.keys()))
        try:
            print(
                "[LAB] Profil complet:\n" +
                json.dumps(
                    {k: {"lab": tuple(v["lab"]), "thr_ab": v["thr_ab"], "L_range": tuple(v["L_range"])}
                     for k, v in COLOR_TARGETS_LAB.items()},
                    indent=2
                )
            )
        except Exception as e:
            print("[LAB] (debug print) Impossible d'afficher le profil complet :", e)
    else:
        print("[LAB] Aucun profil JSON -> cibles par défaut (à calibrer)")

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
        WORLD_ENABLED = False
        print("[WORLD] Homography introuvable -> coordonnées monde désactivées (affichage u,v en pixels).")
    else:
        WORLD_ENABLED = True
        H_cm, IMGW, IMGH, CAM_CENTER_WORLD = loaded
        CAM_CENTER_WORLD = CAM_CENTER_WORLD.reshape(2)
        print("Homography loaded. Camera-center world offset:", CAM_CENTER_WORLD)

    win = "LIVE carre (detect LAB)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, WINDOW_SIZE, WINDOW_SIZE)
    print("Commandes : q=quitter | c=letterbox/crop | v=toggle overlay | p=toggle HSV | m=mask ON/OFF | g=mask color | [ ] thr_ab -/+ | { } Lmin -/+ | ( ) Lmax -/+ | l=toggle L*")

    last = time.time()
    frames = 0
    fps_est = 0.0

    target_dt = 1.0 / float(FPS)

    while not stop_flag["stop"]:
        t0 = time.time()

        ok, frame = cap.read()
        if not ok or frame is None:
            cap.release()
            time.sleep(0.1)
            cap = open_cap()
            continue

        h, w = frame.shape[:2]

        # --- Détection (HSV->Lab) ---
        detections = detect_colored_blocks_lab(
            frame,
            COLOR_TARGETS_LAB,
            min_area_px=MIN_AREA_PX,
            color_std_thresh=COLOR_STD_THRESH,
            rect_angle_tol_deg=RECT_ANGLE_TOL_DEG,
            rect_area_ratio_min=RECT_AREA_RATIO_MIN,
            aspect_ratio_range=ASPECT_RATIO_RANGE,
            use_L=USE_L_CONSTRAINT,
            prehsv_enabled=PREHSV_ENABLED,
        )

        # --- Image carrée + mapping ---
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

                if SQUARE_MODE == "letterbox":
                    px, py = map_point_to_square_letterbox((cx, cy), *mode_params)
                    box_disp = map_poly_to_square(box, mode_params, "letterbox")
                else:
                    px, py = map_point_to_square_crop((cx, cy), *mode_params)
                    box_disp = map_poly_to_square(box, mode_params, "crop")

                color_bgr = {
                    "red": (0, 0, 255),
                    "green": (0, 255, 0),
                    "blue": (255, 0, 0),
                    "yellow": (0, 255, 255),
                    "orange": (0, 165, 255),
                    "purple": (255, 0, 255),
                    "white": (220, 220, 220),
                }.get(col, (255, 255, 255))

                if WORLD_ENABLED:
                    xy_world = pix_to_world_cm((cx, cy), H_cm)
                    if xy_world is None:
                        Xcm, Ycm = float('nan'), float('nan')
                    else:
                        Xcm = xy_world[0] - CAM_CENTER_WORLD[0]
                        Ycm = xy_world[1] - CAM_CENTER_WORLD[1]
                    label = f"{col} | X={Xcm:+.1f}cm Y={Ycm:+.1f}cm | {ang:+.1f}°"
                else:
                    label = f"{col} | u={int(cx)} v={int(cy)} | {ang:+.1f}°"

                cv2.drawContours(square, [box_disp], 0, color_bgr, 2)
                cv2.circle(square, (px, py), 5, color_bgr, -1)
                cv2.putText(square, label, (px + 8, py - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_bgr, 2, cv2.LINE_AA)

        # FPS estimé
        frames += 1
        now = time.time()
        if now - last >= 1.0:
            fps_est = frames / (now - last)
            frames = 0
            last = now

        # HUD
        if COLOR_TARGETS_LAB:
            any_name = next(iter(COLOR_TARGETS_LAB))
            thr_ab_hud = COLOR_TARGETS_LAB[any_name]["thr_ab"]
            L_range_hud = COLOR_TARGETS_LAB[any_name]["L_range"]
        else:
            thr_ab_hud = THR_AB_DEFAULT
            L_range_hud = L_RANGE_DEFAULT

        txt = f"{mode_text} | src {w}x{h} | ~{fps_est:.1f} fps | {len(detections)} bloc(s)"
        cv2.putText(square, txt, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

        draw_hud(square,
                 MIN_AREA_PX,
                 COLOR_STD_THRESH,
                 DOMINANT_FRAC_THRESH,
                 RECT_ANGLE_TOL_DEG,
                 RECT_AREA_RATIO_MIN,
                 ASPECT_RATIO_RANGE,
                 thr_ab_hud,
                 L_range_hud,
                 USE_L_CONSTRAINT,
                 PREHSV_ENABLED)

        cv2.imshow(win, square)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('c'):
            SQUARE_MODE = "crop" if SQUARE_MODE == "letterbox" else "letterbox"
        elif key == ord('v'):
            SHOW_VIS = not SHOW_VIS
        elif key == ord('p'):
            PREHSV_ENABLED = not PREHSV_ENABLED
            print("HSV prefilter:", PREHSV_ENABLED)

        elif key == ord('m'):
            DEBUG_MASK = not DEBUG_MASK
            print("DEBUG_MASK:", DEBUG_MASK)
        elif key == ord('g'):
            keys = list(COLOR_TARGETS_LAB.keys())
            if keys:
                DEBUG_COLOR = keys[0] if DEBUG_COLOR is None else keys[(keys.index(DEBUG_COLOR) + 1) % len(keys)]
                print("DEBUG_COLOR:", DEBUG_COLOR)

        # --- Live tuning (hérités) ---
        elif key == ord('1'):
            MIN_AREA_PX = int(min(20000, MIN_AREA_PX + 200))
        elif key == ord('!'):
            MIN_AREA_PX = int(max(0, MIN_AREA_PX - 200))
        elif key == ord('2'):
            COLOR_STD_THRESH = float(min(50.0, COLOR_STD_THRESH + 1.0))
        elif key == ord('"'):
            COLOR_STD_THRESH = float(max(0.5, COLOR_STD_THRESH - 1.0))
        elif key == ord('4'):
            RECT_ANGLE_TOL_DEG = float(min(45.0, RECT_ANGLE_TOL_DEG + 2.5))
        elif key == ord('$'):
            RECT_ANGLE_TOL_DEG = float(max(2.5, RECT_ANGLE_TOL_DEG - 2.5))
        elif key == ord('5'):
            RECT_AREA_RATIO_MIN = float(min(0.98, RECT_AREA_RATIO_MIN + 0.02))
        elif key == ord('%'):
            RECT_AREA_RATIO_MIN = float(max(0.50, RECT_AREA_RATIO_MIN - 0.02))
        elif key == ord('6'):
            if ASPECT_RATIO_RANGE is None:
                ASPECT_RATIO_RANGE = (0.70, 4.00)
            else:
                ASPECT_RATIO_RANGE = None

        # --- Live tuning (Lab) ---
        elif key == ord('['):  # thr_ab -
            for k in COLOR_TARGETS_LAB:
                COLOR_TARGETS_LAB[k]["thr_ab"] = max(1.0, COLOR_TARGETS_LAB[k]["thr_ab"] - 1.0)
        elif key == ord(']'):  # thr_ab +
            for k in COLOR_TARGETS_LAB:
                COLOR_TARGETS_LAB[k]["thr_ab"] = min(60.0, COLOR_TARGETS_LAB[k]["thr_ab"] + 1.0)
        elif key == ord('{'):  # Lmin -
            for k in COLOR_TARGETS_LAB:
                Lmin, Lmax = COLOR_TARGETS_LAB[k]["L_range"]
                Lmin = max(0.0, Lmin - 1.0)
                COLOR_TARGETS_LAB[k]["L_range"] = (Lmin, Lmax)
        elif key == ord('}'):  # Lmin +
            for k in COLOR_TARGETS_LAB:
                Lmin, Lmax = COLOR_TARGETS_LAB[k]["L_range"]
                Lmin = min(Lmax, Lmin + 1.0)
                COLOR_TARGETS_LAB[k]["L_range"] = (Lmin, Lmax)
        elif key == ord('('):  # Lmax -
            for k in COLOR_TARGETS_LAB:
                Lmin, Lmax = COLOR_TARGETS_LAB[k]["L_range"]
                Lmax = max(Lmin, Lmax - 1.0)
                COLOR_TARGETS_LAB[k]["L_range"] = (Lmin, Lmax)
        elif key == ord(')'):  # Lmax +
            for k in COLOR_TARGETS_LAB:
                Lmin, Lmax = COLOR_TARGETS_LAB[k]["L_range"]
                Lmax = min(100.0, Lmax + 1.0)
                COLOR_TARGETS_LAB[k]["L_range"] = (Lmin, Lmax)
        elif key == ord('l'):
            USE_L_CONSTRAINT = not USE_L_CONSTRAINT

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