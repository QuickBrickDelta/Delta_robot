#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Stable block output (API style)
- main(): boucle infinie qui appelle une fonction
- print_stable_blocks_once(ctx): bloque jusqu'à un set stable, puis imprime le tuple

Tuple imprimé: np.array(dtype=object) de (color, "2x4", Xcm, Ycm, angle_deg)
"""

import time
import statistics
import numpy as np

from bloc_detection import (
    detect_colored_blocks,
    COLOR_RANGES,
    pix_to_world_cm,
    open_cap,
    load_homography,
)

# ----------------------------
# Detector thresholds (yours)
# ----------------------------
DET_MIN_AREA_PX = 3600
DET_COLOR_STD_THRESH = 15.0
DET_DOMINANT_FRAC = 0.80
DET_RECT_ANGLE_TOL_DEG = 5.0
DET_RECT_AREA_RATIO_MIN = 0.92
DET_ASPECT = None  # OFF

# ----------------------------
# Stability parameters
# ----------------------------
FRAME_TARGET = 5             # nombre de frames stables requis (tu peux remettre 10)
POS_TOLERANCE_CM = 1.0       # trimming autour de la médiane pour X,Y
ANGLE_TOLERANCE_DEG = 8.0    # trimming autour de la médiane pour angle
OUTPUT_BRICK_TYPE = "2x4"    # pour l'instant, toutes les briques sont "2x4"

# ----------------------------
# Helpers
# ----------------------------
def reject_outliers(values, tolerance):
    """Retourne les valeurs dans ±tolerance autour de la médiane."""
    if not values:
        return []
    med = statistics.median(values)
    return [v for v in values if abs(v - med) <= tolerance]

def pack_tuple(blocks):
    """
    blocks: list de dict avec:
        color, X, Y, angle
    -> numpy array dtype=object:
        (color, "2x4", X, Y, angle)
    """
    rows = []
    for b in blocks:
        rows.append((b["color"], OUTPUT_BRICK_TYPE,
                     round(b["X"], 3), round(b["Y"], 3), round(b["angle"], 3)))
    return np.array(rows, dtype=object)

# ----------------------------
# Contexte partagé (caméra + homographie)
# ----------------------------
class Context:
    def __init__(self):
        loaded = load_homography()
        if loaded is None:
            raise SystemExit("Homography missing. Run calibrate_homography.py first.")
        H, IMGW, IMGH, CAM_CENTER_WORLD = loaded
        self.H = H
        self.CAM_CENTER_WORLD = CAM_CENTER_WORLD.reshape(2)

        cap = open_cap()
        if not cap.isOpened():
            raise SystemExit("Camera could not be opened via GStreamer/libcamera.")
        self.cap = cap

    def close(self):
        try:
            self.cap.release()
        except Exception:
            pass

# ----------------------------
# Fonction unique: bloque jusqu'à set stable, imprime le tuple
# ----------------------------
def print_stable_blocks_once(ctx: Context):
    """
    Bloque jusqu'à obtenir FRAME_TARGET frames stables (même multiset de couleurs),
    effectue un rejet des valeurs aberrantes, calcule la moyenne, imprime le tuple,
    puis retourne (pour que le main relance un nouveau cycle).
    """
    ring = []  # buffer des dernières frames (list[list[dict]])

    while True:
        ok, frame = ctx.cap.read()
        if not ok or frame is None:
            time.sleep(0.01)
            continue

        # --- Détection stricte avec tes seuils ---
        dets = detect_colored_blocks(
            frame,
            COLOR_RANGES,
            min_area_px=DET_MIN_AREA_PX,
            color_std_thresh=DET_COLOR_STD_THRESH,
            dominant_frac_thresh=DET_DOMINANT_FRAC,
            rect_angle_tol_deg=DET_RECT_ANGLE_TOL_DEG,
            rect_area_ratio_min=DET_RECT_AREA_RATIO_MIN,
            aspect_ratio_range=DET_ASPECT
        )

        # Convertit en coordonnées monde (cm)
        frame_blocks = []
        for d in dets:
            (cx, cy) = d["center"]
            ang = float(d["angle_deg"])

            xy = pix_to_world_cm((cx, cy), ctx.H)
            if xy is None:
                continue
            Xcm = float(xy[0] - ctx.CAM_CENTER_WORLD[0])
            Ycm = float(xy[1] - ctx.CAM_CENTER_WORLD[1])

            frame_blocks.append({
                "color": d["color"],
                "X": Xcm,
                "Y": Ycm,
                "angle": ang
            })

        # buffer (on garde les FRAME_TARGET dernières)
        ring.append(frame_blocks)
        if len(ring) > FRAME_TARGET:
            ring.pop(0)

        # besoin d'au moins FRAME_TARGET frames
        if len(ring) < FRAME_TARGET:
            continue

        # stabilité: même multiset de couleurs sur les FRAME_TARGET frames
        sigs = [tuple(sorted(b["color"] for b in fr)) for fr in ring]
        if not all(sig == sigs[0] for sig in sigs):
            # pas stable en compte/couleurs
            continue

        colors_sorted = list(sigs[0])  # ensemble stable de couleurs

        # agrégation des X, Y, angle par couleur
        final_blocks = []
        for color in colors_sorted:
            X_vals, Y_vals, A_vals = [], [], []
            for fr in ring:
                # Hypothèse actuelle: au plus un bloc par couleur.
                # S'il peut y avoir des doublons, on ajoutera un appariement (Hungarian).
                for b in fr:
                    if b["color"] == color:
                        X_vals.append(b["X"])
                        Y_vals.append(b["Y"])
                        A_vals.append(b["angle"])
                        break

            # rejet d'outliers autour de la médiane
            X_clean = reject_outliers(X_vals, POS_TOLERANCE_CM)
            Y_clean = reject_outliers(Y_vals, POS_TOLERANCE_CM)
            A_clean = reject_outliers(A_vals, ANGLE_TOLERANCE_DEG)

            if not X_clean or not Y_clean or not A_clean:
                # trop instable -> ignore la couleur
                continue

            X_mean = statistics.mean(X_clean)
            Y_mean = statistics.mean(Y_clean)
            A_mean = statistics.mean(A_clean)

            final_blocks.append({
                "color": color,
                "X": X_mean,
                "Y": Y_mean,
                "angle": A_mean
            })

        if not final_blocks:
            # rien de stable pour l'instant
            continue

        # construire et imprimer le tuple
        blocs = pack_tuple(final_blocks)
        print("\n=== STABLE BLOCKS ({}-frame average, outliers removed) ===".format(FRAME_TARGET))
        print(blocs)
        print("===========================================================\n")

        # on retourne vers le main (qui relancera un nouveau cycle)
        return

# ----------------------------
# Main: appelle la fonction en boucle
# ----------------------------
def main():
    ctx = Context()
    print(f"Ready. Will print a tuple every time a stable set of {FRAME_TARGET} frames is reached.")
    try:
        while True:
            print_stable_blocks_once(ctx)
            # petite pause pour éviter spam si la scène reste strictement identique
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        ctx.close()

if __name__ == "__main__":
    main()