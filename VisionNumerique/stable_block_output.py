#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Stable block output
- Reads frames from camera
- Runs your strict detector with your provided thresholds
- Keeps last 10 frames and checks stability:
    * same number of blocks
    * same set of colors
    * position/angle consistency
- Removes outliers (median-based) and averages the remaining values
- Prints a tuple-like numpy array of (color, "2x4", Xcm, Ycm, angle_deg)
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
FRAME_TARGET = 5            # Require 10 consistent frames
POS_TOLERANCE_CM = 1.0       # median trimming tolerance for X,Y
ANGLE_TOLERANCE_DEG = 8.0    # median trimming tolerance for angle
OUTPUT_BRICK_TYPE = "2x4"    # for now all are 2x4 as requested

# ----------------------------
# Helpers
# ----------------------------
def reject_outliers(values, tolerance):
    """Return subset within ±tolerance of the median."""
    if not values:
        return []
    med = statistics.median(values)
    return [v for v in values if abs(v - med) <= tolerance]

def pack_tuple(blocks):
    """
    blocks: list of dict with keys:
        color, X, Y, angle
    Returns numpy array dtype=object:
        (color, "2x4", X, Y, angle)
    """
    rows = []
    for b in blocks:
        rows.append((b["color"], OUTPUT_BRICK_TYPE,
                     round(b["X"], 3), round(b["Y"], 3), round(b["angle"], 3)))
    return np.array(rows, dtype=object)

# ----------------------------
# Main
# ----------------------------
def main():
    # Load homography
    loaded = load_homography()
    if loaded is None:
        raise SystemExit("Homography missing. Run calibrate_homography.py first.")
    H, IMGW, IMGH, CAM_CENTER_WORLD = loaded
    CAM_CENTER_WORLD = CAM_CENTER_WORLD.reshape(2)

    # Open camera
    cap = open_cap()
    if not cap.isOpened():
        raise SystemExit("Camera could not be opened via GStreamer/libcamera.")

    print("Looking for 10 stable frames... (same colors & consistent positions/angles)")

    # buffer of last frames; each item = list[dict(color,X,Y,angle)]
    ring = []

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.01)
                continue

            # --- Run detection with your exact thresholds ---
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

            # Convert to world (cm)
            frame_blocks = []
            for d in dets:
                (cx, cy) = d["center"]
                ang = float(d["angle_deg"])

                xy = pix_to_world_cm((cx, cy), H)
                if xy is None:
                    continue
                Xcm = float(xy[0] - CAM_CENTER_WORLD[0])
                Ycm = float(xy[1] - CAM_CENTER_WORLD[1])

                frame_blocks.append({
                    "color": d["color"],
                    "X": Xcm,
                    "Y": Ycm,
                    "angle": ang
                })

            # push into ring buffer (keep last 10)
            ring.append(frame_blocks)
            if len(ring) > FRAME_TARGET:
                ring.pop(0)

            # need 10 frames
            if len(ring) < FRAME_TARGET:
                continue

            # ---- Stability check: same multiset of colors across the 10 frames ----
            sigs = [tuple(sorted(b["color"] for b in fr)) for fr in ring]
            if not all(sig == sigs[0] for sig in sigs):
                # not stable in count/colors
                continue

            colors_sorted = list(sigs[0])  # stable set of colors in sorted order

            # ---- For each color, aggregate X, Y, angle across the 10 frames ----
            final_blocks = []
            for color in colors_sorted:
                X_vals, Y_vals, A_vals = [], [], []
                for fr in ring:
                    # NOTE: if multiple of same color appear (rare), take nearest to median later;
                    # here we assume 0/1 per color in your scenario. If duplicates are possible, we can
                    # do matching by nearest neighbor—tell me and I’ll add it.
                    for b in fr:
                        if b["color"] == color:
                            X_vals.append(b["X"])
                            Y_vals.append(b["Y"])
                            A_vals.append(b["angle"])
                            break

                # Reject outliers based on median
                X_clean = reject_outliers(X_vals, POS_TOLERANCE_CM)
                Y_clean = reject_outliers(Y_vals, POS_TOLERANCE_CM)
                A_clean = reject_outliers(A_vals, ANGLE_TOLERANCE_DEG)

                # If angles wrap (e.g., +175 vs -185), we could normalize; for now we expect
                # your detector to be near a mode (e.g., ~15°). If needed, I can add circular stats.
                if not X_clean or not Y_clean or not A_clean:
                    # too unstable → skip this color
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
                continue

            # ---- Print the final tuple and reset buffer for the next snapshot ----
            blocs = pack_tuple(final_blocks)
            print("\n=== STABLE BLOCKS (5-frame average, outliers removed) ===")
            print(blocs)
            print("===========================================================\n")

            # reset to detect the next stable scene
            ring.clear()
            time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            cap.release()
        except Exception:
            pass


if __name__ == "__main__":
    main()