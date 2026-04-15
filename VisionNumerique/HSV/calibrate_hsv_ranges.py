#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Calibration HSV (robuste luminosité)

But:
- Drag une ROI sur un bloc de couleur
- Appuie sur 1..9 pour assigner la DERNIÈRE ROI à un nom de couleur
- Exporte `hsv_ranges.json` (plages HSV OpenCV) réutilisable par les détecteurs

Notes:
- HSV OpenCV: H ∈ [0..179], S ∈ [0..255], V ∈ [0..255]
- Pour le rouge (wrap H autour de 0), le script peut produire 2 intervalles.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


# ---------------- Camera pipeline (copie cohérente avec tes scripts) ----------------
WIDTH = 1920
HEIGHT = 1080
FPS = 5
PIPELINE = (
    "libcamerasrc af-mode=manual lens-position=3.4 ! "
    f"video/x-raw,format=NV12,width={WIDTH},height={HEIGHT},framerate={FPS}/1 ! "
    "videoconvert ! video/x-raw,format=BGR ! "
    "appsink drop=true max-buffers=1 sync=false"
)


# ---------------- Calibration parameters ----------------
# Range suggestion: mean +/- K_SIGMA * std, then clamped
K_SIGMA = 2.5

# Reject low saturation/value pixels in the ROI (avoid background / glare)
MIN_S = 40
MIN_V = 40

# If H window crosses 0 (wrap), split into 2 ranges when useful
ALLOW_H_WRAP_SPLIT = True


# ---------------- UI state ----------------
WIN = "Calib HSV (drag ROI) | 1..9 assign | e export | r reset | q quit"
dragging = False
pt0: tuple[int, int] | None = None
pt1: tuple[int, int] | None = None
last_frame: np.ndarray | None = None
last_roi: dict | None = None


KEY_TO_COLOR = {
    ord("1"): "red",
    ord("2"): "green_dark",
    ord("3"): "blue",
    ord("4"): "yellow",
    ord("5"): "orange",
    ord("6"): "purple",
    ord("7"): "white",
    ord("8"): "black",
    ord("9"): "brown",
    ord("0"): "green_light",
}


@dataclass(frozen=True)
class HSVRange:
    lo: tuple[int, int, int]  # (H,S,V)
    hi: tuple[int, int, int]


def _open_camera() -> cv2.VideoCapture:
    cap = cv2.VideoCapture(PIPELINE, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise SystemExit("Erreur : impossible d'ouvrir la caméra via GStreamer.")
    return cap


def _clamp_int(x: float, lo: int, hi: int) -> int:
    return int(max(lo, min(hi, int(round(x)))))


def _hsv_roi_stats(bgr: np.ndarray, x0: int, y0: int, x1: int, y1: int) -> dict | None:
    x0, x1 = sorted([int(x0), int(x1)])
    y0, y1 = sorted([int(y0), int(y1)])
    h, w = bgr.shape[:2]
    x0 = int(np.clip(x0, 0, w - 1))
    x1 = int(np.clip(x1, 0, w - 1))
    y0 = int(np.clip(y0, 0, h - 1))
    y1 = int(np.clip(y1, 0, h - 1))
    if x1 <= x0 or y1 <= y0:
        return None

    roi = bgr[y0:y1, x0:x1]
    if roi.size == 0:
        return None

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    H = hsv[:, :, 0].astype(np.float32)
    S = hsv[:, :, 1].astype(np.float32)
    V = hsv[:, :, 2].astype(np.float32)

    mask = (S >= float(MIN_S)) & (V >= float(MIN_V))
    if int(np.count_nonzero(mask)) < 50:
        # fallback: stats over whole ROI
        mask = np.ones_like(H, dtype=bool)

    Hm = float(np.mean(H[mask]))
    Hs = float(np.std(H[mask]))
    Sm = float(np.mean(S[mask]))
    Ss = float(np.std(S[mask]))
    Vm = float(np.mean(V[mask]))
    Vs = float(np.std(V[mask]))

    return {
        "rect": (x0, y0, x1, y1),
        "mean": (Hm, Sm, Vm),
        "std": (Hs, Ss, Vs),
        "n": int(np.count_nonzero(mask)),
        "min_sv": (MIN_S, MIN_V),
        "k_sigma": float(K_SIGMA),
    }


def _suggest_ranges_from_stats(stats: dict) -> list[HSVRange]:
    (Hm, Sm, Vm) = stats["mean"]
    (Hs, Ss, Vs) = stats["std"]

    h_lo = Hm - K_SIGMA * Hs
    h_hi = Hm + K_SIGMA * Hs
    s_lo = Sm - K_SIGMA * Ss
    s_hi = Sm + K_SIGMA * Ss
    v_lo = Vm - K_SIGMA * Vs
    v_hi = Vm + K_SIGMA * Vs

    # Clamp in OpenCV ranges
    s_lo_i = _clamp_int(s_lo, 0, 255)
    s_hi_i = _clamp_int(s_hi, 0, 255)
    v_lo_i = _clamp_int(v_lo, 0, 255)
    v_hi_i = _clamp_int(v_hi, 0, 255)

    # Enforce minimum S/V floors (keep robustness vs illumination)
    s_lo_i = max(s_lo_i, MIN_S)
    v_lo_i = max(v_lo_i, MIN_V)

    # H wrap handling:
    # If interval goes below 0 or above 179, optionally split into two.
    if not ALLOW_H_WRAP_SPLIT:
        h_lo_i = _clamp_int(h_lo, 0, 179)
        h_hi_i = _clamp_int(h_hi, 0, 179)
        if h_lo_i > h_hi_i:
            h_lo_i, h_hi_i = h_hi_i, h_lo_i
        return [HSVRange((h_lo_i, s_lo_i, v_lo_i), (h_hi_i, s_hi_i, v_hi_i))]

    if h_lo < 0:
        # Example: [-8, 6] -> [0,6] and [172,179]
        h_hi_i = _clamp_int(h_hi, 0, 179)
        h_lo_wrap = _clamp_int(180 + h_lo, 0, 179)
        return [
            HSVRange((0, s_lo_i, v_lo_i), (h_hi_i, s_hi_i, v_hi_i)),
            HSVRange((h_lo_wrap, s_lo_i, v_lo_i), (179, s_hi_i, v_hi_i)),
        ]
    if h_hi > 179:
        # Example: [174, 187] -> [174,179] and [0,8]
        h_lo_i = _clamp_int(h_lo, 0, 179)
        h_hi_wrap = _clamp_int(h_hi - 180, 0, 179)
        return [
            HSVRange((h_lo_i, s_lo_i, v_lo_i), (179, s_hi_i, v_hi_i)),
            HSVRange((0, s_lo_i, v_lo_i), (h_hi_wrap, s_hi_i, v_hi_i)),
        ]

    h_lo_i = _clamp_int(h_lo, 0, 179)
    h_hi_i = _clamp_int(h_hi, 0, 179)
    if h_lo_i > h_hi_i:
        h_lo_i, h_hi_i = h_hi_i, h_lo_i
    return [HSVRange((h_lo_i, s_lo_i, v_lo_i), (h_hi_i, s_hi_i, v_hi_i))]


def _draw_overlay(img: np.ndarray, stats: dict | None) -> np.ndarray:
    overlay = img.copy()
    if dragging and pt0 and pt1:
        cv2.rectangle(overlay, pt0, pt1, (0, 255, 255), 2)

    cv2.putText(
        overlay,
        "Drag ROI | 1..9 assign | e export | r reset | q quit",
        (15, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (20, 255, 20),
        2,
        cv2.LINE_AA,
    )

    if stats is not None:
        (Hm, Sm, Vm) = stats["mean"]
        (Hs, Ss, Vs) = stats["std"]
        rr = _suggest_ranges_from_stats(stats)
        line1 = f"HSV mean=({Hm:.1f},{Sm:.1f},{Vm:.1f})  std=({Hs:.1f},{Ss:.1f},{Vs:.1f})  n={stats['n']}"
        cv2.putText(overlay, line1, (15, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(overlay, line1, (15, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)
        for i, r in enumerate(rr[:2]):
            line = f"range{i+1}: lo={r.lo} hi={r.hi}"
            y = 78 + i * 20
            cv2.putText(overlay, line, (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(overlay, line, (15, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)

    return overlay


def _on_mouse(event, x, y, _flags, _param) -> None:
    global dragging, pt0, pt1, last_roi, last_frame
    if last_frame is None:
        return

    if event == cv2.EVENT_LBUTTONDOWN:
        dragging = True
        pt0 = (int(x), int(y))
        pt1 = (int(x), int(y))
    elif event == cv2.EVENT_MOUSEMOVE and dragging:
        pt1 = (int(x), int(y))
    elif event == cv2.EVENT_LBUTTONUP:
        dragging = False
        pt1 = (int(x), int(y))
        if pt0 is None:
            return
        dx = abs(pt1[0] - pt0[0])
        dy = abs(pt1[1] - pt0[1])
        if dx < 5 and dy < 5:
            # pixel probe (for info)
            b, g, r = last_frame[int(y), int(x)].astype(int).tolist()
            hsv = cv2.cvtColor(np.uint8([[[b, g, r]]]), cv2.COLOR_BGR2HSV)[0, 0]
            print(f"[PIXEL] (x={x}, y={y})  BGR=({b},{g},{r})  HSV={tuple(int(v) for v in hsv)}")
            return

        stats = _hsv_roi_stats(last_frame, pt0[0], pt0[1], pt1[0], pt1[1])
        if stats is None:
            return
        last_roi = stats
        (Hm, Sm, Vm) = stats["mean"]
        (Hs, Ss, Vs) = stats["std"]
        print(f"[ROI] HSV mean=({Hm:.2f},{Sm:.2f},{Vm:.2f}) std=({Hs:.2f},{Ss:.2f},{Vs:.2f}) rect={stats['rect']} n={stats['n']}")


def _serialize_ranges(ranges: list[HSVRange]) -> list[list[list[int]]]:
    # Output format compatible with COLOR_RANGES: list of [lo, hi], with lo/hi as [H,S,V]
    return [[list(r.lo), list(r.hi)] for r in ranges]


def main() -> None:
    global last_frame, last_roi

    out_dir = Path(__file__).resolve().parent
    out_path = out_dir / "hsv_ranges.json"

    assigned: dict[str, dict] = {}

    cap = _open_camera()
    # warmup
    for _ in range(8):
        cap.read()

    ok, frame = cap.read()
    if not ok or frame is None:
        cap.release()
        raise SystemExit("Capture échouée.")

    last_frame = frame.copy()

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1280, int(1280 * (frame.shape[0] / frame.shape[1])))
    cv2.setMouseCallback(WIN, _on_mouse)

    print("Commandes :")
    print("  - Drag: ROI -> stats + suggestion de plages HSV")
    print("  - 1..9: assigne la dernière ROI au nom de couleur")
    print("  - e   : exporte hsv_ranges.json")
    print("  - r   : reset (efface assignations)")
    print("  - q/ESC : quitter")

    while True:
        ok, frame = cap.read()
        if ok and frame is not None:
            last_frame = frame

        disp = _draw_overlay(last_frame, last_roi)
        cv2.imshow(WIN, disp)

        k = cv2.waitKey(20) & 0xFF
        if k in (27, ord("q")):
            break

        if k == ord("r"):
            assigned.clear()
            last_roi = None
            print("Reset: assignations effacées.")

        elif k in KEY_TO_COLOR and last_roi is not None:
            cname = KEY_TO_COLOR[k]
            ranges = _suggest_ranges_from_stats(last_roi)
            assigned[cname] = {
                "ranges": _serialize_ranges(ranges),
                "roi_stats": {
                    "rect": list(last_roi["rect"]),
                    "mean": [float(v) for v in last_roi["mean"]],
                    "std": [float(v) for v in last_roi["std"]],
                    "n": int(last_roi["n"]),
                    "min_sv": list(last_roi["min_sv"]),
                    "k_sigma": float(last_roi["k_sigma"]),
                },
            }
            print(f"Couleur '{cname}' assignée: {assigned[cname]['ranges']}")

        elif k == ord("e"):
            if not assigned:
                print("Rien à exporter (assigne au moins une couleur avec 1..9).")
                continue
            payload = {
                "meta": {
                    "generated_at": datetime.now().isoformat(timespec="seconds"),
                    "opencv_hsv": {"H": [0, 179], "S": [0, 255], "V": [0, 255]},
                    "k_sigma": float(K_SIGMA),
                    "min_s": int(MIN_S),
                    "min_v": int(MIN_V),
                },
                "colors": assigned,
            }
            with open(out_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2)
            print("Export ->", out_path)

        time.sleep(0.001)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

