#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
flash_ambient_filter_manual.py
Capture manuelle du couple OFF/ON (LEDs éteintes/allumées) pour construire un filtre
d'atténuation de lumière ambiante. Aperçu live + touches de contrôle :

  - 'o' : capturer OFF (LEDs éteintes)
  - 'l' : capturer ON  (LEDs allumées)
  - 'c' : calculer le filtre (si OFF et ON présents)
  - 's' : sauvegarder le filtre (ambient_ref.png, led_contrib.png, led_mask.png, flash_params.npz)
  - 'f' : basculer l'aperçu entre Original et Filtré (si filtre calculé)
  - 'r' : réinitialiser (efface OFF/ON/filtre en mémoire)
  - 'q' : quitter

Dépendances:
    pip install opencv-python numpy
"""

import os
import time
import json
from typing import Dict, Any, Tuple
import cv2
import numpy as np

# ---------- Réglages caméra ----------
PIPELINE = (
    "libcamerasrc ! "
    "video/x-raw,format=NV12,width=4608,height=2592,framerate=5/1 ! "
    "videoscale ! video/x-raw,width=1920,height=1080 ! "
    "queue max-size-buffers=1 leaky=downstream ! "
    "videoconvert ! video/x-raw,format=BGR ! "
    "appsink drop=true max-buffers=1 sync=false"
)

# ---------- Réglages filtre ----------
SAVE_DIR = "calib_flash"
os.makedirs(SAVE_DIR, exist_ok=True)

ALPHA_AMBIENT = 0.85     # force de soustraction de l'ambiante (0.6..1.2 typique)
GAUSS_BLUR_K = (5, 5)    # anti-bruit
MIN_LED_DELTA = 4        # delta min (niveaux) pour considérer une vraie contribution LED
MASK_SMOOTH_K = 21       # lissage du masque LED
MASK_GAMMA = 0.7         # renforcement doux du masque (0.6..0.9)
ECC_ALIGN = False        # activer l'alignement ECC ON->OFF si nécessaire (lent mais utile si bougé)

# ---------- Utilitaires ----------
def open_cap():
    cap = cv2.VideoCapture(PIPELINE, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise SystemExit("Erreur: impossible d'ouvrir la caméra via GStreamer.")
    return cap

def grab_stable_frame(cap, n_discard=2, wait=0.02):
    for _ in range(n_discard):
        cap.read()
        time.sleep(wait)
    ok, frame = cap.read()
    if not ok or frame is None:
        raise RuntimeError("Capture frame échouée.")
    return frame

def gaussian(img):
    return cv2.GaussianBlur(img, GAUSS_BLUR_K, 0)

def align_ecc(template_bgr: np.ndarray, moving_bgr: np.ndarray) -> np.ndarray:
    """
    Aligne moving_bgr sur template_bgr (niveau de gris, transformation euclidienne).
    Retourne l'image alignée. Si échec, retourne moving_bgr.
    """
    try:
        im1 = cv2.cvtColor(template_bgr, cv2.COLOR_BGR2GRAY)
        im2 = cv2.cvtColor(moving_bgr,  cv2.COLOR_BGR2GRAY)
        im1 = cv2.equalizeHist(im1)
        im2 = cv2.equalizeHist(im2)

        warp_mode = cv2.MOTION_EUCLIDEAN
        warp_matrix = np.eye(2, 3, dtype=np.float32)
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 80, 1e-5)
        cc, warp_matrix = cv2.findTransformECC(im1, im2, warp_matrix, warp_mode, criteria, None, 5)
        aligned = cv2.warpAffine(moving_bgr, warp_matrix, (moving_bgr.shape[1], moving_bgr.shape[0]),
                                 flags=cv2.INTER_LINEAR + cv2.WARP_INVERSE_MAP)
        return aligned
    except Exception:
        return moving_bgr

def compute_filter(img_off_bgr: np.ndarray, img_on_bgr: np.ndarray) -> Dict[str, Any]:
    # Optionnel: aligner ON sur OFF si la scène a légèrement bougé
    if ECC_ALIGN:
        img_on_bgr = align_ecc(img_off_bgr, img_on_bgr)

    img_off = gaussian(img_off_bgr)
    img_on  = gaussian(img_on_bgr)

    # Contribution LED = max(0, ON - OFF)
    led_contrib = cv2.subtract(img_on, img_off)
    led_contrib = np.where(led_contrib < MIN_LED_DELTA, 0, led_contrib).astype(np.uint8)

    ambient_ref = img_off.copy()

    # Masque LED 0..1
    eps = 1.0
    on_f  = img_on.astype(np.float32)
    off_f = img_off.astype(np.float32)
    delta = np.clip(on_f - off_f, 0, None)
    denom = np.maximum(on_f, eps)
    ratio = delta / denom
    led_mask = np.mean(ratio, axis=2)  # fusion canaux
    led_mask = cv2.GaussianBlur(led_mask, (MASK_SMOOTH_K, MASK_SMOOTH_K), 0)
    led_mask = np.clip(led_mask, 0.0, 1.0)
    led_mask = led_mask ** MASK_GAMMA

    return {
        "ambient_ref": ambient_ref,
        "led_contrib": led_contrib,
        "led_mask": led_mask.astype(np.float32),
        "alpha": float(ALPHA_AMBIENT),
    }

def robust_gain(src_f: np.ndarray, ref_f: np.ndarray, samp_step=16) -> np.ndarray:
    s = src_f[::samp_step, ::samp_step, :].reshape(-1, 3).astype(np.float32) + 1.0
    r = ref_f[::samp_step, ::samp_step, :].reshape(-1, 3).astype(np.float32) + 1.0
    gain = np.median(s / r, axis=0)
    gain = np.clip(gain, 0.5, 2.0)
    return gain

def apply_ambient_filter(frame_bgr: np.ndarray, filt: Dict[str, Any]) -> np.ndarray:
    ambient = filt["ambient_ref"].astype(np.float32)
    mask    = filt["led_mask"].astype(np.float32)  # 0..1
    alpha   = float(filt["alpha"])
    f = frame_bgr.astype(np.float32)

    # Ajuster l'ambiante si l'exposition globale a un peu varié
    gain = robust_gain(f, ambient)
    ambient_adj = np.clip(ambient * gain.reshape(1,1,3), 0, 255)

    # Pondération spatiale: (1 - mask) => plus on est hors zone LED, plus on retire d'ambiante
    w = (1.0 - mask)[..., None]
    ambient_term = alpha * w * ambient_adj

    out = f - ambient_term
    out = np.clip(out, 0, 255).astype(np.uint8)
    return out

def save_filter(filt: Dict[str, Any], save_dir: str = SAVE_DIR):
    os.makedirs(save_dir, exist_ok=True)
    ambient_ref = filt["ambient_ref"]
    led_contrib = filt["led_contrib"]
    led_mask    = filt["led_mask"]
    alpha       = float(filt["alpha"])

    cv2.imwrite(os.path.join(save_dir, "ambient_ref.png"), ambient_ref)
    cv2.imwrite(os.path.join(save_dir, "led_contrib.png"), led_contrib)
    cv2.imwrite(os.path.join(save_dir, "led_mask.png"),
                (255.0 * np.clip(led_mask, 0, 1)).astype(np.uint8))
    np.savez_compressed(os.path.join(save_dir, "flash_params.npz"),
                        ambient_ref=ambient_ref,
                        led_mask=led_mask.astype(np.float32),
                        alpha=np.array([alpha], dtype=np.float32))
    with open(os.path.join(save_dir, "flash_params.json"), "w") as f:
        json.dump({"alpha_ambient": alpha}, f, indent=2)

# ---------- Main interactif ----------
def main():
    cap = open_cap()
    print("=== Calibration flash/no-flash (manuel) ===")
    print("Étapes :")
    print("  1) ÉTEINS tes LEDs, cadre la scène, puis appuie sur 'o' pour CAPTURER OFF.")
    print("  2) ALLUME tes LEDs, puis appuie sur 'l' pour CAPTURER ON.")
    print("  3) Appuie sur 'c' pour CALCULER le filtre.")
    print("  4) Appuie sur 'f' pour basculer Original/Filtré (prévisualisation).")
    print("  5) Appuie sur 's' pour SAUVEGARDER. 'r' pour réinitialiser. 'q' pour quitter.")

    win = "flash/manual"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    show_filtered = False

    img_off = None
    img_on  = None
    filt    = None

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        disp = frame.copy()
        h, w = frame.shape[:2]

        # Overlay état
        status = []
        status.append(f"OFF: {'OK' if img_off is not None else '---'}")
        status.append(f"ON: {'OK' if img_on  is not None else '---'}")
        status.append(f"FILTR: {'OK' if filt is not None else '---'}")
        status.append(f"VIEW: {'filtered' if (show_filtered and filt is not None) else 'original'}")
        y0 = 24
        for txt in status:
            cv2.putText(disp, txt, (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2, cv2.LINE_AA)
            y0 += 26

        help1 = "o: capture OFF | l: capture ON | c: compute | f: toggle view | s: save | r: reset | q: quit"
        cv2.putText(disp, help1, (10, h-14), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2, cv2.LINE_AA)
        cv2.putText(disp, help1, (10, h-14), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 1, cv2.LINE_AA)

        # Affichage filtré si demandé et filtre dispo
        if show_filtered and (filt is not None):
            disp = apply_ambient_filter(frame, filt)

        cv2.imshow(win, disp)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('o'):
            # demander de bien éteindre les LEDs
            print("[OFF] Assure-toi que les LEDs sont ÉTEINTES... capture dans 0.2 s")
            time.sleep(0.2)
            img_off = grab_stable_frame(cap)
            print("[OFF] Capturé.")
            filt = None  # invalider filtre courant
        elif key == ord('l'):
            # demander d’allumer les LEDs
            print("[ON] Assure-toi que les LEDs sont ALLUMÉES... capture dans 0.2 s")
            time.sleep(0.2)
            img_on = grab_stable_frame(cap)
            print("[ON] Capturé.")
            filt = None
        elif key == ord('c'):
            if img_off is None or img_on is None:
                print("[WARN] Capture OFF/ON incomplète.")
            else:
                filt = compute_filter(img_off, img_on)
                print("[OK] Filtre calculé. Tu peux appuyer sur 'f' pour voir l'effet, 's' pour sauvegarder.")
        elif key == ord('f'):
            show_filtered = not show_filtered
        elif key == ord('s'):
            if filt is None:
                print("[WARN] Aucun filtre calculé.")
            else:
                save_filter(filt, SAVE_DIR)
                print(f"[OK] Sauvegardé dans {SAVE_DIR}/ (ambient_ref.png, led_contrib.png, led_mask.png, flash_params.npz)")
        elif key == ord('r'):
            img_off = None
            img_on  = None
            filt    = None
            print("[RESET] Réinitialisé.")

    cap.release()
    cv2.destroyAllWindows()

# ---------- Point d'entrée ----------
if __name__ == "__main__":
    main()