#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
picam2_focus_manual.py
-----------------------------------
Aperçu caméra Module 3 (IMX708) avec focus manuel.
Utilise Picamera2 à la place de libcamerasrc.

Touches :
  q    = quitter
  j    = diminuer LensPosition (focus plus loin)
  k    = augmenter LensPosition (focus plus proche)

LensPosition = dioptries :
  - 0   = infini
  - 10  = ≈ 10 cm
"""

import time
import cv2
from picamera2 import Picamera2
from libcamera import controls

# ---------------- CONFIG ----------------
WIDTH = 1920
HEIGHT = 1080
FPS = 5.0

# Distance d'intérêt (≈30–40 cm → ~2.0 à 3.3)
LENS_POS_INIT = 3.0           # 3.0 → environ 33 cm
LENS_STEP = 0.1               # incrément avec j/k

# -----------------------------------------
def main():

    picam2 = Picamera2()

    # Configuration du flux :
    config = picam2.create_video_configuration(
        main={"size": (WIDTH, HEIGHT), "format": "BGR888"},
        controls={"FrameRate": FPS}
    )
    picam2.configure(config)

    # --- Mise au point MANUELLE ---
    # Selon docs RPi : AfMode=Manual + LensPosition en dioptries
    picam2.set_controls({
        "AfMode": controls.AfModeEnum.Manual,
        "LensPosition": float(LENS_POS_INIT)
    })

    picam2.start()
    time.sleep(0.2)  # petit délai pour que les premiers frames soient stables

    lens_pos = LENS_POS_INIT

    print("=== APERÇU CAMERA (Picamera2) ===")
    print("LensPosition (dioptries) : 0=infini, 10≈10cm  (RPi engineering)")

    while True:
        frame = picam2.capture_array("main")

        # overlay texte
        txt = f"LensPosition = {lens_pos:.2f} (dioptries)"
        cv2.putText(frame, txt, (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

        cv2.imshow("Picamera2 Focus Manual", frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('j'):
            lens_pos = max(0.0, lens_pos - LENS_STEP)
            picam2.set_controls({
                "AfMode": controls.AfModeEnum.Manual,
                "LensPosition": float(lens_pos)
            })
        elif key == ord('k'):
            lens_pos = min(10.0, lens_pos + LENS_STEP)
            picam2.set_controls({
                "AfMode": controls.AfModeEnum.Manual,
                "LensPosition": float(lens_pos)
            })

    picam2.stop()
    picam2.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()