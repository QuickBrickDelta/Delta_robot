#!/usr/bin/env python3
import cv2
import numpy as np
from pathlib import Path

## Calibration de la caméra pour obtenir l'homographie entre les pixels et les coordonnées du plan de travail (en cm).
## On clique sur les 4 coins d'un rectangle de dimensions réelles connues (ex: cellulaire posée à plat),
#    et on sauvegarde l'homographie dans un fichier .npz pour l'utiliser dans le projet de détection d'objets.

# --- Use the SAME pipeline as your detector ---
pipeline = (
    'libcamerasrc af-mode=manual lens-position=3.4 ! '
    'video/x-raw,format=NV12,width=4608,height=2592,framerate=5/1 ! '
    'videoscale ! video/x-raw,width=1920,height=1080 ! '
    'queue max-size-buffers=1 leaky=downstream ! '
    'videoconvert ! video/x-raw,format=BGR ! '
    'appsink drop=true max-buffers=1 sync=false'
)

SAVE_PATH = Path(__file__).resolve().parent / "homography_plane.npz"

# Click 4 points: TL, TR, BR, BL
clicked = []
def on_mouse(event, x, y, flags, param):
    global clicked
    if event == cv2.EVENT_LBUTTONDOWN and len(clicked) < 4:
        clicked.append((x, y))
        print(f"Clicked {len(clicked)}: ({x}, {y})")

def main():
    global clicked

    W_cm = float(input("Largeur réelle du rectangle (en cm) (ex: 29.7) : ").strip())
    H_cm = float(input("Hauteur réelle du rectangle (en cm) (ex: 21.0) : ").strip())
    print("\nClique les 4 coins dans l'ordre: TL, TR, BR, BL.\nAppuie sur 'r' pour reset si tu t'es trompé.\n'appuie sur 'q' pour quitter sans sauver.\n")

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise SystemExit("Impossible d'ouvrir la caméra via GStreamer.")

    win = "Calibration homography (click TL,TR,BR,BL)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, on_mouse)

    H_img, W_img = None, None

    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        if H_img is None:
            H_img, W_img = frame.shape[:2]

        # draw current clicks
        disp = frame.copy()
        for i, (x, y) in enumerate(clicked):
            cv2.circle(disp, (x, y), 6, (0, 255, 255), -1)
            cv2.putText(disp, f"{i+1}", (x+8, y-8), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)

        cv2.imshow(win, disp)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('r'):
            clicked = []
            print("Reset points.")
        elif key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            return
        elif len(clicked) == 4:
            # Pixel points (u,v)
            pts_img = np.array(clicked, dtype=np.float32)

            # World points in cm for TL,TR,BR,BL
            # We'll define world frame with the RECT center at (0,0) for calibration,
            # then later re-center to camera center.
            halfW = W_cm / 2.0
            halfH = H_cm / 2.0
            pts_wrld = np.array([
                [-halfW, -halfH],  # TL
                [ halfW, -halfH],  # TR
                [ halfW,  halfH],  # BR
                [-halfW,  halfH],  # BL
            ], dtype=np.float32)

            # Compute homography: pixels -> world(cm)
            H, mask = cv2.findHomography(pts_img, pts_wrld, method=cv2.RANSAC)
            if H is None:
                print("Echec: homography non calculée.")
                clicked = []
                continue

            # Save H and also save the world coordinate of image center so we can set camera-center (0,0)
            img_center = np.array([ [W_img/2.0, H_img/2.0] ], dtype=np.float32)
            one = np.array([[1.0]], dtype=np.float32)
            ph = H @ np.vstack([img_center.T, one])
            ph /= ph[2, :]
            cam_center_world = ph[:2, 0].astype(np.float32)  # (Xc, Yc) in cm

            np.savez(str(SAVE_PATH), H=H, imgW=W_img, imgH=H_img, cam_center_world=cam_center_world)
            print(f"\nHomography saved to {SAVE_PATH}")
            print(f"Image size: {W_img}x{H_img}, camera-center (world cm): {cam_center_world}")
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()