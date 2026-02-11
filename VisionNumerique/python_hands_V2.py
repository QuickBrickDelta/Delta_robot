#!/usr/bin/env python3
import cv2
import mediapipe as mp

pipeline = (
    "libcamerasrc ! "
    "video/x-raw,format=NV12,width=1920,height=1080,framerate=5/1 ! "
    "videoconvert ! video/x-raw,format=BGR ! "
    "appsink drop=true max-buffers=1 sync=false"
)

def main():
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise SystemExit("Erreur : caméra inaccessible.")

    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        model_complexity=0,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )

    print("Détection active. Print lorsqu'une main est détectée.")

    while True:
        ok, frame = cap.read()
        if not ok:
            continue

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(rgb)

        # --------- 🔥 PRINT SI MAIN DÉTECTÉE ----------
        if result.multi_hand_landmarks:
            print("👉 Main détectée")
        # ------------------------------------------------

        # (Affichage optionnel - retire pour +FPS)
        cv2.imshow("Détection main", frame)
        if cv2.waitKey(1) == ord('q'):
            break

    hands.close()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()