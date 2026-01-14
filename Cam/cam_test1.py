
import cv2

# Ouvrir la caméra (0 = caméra par défaut)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Erreur : impossible d'ouvrir la caméra.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Erreur : impossible de lire la vidéo.")
        break

    # Convertir en niveaux de gris
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Détection de bords avec Canny
    edges = cv2.Canny(gray, 50, 150)

    # Afficher le résultat
    cv2.imshow("Camera", frame)
    cv2.imshow("Edges", edges)

    # Quitter si 'q' est pressé
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libérer les ressources
cap.release()
cv2.destroyAllWindows()
