# Delta Robot — S4 Génie Robotique

> Projet de session — Baccalauréat en Génie Robotique, Université de Sherbrooke.  
> Robot delta capable de trier des blocs colorés en temps réel.

---

## Aperçu

<!-- TODO: Ajouter une photo du robot complet monté -->
| Vue générale | Flux caméra | Interface UI |
|:---:|:---:|:---:|
| ![robot](docs/img/robot_general.jpg) | ![camera](docs/img/camera_feed.jpg) | ![ui](docs/img/ui_screenshot.jpg) |

---

## Introduction

Ce projet intègre plusieurs sous-systèmes qui travaillent ensemble pour réaliser une tâche de tri automatisé :

1. **La vision numérique** détecte les blocs et leur associ une couleur, une position et un angle de rotation.
2. **La de trajectoire** calcule l'ordre optimal de ramassage et génère une séquence de points intermédiaires.
3. **La cinématique inverse** convertit les positions XYZ en angles pour les 3 servomoteurs.
4. **Une interface graphique** visualise le robot en 3D, la séquence de tri optimale et le flux caméra en temps réel.
5. **Le contrôleur OpenRB** reçoit les commandes et pilote les moteurs Dynamixel et les petits servos-moteurs de la pince.

![schema_prog](docs/img/schema_prog.jpg)

---

## 🗂️ Structure des dossiers

```
Delta_robot/
├── Arduino/            # Firmware embarqué sur l'OpenRB-150
├── CinématiqueRobot/   # Modèle géométrique direct & inverse
├── Communication/      # Envoi des commandes Raspberry Pi → Arduino
├── Trajectoire/        # Planification de trajectoire & optimisation
├── UI/                 # Interface graphique PyQt6 (VibeCode UI)
└── VisionNumerique/    # Détection des blocs par vision HSV
```

---

## 📁 Détail des modules

### 🔵 `VisionNumerique/`
Détection des blocs colorés par traitement d'image OpenCV.

- **`HSV/`** — Pipeline principal de détection
  - `bloc_detection_w_filter.py` — Détecteur actif (filtre LAB + HSV + géométrie rectangulaire)
  - `hsv_ranges.json` — Plages HSV calibrées par couleur (`red`, `blue`, `green_dark`, `green_light`, `yellow`, `orange`)
  - `calibrate_hsv_ranges.py` — Outil de calibration interactive des couleurs
- **`calibration/`** — Calibration spatiale caméra ↔ plan de travail
  - `calculate_homography.py` — Calcule la matrice d'homographie pixel → cm (4 clics sur un rectangle réel)
  - `homography_plane.npz` — Matrice H sauvegardée (utilisée par tous les modules)
- **`lab/`** — Explorateur alternatif basé sur l'espace colorimétrique CIELAB *(expérimental)*

<!-- TODO: Ajouter une capture du feed caméra avec les bounding boxes colorées -->

---

### 🟠 `Trajectoire/`
Planification de l'ordre optimal de ramassage des blocs.

- **`plannif_trajectoire/`**
  - `plannif_trajectoire.py` — Point d'entrée : génère la trajectoire complète à partir d'une liste de blocs
  - `shortest_path_algorithms.py` — Algorithme Branch & Bound (BnB) pour minimiser la distance totale parcourue
  - `other_fct_traj.py` — Calcul des distances et des positions de sortie par couleur
  - `config_traj.py` — Paramètres globaux : hauteur table, positions des bacs de dépôt, vitesses
  - `animation_traj.py` — Visualisation matplotlib de la trajectoire planifiée

<!-- TODO: Ajouter une image de la trajectoire planifiée (matplotlib) -->

---

### 🟢 `CinématiqueRobot/`
Modèle géométrique du robot delta 3 bras.

- `Cinematique_delta3bras.py` — Cinématique **directe** et **inverse** : `(x, y, z)` ↔ `(θ1, θ2, θ3)`
- `MouvementRobot.py` — Interpolations linéaire et articulaire entre points de passage
- `MouvementConnecte.py` — Orchestre la trajectoire complète : charge les blocs détectés, planifie, interpole et génère la liste finale de commandes angulaires

---

### 🔴 `Communication/`
Envoi des commandes du Raspberry Pi vers l'Arduino.

- `PieToArduino.py` — Lit les commandes générées par `MouvementConnecte`, sélectionne le port série, et envoie les angles + état pince en temps réel via USB/Bluetooth

---

### 🟣 `Arduino/`
Firmware embarqué sur la carte **OpenRB-150**.

- **`OpenRB_DeltaRobot/`** — Sketch principal : reçoit les commandes série et pilote les 3 servomoteurs Dynamixel + la pince électromagnétique
- `DiagnosticPince/`, `TestPince/` — Utilitaires de test des actionneurs

---

### 💻 `UI/`
Interface graphique de contrôle et de visualisation.

- `main.py` — Application PyQt6 :
  - Flux caméra en direct avec overlay des blocs détectés
  - Visualisation 3D du robot en animation (matplotlib)
  - Affichage de la position `(X, Y, Z)` en temps réel
  - Bouton **DÉMARRER** : recalcule la trajectoire à partir des blocs visibles et lance le mouvement
- `detected_blocks.json` — Fichier intermédiaire — les blocs vus par la caméra sont écrits ici avant d'être passés au planificateur

<!-- TODO: Ajouter une capture de l'UI complète (3D + caméra) -->

---

## Vision numérique

La caméra est montée au-dessus de la surface de travail et communique avec le Raspberry Pi via un pipeline **GStreamer** (libcamera, résolution 1920×1080).

Chaque image est traitée pour :
1. Masquer les pixels correspondant aux plages HSV de chaque couleur
2. Filtrer les contours non-rectangulaires (critère de solidité et ratio d'aire)
3. Vérifier l'uniformité de couleur dans la région (espace LAB)
4. Convertir les coordonnées pixel du centre en **centimètres** via la matrice d'homographie

<!-- TODO: Schéma du pipeline de détection (masque HSV → contours → homographie) -->

---

## Planification de trajectoire

Le planificateur reçoit une liste de blocs `(couleur, type, x, y)` et :
1. Calcule le coût de chaque séquence possible (distance totale)
2. Utilise un algorithme **Branch & Bound** pour trouver l'ordre optimal sans explorer toutes les permutations
3. Retourne une trajectoire ordonnée incluant les points de passage (home → bloc → bac de dépôt → home…)

---

## Cinématique inverse

Le modèle géométrique transforme une position cartésienne `(x, y, z)` en angles `(θ1, θ2, θ3)` pour les 3 servomoteurs du robot delta, en résolvant analytiquement les contraintes de chaque bras parallèle.

<!-- TODO: Ajouter une figure du schéma cinématique du bras -->

---

## Séquence d'opération

```
1. Lancer main.py (UI)
2. La caméra détecte les blocs en temps réel
3. L'utilisateur appuie sur ▶ DÉMARRER
4. Les blocs visibles sont sauvegardés → detected_blocks.json
5. Le planificateur calcule l'ordre optimal
6. La cinématique inverse génère les commandes angulaires
7. PieToArduino envoie les commandes à l'Arduino
8. Les moteurs Dynamixel exécutent le mouvement
```

---

## ⚙️ Dépendances

| Environnement | Dépendances |
|---|---|
| Raspberry Pi | `opencv-python`, `numpy`, `pyserial` (+ GStreamer/libcamera) |
| PC (dev/sim) | `PyQt6`, `matplotlib`, `numpy`, `opencv-python`, `mediapipe` |
| Arduino | Bibliothèque Dynamixel SDK |

---

## Conclusion

Ce projet V1 démontre l'intégration d'un pipeline perception-planification-action complet sur un robot delta réel.  
Les axes de développement futurs incluent : calibration automatique des axes caméra/robot, détection plus robuste sous éclairage variable, et optimisation du temps de cycle.

---

*Université de Sherbrooke — GRO S4 — 2025*
