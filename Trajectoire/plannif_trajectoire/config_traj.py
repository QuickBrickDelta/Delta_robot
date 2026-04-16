import numpy as np
import matplotlib.pyplot as plt

# --- Paramètres globaux ---
ROBOT_ROTATION_OFFSET_DEG = -30.0    # Ajuster si les axes X/Y sont décalés par rapport au châssis
MAX_BLOCS_OPTIMAL = 7
speed_joint_move_global = 55.0  # cm/s
speed_approach_move_global = 25.0  # cm/s
speed_approach_hub = 10.0
DETECTION_X_OFFSET_CM = 1.3  # Offset X (cm) appliqué aux positions détectées → valeur écrite dans detected_blocks.json
DETECTION_Y_OFFSET_CM = 1  # Offset Y (cm) appliqué aux positions détectées
OFFSET_X_MULTIPLICATOR = 1.1  # Multiplicateur X (Correction d'échelle)
OFFSET_Y_MULTIPLICATOR = 1.1  # Multiplicateur Y (Correction d'échelle)
WRIST_ANGLE_OFFSET_DEG = -70  # Décalage en degrés de la pince par rapport à la vue caméra

# --- Calibration Servos PWM ---
PULSE_PINCE_OUVERTE = 1500
PULSE_PINCE_FERMEE  = 1750
PULSE_WRIST_0_DEG   = 1700  # Valeur calibrée pour 0° (Ouvert)
PULSE_WRIST_90_DEG  = 550   # Valeur calibrée pour 90°
# --- Coordonnées du Robot (Centimètres) ---
# Le robot travaille en Z NEGATIF (sous les moteurs).
# Avec Lc=15 et Lb=32:
# Zone valide Z approx: [-20, -45] selon l'éloignement du centre.

z_table = -33.0    # Hauteur de la table adaptée pour Lc=15/Lb=32
z_drop = z_table + 5
z_home = -17.0        # Hauteur de sécurité (Home - Mise à jour)
z_home_intermediaire = z_table + 5.0 # Hauteur intermédiaire pour sortie/entrée clean

home_position = (0.0, 0.0, z_home)
home_intermediaire_position = (0.0, 0.0, z_home_intermediaire)

# Positions de sortie (Drop Bacs)
# 3 bacs à r=18cm, même disposition que les pignons mais décalé de 90° en XY
# Pignons: 0°, 120°, 240°  →  Drops: 90°, 210°, 330°
import math
r_drop = 25
drop_bac1_position = ( 0.0,                            r_drop,                          z_drop)  # 90°
drop_bac2_position = (-math.sin(math.radians(60))*r_drop, -math.cos(math.radians(60))*r_drop, z_drop)  # 210°
drop_bac3_position = ( math.sin(math.radians(60))*r_drop, -math.cos(math.radians(60))*r_drop, z_drop)  # 330°

# Mapping dynamique via color_mapping.json
import json
import os

mapping = {
    "red": 1,
    "blue": 2,
    "green_dark": 3,
    "green_light": 1,
    "yellow": 1,
    "orange": 2
}

try:
    map_file = os.path.join(os.path.dirname(__file__), "..", "UI_VIBECODE", "color_mapping.json")
    if os.path.exists(map_file):
        with open(map_file, "r") as f:
            mapping.update(json.load(f))
except Exception:
    pass

bacs = {
    1: drop_bac1_position,
    2: drop_bac2_position,
    3: drop_bac3_position
}

red_output_position    = bacs.get(mapping.get("red", 1), drop_bac1_position)
blue_output_position   = bacs.get(mapping.get("blue", 2), drop_bac2_position)
green_dark_output_position  = bacs.get(mapping.get("green_dark", 3), drop_bac3_position)
green_light_output_position = bacs.get(mapping.get("green_light", 1), drop_bac1_position)
yellow_output_position = bacs.get(mapping.get("yellow", 1), drop_bac1_position)
orange_output_position = bacs.get(mapping.get("orange", 2), drop_bac2_position)

# --- Blocs ---

# Fixed Z values within [z_table, z_home] (no randomness)
blocs = np.array([
    ("red", "2x4", 4.0, -5.0, z_table, 0.0),
    ("yellow", "2x8", -7.0, 8.0, z_table, 45.0),
    ("green_dark", "1x4", -5.0, 6.0, z_table, 90.0),
    ("blue", "2x2", 5.0, 5.0, z_table, -45.0),
], dtype=object)
