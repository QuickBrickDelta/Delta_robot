import numpy as np
import matplotlib.pyplot as plt

# --- Paramètres globaux ---
ROBOT_ROTATION_OFFSET_DEG = -30.0    # Ajuster si les axes X/Y sont décalés par rapport au châssis
MAX_BLOCS_OPTIMAL = 7
speed_joint_move_global = 80.0  # cm/s
speed_approach_move_global = 40.0  # cm/s
speed_approach_hub = 10.0
DETECTION_X_OFFSET_CM = 1.3  # Offset X (cm) appliqué aux positions détectées → valeur écrite dans detected_blocks.json
DETECTION_Y_OFFSET_CM = 1  # Offset Y (cm) appliqué aux positions détectées
OFFSET_X_MULTIPLICATOR = 1.1  # Multiplicateur X (Correction d'échelle)
OFFSET_Y_MULTIPLICATOR = 1.1  # Multiplicateur Y (Correction d'échelle)
WRIST_ANGLE_OFFSET_DEG = -10  # Décalage en degrés de la pince par rapport à la vue caméra

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
z_drop = z_table + 3
z_home = -17.0        # Hauteur de sécurité (Home - Mise à jour)
z_home_intermediaire = z_table + 5.0 # Hauteur intermédiaire pour sortie/entrée clean

home_position = (0.0, 0.0, z_home)
home_intermediaire_position = (0.0, 0.0, z_home_intermediaire)

# Positions de sortie (Drop Bacs)
# 3 bacs à r=18cm, même disposition que les pignons mais décalé de 90° en XY
# Pignons: 0°, 120°, 240°  →  Drops: 90°, 210°, 330°
import math
# --- Positions de sortie (Teachable & Auto-compensées) ---
# On compense le ROBOT_ROTATION_OFFSET_DEG pour que les bacs ne bougent pas 
# physiquement quand on ajuste la rotation du robot.
r_drop = 21.0
ang1 = 90.0  + ROBOT_ROTATION_OFFSET_DEG
ang2 = 210.0 + ROBOT_ROTATION_OFFSET_DEG
ang3 = 330.0 + ROBOT_ROTATION_OFFSET_DEG

drop_bac1_position = (r_drop * math.cos(math.radians(ang1)), r_drop * math.sin(math.radians(ang1)), z_drop)
drop_bac2_position = (r_drop * math.cos(math.radians(ang2)), r_drop * math.sin(math.radians(ang2)), z_drop)
drop_bac3_position = (r_drop * math.cos(math.radians(ang3)), r_drop * math.sin(math.radians(ang3)), z_drop)

s_offset = 10.5 # Décalage entre les slots

def get_tangent(angle_deg):
    rad = math.radians(angle_deg + 90) # Tangente = Normale + 90°
    return (math.cos(rad), math.sin(rad))

# Calcul des 9 Slots relatifs aux 3 centres
# Segment 1 (Top)
t1 = get_tangent(ang1)
p2 = drop_bac1_position
p3 = (p2[0] - s_offset*t1[0], p2[1] - s_offset*t1[1], p2[2]) # Slot 3
p1 = (p2[0] + s_offset*t1[0], p2[1] + s_offset*t1[1], p2[2]) # Slot 1

# Segment 2 (Bas-Gauche)
t2 = get_tangent(ang2)
p5 = drop_bac2_position
p4 = (p5[0] - s_offset*t2[0], p5[1] - s_offset*t2[1], p5[2]) # Slot 4
p6 = (p5[0] + s_offset*t2[0], p5[1] + s_offset*t2[1], p5[2]) # Slot 6

# Segment 3 (Bas-Droite)
t3 = get_tangent(ang3)
p8 = drop_bac3_position
p9 = (p8[0] - s_offset*t3[0], p8[1] - s_offset*t3[1], p8[2]) # Slot 9
p7 = (p8[0] + s_offset*t3[0], p8[1] + s_offset*t3[1], p8[2]) # Slot 7

# Dictionnaire final des Bacs
bacs = {
    1: p1, 2: p2, 3: p3,
    4: p4, 5: p5, 6: p6,
    7: p7, 8: p8, 9: p9
}

# Mapping dynamique via color_mapping.json
import json
import os

# Fallback mapping
mapping = {"red": 1, "blue": 2, "green_dark": 3, "green_light": 4, "yellow": 5, "orange": 6}

try:
    map_file = os.path.join(os.path.dirname(__file__), "..", "..", "UI_VIBECODE", "color_mapping.json")
    if os.path.exists(map_file):
        with open(map_file, "r") as f:
            mapping.update(json.load(f))
except Exception:
    pass

# Helper pour récupérer la position de sortie d'une couleur
def get_output_pos(color_name):
    slot_id = mapping.get(color_name, 0)
    return bacs.get(slot_id, drop_bac1_position if "drop_bac1_position" in globals() else p_top_mid)

red_output_position    = get_output_pos("red")
blue_output_position   = get_output_pos("blue")
green_dark_output_position  = get_output_pos("green_dark")
green_light_output_position = get_output_pos("green_light")
yellow_output_position = get_output_pos("yellow")
orange_output_position = get_output_pos("orange")

# --- Blocs ---

# Fixed Z values within [z_table, z_home] (no randomness)
blocs = np.array([
    ("red", "2x4", 4.0, -5.0, z_table, 0.0),
    ("yellow", "2x8", -7.0, 8.0, z_table, 45.0),
    ("green_dark", "1x4", -5.0, 6.0, z_table, 90.0),
    ("blue", "2x2", 5.0, 5.0, z_table, -45.0),
], dtype=object)
