import numpy as np
import matplotlib.pyplot as plt

# --- Paramètres globaux ---
ROBOT_ROTATION_OFFSET_DEG = -30.0    # Ajuster si les axes X/Y sont décalés par rapport au châssis
ROBOT_SCALE_X = 1.0                 # Multiplicateur X (ex: 1.05 = +5% de distance)
ROBOT_SCALE_Y = 1.0                 # Multiplicateur Y
MAX_BLOCS_OPTIMAL = 7
speed_joint_move_global = 40.0  # cm/s
speed_approach_move_global = 15.0  # cm/s

# --- Coordonnées du Robot (Centimètres) ---
# Le robot travaille en Z NEGATIF (sous les moteurs).
# Avec Lc=15 et Lb=32:
# Zone valide Z approx: [-20, -45] selon l'éloignement du centre.

z_table = -35.0    # Hauteur de la table adaptée pour Lc=15/Lb=32
z_drop = z_table + 5
z_home = -19.0        # Hauteur de sécurité (Home - Mise à jour)
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

# Mapping couleur → bac (3 bacs, 4 couleurs)
red_output_position    = drop_bac1_position   # Bac 1
blue_output_position   = drop_bac2_position   # Bac 2
green_dark_output_position  = drop_bac3_position   # Bac 3
green_light_output_position = drop_bac1_position   # Bac 1 (partagé avec red)
yellow_output_position = drop_bac1_position   # Bac 1 (partagé avec red)
orange_output_position = drop_bac2_position   # Bac 2 (partagé avec blue)

# --- Blocs ---

# Fixed Z values within [z_table, z_home] (no randomness)
blocs = np.array([
    ("red","2x4",  4.0,  -5.0, z_table),
    ("yellow","2x8", -7.0,  8.0, z_table),
    ("green_dark","1x4",   -5.0,  6.0, z_table),
    ("blue","2x2",  5.0,  5.0, z_table),
    
], dtype=object)
