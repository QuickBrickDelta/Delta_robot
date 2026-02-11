import numpy as np
import matplotlib.pyplot as plt

# --- Paramètres globaux ---
MAX_BLOCS_OPTIMAL = 7
speed_joint_move_global = 100.0  # cm/s
speed_approach_move_global = 15.0  # cm/s

# --- Coordonnées du Robot (Centimètres) ---
# Le robot travaille en Z NEGATIF (sous les moteurs).
# Avec Lc=15 et Lb=32:
# Zone valide Z approx: [-20, -45] selon l'éloignement du centre.

z_table = -40.0       # Hauteur de la table adaptée pour Lc=15/Lb=32
z_home = -25.0        # Hauteur de sécurité (Home)

home_position = (0.0, 0.0, z_home)

# Positions de sortie (Output)
# On réduit l'écartement car le robot est plus petit
offset_output = 8.0
red_output_position = (offset_output, 0.0, z_table)
blue_output_position = (-offset_output, 0.0, z_table)
green_output_position = (0.0, offset_output, z_table)
yellow_output_position = (0.0, -offset_output, z_table)

# --- Blocs ---

# Fixed Z values within [z_table, z_home] (no randomness)
blocs = np.array([
    ("red",    "2x4",   -10.0,   2.0, -30.0),
    ("blue",   "2x2",  8.0,   0.0, -30.0),
    ("green",  "2x4",   6.0,  8.0, -30.0),
    ("yellow", "2x2", -9.0, -7.0, -30.0),
    ("red",    "2x4",  4.0,  -5.0, -30.0),
    ("yellow", "2x8", -9.0,  8.0, -30.0),
    ("green",  "1x4",   -5.0,  6.0, -30.0),
    ("blue",   "2x2",  5.0,  6.0, -30.0),
    ("red",    "2x2",  -5.0,   0.0, -30.0)
], dtype=object)
