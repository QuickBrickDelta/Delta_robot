import numpy as np
import matplotlib.pyplot as plt

# --- Paramètres globaux ---
ROBOT_ROTATION_OFFSET_DEG = -30.0    # Ajuster si les axes X/Y sont décalés par rapport au châssis
MAX_BLOCS_OPTIMAL = 7
speed_joint_move_global = 55.0  # cm/s
speed_approach_move_global = 25.0  # cm/s
speed_approach_hub = 10.0
DETECTION_X_OFFSET_CM = 2.0  # Offset X (cm) appliqué aux positions détectées → valeur écrite dans detected_blocks.json
OFFSET_MULTIPLICATOR = 1.2

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

# --- Paramètres de géométrie ---
r_drop = 20    # Distance du centre au milieu de chaque face
r_next = 10.5  # Décalage entre les bacs sur une même face

# --- Fonctions utilitaires pour la clarté ---
def calc_bac_face(angle_deg):
    theta = np.radians(angle_deg)
    # Point milieu de la face
    m_x = r_drop * np.cos(theta)
    m_y = r_drop * np.sin(theta)
    # Vecteur tangent à la face (direction perpendiculaire au rayon)
    t_x = -np.sin(theta)
    t_y = np.cos(theta)
    
    # Retourne les 3 points : Milieu - r_next, Milieu, Milieu + r_next
    return [
        (m_x - r_next * t_x, m_y - r_next * t_y, z_drop), # Bac gauche
        (m_x, m_y, z_drop),                              # Bac milieu
        (m_x + r_next * t_x, m_y + r_next * t_y, z_drop)  # Bac droit
    ]

# Génération des 9 positions
# Face 1 (Bacs 1,2,3) à -60°
drop_bac3_position, drop_bac2_position, drop_bac1_position = calc_bac_face(-60)
# Face 2 (Bacs 4,5,6) à 180°
drop_bac6_position, drop_bac5_position, drop_bac4_position = calc_bac_face(180)
# Face 3 (Bacs 7,8,9) à 60°
drop_bac9_position, drop_bac8_position, drop_bac7_position = calc_bac_face(60)

# Mapping couleur → bac (3 bacs, 4 couleurs)
red_output_position    = drop_bac1_position   # Bac 1
blue_output_position   = drop_bac2_position   # Bac 2
green_dark_output_position  = drop_bac3_position   # Bac 3
green_light_output_position = drop_bac4_position   # Bac 4
yellow_output_position = drop_bac5_position   # Bac 5
orange_output_position = drop_bac6_position   # Bac 6

# --- Blocs ---

# Fixed Z values within [z_table, z_home] (no randomness)
blocs = np.array([
    ("red", 4.0, -5.0, z_table, 0.0),
    ("yellow", -7.0, 8.0, z_table, 45.0),
    ("green_dark", -5.0, 6.0, z_table, 90.0),
    ("blue", 5.0, 5.0, z_table, -45.0),
    ("green_light", 0.0, -10.0, z_table, 30.0),
    ("orange", -3.0, -4.0, z_table, -30.0)
], dtype=object)


import matplotlib.pyplot as plt

def main():
    # Liste pour l'affichage
    bacs = [
        drop_bac1_position, drop_bac2_position, drop_bac3_position,
        drop_bac4_position, drop_bac5_position, drop_bac6_position,
        drop_bac7_position, drop_bac8_position, drop_bac9_position
    ]

    plt.figure(figsize=(10, 10))
    plt.axhline(0, color='black', alpha=0.3)
    plt.axvline(0, color='black', alpha=0.3)
    
    # Tracer le centre
    plt.scatter(0, 0, c='black', marker='+', s=200, label='Centre Robot')

    # Tracer les bacs
    for i, pos in enumerate(bacs):
        x, y, z = pos
        plt.scatter(x, y, s=150, edgecolors='black')
        plt.text(x+0.5, y+0.5, f"B{i+1}", fontsize=12, fontweight='bold')

    plt.gca().invert_xaxis()  # Positifs à gauche, Négatifs à droite
    plt.gca().invert_yaxis()  # Inverse aussi le Y si nécessaire pour ta vue

    plt.title("Visualisation 2D des Bacs de Sortie (Triangle Équilatéral)")
    plt.xlabel("X (cm)")
    plt.ylabel("Y (cm)")
    plt.axis('equal')
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.show()

if __name__ == "__main__":
    main()