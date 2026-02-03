import numpy as np
import matplotlib.pyplot as plt

## On suppose que l'aire de travil est un cube de 100x100x100 unités

# Nombre max de bloc pour la planification optimale (bruteforce) O(n!)
MAX_BLOCS_OPTIMAL = 7
speed_joint_move_global = 100.0  # unité/sec
speed_approach_move_global = 15.0  # unité/sec

# Array de tuple représentant des blocs avec (couleur, position_x, position_y, angle), pour le test
""""
blocs = np.array([
    ("red",15, 40),
    ("blue", 80, 90),
    ("green", 50, 30),
    ("yellow", 30, 90), 
    ("red", 70, 20)
    ], dtype=object)
"""
blocs = np.array([
    ("red","2x4",15.0, 40.0, 0.0),
    ("blue", "2x2", 80.0, 90.0, 30.0),
    ("green", "2x4", 50.0, 30.0, 0.0),
    ("yellow", "2x2", 30.0, 90.0, 80.0), 
    ("red", "2x4", 70.0, 20.0, 60.0),
    ("yellow", "2x8", 20.0, 90.0, 0.0),
    ("green", "1x4", 50.0, 90.0, 20.0),
    ("blue", "2x2", 60.0, 60.0, 0.0),
    ("red", "2x2", 5.0, 0.0, 10.0)
    ], dtype=object)

home_position = (50, 50, 70)  # Position de départ du robot
red_output_position = (25, 0, 0)  # Position de sortie pour les blocs rouges
blue_output_position = (75, 0, 0)  # Position de sortie pour les blocs bleus
green_output_position = (25, 100, 0)  # Position de sortie pour
yellow_output_position = (75, 100, 0)  # Position de sortie pour les blocs jaunes
