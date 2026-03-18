import sys
import os
import numpy

# Configuration des chemins pour les imports
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
traj_dir = os.path.join(project_root, "Trajectoire", "plannif_trajectoire")

if project_root not in sys.path:
    sys.path.append(project_root)

if traj_dir not in sys.path:
    sys.path.append(traj_dir)

# Imports locaux
try:
    # Simulation + cinématique directe (XYZ -> angles)
    from MouvementRobot import (
        run_simulation_realtime,
        get_all_thetas,
        interpolate_linear,
        interpolate_joint,
    )
except ImportError:
    from CinématiqueRobot.MouvementRobot import (
        run_simulation_realtime,
        get_all_thetas,
        interpolate_linear,
        interpolate_joint,
    )

# Imports des modules de Trajectoire
import plannif_trajectoire
import config_traj


# ===============================
#  Génération des commandes
# ===============================

# Trajectoire haut niveau (positions XYZ + info pince)
blocs = config_traj.blocs
Trajectory = plannif_trajectoire.plan_full_trajectory(blocs)

# Commandes pour la simulation (XYZ + mode L/J)
Motor_command_xyz = []
pince_states = []

# Nouvelles commandes bas niveau pour la communication :
# liste dense de [angle1, angle2, angle3, pince_fermee] après interpolation
Motor_command_angles = []

# ===============================
# 1) Construire les waypoints XYZ + mode + pince
# ===============================
for step in Trajectory:
    # step structure:
    # (bloc_carried, bloc_type, movement_type, speed,
    #  x, y, z, angle, pince_fermee)
    # Indices utiles:
    #   2 = movement_type, 4 = x, 5 = y, 6 = z, 8 = pince_fermee
    move_type = step[2]
    x, y, z = float(step[4]), float(step[5]), float(step[6])
    pince_fermee = bool(step[8])

    code_mouv = None
    if move_type in ["home", "joint"]:
        code_mouv = "J"
    elif move_type == "linear":
        code_mouv = "L"

    if code_mouv:
        # Waypoints cartésiens pour la simulation
        Motor_command_xyz.append([x, y, z, code_mouv])
        # État de la pince associé à ce waypoint
        pince_states.append(pince_fermee)


# ===============================
# 2) Générer la trajectoire en ANGLES + pince (interpolation)
# ===============================
if Motor_command_xyz:
    # Nombre de pas d'interpolation par segment
    steps_per_move = 30

    current_pos = Motor_command_xyz[0][:3]

    for i in range(1, len(Motor_command_xyz)):
        target_pt = Motor_command_xyz[i]
        pos_xyz = target_pt[:3]
        mode = target_pt[3]

        # État de la pince pour tout ce segment
        pince_seg = pince_states[i]

        # Choix de l'interpolation (même logique que run_simulation_realtime)
        if mode == "J":
            traj_points = interpolate_joint(current_pos, pos_xyz, steps_per_move)
        else:
            traj_points = interpolate_linear(current_pos, pos_xyz, steps_per_move)

        for pos in traj_points:
            thetas = get_all_thetas(pos)
            if thetas is None:
                # Point hors limite : on ignore ce sous-point
                continue

            theta1, theta2, theta3 = [float(t) for t in thetas]
            Motor_command_angles.append([theta1, theta2, theta3, pince_seg])

        current_pos = pos_xyz


# Lancer la simulation si exécuté directement
if __name__ == "__main__":
    try:
        run_simulation_realtime(Motor_command_xyz)
    except Exception as e:
        print(f"Erreur lors du lancement de la simulation: {e}")
