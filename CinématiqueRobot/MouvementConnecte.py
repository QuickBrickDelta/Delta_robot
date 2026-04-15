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

import json

# Trajectoire haut niveau (positions XYZ + info pince)
detected_path = os.path.join(project_root, "UI_VIBECODE", "detected_blocks.json")
blocs = config_traj.blocs

if os.path.exists(detected_path):
    print(f"Chargement des blocs détectés depuis {detected_path}")
    try:
        with open(detected_path, 'r') as f:
            data = json.load(f)
            if data and len(data) > 0:
                # Convertir en tableau 2D d'objets comme dans config_traj
                blocs = numpy.array(data, dtype=object)
    except Exception as e:
        print(f"Erreur de lecture des blocs: {e}")

Trajectory, blocs_sorted = plannif_trajectoire.plan_full_trajectory(blocs)

# Commandes pour la simulation (XYZ + mode L/J/G)
Motor_command_xyz = []
pince_states = []

# Commandes bas niveau pour la communication :
# liste dense de [angle1, angle2, angle3, pince_fermee] après interpolation
Motor_command_angles = []

# Nombre de commandes de pause pour laisser la pince s'ouvrir/fermer
# 8 steps × 50ms = 0.4s de délai (Divisé par ~3)
GRIPPER_HOLD_STEPS = 8

# ===============================
# 1) Construire les waypoints XYZ + mode + pince
# ===============================
for step in Trajectory:
    # step structure:
    # (bloc_carried, bloc_type, movement_type, speed,
    #  x, y, z, angle, pince_fermee)
    move_type = step[2]
    x, y, z = float(step[4]), float(step[5]), float(step[6])
    pince_fermee = bool(step[8])

    code_mouv = None
    if move_type in ["home", "joint"]:
        code_mouv = "J"
    elif move_type == "linear":
        code_mouv = "L"
    elif move_type in ["closeGripper", "openGripper"]:
        code_mouv = "G"  # Gripper action — maintenir position + changer pince

    if code_mouv:
        angle = float(step[7])  # Récupérer l'angle depuis la trajectoire
        Motor_command_xyz.append([x, y, z, code_mouv, angle])
        pince_states.append(pince_fermee)


# ===============================
# 2) Générer la trajectoire en ANGLES + pince (interpolation)
# ===============================
if Motor_command_xyz:
    # Fréquence cible : 50Hz (20ms)
    DEFAULT_DT = 0.02 

    current_pos = Motor_command_xyz[0][:3]

    for i in range(1, len(Motor_command_xyz)):
        target_pt = Motor_command_xyz[i]
        pos_xyz = target_pt[0:3]
        mode = target_pt[3]
        angle = target_pt[4] # Récupérer l'angle
        pince_seg = pince_states[i]
        
        # Récupérer la vitesse depuis la trajectoire d'origine
        speed = float(Trajectory[i][3]) if i < len(Trajectory) else 20.0
        if speed <= 0: speed = 20.0

        if mode == "G":
            # Action pince : maintenir la position actuelle + changer l'état pince
            thetas = get_all_thetas(current_pos)
            if thetas is not None:
                theta1, theta2, theta3 = [float(t) for t in thetas]
                # Délai réduit pour la pince
                for _ in range(GRIPPER_HOLD_STEPS):
                    Motor_command_angles.append([theta1, theta2, theta3, pince_seg, angle])
            continue

        # Calcul de la distance
        dist = numpy.linalg.norm(numpy.array(pos_xyz) - numpy.array(current_pos))
        
        # Sauter les mouvements de distance zéro
        if dist < 0.01:
            current_pos = pos_xyz
            continue

        # CALCUL DYNAMIQUE DU NOMBRE DE POINTS
        # steps = (Distance / Vitesse) / DT
        # On assure un minimum de 10 points pour la fluidité même sur petits trajets
        steps_dynamic = max(10, int((dist / speed) / DEFAULT_DT))

        # Choix de l'interpolation (joint avec fallback linéaire)
        if mode == "J":
            # Vérifier si les deux extrémités sont atteignables avant joint
            start_ok = get_all_thetas(current_pos) is not None
            end_ok = get_all_thetas(pos_xyz) is not None
            if start_ok and end_ok:
                traj_points = interpolate_joint(current_pos, pos_xyz, steps_dynamic)
            else:
                traj_points = interpolate_linear(current_pos, pos_xyz, steps_dynamic)
        else:
            traj_points = interpolate_linear(current_pos, pos_xyz, steps_dynamic)

        for pos in traj_points:
            thetas = get_all_thetas(pos)
            if thetas is None:
                continue

            theta1, theta2, theta3 = [float(t) for t in thetas]
            Motor_command_angles.append([theta1, theta2, theta3, pince_seg, angle])

        # Diagnostic léger toutes les 50 commandes générées
        if len(Motor_command_angles) % 50 == 0:
            print(f"  [DEBUG] Cmd #{len(Motor_command_angles)}: XYZ=({pos_xyz[0]:.1f}, {pos_xyz[1]:.1f}, {pos_xyz[2]:.1f}) P={pince_seg} W={angle:.1f}°")

        current_pos = pos_xyz

    # Debug : afficher les transitions de pince
    print(f"Total commandes: {len(Motor_command_angles)}")
    prev_p = None
    for idx, cmd in enumerate(Motor_command_angles):
        p = cmd[3]
        if p != prev_p:
            print(f"  Cmd #{idx}: pince={'FERME' if p else 'OUVERT'}")
            prev_p = p


# Lancer la simulation si exécuté directement
if __name__ == "__main__":
    try:
        run_simulation_realtime(Motor_command_xyz)
    except Exception as e:
        print(f"Erreur lors du lancement de la simulation: {e}")
