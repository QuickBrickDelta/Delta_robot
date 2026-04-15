import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import itertools

import config_traj  # Pour les variables globales
import animation_and_plot_traj # Pour l'animation (utilise cette même file)
import shortest_path_algorithms  # Pour les algorithmes de planification
import other_fct_traj  # Pour les fonctions utilitaires

# Importer les variables globales depuis config_traj.py
blocs = config_traj.blocs
home_position = config_traj.home_position
red_output_position = config_traj.red_output_position
blue_output_position = config_traj.blue_output_position
green_dark_output_position = config_traj.green_dark_output_position
green_light_output_position = config_traj.green_light_output_position
yellow_output_position = config_traj.yellow_output_position
orange_output_position = config_traj.orange_output_position
speed_joint_move_global = config_traj.speed_joint_move_global
speed_approach_move_global = config_traj.speed_approach_move_global
speed_approach_hub = config_traj.speed_approach_hub

# Importer les fonctions depuis animation_and_plot_traj.py
plot_blocks_2D = animation_and_plot_traj.plot_blocks_2D
plot_blocks_3D = animation_and_plot_traj.plot_blocks_3D
plot_route_2D = animation_and_plot_traj.plot_route_2D
animate_full_trajectory_2D = animation_and_plot_traj.animate_full_trajectory_2D
animate_full_trajectory_3D = animation_and_plot_traj.animate_full_trajectory_3D

# Importer les fonctions depuis other_fct_traj.py
distance_from_output = other_fct_traj.distance_from_output
distance_between_points = other_fct_traj.distance_between_points
distance_between_3_points = other_fct_traj.distance_between_3_points
output_pos_for_color = other_fct_traj.output_pos_for_color
cost_do_bloc_from = other_fct_traj.cost_do_bloc_from

# Importer les fonctions depuis shortest_path_algorithms.py
from shortest_path_algorithms import plan_bnb_basic, plan_exact_tsp, plan_cheapest_insertion

## full trajectoire function
def plan_full_trajectory(blocs):
    # path: [(bloc_carried, bloc_type, movement_type, speed, x, y, z, angle, pince_fermee), ...]
    path = []
    speed_joint = speed_joint_move_global
    speed_approach = speed_approach_move_global
    distance_approach = 10.0

    # Trier les blocs selon un algorithme de planification
    if len(blocs) < 11:
        blocs_sorted, total_distance = plan_bnb_basic(blocs, home_position)
    elif len(blocs) < 14:
        blocs_sorted, total_distance = plan_exact_tsp(blocs, home_position)
    else:
        blocs_sorted, total_distance = plan_cheapest_insertion(blocs, home_position)

    # 1) Départ au home (Haut)
    path.append((None, None, "home", speed_approach_hub,
                 home_position[0], home_position[1], home_position[2], 0.0, False))

    # 1b) Point de descente centrale (sortie hub - rentrée smooth)
    path.append((None, None, "joint", speed_approach_hub,
                 config_traj.home_intermediaire_position[0], 
                 config_traj.home_intermediaire_position[1], 
                 config_traj.home_intermediaire_position[2], 
                 0.0, False))

    for bloc in blocs_sorted:
        # bloc format: (couleur, bloc_type, x, y, z, angle)
        if len(bloc) >= 6:
            couleur, bloc_type, x, y, z_ignored, angle, *_ = bloc
            p_bloc = (float(x), float(y), config_traj.z_table)
        else:
            # Fallback pour compatibilité ancienne
            couleur, x, y, z_ignored, angle = bloc[:5]
            bloc_type = "None"
            p_bloc = (float(x), float(y), config_traj.z_table)
            
        # Application de l'offset du poignet et wrap arithmétique pour rester entre -90 et 90
        angle = float(angle) + getattr(config_traj, 'WRIST_ANGLE_OFFSET_DEG', 0.0)
        while angle > 90.0:
            angle -= 180.0
        while angle < -90.0:
            angle += 180.0
            
        p_out  = output_pos_for_color(couleur)

        # 2) Aller au-dessus du bloc (pince ouverte)
        path.append((None, None, "joint", speed_joint,
                     p_bloc[0], p_bloc[1], p_bloc[2] + distance_approach, angle, False))

        # 3) Descendre sur le bloc
        path.append((None, None, "joint", speed_approach,
                     p_bloc[0], p_bloc[1], p_bloc[2], angle, False))

        # 4) Fermer la pince
        path.append((couleur, bloc_type, "closeGripper", 0.0,
                     p_bloc[0], p_bloc[1], p_bloc[2], angle, True))

        # 5) Remonter avec le bloc
        path.append((couleur, bloc_type, "joint", speed_approach,
                     p_bloc[0], p_bloc[1], p_bloc[2] + distance_approach, angle, True))

        # 6) Aller au-dessus du bac de sortie
        path.append((couleur, bloc_type, "joint", speed_joint,
                     p_out[0], p_out[1], p_out[2] + distance_approach, 0, True))

        # 7) Ouvrir la pince (drop)
        path.append((None, None, "openGripper", 0.0,
                     p_out[0], p_out[1], p_out[2] + distance_approach, 0, False))

    # 8) Point de passage centre table (rentrée smooth)
    path.append((None, None, "joint", speed_joint,
                 config_traj.home_intermediaire_position[0], 
                 config_traj.home_intermediaire_position[1], 
                 config_traj.home_intermediaire_position[2], 
                 0, False))

    # 9) Retour final au home
    path.append((None, None, "joint", speed_approach_hub,
                 home_position[0], home_position[1], home_position[2], 0, False))

    return path

def main():
    #plot_blocks_2D(blocs)
    plot_blocks_3D(blocs, home_position)
    distances = [distance_from_output(bloc) for bloc in blocs]
    for bloc, dist in zip(blocs, distances):
        print(f"Distance du bloc {bloc} a sa position de sortie: {dist:.2f} unites")

    print(f"Distance entre la position de depart et la position de sortie rouge: {distance_between_points(home_position, red_output_position):.2f} unites")

    # Trier les blocs selon un algorithme de planification
    # Branch and Bound, TSP exact, ou Cheapest Insertion selon le nombre de blocs
    if len(blocs) < 11:
        blocs_sorted, total_distance = plan_bnb_basic(blocs, home_position)
        print("Ordre optimal:", blocs_sorted)
        print(f"Distance totale optimale: {total_distance:.2f}")
    elif len(blocs) < 14:
        blocs_sorted, total_distance = plan_exact_tsp(blocs, home_position)
        print("Ordre optimal:", blocs_sorted)
        print(f"Distance totale optimale: {total_distance:.2f}")
    else:
        blocs_sorted, total_distance = plan_cheapest_insertion(blocs, home_position)
        print("Ordre non-optimal:", blocs_sorted)
        print(f"Distance totale non-optimale: {total_distance:.2f}")

    plot_route_2D(blocs_sorted, home_position)

    # Full trajectory
    full_path = plan_full_trajectory(blocs_sorted)
    print("Trajectoire complète:")
    for step in full_path:
        print(step)
    #animate_full_trajectory_2D(full_path, blocs=blocs, home_position=home_position, dt=0.05, show_trace=True)
    animate_full_trajectory_3D(full_path, blocs=blocs_sorted, home_position=home_position, dt=0.05, show_trace=True)

if __name__ == "__main__":
    main()