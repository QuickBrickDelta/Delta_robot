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
green_output_position = config_traj.green_output_position
yellow_output_position = config_traj.yellow_output_position
MAX_BLOCS_OPTIMAL = config_traj.MAX_BLOCS_OPTIMAL
speed_joint_move_global = config_traj.speed_joint_move_global
speed_approach_move_global = config_traj.speed_approach_move_global

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
from shortest_path_algorithms import plan_optimal_bruteforce, plan_nearest_neighbor

## full trajectoire function
def plan_full_trajectory(blocs):
    # path: [(bloc_carried, movement_type, speed, x, y, z, pince_fermee), ...]
    path = []
    speed_joint_move = speed_joint_move_global
    speed_approach_move = speed_approach_move_global
    distance_approach = 10.0

    # Trier les blocs
    if len(blocs) <= MAX_BLOCS_OPTIMAL:
        blocs_sorted, total_distance = plan_optimal_bruteforce(blocs, home_position)
    else:
        blocs_sorted, total_distance = plan_low_computing(blocs, home_position)

    # Début à la maison (rien dans la pince)
    path.append((None, "home",  speed_joint_move,
                 home_position[0], home_position[1], home_position[2], False))
    for bloc in blocs_sorted:
        couleur, x, y = bloc
        p_bloc = (float(x), float(y), 0.0)
        p_out  = output_pos_for_color(couleur)

        # Aller au-dessus du bloc (pince ouverte)
        path.append((None, "joint", speed_joint_move,
                     p_bloc[0], p_bloc[1], p_bloc[2] + distance_approach, False))

        # Descendre sur le bloc (pince ouverte)
        path.append((None, "linear", speed_approach_move,
                     p_bloc[0], p_bloc[1], p_bloc[2], False))

        # Remonter AVEC le bloc (pince fermée + bloc_carried = couleur)
        path.append((couleur, "linear", speed_approach_move,
                     p_bloc[0], p_bloc[1], p_bloc[2] + distance_approach, True))

        # Aller au-dessus de la sortie (toujours avec le bloc)
        path.append((couleur, "joint", speed_joint_move,
                     p_out[0], p_out[1], p_out[2] + distance_approach, True))

        # Descendre à la sortie (toujours avec le bloc)
        path.append((couleur, "linear", speed_approach_move,
                     p_out[0], p_out[1], p_out[2], True))

        # Remonter SANS le bloc (pince ouverte + bloc_carried=None)
        path.append((None, "linear", speed_approach_move,
                     p_out[0], p_out[1], p_out[2] + distance_approach, False))

    # Retour à la maison
    path.append((None, "joint", speed_joint_move,
                 home_position[0], home_position[1], home_position[2], False))

    return path

def main():
    #plot_blocks_2D(blocs)
    plot_blocks_3D(blocs, home_position)
    distances = [distance_from_output(bloc) for bloc in blocs]
    for bloc, dist in zip(blocs, distances):
        print(f"Distance du bloc {bloc} a sa position de sortie: {dist:.2f} unites")

    print(f"Distance entre la position de depart et la position de sortie rouge: {distance_between_points(home_position, red_output_position):.2f} unites")

    # Low computing (greedy)
    order_g, total_g = plan_nearest_neighbor(blocs, home_position)
    print("Ordre greedy:", order_g)
    print(f"Distance totale greedy: {total_g:.2f}")

    plot_route_2D(order_g, home_position)

    if len(blocs) <= MAX_BLOCS_OPTIMAL:
        print("Calcul de la solution optimale (bruteforce)...")
        # Optimal (exact) — parfait pour <= 9-10 blocs
        order_o, total_o = plan_optimal_bruteforce(blocs, home_position)
        print("Ordre optimal:", order_o)
        print(f"Distance totale optimale: {total_o:.2f}")

        plot_route_2D(order_o, home_position)

    # Full trajectory
    full_path = plan_full_trajectory(blocs)
    print("Trajectoire complète:")
    for step in full_path:
        print(step)
    #animate_full_trajectory_2D(full_path, blocs=blocs, home_position=home_position, dt=0.05, show_trace=True)
    animate_full_trajectory_3D(full_path, blocs=blocs, home_position=home_position, dt=0.05, show_trace=True)

if __name__ == "__main__":
    main()