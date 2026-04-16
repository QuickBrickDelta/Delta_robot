import numpy as np
import config_traj  # Pour les variables globales

# Importation de `config_traj` pour un accès dynamique aux paramètres

## Get position functions
def bloc_pos(bloc):
    # Supporte 5 ou 6 éléments [couleur, type, x, y, z, angle] ou [couleur, type, x, y, angle]
    if len(bloc) >= 6:
        couleur, bloc_type, x, y, z, angle = bloc
    else:
        couleur, x, y, z, angle = bloc
    return (float(x), float(y), 0.0)

def output_pos_for_color(color):
    return {
        'red': config_traj.red_output_position,
        'blue': config_traj.blue_output_position,
        'green_dark': config_traj.green_dark_output_position,
        'green_light': config_traj.green_light_output_position,
        'yellow': config_traj.yellow_output_position,
        'orange': config_traj.orange_output_position,
    }[color]

## Cost calculation functions
def cost_do_bloc_from(pos, bloc):
    if len(bloc) >= 6:
        c, bloc_type, x, y, z, angle = bloc
    else:
        c, x, y, z, angle = bloc
    p_bloc = (float(x), float(y), 0.0)
    p_out  = output_pos_for_color(c)
    return distance_between_3_points(pos, p_bloc, p_out)

## Distance calculation functions
def distance_from_output(bloc):
    """Calcule la distance entre un bloc et sa position de sortie."""
    if len(bloc) >= 6:
        couleur, bloc_type, x, y, z_val, angle = bloc
    else:
        couleur, x, y, z_val, angle = bloc
    z = 0  # Tous les blocs sont au niveau z=0 pour le calcul 2D
    output_pos = output_pos_for_color(couleur)
    
    dist = np.sqrt((x - output_pos[0])**2 + (y - output_pos[1])**2 + (z - output_pos[2])**2)
    return dist

def distance_between_points(p1, p2):
    """Calcule la distance euclidienne entre deux points 3D."""
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)

def distance_between_3_points(p1, p2, p3):
    """Calcule la distance totale pour aller de p1 à p2 puis à p3."""
    dist1 = distance_between_points(p1, p2)
    dist2 = distance_between_points(p2, p3)
    return dist1 + dist2
