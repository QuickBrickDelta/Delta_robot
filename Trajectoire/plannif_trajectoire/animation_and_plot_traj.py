import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import config_traj  # Pour les variables globales

# =============================================================================
# 1. CONFIGURATION ET IMPORTS DES POSITIONS DE SORTIE
# =============================================================================

# Récupération des coordonnées cibles depuis le fichier de configuration
red_output_position = config_traj.red_output_position
blue_output_position = config_traj.blue_output_position
green_dark_output_position = config_traj.green_dark_output_position
green_light_output_position = config_traj.green_light_output_position
yellow_output_position = config_traj.yellow_output_position
orange_output_position = config_traj.orange_output_position

# Importer les fonctions utilitaires nécessaires
try:
    from other_fct_traj import output_pos_for_color
except ImportError:
    # Fonction de secours si le fichier est manquant
    def output_pos_for_color(color):
        mapping = {
            'red': red_output_position,
            'blue': blue_output_position,
            'green_dark': green_dark_output_position,
            'green_light': green_light_output_position,
            'yellow': yellow_output_position,
            'orange': orange_output_position
        }
        return mapping.get(color, [0, 0, config_traj.z_table])

# MAPPING DES COULEURS : Pour éviter l'erreur "ValueError: 'green_dark' is not a valid color"
# Convertit tes étiquettes de détection en couleurs standards Matplotlib
COLOR_MAP = {
    'green_dark': 'darkgreen',
    'green_light': 'lime',
    'red': 'red',
    'yellow': 'gold',
    'blue': 'blue',
    'orange': 'orange',
    'black': 'black'
}

# =============================================================================
# 2. FONCTIONS GÉOMÉTRIQUES DE BASE
# =============================================================================

def get_triangle_vertices(side_length=30):
    """
    Calcule les sommets du triangle équilatéral de la zone de travail.
    Le triangle est centré à (0,0) avec un sommet sur l'axe X positif.
    """
    r = side_length / np.sqrt(3)
    v1 = np.array([r, 0])
    v2 = np.array([-r/2, r*np.sqrt(3)/2])
    v3 = np.array([-r/2, -r*np.sqrt(3)/2])
    return np.array([v1, v2, v3])

def plot_triangle(side_length=30):
    """Dessine le triangle de la zone de travail."""
    vertices = get_triangle_vertices(side_length)
    triangle = np.vstack([vertices, vertices[0]])
    plt.plot(triangle[:, 0], triangle[:, 1], 'k-', linewidth=1.2, label='Zone de travail')

# =============================================================================
# 3. FONCTIONS DE VISUALISATION STATIQUE
# =============================================================================

def plot_blocks_2D(blocs):
    """Affiche les blocs détectés en 2D avec leurs angles."""
    plt.figure(figsize=(8, 8))
    plot_triangle(30)
    
    for bloc in blocs:
        # Déballage flexible avec *_ pour ignorer les données supplémentaires
        couleur, x, y, angle, *_ = bloc
        c_plot = COLOR_MAP.get(couleur, couleur)
        
        plt.scatter(x, y, c=c_plot, s=150, edgecolors='black', zorder=5)
        plt.text(x + 0.5, y + 0.5, f"{couleur}\n{angle:.1f}°", fontsize=8, fontweight='bold')

    plt.xlim(-15, 20)
    plt.ylim(-18, 18)
    plt.gca().set_aspect('equal')
    plt.xlabel('X (cm)')
    plt.ylabel('Y (cm)')
    plt.title('Vue 2D : Détection des blocs')
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.show()

def plot_blocks_3D(blocs, home_position):
    """Visualisation 3D de l'espace de travail complet."""
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    
    # Blocs sur la table
    for bloc in blocs:
        z = config_traj.z_table
        couleur, x, y, angle, *_ = bloc
        c_plot = COLOR_MAP.get(couleur, 'gray')
        ax.scatter(x, y, z, c=c_plot, s=100, edgecolors='black')
        ax.text(x, y, z + 1, f"{angle:.0f}°", fontsize=7)

    # Position de repos du robot (Home)
    ax.scatter(home_position[0], home_position[1], home_position[2], c='black', s=200, marker='^', label='Home')

    # Boîtes de sortie
    outputs = {
        'red': red_output_position, 'blue': blue_output_position,
        'green_dark': green_dark_output_position, 'yellow': yellow_output_position
    }
    for key, pos in outputs.items():
        c_out = COLOR_MAP.get(key, key)
        ax.scatter(pos[0], pos[1], pos[2], c=c_out, s=150, marker='s', edgecolors='black')

    # Triangle au sol
    vertices = get_triangle_vertices(30)
    tri = np.vstack([vertices, vertices[0]])
    ax.plot(tri[:, 0], tri[:, 1], [config_traj.z_table]*len(tri), 'k--', alpha=0.4)
    
    ax.set_zlim(config_traj.z_table - 2, 5)
    ax.set_title('Environnement 3D du Delta Robot')
    plt.show()

def plot_route_2D(order, start_pos):
    """Affiche le trajet complet : Home -> Bloc -> Sortie -> Bloc..."""
    plt.figure(figsize=(9, 9))
    plot_triangle(30)
    
    cur_x, cur_y = start_pos[0], start_pos[1]
    plt.scatter(cur_x, cur_y, c='black', s=150, marker='^', label='Home')

    for i, b in enumerate(order, 1):
        c, x, y, z, angle, *_ = b
        p_out = output_pos_for_color(c)
        c_plot = COLOR_MAP.get(c, c)

        # Mouvement à vide
        plt.annotate("", xy=(x, y), xytext=(cur_x, cur_y),
                     arrowprops=dict(arrowstyle="->", color='gray', linestyle=':', alpha=0.5))
        
        # Le bloc
        plt.scatter(x, y, c=c_plot, s=120, edgecolors='black')
        plt.text(x, y-1.5, f"B{i}", ha='center', weight='bold')

        # Mouvement en charge vers la sortie
        plt.annotate("", xy=(p_out[0], p_out[1]), xytext=(x, y),
                     arrowprops=dict(arrowstyle="->", color=c_plot, linewidth=2))
        
        plt.scatter(p_out[0], p_out[1], c=c_plot, s=180, marker='s', edgecolors='black')
        
        cur_x, cur_y = p_out[0], p_out[1]

    plt.title("Planification de trajectoire optimale")
    plt.gca().set_aspect('equal')
    plt.grid(True, alpha=0.3)
    plt.show()

# =============================================================================
# 4. LOGIQUE D'ANIMATION ET D'INTERPOLATION
# =============================================================================

def animate_full_trajectory_2D(full_path, blocs=None, home_position=None, dt=0.05, show_trace=True):
    """
    Anime le déplacement du robot sur le plan XY.
    Gère automatiquement la couleur du robot selon qu'il transporte un bloc ou non.
    """
    if not full_path or len(full_path) < 2: return

    # --- Interpolation des points pour une animation fluide ---
    frames = []
    for i in range(len(full_path) - 1):
        # Format attendu : (carried_color, type, movement_type, speed, x, y, z, angle, grip)
        c_col1, _, mt1, sp1, x1, y1, z1, _, g1 = full_path[i][:9]
        c_col2, _, mt2, sp2, x2, y2, z2, _, g2 = full_path[i+1][:9]
        
        p1, p2 = np.array([x1, y1]), np.array([x2, y2])
        dist = np.linalg.norm(p2 - p1)
        
        # Calcul des étapes selon la vitesse (évite division par zéro)
        vitesse = max(float(sp2), 0.1)
        steps = max(1, int(np.ceil((dist / vitesse) / dt)))
        
        for s in range(steps):
            t = (s + 1) / steps
            pos = (1 - t) * p1 + t * p2
            # On stocke (x, y, grip, type_mouv, couleur_portée)
            frames.append((pos[0], pos[1], bool(g2), mt2, c_col2))

    fig, ax = plt.subplots(figsize=(8, 8))
    plot_triangle(30)
    
    # Affichage des blocs statiques en arrière-plan (fantômes)
    if blocs:
        for b in blocs:
            c, bx, by, *_ = b
            ax.scatter(bx, by, c=COLOR_MAP.get(c, c), s=80, alpha=0.3)

    robot_marker = ax.scatter([], [], s=160, marker='o', edgecolors='white', zorder=10)
    trace, = ax.plot([], [], 'b-', alpha=0.2, linewidth=1)
    tx, ty = [], []

    def update(k):
        x, y, grip, mtype, carried = frames[k]
        robot_marker.set_offsets(np.array([[x, y]]))
        
        # Changement de couleur si l'objet est saisi
        if grip and carried:
            robot_marker.set_color(COLOR_MAP.get(carried, 'black'))
        else:
            robot_marker.set_color('black')
        
        if show_trace:
            tx.append(x); ty.append(y)
            trace.set_data(tx, ty)
        return robot_marker, trace

    ax.set_xlim(-15, 20); ax.set_ylim(-18, 18)
    ax.set_aspect('equal')
    ani = FuncAnimation(fig, update, frames=len(frames), interval=int(dt*1000), blit=True, repeat=False)
    plt.show()
    return ani

def animate_full_trajectory_3D(full_path, blocs=None, home_position=None, dt=0.05, show_trace=True):
    # AJOUT DE show_trace=True pour corriger le TypeError
    frames = []
    for i in range(len(full_path) - 1):
        # Unpacking des 9 colonnes de ta trajectoire
        c1, ct1, mt1, sp1, x1, y1, z1, a1, g1 = full_path[i]
        c2, ct2, mt2, sp2, x2, y2, z2, a2, g2 = full_path[i+1]
        p1, p2 = np.array([x1, y1, z1]), np.array([x2, y2, z2])
        dist = np.linalg.norm(p2 - p1)
        steps = max(1, int(np.ceil((dist / max(float(sp2), 0.1)) / dt)))
        for s in range(steps):
            p = (1 - (s+1)/steps) * p1 + ((s+1)/steps) * p2
            frames.append((p[0], p[1], p[2], g2, c2))

    fig = plt.figure(); ax = fig.add_subplot(111, projection='3d')
    robot = ax.scatter([], [], [], s=140, marker='o')
    
    if show_trace:
        line, = ax.plot([], [], [], 'b-', alpha=0.3)
        history = []

    def update(k):
        x, y, z, grip, carried = frames[k]
        robot._offsets3d = ([x], [y], [z])
        c_robot = COLOR_MAP.get(carried, 'black') if (grip and carried) else 'black'
        robot.set_color(c_robot)
        if show_trace:
            history.append([x, y, z]); h = np.array(history)
            line.set_data(h[:,0], h[:,1]); line.set_3d_properties(h[:,2])
        return robot,

    ani = FuncAnimation(fig, update, frames=len(frames), interval=int(dt*1000))
    ax.set_xlim(-20, 20)
    ax.set_ylim(-20, 20)
    ax.set_zlim(config_traj.z_table - 2, 10) # Force la vue sur la table et le robot

    ax.set_xlabel("X (cm)")
    ax.set_ylabel("Y (cm)")
    ax.set_zlabel("Z (cm)")
    plt.show()
    return ani