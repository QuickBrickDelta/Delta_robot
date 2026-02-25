import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import config_traj  # Pour les variables globales

# Importer les variables globales depuis config_traj
red_output_position = config_traj.red_output_position
blue_output_position = config_traj.blue_output_position
green_output_position = config_traj.green_output_position
yellow_output_position = config_traj.yellow_output_position

# Importer les fonctions depuis other_fct_traj.py
from other_fct_traj import output_pos_for_color

## Plotting functions
def plot_blocks_2D(blocs):
    """Affiche les blocs sur un graphique 2D."""
    for bloc in blocs:
        couleur, bloc_type, x, y, angle = bloc
        plt.scatter(x, y, c=couleur, s=100)  # s est la taille du point
        plt.text(x, y, f'({x},{y})', fontsize=9, ha='right')
    plt.xlim(-10, 10)
    plt.ylim(-10, 10)
    plt.xlabel('Position X')
    plt.ylabel('Position Y')
    plt.title('Positions des blocs')
    plt.grid()
    plt.show()

def plot_blocks_3D(blocs, home_position):
    """Affiche les blocs sur un graphique 3D."""
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for bloc in blocs:
        z = config_traj.z_table  # Tous les blocs sont au niveau z=-38.5
        couleur, bloc_type, x, y, angle = bloc
        ax.scatter(x, y, z, c=couleur, s=100)
    # Afficher la position de départ du robot
    ax.scatter(home_position[0], home_position[1], home_position[2], c='black', s=150, marker='^')
    ax.text(home_position[0], home_position[1], home_position[2], ' Home', fontsize=10, color='black')
    # Afficher les positions de sortie
    output_positions = {
        'red': red_output_position,
        'blue': blue_output_position,
        'green': green_output_position,
        'yellow': yellow_output_position
    }
    for color, pos in output_positions.items():
        ax.scatter(pos[0], pos[1], pos[2], c=color, s=150, marker='s')
        ax.text(pos[0], pos[1], pos[2], f' {color} output', fontsize=10, color=color)
    
    # Dessiner le triangle à z=-38.5 (niveau de la table)
    vertices = get_triangle_vertices(side_length=30)
    triangle = np.vstack([vertices, vertices[0]])
    ax.plot(triangle[:, 0], triangle[:, 1], [-38.5]*len(triangle), 'k-', linewidth=0.8, label='Zone de travail')
    
    # Définir les limites et les labels
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(-38.5, 10)
    ax.set_xlabel('Position X')
    ax.set_ylabel('Position Y')
    ax.set_zlabel('Position Z')
    ax.set_title('Positions des blocs en 3D')
    ax.legend()
    plt.show()

## Triangle helper function
def get_triangle_vertices(side_length=30):
    """Retourne les vertices du triangle équilatéral.
    Un coin est aligné avec l'axe X positif."""
    r = side_length / np.sqrt(3)
    v1 = np.array([r, 0])
    v2 = np.array([-r/2, r*np.sqrt(3)/2])
    v3 = np.array([-r/2, -r*np.sqrt(3)/2])
    return np.array([v1, v2, v3])

## Triangle plotting function
def plot_triangle(side_length=30):
    """Dessine un triangle équilatéral noir centré à l'origine.
    Un coin est aligné avec l'axe X positif."""
    vertices = get_triangle_vertices(side_length)
    triangle = np.vstack([vertices, vertices[0]])  # Fermer le triangle
    plt.plot(triangle[:, 0], triangle[:, 1], 'k-', linewidth=0.8, label='Zone de travail')

## Trajectory plotting function
def plot_route_2D(order, start_pos):
    x0, y0, _ = start_pos

    plt.figure()

    # Dessiner le triangle de travail
    plot_triangle(side_length=30)

    # Point home
    plt.scatter(x0, y0, c='black', s=120, marker='^', label='Home')

    # Trajet segment par segment (avec flèches)
    cur = start_pos
    for i, b in enumerate(order, 1):
        c, bloc_type, x, y, angle = b
        p_bloc = (float(x), float(y), -38.5)  # Z fixe pour les blocs
        p_out  = output_pos_for_color(c)

        # cur -> bloc
        plt.annotate("", xy=(p_bloc[0], p_bloc[1]), xytext=(cur[0], cur[1]),
                     arrowprops=dict(arrowstyle="->"))
        plt.scatter(p_bloc[0], p_bloc[1], c=c, s=100)
        plt.text(p_bloc[0], p_bloc[1], f"B{i}", ha='left', va='bottom')

        # bloc -> output (inclut le dernier bloc aussi)
        plt.annotate("", xy=(p_out[0], p_out[1]), xytext=(p_bloc[0], p_bloc[1]),
                     arrowprops=dict(arrowstyle="->"))
        plt.scatter(p_out[0], p_out[1], c=c, s=150, marker='s')
        plt.text(p_out[0], p_out[1], f"O{i}", ha='left', va='bottom')

        cur = p_out  # prochaine position = sortie

    plt.xlim(-15, 20)
    plt.ylim(-18, 18)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    plt.title("Trajectoire (projection 2D) – bloc → output inclus")
    plt.legend()
    plt.show()

## animation function

def animate_full_trajectory_2D(full_path, blocs=None, home_position=None, dt=0.05, show_trace=True):
    """
    full_path: [(bloc_carried, movement_type, speed, x, y, z, pince_fermee), ...]
    dt: pas de temps simulé (sec) pour l'interpolation
    """

    if len(full_path) < 2:
        raise ValueError("full_path doit contenir au moins 2 points")

    # --- 1) Générer des frames interpolées ---
    frames = []  # (x, y, grip, mtype, carried_color)
    for i in range(len(full_path) - 1):
        carried1, mtype1, speed1, x1, y1, z1, grip1 = full_path[i]
        carried2, mtype2, speed2, x2, y2, z2, grip2 = full_path[i + 1]

        p1 = np.array([float(x1), float(y1), float(z1)], dtype=float)
        p2 = np.array([float(x2), float(y2), float(z2)], dtype=float)

        dist = np.linalg.norm(p2 - p1)
        speed = max(float(speed1), 1e-6)
        T = dist / speed
        steps = max(1, int(np.ceil(T / dt)))

        for s in range(steps):
            t = (s + 1) / steps
            p = (1 - t) * p1 + t * p2
            frames.append((p[0], p[1], bool(grip2), mtype2, carried2))

    # --- 2) Setup figure ---
    fig, ax = plt.subplots()
    ax.set_xlim(-15, 20)
    ax.set_ylim(-18, 18)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Animation trajectoire (2D)")
    ax.grid(True)
    ax.set_aspect('equal', adjustable='box')
    
    # Dessiner le triangle
    vertices = get_triangle_vertices(side_length=30)
    triangle = np.vstack([vertices, vertices[0]])
    ax.plot(triangle[:, 0], triangle[:, 1], 'k-', linewidth=0.8, label='Zone de travail')

    # Blocs
    if blocs is not None:
        for couleur, x, y in blocs:
            ax.scatter(float(x), float(y), c=couleur, s=80)

    # Home
    if home_position is not None:
        ax.scatter(home_position[0], home_position[1], c='black', s=120, marker='^')
        ax.text(home_position[0], home_position[1], " Home", fontsize=9)

    # Outputs (si tes variables globales existent)
    outputs = {
        'red': red_output_position,
        'blue': blue_output_position,
        'green': green_output_position,
        'yellow': yellow_output_position
    }
    for color, pos in outputs.items():
        ax.scatter(pos[0], pos[1], c=color, s=120, marker='s')
        ax.text(pos[0], pos[1], f" {color}", fontsize=9)

    ax.legend()
    robot_point = ax.scatter([], [], s=140, marker='o')
    status_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top")

    if show_trace:
        trace_line, = ax.plot([], [], linewidth=2)
        trace_x, trace_y = [], []
    else:
        trace_line = None

    def init():
        x, y, grip, mtype, carried = frames[0]
        robot_point.set_offsets(np.array([[x, y]]))
        robot_point.set_color(carried if (grip and carried is not None) else 'black')
        status_text.set_text("")
        if show_trace:
            trace_line.set_data([], [])
        return (robot_point, status_text) if not show_trace else (robot_point, status_text, trace_line)

    def update(k):
        x, y, grip, mtype, carried = frames[k]

        # Couleur selon bloc_carried fourni par full_path
        if grip and carried is not None:
            robot_point.set_color(carried)
        else:
            robot_point.set_color('black')

        robot_point.set_offsets(np.array([[x, y]]))

        status_text.set_text(
            f"Frame {k+1}/{len(frames)} | move={mtype} | grip={'CLOSED' if grip else 'OPEN'} | carried={carried}"
        )

        if show_trace:
            trace_x.append(x)
            trace_y.append(y)
            trace_line.set_data(trace_x, trace_y)
            return robot_point, status_text, trace_line

        return robot_point, status_text

    anim = FuncAnimation(fig, update, frames=len(frames), init_func=init,
                         interval=int(dt * 1000), blit=True, repeat=False)

    plt.show()
    return anim

def animate_full_trajectory_3D(full_path, blocs=None, home_position=None, dt=0.05, show_trace=True):
    """
    full_path: [(bloc_carried, movement_type, speed, x, y, z, pince_fermee), ...]
    dt: pas de temps simulé (sec) pour l'interpolation
    """

    if len(full_path) < 2:
        raise ValueError("full_path doit contenir au moins 2 points")

    # --- 1) Frames interpolées ---
    frames = []  # (x, y, z, angle, grip, mtype, carried_color)
    for i in range(len(full_path) - 1):
        carried_color1, carried_type1, mtype1, speed1, x1, y1, z1, angle1, grip1 = full_path[i]
        carried_color2, carried_type2, mtype2, speed2, x2, y2, z2, angle2, grip2 = full_path[i + 1]

        p1 = np.array([float(x1), float(y1), float(z1)], dtype=float)
        p2 = np.array([float(x2), float(y2), float(z2)], dtype=float)

        dist = np.linalg.norm(p2 - p1)
        speed = max(float(speed2), 1e-6)
        T = dist / speed
        steps = max(1, int(np.ceil(T / dt)))

        for s in range(steps):
            t = (s + 1) / steps
            p = (1 - t) * p1 + t * p2
            frames.append((p[0], p[1], p[2], bool(grip2), mtype2, carried_color2))

    # --- 2) Figure 3D ---
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(-38.5, 10)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Animation trajectoire (3D)")
    
    # Dessiner le triangle à z=-38.5 (niveau de la table)
    vertices = get_triangle_vertices(side_length=30)
    triangle = np.vstack([vertices, vertices[0]])
    ax.plot(triangle[:, 0], triangle[:, 1], [-38.5]*len(triangle), 'k-', linewidth=0.8, label='Zone de travail')

    # Blocs
    if blocs is not None:
        for couleur, bloc_type, x, y, angle in blocs:
            ax.scatter(float(x), float(y), -38.5, c=couleur, s=60)

    # Home
    if home_position is not None:
        ax.scatter(home_position[0], home_position[1], home_position[2], c='black', s=120, marker='^')
        ax.text(home_position[0], home_position[1], home_position[2], " Home", fontsize=9, color='black')

    # Outputs
    outputs = {
        'red': red_output_position,
        'blue': blue_output_position,
        'green': green_output_position,
        'yellow': yellow_output_position
    }
    for color, pos in outputs.items():
        ax.scatter(pos[0], pos[1], pos[2], c=color, s=120, marker='s')
        ax.text(pos[0], pos[1], pos[2], f" {color} out", fontsize=9, color=color)

    ax.legend()
    robot = ax.scatter([], [], [], s=140, marker='o')

    if show_trace:
        trace_line, = ax.plot([], [], [], linewidth=2)
        tx, ty, tz = [], [], []
    else:
        trace_line = None

    status_text = fig.text(0.02, 0.95, "", ha="left", va="top")

    def init():
        x, y, z, grip, mtype, carried = frames[0]
        robot._offsets3d = ([x], [y], [z])
        robot.set_color(carried if (grip and carried is not None) else 'black')
        status_text.set_text("")
        if show_trace:
            trace_line.set_data([], [])
            trace_line.set_3d_properties([])
        return (robot, status_text) if not show_trace else (robot, status_text, trace_line)

    def update(k):
        x, y, z, grip, mtype, carried = frames[k]

        robot._offsets3d = ([x], [y], [z])
        robot.set_color(carried if (grip and carried is not None) else 'black')

        status_text.set_text(
            f"Frame {k+1}/{len(frames)} | move={mtype} | grip={'CLOSED' if grip else 'OPEN'} | carried={carried}"
        )

        if show_trace:
            tx.append(x); ty.append(y); tz.append(z)
            trace_line.set_data(tx, ty)
            trace_line.set_3d_properties(tz)
            return robot, status_text, trace_line

        return robot, status_text

    anim = FuncAnimation(
        fig, update, frames=len(frames), init_func=init,
        interval=int(dt * 1000),
        blit=False,  # 3D: plus stable
        repeat=False
    )

    plt.show()
    return anim
