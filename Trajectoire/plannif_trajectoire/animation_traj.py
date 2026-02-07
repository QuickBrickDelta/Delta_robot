import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import config_traj  # Pour les variables globales

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
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Animation trajectoire (2D)")
    ax.grid(True)

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
    frames = []  # (x, y, z, grip, mtype, carried_color)
    for i in range(len(full_path) - 1):
        carried1, mtype1, speed1, x1, y1, z1, grip1 = full_path[i]
        carried2, mtype2, speed2, x2, y2, z2, grip2 = full_path[i + 1]

        p1 = np.array([float(x1), float(y1), float(z1)], dtype=float)
        p2 = np.array([float(x2), float(y2), float(z2)], dtype=float)

        dist = np.linalg.norm(p2 - p1)
        speed = max(float(speed2), 1e-6)
        T = dist / speed
        steps = max(1, int(np.ceil(T / dt)))

        for s in range(steps):
            t = (s + 1) / steps
            p = (1 - t) * p1 + t * p2
            frames.append((p[0], p[1], p[2], bool(grip2), mtype2, carried2))

    # --- 2) Figure 3D ---
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(-40, 10)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Animation trajectoire (3D)")

    # Blocs
    if blocs is not None:
        for couleur, x, y in blocs:
            ax.scatter(float(x), float(y), 0.0, c=couleur, s=60)

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
