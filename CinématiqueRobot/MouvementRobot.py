import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import root  # <--- Ajout nécessaire pour le mode Joint
from Cinematique_delta3bras import rotZ, GetAngleMoteur1, GetBrasComplet
import matplotlib
matplotlib.use('TkAgg')

# ==========================================
# 1. LOGIQUE D'INTERPOLATION 
# ==========================================

def get_all_thetas(pos):
    """ Helper : Récupère les 3 angles pour une position XYZ donnée """
    phis = [0, np.radians(120), np.radians(240)]
    thetas = []
    for phi in phis:
        p_rot = rotZ(np.array(pos), -phi)
        res = GetAngleMoteur1(p_rot[0], p_rot[1], p_rot[2], phi)
        if res[0] is None: return None # Position impossible
        thetas.append(res[0])
    return np.array(thetas)

def interpolate_linear(p_start, p_end, steps):
    """ Ligne droite cartésienne (X,Y,Z) """
    traj = []
    xs = np.linspace(p_start[0], p_end[0], steps)
    ys = np.linspace(p_start[1], p_end[1], steps)
    zs = np.linspace(p_start[2], p_end[2], steps)
    for i in range(steps):
        traj.append([xs[i], ys[i], zs[i]])
    return traj

def interpolate_joint(p_start, p_end, steps):
    """ Interpolation des ANGLES (Courbe naturelle) """
    traj = []
    
    # 1. On récupère les angles de début et fin
    angles_start = get_all_thetas(p_start)
    angles_end = get_all_thetas(p_end)
    
    if angles_start is None or angles_end is None:
        print("Erreur: Point de départ ou d'arrivée hors limite.")
        return [p_start] * steps

    # 2. On interpole les angles linéairement
    t_interp = np.linspace(angles_start, angles_end, steps)
    
    # 3. Solveur : On cherche le XYZ qui correspond aux angles interpolés
    current_pos = p_start # "Guess" initial pour le solveur
    
    for i in range(steps):
        target_angles = t_interp[i]
        
        # Fonction d'erreur à minimiser : (Angles_Calculés - Angles_Cibles)
        def error_func(x):
            calc = get_all_thetas(x)
            if calc is None: return np.ones(3)*999 # Pénalité si hors limite
            return calc - target_angles

        # Résolution numérique
        sol = root(error_func, current_pos)
        
        if sol.success:
            traj.append(sol.x)
            current_pos = sol.x # Met à jour le guess pour la prochaine frame (plus rapide)
        else:
            traj.append(current_pos)
            
    return traj

# ==========================================
# 2. SIMULATION (Mise à jour pour gérer L/J)
# ==========================================

def run_simulation_realtime(points_data, steps_per_move=30):
    plt.ion()
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    lim = 40.0
    z_min, z_max = -80.0, 10.0
    colors = ['red', 'green', 'blue']
    angles_phi = [0, np.radians(120), np.radians(240)]
    
    print("--- Démarrage Simulation ---")
    current_pos = points_data[0][:3] 

    # Boucle sur les segments
    for i in range(1, len(points_data)):
        target_pt = points_data[i]
        pos_xyz = target_pt[:3]
        
        # Lecture du mode ('L' par défaut si absent)
        mode = target_pt[3] if len(target_pt) > 3 else 'L'
        
        print(f"Vers {pos_xyz} [{mode}]")
        
        # Choix de l'interpolation
        if mode == 'J':
            traj_points = interpolate_joint(current_pos, pos_xyz, steps_per_move)
        else:
            traj_points = interpolate_linear(current_pos, pos_xyz, steps_per_move)
        
        # Animation
        for pos in traj_points:
            ax.cla()
            ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(z_min, z_max)
            ax.set_title(f"Mode: {'JOINT' if mode=='J' else 'LINEAIRE'}")
            # raph aime les cocks
            # Base
            base_pts = [rotZ(np.array([0, 10.0, 0]), a) for a in angles_phi]
            base_pts.append(base_pts[0])
            ax.plot(np.array(base_pts)[:,0], np.array(base_pts)[:,1], np.array(base_pts)[:,2], 'k-')

            # Bras
            for k, phi in enumerate(angles_phi):
                res = GetBrasComplet(pos[0], pos[1], pos[2], phi)
                if res is None: continue
                c = colors[k]
                # --- DESSIN DU BRAS ---
                c = colors[k]
                # Bras Proximal (Moteur -> Coude)
                ax.plot([res["M"][0], res["B"][0]], [res["M"][1], res["B"][1]], [res["M"][2], res["B"][2]], color=c, lw=3)
                # Bras Distal (Parallélogramme)
                ax.plot([res["BG"][0], res["EG"][0]], [res["BG"][1], res["EG"][1]], [res["BG"][2], res["EG"][2]], color='orange')
                ax.plot([res["BD"][0], res["ED"][0]], [res["BD"][1], res["ED"][1]], [res["BD"][2], res["ED"][2]], color='teal')
                # Entretoises
                ax.plot([res["BG"][0], res["BD"][0]], [res["BG"][1], res["BD"][1]], [res["BG"][2], res["BD"][2]], color='black')
                ax.plot([res["EG"][0], res["ED"][0]], [res["EG"][1], res["ED"][1]], [res["EG"][2], res["ED"][2]], color='purple')
            
            plt.pause(0.001)
            
        current_pos = pos_xyz 

    plt.ioff()
    plt.show()


###Test simulation###

# ==========================================
# 3. EXÉCUTION
# ==========================================

# Format: [X, Y, Z, 'Mode']
points_passage = [
    [0.0, 0.0, -30.0, 'L'],     # 1. Home : Centre, hauteur de sécurité
    [8.0, 0.0, -30.0, 'L'],    # 2. Approche X+ : On se met au dessus de l'objet
    [8.0, 0.0, -33.0, 'J'],    # 3. Descente : On descend attraper l'objet (Courbe fluide)
    [8.0, 0.0, -30.0, 'L'],    # 4. Remontée : On remonte avec l'objet
    [-8.0, 8.0, -30.0, 'L'],    # 5. Traversée : On va vers la zone de dépôt
    [-8.0, 8.0, -33.0, 'J'],    # 6. Descente : On dépose l'objet
    [0.0, 0.0, -32.0, 'J']      # 7. Retour Home
]

# run_simulation_realtime(points_passage, steps_per_move=40)