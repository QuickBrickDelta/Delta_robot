import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import root  # <--- Ajout nécessaire pour le mode Joint
from Cinematique_delta3bras import rotZ, GetAngleMoteur1, GetBrasComplet
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

# ==========================================
# 1. LOGIQUE D'INTERPOLATION 
# ==========================================

# --- Import de la calibration rotation ---
try:
    import sys, os
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_path = os.path.join(project_root, "Trajectoire", "plannif_trajectoire")
    if config_path not in sys.path:
        sys.path.append(config_path)
    import config_traj
    ROBOT_ROTATION_OFFSET_DEG = config_traj.ROBOT_ROTATION_OFFSET_DEG
except ImportError:
    ROBOT_ROTATION_OFFSET_DEG = 0.0

def get_all_thetas(pos):
    """ Helper : Récupère les 3 angles pour une position XYZ donnée """
    # --- Compensation du Z lié au scale ---
    # Le scale réduit les dimensions virtuelles du robot. Sans compensation,
    # Z est aussi affecté et le robot descend trop bas.
    # On divise Z par le scale pour annuler cet effet : Z_physique = Z_cible.
    scale = getattr(config_traj, 'ROBOT_CALIBRATION_SCALE', 1.0)
    x, y, z = pos
    pos_comp = [x, y, z / scale]

    offset_rad = np.radians(ROBOT_ROTATION_OFFSET_DEG)
    phis = [0 + offset_rad, np.radians(120) + offset_rad, np.radians(240) + offset_rad]
    thetas = []
    for phi in phis:
        # On tourne le point dans le repère local du moteur en question
        p_rot = rotZ(np.array(pos_comp), -phi)
        res = GetAngleMoteur1(p_rot[0], p_rot[1], p_rot[2], phi)
        if res[0] is None: return None # Position impossible
        thetas.append(res[0])
    return np.array(thetas)

def interpolate_linear(p_start, p_end, steps):
    """ Ligne droite cartésienne (X,Y,Z) avec profil de vitesse S-Curve """
    if steps < 2: return [p_end]
    traj = []
    
    # 1. Génération du profil S-Curve (vitesse progressive)
    # i/steps-1 va de 0 à 1 linéairement.
    # On le transforme en sinusoïde pour une accélération douce.
    indices = np.arange(steps)
    t = (1 - np.cos(np.pi * indices / (steps - 1))) / 2
    
    for val_t in t:
        pos = np.array(p_start) + val_t * (np.array(p_end) - np.array(p_start))
        traj.append(pos.tolist())
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

    # 2. On génère le profil S-Curve sur les ANGLES
    indices = np.arange(steps)
    t = (1 - np.cos(np.pi * indices / (steps - 1))) / 2
    
    # On interpole les angles de manière sinusoïdale
    t_interp = []
    for val_t in t:
        t_interp.append(angles_start + val_t * (angles_end - angles_start))
    
    # 3. Solveur : On cherche le XYZ qui correspond aux angles interpolés
    current_pos = p_start # "Guess" initial pour le solveur
    
    for target_angles in t_interp:
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
            traj.append(current_pos.tolist())
            
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
        
        #print(f"Vers {pos_xyz} [{mode}]")
        
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

            # --- Repère : axes X, Y, Z avec flèches et labels ---
            arrow_len = 10.0
            ax.quiver(0, 0, 0, arrow_len, 0, 0, color='red',   arrow_length_ratio=0.15, linewidth=2)
            ax.quiver(0, 0, 0, 0, arrow_len, 0, color='green', arrow_length_ratio=0.15, linewidth=2)
            ax.quiver(0, 0, 0, 0, 0, arrow_len, color='blue',  arrow_length_ratio=0.15, linewidth=2)
            ax.text(arrow_len * 1.15, 0, 0, 'X', color='red',   fontsize=12, fontweight='bold')
            ax.text(0, arrow_len * 1.15, 0, 'Y', color='green', fontsize=12, fontweight='bold')
            ax.text(0, 0, arrow_len * 1.15, 'Z', color='blue',  fontsize=12, fontweight='bold')
            ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
            
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
                
                # Print la longueur de la barre (BG-EG) pour vérifier qu'elle est constante
                if k == 0: # On print juste pour le Bras 1 pour ne pas spammer la console
                    long_tige = np.linalg.norm(res["BG"] - res["EG"])
                    print(f"Frame | Longueur vraie de la tige BG-EG: {long_tige:.4f} cm")
            
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