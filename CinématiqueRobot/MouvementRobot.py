import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import root  # <--- Ajout nécessaire pour le mode Joint

# ==========================================
# 1. TA CINÉMATIQUE (Intouchée)
# ==========================================

# --- Paramètres du robot ---
f = 0.1   # Rayon base moteur
e = 0.1   # Rayon effecteur
r_parralelogramme = 0.05 
Lc = 0.3  # Longueur bras supérieur (proximal)
Lb = 0.5  # Longueur bras inférieur (distal)

def rotZ(p, phi):
    """Rotation d'un point p autour de l'axe Z."""
    R = np.array([
        [np.cos(phi), -np.sin(phi), 0],
        [np.sin(phi),  np.cos(phi), 0],
        [0, 0, 1]
    ])
    return R @ p

def GetAngleMoteur1(x_eff, y_eff, z_eff):
    """ Calcule l'angle valide dans [0, 90]. """
    y_E = y_eff + e
    z_E = z_eff
    
    A = 2 * (f - y_E)
    B = -2 * z_E
    C = f**2 - y_E**2 - z_E**2 + Lb**2 - Lc**2

    if abs(B) < 1e-6:
        if abs(A) < 1e-6: return None, None, None
        y_sol = C / A
        delta_z = Lc**2 - (y_sol - f)**2
        if delta_z < 0: return None, None, None
        solutions = [(y_sol, -np.sqrt(delta_z)), (y_sol, np.sqrt(delta_z))]
    else:
        a_q = 1 + (A/B)**2
        b_q = -2*f - 2*(A*C)/(B**2)
        c_q = f**2 + (C/B)**2 - Lc**2
        delta = b_q**2 - 4*a_q*c_q
        if delta < 0: return None, None, None
        y1 = (-b_q - np.sqrt(delta)) / (2*a_q)
        y2 = (-b_q + np.sqrt(delta)) / (2*a_q)
        solutions = [(y1, (C - A*y1) / B), (y2, (C - A*y2) / B)]

    valid_solution = None
    for y_B, z_B in solutions:
        theta_candidate = np.arctan2(y_B - f, -z_B)
        deg = np.degrees(theta_candidate)
        if -0.1 <= deg <= 90.1:
            if valid_solution is None or z_B < valid_solution[2]:
                valid_solution = (theta_candidate, y_B, z_B)

    if valid_solution:
        return valid_solution # theta, y_B, z_B
    else:
        return None, None, None

def GetBrasComplet(x_eff_glob, y_eff_glob, z_eff_glob, phi):
    Eff_glob = np.array([x_eff_glob, y_eff_glob, z_eff_glob])
    Eff_loc = rotZ(Eff_glob, -phi)
    
    theta, y_B, z_B = GetAngleMoteur1(Eff_loc[0], Eff_loc[1], Eff_loc[2])
    
    if theta is None: return None

    M_loc = np.array([0, f, 0])
    B_loc = np.array([0, y_B, z_B])
    
    BG_loc = B_loc + [r_parralelogramme, 0, 0]
    BD_loc = B_loc - [r_parralelogramme, 0, 0]
    
    E_loc = np.array([Eff_loc[0], Eff_loc[1] + e, Eff_loc[2]])
    EG_loc = E_loc + [r_parralelogramme, 0, 0]
    ED_loc = E_loc - [r_parralelogramme, 0, 0]

    return {
        "M":  rotZ(M_loc, phi), "B":  rotZ(B_loc, phi),
        "BG": rotZ(BG_loc, phi), "BD": rotZ(BD_loc, phi),
        "EG": rotZ(EG_loc, phi), "ED": rotZ(ED_loc, phi),
        "Eff": Eff_glob
    }

# ==========================================
# 2. LOGIQUE D'INTERPOLATION (Modifiée)
# ==========================================

def get_all_thetas(pos):
    """ Helper : Récupère les 3 angles pour une position XYZ donnée """
    phis = [0, np.radians(120), np.radians(240)]
    thetas = []
    for phi in phis:
        p_rot = rotZ(np.array(pos), -phi)
        res = GetAngleMoteur1(p_rot[0], p_rot[1], p_rot[2])
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
# 3. SIMULATION (Mise à jour pour gérer L/J)
# ==========================================

def run_simulation_realtime(points_data, steps_per_move=30):
    plt.ion()
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    lim = 0.4
    z_min, z_max = -0.8, 0.1
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
            
            # Base
            base_pts = [rotZ(np.array([0, 0.1, 0]), a) for a in angles_phi]
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

# ==========================================
# 4. EXÉCUTION
# ==========================================

# Format: [X, Y, Z, 'Mode']
points_passage = [
    [0.0, 0.0, -0.65, 'L'],   # Départ
    [0.2, 0.0, -0.60, 'L'],   # Mouvement Linéaire
    [0.0, 0.2, -0.55, 'J'],   # Mouvement Joint (Courbe)
    [-0.2, -0.1, -0.70, 'L'], 
    [0.0, 0.0, -0.65, 'J']    
]

run_simulation_realtime(points_passage, steps_per_move=40)