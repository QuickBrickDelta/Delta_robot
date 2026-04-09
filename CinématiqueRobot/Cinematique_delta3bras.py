import numpy as np
import matplotlib.pyplot as plt


# --- Paramètres du robot ---
# --- Paramètres du robot ---
f_base = 5.5  # Rayon base moteur
e_eff  = 4    # Rayon effecteur
Lc_arm = 15   # Longueur bras supérieur
Lb_arm = 32.5 # Longueur bras inférieur 
r_par  = 5    # Rayon parallélogramme

# On récupère l'échelle de calibration
try:
    import sys, os
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_path = os.path.join(project_root, "Trajectoire", "plannif_trajectoire")
    if config_path not in sys.path: sys.path.append(config_path)
    import config_traj
    scale = getattr(config_traj, 'ROBOT_CALIBRATION_SCALE', 1.0)
except:
    scale = 1.0

# On divise les dimensions par l'échelle. 
# Si scale = 1.5, le robot "mathématique" est plus petit, 
# donc il commandera des angles plus grands, ce qui fera aller le robot réel plus loin.
f = f_base / scale
e = e_eff  / scale
Lc = Lc_arm / scale
Lb = Lb_arm / scale
r_parralelogramme = r_par / scale

def rotZ(p, phi):
    """Rotation d'un point p autour de l'axe Z (rad)."""
    R = np.array([
        [np.cos(phi), -np.sin(phi), 0],
        [np.sin(phi),  np.cos(phi), 0],
        [0, 0, 1]
    ])
    return R @ p

# --- Import de la calibration rotation ---
import math
ROBOT_ROTATION_OFFSET_DEG = 0.0
try:
    import sys, os
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_path = os.path.join(project_root, "Trajectoire", "plannif_trajectoire")
    if config_path not in sys.path:
        sys.path.append(config_path)
    import config_traj
    ROBOT_ROTATION_OFFSET_DEG = config_traj.ROBOT_ROTATION_OFFSET_DEG
except ImportError:
    pass

def GetAngleMoteur1(x_eff, y_eff, z_eff, phi):
    """
    Calcule l'angle valide dans [0, 90].
    Affiche "Position non atteignable" si aucune solution réelle ou physique n'existe.
    """
    y_E = y_eff + e
    z_E = z_eff
    
    # Projection 3D: la longueur de la barre dans le plan 2D est raccourcie en fonction du décalage X
    Lb_eff2 = Lb**2 - x_eff**2
    if Lb_eff2 < 0:
        return None, None, None  # Le décalage X est physiquement impossible (plus long que le bras)
        
    # Paramètres de l'intersection cercle-cercle (2D)
    A = 2 * (f - y_E)
    B = -2 * z_E
    C = f**2 - y_E**2 - z_E**2 + Lb_eff2 - Lc**2

    # --- 1. Résolution mathématique (Intersection des cercles) ---
    if abs(B) < 1e-6:
        # Cas Z=0 (rare)
        if abs(A) < 1e-6: 
            #print("Position non atteignable")
            return None, None, None
        y_sol = C / A
        delta_z = Lc**2 - (y_sol - f)**2
        if delta_z < 0: 
           # print("Position non atteignable")
            return None, None, None
        solutions = [(y_sol, -np.sqrt(delta_z)), (y_sol, np.sqrt(delta_z))]
    else:
        # Cas général
        a_q = 1 + (A/B)**2
        b_q = -2*f - 2*(A*C)/(B**2)
        c_q = f**2 + (C/B)**2 - Lc**2
        
        delta = b_q**2 - 4*a_q*c_q
        
        # Pas de solution réelle (trop loin)
        if delta < 0: 
            #print("Position non atteignable")
            return None, None, None
        
        y1 = (-b_q - np.sqrt(delta)) / (2*a_q)
        y2 = (-b_q + np.sqrt(delta)) / (2*a_q)
        solutions = [(y1, (C - A*y1) / B), (y2, (C - A*y2) / B)]

    # --- 2. Filtrage physique [0°, 90°] ---
    valid_solution = None
    
    for y_B, z_B in solutions:
        theta_candidate = np.arctan2(y_B - f, -z_B)
        deg = np.degrees(theta_candidate)
        
        # On accepte une petite tolérance (-0.1) pour inclure 0 pile
        if -0.1 <= deg <= 165.1:
            # Si deux solutions valides, on garde celle avec le Z le plus bas
            if valid_solution is None or z_B < valid_solution[2]:
                valid_solution = (theta_candidate, y_B, z_B)

    if valid_solution:
        theta_phys, y_B, z_B = valid_solution
        
        offset_rad = math.radians(ROBOT_ROTATION_OFFSET_DEG)
        if math.isclose(phi, 0 + offset_rad, abs_tol=1e-5):
            moteur = 1
        elif math.isclose(phi, math.radians(120) + offset_rad, abs_tol=1e-5):
            moteur = 2
        else:
            moteur = 3

        return theta_phys, y_B, z_B
    else:
        # Solutions mathématiques existent mais sont hors limites (angles négatifs ou > 90)
        #print("Position non atteignable")
        return None, None, None

def GetBrasComplet(x_eff_glob, y_eff_glob, z_eff_glob, phi):
    Eff_glob = np.array([x_eff_glob, y_eff_glob, z_eff_glob])
    Eff_loc = rotZ(Eff_glob, -phi)
    
    theta, y_B, z_B = GetAngleMoteur1(Eff_loc[0], Eff_loc[1], Eff_loc[2], phi)
    
    if theta is None: return None

    # Construction géométrique
    M_loc = np.array([0, f, 0])
    B_loc = np.array([0, y_B, z_B])
    
    BG_loc = B_loc + [r_parralelogramme, 0, 0]
    BD_loc = B_loc - [r_parralelogramme, 0, 0]
    
    E_loc = np.array([Eff_loc[0], Eff_loc[1] + e, Eff_loc[2]])
    EG_loc = E_loc + [r_parralelogramme, 0, 0]
    ED_loc = E_loc - [r_parralelogramme, 0, 0]

    return {
        "M":  rotZ(M_loc, phi),
        "B":  rotZ(B_loc, phi),
        "BG": rotZ(BG_loc, phi),
        "BD": rotZ(BD_loc, phi),
        "EG": rotZ(EG_loc, phi),
        "ED": rotZ(ED_loc, phi),
        "Eff": Eff_glob
    }

# # --- TEST ---
# # Position excentrée (X=0.2)
# target = [0.1, 0.0, -0.5] 

# # Pour tester l'erreur, tu peux essayer une cible impossible, ex : [0.8, 0, -0.6]
# # target = [0.8, 0.0, -0.6] 

# fig = plt.figure(figsize=(10, 8))
# ax = fig.add_subplot(111, projection='3d')

# angles_phi = [0, np.radians(120), np.radians(240)]
# colors = ['red', 'green', 'blue']

# for i, phi in enumerate(angles_phi):
#     res = GetBrasComplet(*target, phi)
#     if res:
#         c = colors[i]
#         ax.plot([res["M"][0], res["B"][0]], [res["M"][1], res["B"][1]], [res["M"][2], res["B"][2]], color=c, lw=4)
#         ax.plot([res["BG"][0], res["EG"][0]], [res["BG"][1], res["EG"][1]], [res["BG"][2], res["EG"][2]], color='orange')
#         ax.plot([res["BD"][0], res["ED"][0]], [res["BD"][1], res["ED"][1]], [res["BD"][2], res["ED"][2]], color='teal')
#         ax.plot([res["BG"][0], res["BD"][0]], [res["BG"][1], res["BD"][1]], [res["BG"][2], res["BD"][2]], color='black')
#         ax.plot([res["EG"][0], res["ED"][0]], [res["EG"][1], res["ED"][1]], [res["EG"][2], res["ED"][2]], color='purple')
#         ax.scatter(*res["M"], color='black', s=40)

# ax.scatter(*target, color='magenta', s=150)
# ax.set_zlim(-0.8, 0.1)
# ax.set_title("Robot Delta - [0°, 90°] ou 'Position non atteignable'")
# plt.show()