from Cinematique_delta3bras import rotZ, GetAngleMoteur1, GetBrasComplet
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- 1. FONCTIONS D'INTERPOLATION ---

def interpolate_linear(p_start, p_end, steps):
    """
    Génère une liste de points (x,y,z) formant une ligne droite.
    """
    traj = []
    xs = np.linspace(p_start[0], p_end[0], steps)
    ys = np.linspace(p_start[1], p_end[1], steps)
    zs = np.linspace(p_start[2], p_end[2], steps)
    
    for i in range(steps):
        traj.append([xs[i], ys[i], zs[i]])
    return traj

def interpolate_joint(p_start, p_end, steps):
    """
    Génère une liste d'ANGLES [theta1, theta2, theta3] interpolés linéairement.
    """
    # Calcul des angles aux extrémités
    angles_start = GetAngleMoteur1(*p_start) # Retourne (theta, y, z) ou juste theta selon ta version
    angles_end = GetAngleMoteur1(*p_end)
    
    # Adaptation selon ce que retourne exactement ta fonction GetAngleMoteur1
    # Si elle retourne un tuple (theta, y, z), on prend juste l'index 0.
    if isinstance(angles_start, tuple): angles_start = angles_start[0]
    if isinstance(angles_end, tuple): angles_end = angles_end[0]

    # Pour les 3 bras (ici on suppose symétrie ou on appelle GetAnglesRobot si dispo)
    # NOTE: Pour simplifier ici, on suppose que GetAngleMoteur1 gère le bras 1.
    # Pour une vraie interpolation JOINT sur les 3 bras, il faudrait calculer les 3 angles de départ et fin.
    # Ici, c'est une structure pour l'exemple.
    
    return [] # Simplifié pour la simulation visuelle qui utilise LIN

# --- 2. FONCTION DE SIMULATION TEMPS RÉEL ---

def run_simulation_realtime(points_passage, steps_per_move=30):
    """
    Parcourt la liste de points et anime le robot en temps réel.
    """
    # --- Configuration du Plot ---
    plt.ion() # Mode Interactif ON
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Limites fixes (CRUCIAL pour l'animation)
    lim = 0.4
    z_min, z_max = -0.8, 0.1
    
    colors = ['red', 'green', 'blue']
    angles_phi = [0, np.radians(120), np.radians(240)]
    
    all_commands = [] # Pour stocker l'historique des angles

    print("--- Démarrage Simulation Temps Réel ---")

    # Boucle sur chaque segment (Point A -> Point B)
    for i in range(len(points_passage) - 1):
        p_start = points_passage[i]
        p_end = points_passage[i+1]
        
        print(f"Déplacement : {p_start} -> {p_end}")
        
        # On génère les points intermédiaires (LIN pour l'affichage visuel)
        traj_points = interpolate_linear(p_start, p_end, steps_per_move)
        
        # Boucle d'animation pour ce segment
        for pos in traj_points:
            ax.cla() # Effacer l'image précédente
            
            # Remettre les limites et labels (cla() efface tout)
            ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(z_min, z_max)
            ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
            ax.set_title(f"Simulation: {pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}")
            
            # Dessiner la Base Fixe (Triangle noir)
            base_pts = [rotZ(np.array([0, 0.1, 0]), a) for a in angles_phi] # f=0.1 supposé
            base_pts.append(base_pts[0]) 
            base_pts = np.array(base_pts)
            ax.plot(base_pts[:,0], base_pts[:,1], base_pts[:,2], 'k-', lw=1)

            current_angles = []

            # Calculer et Dessiner les 3 bras
            for k, phi in enumerate(angles_phi):
                # Appel à ta fonction cinématique inverse complète
                res = GetBrasComplet(pos[0], pos[1], pos[2], phi)
                
                if res is None:
                    continue # Skip si hors limite
                
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
                
                # Récupération de l'angle (dépend de ce que GetBrasComplet ou GetAngleMoteur1 retourne)
                # Ici on recalcule juste pour l'affichage ou on le stocke si GetBrasComplet le donne
                # Supposons qu'on le recalcule ou qu'il est dans 'res' si tu l'as ajouté.
                # Pour l'instant, on stocke la position pour simuler la commande.
                
            # Effecteur
            ax.scatter(pos[0], pos[1], pos[2], color='magenta', s=50)
            
            # Pause pour l'animation (vitesse)
            plt.pause(0.01) 
            
    plt.ioff() # Fin du mode interactif
    print("Simulation terminée.")
    plt.show()

# --- 3. EXÉCUTION ---

# Liste de points à parcourir (X, Y, Z)
points_passage = [
    [0.0, 0.0, -0.65],   # Point 1 (Centre bas)
    [0.2, 0.0, -0.60],   # Point 2 (Droite)
    [0.0, 0.2, -0.55],   # Point 3 (Haut Gauche)
    [-0.2, -0.1, -0.70], # Point 4 (Bas Gauche Profond)
    [0.0, 0.0, -0.65]    # Retour au centre
]

# Lancement
run_simulation_realtime(points_passage, steps_per_move=40)