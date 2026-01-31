import numpy as np
import matplotlib.pyplot as plt

# --- Paramètres du robot ---
f = 0.1   # Rayon base moteur
e = 0.1   # Rayon effecteur
r_parralelogramme = 0.05 
Lc = 0.3  # Longueur bras supérieur (proximal)
Lb = 0.5  # Longueur bras inférieur (distal)

def rotZ(p, phi):
    """Rotation d'un point p autour de l'axe Z d'un angle phi."""
    R = np.array([
        [np.cos(phi), -np.sin(phi), 0],
        [np.sin(phi),  np.cos(phi), 0],
        [0, 0, 1]
    ])
    return R @ p

def GetAngleMoteur1(x_eff, y_eff, z_eff):
    # Coordonnées relatives
    y_rel = y_eff + e - f
    
    # Équation du cercle pour l'intersection
    a1 = -(y_rel)
    b1 = z_eff
    c1 = (Lb**2 - x_eff**2 - y_rel**2 - z_eff**2 - Lc**2) / (2 * Lc)
    
    Denom = np.sqrt(a1**2 + b1**2)
    
    # Sécurité pour éviter les erreurs numériques
    val = np.clip(c1 / Denom, -1.0, 1.0)
    
    # Calcul de l'angle
    # On utilise - arccos pour la solution "bras vers le bas/intérieur"
    theta1 = np.arctan2(b1, a1) - np.arccos(val)
    
    # Affichage en degrés pour debug
    print(f"Angle: {np.degrees(theta1)%360:.2f}°")
    
    return theta1

def GetBrasComplet(x_eff_glob, y_eff_glob, z_eff_glob, phi):
    """Calcule les points d'un bras en forçant l'orientation interne."""
    Eff_glob = np.array([x_eff_glob, y_eff_glob, z_eff_glob])
    # Passage au repère local du bras
    Eff_loc = rotZ(Eff_glob, -phi)
    
    theta = GetAngleMoteur1(Eff_loc[0], Eff_loc[1], Eff_loc[2])
    
    # --- Construction du bras dans son plan local ---
    M_loc = np.array([0, f, 0])
    
    # Pour que le bras soit vers l'intérieur, le coude B 
    # doit avoir une coordonnée Y locale inférieure à f (plus proche du centre)
    # Ta formule le fait déjà, mais on s'assure de la construction :
    y_B = f + Lc * np.sin(theta)
    z_B = -Lc * np.cos(theta)
    B_loc = np.array([0, y_B, z_B])
    
    # Parallélogramme (Coude)
    BG_loc = B_loc + [r_parralelogramme, 0, 0]
    BD_loc = B_loc - [r_parralelogramme, 0, 0]
    
    # Attache Effecteur
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

# --- Visualisation ---
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

#Point à atteindre
target = [0.0, 0.00, -1.1] 

angles = [0, np.radians(120), np.radians(240)]
colors = ['red', 'green', 'blue']

for i, phi in enumerate(angles):
    res = GetBrasComplet(*target, phi)
    c = colors[i]
    
    # Bras Proximal (Moteur -> Coude)
    ax.plot([res["M"][0], res["B"][0]], [res["M"][1], res["B"][1]], [res["M"][2], res["B"][2]], color=c, lw=4)
    
    # Bras Distal (Parallélogramme)
    ax.plot([res["BG"][0], res["EG"][0]], [res["BG"][1], res["EG"][1]], [res["BG"][2], res["EG"][2]], color='orange', alpha=0.8)
    ax.plot([res["BD"][0], res["ED"][0]], [res["BD"][1], res["ED"][1]], [res["BD"][2], res["ED"][2]], color='teal', alpha=0.8)
    
    # Barres transversales
    ax.plot([res["BG"][0], res["BD"][0]], [res["BG"][1], res["BD"][1]], [res["BG"][2], res["BD"][2]], color='black')
    ax.plot([res["EG"][0], res["ED"][0]], [res["EG"][1], res["ED"][1]], [res["EG"][2], res["ED"][2]], color='purple')
    
    # Pointillés Base et Effecteur
    ax.plot([0, res["M"][0]], [0, res["M"][1]], [0, res["M"][2]], 'k--', alpha=0.2)
    ax.plot([res["EG"][0], res["Eff"][0]], [res["EG"][1], res["Eff"][1]], [res["EG"][2], res["Eff"][2]], 'm--', alpha=0.2)
    ax.plot([res["ED"][0], res["Eff"][0]], [res["ED"][1], res["Eff"][1]], [res["ED"][2], res["Eff"][2]], 'm--', alpha=0.2)
    
    ax.scatter(*res["M"], color='black', s=40)

ax.scatter(*target, color='magenta', s=150)
ax.set_zlim(-0.6, 0.1)
ax.set_title("Robot Delta : Tous les bras vers l'intérieur")
plt.show()