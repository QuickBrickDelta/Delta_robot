import numpy as np
import matplotlib.pyplot as plt

# Paramètres du robot
f = 0.1   # Rayon base moteur
e = 0.1   # Rayon effecteur
r_parralelogramme = 0.05 # Demi-largeur parallélogramme
Lc = 0.3  # Longueur bras inférieur (parallélogramme)
Lb = 0.5  # Longueur bras supérieur

def rotZ(p, phi):
    """Rotation autour de Z"""
    R = np.array([
        [np.cos(phi), -np.sin(phi), 0],
        [np.sin(phi),  np.cos(phi), 0],
        [0, 0, 1]
    ])
    return R @ p

def GetAngleMoteur(x_eff, y_eff, z_eff, f_local):
    """Inverse cinématique pour un bras donné dans son plan"""
    a = -(y_eff + e - f_local)
    b = z_eff
    c = Lb**2 - x_eff**2 - z_eff**2 - Lc**2 - (y_eff + e - f_local)**2
    D = np.sqrt(a**2 + b**2)
    val = np.clip(c / (2*Lc*D), -1.0, 1.0)
    theta = np.arctan2(b, a) - np.arccos(val)
    return theta

def GetBras(x_eff, y_eff, z_eff, phi):
    """Calcule les points d'un bras à partir de l'effecteur et de phi"""
    # Position de l'effecteur dans le plan local
    Eff = np.array([x_eff, y_eff, z_eff])
    E_center = Eff + np.array([0,e, 0])
    E_G = E_center + np.array([r_parralelogramme, 0, 0])
    E_D = E_center - np.array([r_parralelogramme, 0, 0])

    # Rotation de l'effecteur et de ses points
    E_center = rotZ(E_center, phi)
    E_G = rotZ(E_G, phi)
    E_D = rotZ(E_D, phi)
    Eff_rot = rotZ(Eff, phi)

    # Calcul angle moteur dans le plan du bras
    theta = GetAngleMoteur(x_eff, y_eff, z_eff, f)
    # Position centrale du coude
    B = np.array([0, f + Lc*np.sin(theta), -Lc*np.cos(theta)])
    B_G = B + np.array([r_parralelogramme, 0, 0])
    B_D = B - np.array([r_parralelogramme, 0, 0])

    # Rotation du bras entier
    B = rotZ(B, phi)
    B_G = rotZ(B_G, phi)
    B_D = rotZ(B_D, phi)

    # Position du moteur
    M = rotZ(np.array([0, f, 0]), phi)

    return {"M": M, "B": B, "BG": B_G, "BD": B_D, "EG": E_G, "ED": E_D, "Eff": Eff_rot}

# Position effecteur
x_eff, y_eff, z_eff = 0.0, 0.0, -0.5

# Calcul des 3 bras
bras = []
for i in range(3):
    phi = np.radians(120 * i)  # 0°, 120°, 240°
    bras.append(GetBras(x_eff, y_eff, z_eff, phi))

# Plot 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

colors = ['red', 'green', 'blue']
Eff = bras[0]["Eff"]  # Effecteur central
ax.scatter(*Eff, color='magenta', s=60, label='Centre effecteur')

for i, r in enumerate(bras):
    ax.scatter(*r["M"], color='black', s=50)
    ax.plot(*zip(r["M"], r["B"]), color=colors[i])
    ax.plot(*zip(r["B"], r["BG"]), color=colors[i])
    ax.plot(*zip(r["B"], r["BD"]), color=colors[i])
    ax.plot(*zip(r["BG"], r["EG"]), color='orange')
    ax.plot(*zip(r["BD"], r["ED"]), color='teal')
    ax.plot(*zip(r["EG"], r["ED"]), color='purple', linestyle='dashed')
    ax.plot(*zip(r["EG"], Eff), color='magenta', linestyle='dashed')
    ax.plot(*zip(r["ED"], Eff), color='magenta', linestyle='dashed')

# Axes
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Delta Robot 3 bras')
ax.legend()

# Échelle identique sur les 3 axes
all_points = np.array([p for r in bras for p in [r["M"], r["B"], r["BG"], r["BD"], r["EG"], r["ED"], r["Eff"]]])
max_range = np.ptp(all_points, axis=0).max() / 2.0
mid = (all_points.max(axis=0) + all_points.min(axis=0)) / 2
ax.set_xlim(mid[0]-max_range, mid[0]+max_range)
ax.set_ylim(mid[1]-max_range, mid[1]+max_range)
ax.set_zlim(mid[2]-max_range, mid[2]+max_range)

plt.show()
