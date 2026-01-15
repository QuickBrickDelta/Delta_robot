import numpy as np
import matplotlib.pyplot as plt


#Définir la géométrie du robot
f = 0.1   # Rayon qui sépare les moteurs du point central fixe
e = 0.1   # Rayon qui sépare les cotés de la plaquette éffecteur 
r_parralelogramme = 0.05 #Rayon de la tige horizontale du parralélogramme

Lc = 0.3  # Longueur du coude
Lb = 0.5  # Longueur du bras


def GetAngleMoteur1(x_eff,y_eff,z_eff): # Fonction qui trouve l'angle de M1 en fonction de la position de l'effecteur
    a1 = -(y_eff+e-f)
    b1 = z_eff
    c1 = Lb**2 - x_eff**2 - z_eff**2 - Lc**2 - (y_eff+e-f)**2
    d1 = c1/ (2*Lc)
    Denom = np.sqrt(a1**2 + b1**2)
    val = d1/Denom

    theta1 = np.arctan2(b1,a1) - np.arccos(val)
    theta_print = np.degrees(theta1 % (2*np.pi))
    print("Angle Theta",theta_print)
    return theta1

def GetGeometrieRobot(x_eff,y_eff,z_eff): # Fonction qui détermine la géométrie du robot en fonction de la position de l'effecteur

    theta1 = GetAngleMoteur1(x_eff,y_eff,z_eff)

    P_fixe = np.array([0,0,0])

    x_M1 = 0
    y_M1 = f
    z_M1 = 0

    M1 = np.array([x_M1, y_M1, z_M1])

    x_B1 = 0
    y_B1 = f + Lc*np.sin(theta1)
    z_B1 = -Lc * np.cos(theta1)

    B1 = np.array([x_B1, y_B1, z_B1]) # Centrale du coude
    B1G = np.array([x_B1 + r_parralelogramme, y_B1, z_B1])  # Point Gauche du coude
    B1D = np.array([x_B1 - r_parralelogramme, y_B1, z_B1])  # Point Droit du coude

    E1 = np.array([x_eff, y_eff + e, z_eff])
    E1G = np.array([x_eff + r_parralelogramme, y_eff + e, z_eff])
    E1D = np.array([x_eff - r_parralelogramme, y_eff + e, z_eff])

    Eff = np.array([x_eff, y_eff, z_eff])

    return P_fixe, M1, B1, B1G, B1D, E1, E1G, E1D, Eff


# Exemple de position de l'effecteur
x_eff, y_eff, z_eff = 0.0, 0.0, -0.5
P_fixe, M1, B1, B1G, B1D, E1, E1G, E1D, Eff = GetGeometrieRobot(x_eff, y_eff, z_eff)

# Plot 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Tracer les points
ax.scatter(*P_fixe, color='brown', s=60, label='Origine P_fixe')
ax.scatter(*M1, color='red', s=50, label='Moteur M1')
ax.scatter(*Eff, color='magenta', s=50, label='Centre effecteur')

# Tracer les segments du bras (parallélogramme)
ax.plot([M1[0], B1[0]], [M1[1], B1[1]], [M1[2], B1[2]], color='black')
ax.plot([B1[0], B1D[0]], [B1[1], B1D[1]], [B1[2], B1D[2]], color='black')
ax.plot([B1[0], B1G[0]], [B1[1], B1G[1]], [B1[2], B1G[2]], color='black')
ax.plot([B1G[0], E1G[0]], [B1G[1], E1G[1]], [B1G[2], E1G[2]], color='orange')
ax.plot([B1D[0], E1D[0]], [B1D[1], E1D[1]], [B1D[2], E1D[2]], color='teal')
ax.plot([E1G[0], E1D[0]], [E1G[1], E1D[1]], [E1G[2], E1D[2]], color='purple', linestyle='dashed')
ax.plot([E1[0], Eff[0]], [E1[1], Eff[1]], [E1[2], Eff[2]], color='magenta', linestyle='dashed')
ax.plot([P_fixe[0], M1[0]], [P_fixe[1], M1[1]], [P_fixe[2], M1[2]], color='brown')

# Axes et labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Bras 1 du robot Delta')
ax.legend()

# Même échelle sur les 3 axes
all_points = np.array([P_fixe, M1, B1, B1G, B1D, E1, E1G, E1D, Eff])
max_range = np.ptp(all_points, axis=0).max() / 2.0
mid_x = (all_points[:,0].max() + all_points[:,0].min()) * 0.5
mid_y = (all_points[:,1].max() + all_points[:,1].min()) * 0.5
mid_z = (all_points[:,2].max() + all_points[:,2].min()) * 0.5
ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

plt.show()
