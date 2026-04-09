import numpy as np
import matplotlib.pyplot as plt
import math

# Paramètres du robot (issus de Cinematique_delta3bras.py)
f = 5.5   # Rayon base moteur
e = 4     # Rayon effecteur
Lc = 15   # Longueur bras supérieur
Lb = 32.5 # Longueur bras inférieur 

def GetAngleMoteur1(x_eff, y_eff, z_eff):
    y_E = y_eff + e
    z_E = z_eff
    Lb_eff2 = Lb**2 - x_eff**2
    if Lb_eff2 < 0: return None, None, None
    A = 2 * (f - y_E)
    B = -2 * z_E
    C = f**2 - y_E**2 - z_E**2 + Lb_eff2 - Lc**2
    
    a_q = 1 + (A/B)**2
    b_q = -2*f - 2*(A*C)/(B**2)
    c_q = f**2 + (C/B)**2 - Lc**2
    delta = b_q**2 - 4*a_q*c_q
    if delta < 0: return None, None, None
    
    y1 = (-b_q - np.sqrt(delta)) / (2*a_q)
    y2 = (-b_q + np.sqrt(delta)) / (2*a_q)
    
    sols = [(y1, (C - A*y1) / B), (y2, (C - A*y2) / B)]
    
    valid = None
    for y_B, z_B in sols:
        theta = np.arctan2(y_B - f, -z_B)
        if valid is None or z_B < valid[2]:
            valid = (theta, y_B, z_B)
    return valid

# Calcul pour une position donnée
x_target, y_target, z_target = 0, 0, -35
res = GetAngleMoteur1(x_target, y_target, z_target)

if res:
    theta_rad, y_B, z_B = res
    theta_deg = np.degrees(theta_rad)
    
    # Points pour le tracé (Plan YZ local)
    O = [0, 0]
    M = [f, 0]
    B = [y_B, z_B]
    E = [y_target + e, z_target]
    P = [y_target, z_target] # Centre effecteur (projeté)

    plt.figure(figsize=(10, 8))
    
    # Tracer les segments
    plt.plot([0, f], [0, 0], 'k--', label='Rayon base (f)', alpha=0.5)
    plt.plot([f, y_B], [0, z_B], 'r-o', lw=3, label=f'Bras Sup (Lc={Lc})')
    plt.plot([y_B, y_target + e], [z_B, z_target], 'b-o', lw=3, label=f'Bras Inf (Lb={Lb})')
    plt.plot([y_target, y_target + e], [z_target, z_target], 'g--', label='Rayon effecteur (e)', alpha=0.5)
    
    # Points clés
    plt.scatter([0], [0], color='black', s=100, zorder=5) # CENTRE BASE
    plt.text(0, 1, 'Centre Base', ha='center')
    
    plt.scatter([f], [0], color='red', s=100, zorder=5) # MOTEUR
    plt.text(f, 1, 'Moteur', ha='center', color='red')
    
    plt.scatter([y_B], [z_B], color='black', s=80, zorder=5) # COUDE
    plt.text(y_B + 1, z_B, 'Articulation B', va='center')
    
    plt.scatter([y_target+e], [z_target], color='blue', s=80, zorder=5) # CHEVILLE
    plt.text(y_target+e + 1, z_target, 'Articulation E', va='center')
    
    plt.scatter([y_target], [z_target], color='green', s=100, zorder=5) # CENTRE EFF
    plt.text(y_target, z_target - 2, 'Cible (Centre Effecteur)', ha='center', color='green')

    # Tracer l'angle moteur
    # L'angle theta est par rapport à la verticale descendante
    plt.plot([f, f], [0, -10], 'k:', alpha=0.3)
    # Arc pour l'angle
    angle_arc = np.linspace(-np.pi/2, -np.pi/2 + theta_rad, 50)
    plt.plot(f + 3*np.cos(angle_arc), 3*np.sin(angle_arc), 'orange', lw=2)
    plt.text(f + 2, -5, r'$\theta_{m1}$', color='orange', fontweight='bold', fontsize=16)

    plt.axhline(0, color='black', lw=1)
    plt.axvline(0, color='black', lw=1)
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.legend()
    plt.axis('equal')
    plt.title(f"Schéma d'un bras de Robot Delta\nAngle moteur calculé pour Z={z_target}")
    plt.xlabel("Y (mm)")
    plt.ylabel("Z (mm)")
    
    plt.savefig('arm_schematic.png', dpi=100)
    print("Graphique généré : arm_schematic.png")
else:
    print("Erreur : Position non atteignable")
