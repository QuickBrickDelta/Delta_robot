import numpy as np

# Paramètres du robot
f = 5.5  # Rayon base moteur
e = 4   # Rayon effecteur
Lc = 15  # Longueur bras supérieur
Lb = 32  # Longueur bras inférieur (Try 32 instead of 32.5)

def GetAngleMoteur1(x_eff, y_eff, z_eff):
    y_E = y_eff + e
    z_E = z_eff
    Lb_eff2 = Lb**2 - x_eff**2
    if Lb_eff2 < 0: return None
    A = 2 * (f - y_E)
    B = -2 * z_E
    C = f**2 - y_E**2 - z_E**2 + Lb_eff2 - Lc**2
    a_q = 1 + (A/B)**2
    b_q = -2*f - 2*(A*C)/(B**2)
    c_q = f**2 + (C/B)**2 - Lc**2
    delta = b_q**2 - 4*a_q*c_q
    if delta < 0: return None
    y1 = (-b_q - np.sqrt(delta)) / (2*a_q)
    z1 = (C - A*y1) / B
    theta = np.arctan2(y1 - f, -z1)
    return theta

# Test for z = -40
print(f"z=-40, Lb=32 -> {GetAngleMoteur1(0, 0, -40)}")
# Test for z = -20
print(f"z=-20, Lb=32 -> {GetAngleMoteur1(0, 0, -20)}")
