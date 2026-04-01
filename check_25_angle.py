import sys
import os
import numpy as np

# Add kinematics dir to path
sys.path.append(r"c:\Users\Raphg\Documents\Projetdrip\Delta_robot\CinématiqueRobot")

from Cinematique_delta3bras import GetAngleMoteur1

# Calculate angle for z = -25.0 (Old Home)
theta, yB, zB = GetAngleMoteur1(0.0, 0.0, -25.0, 0.0)

if theta is not None:
    print(f"THETA_RAD: {theta}")
    print(f"THETA_DEG: {np.degrees(theta)}")
else:
    print("Error: Position not reachable!")
