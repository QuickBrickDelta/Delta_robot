import sys
import os
import json
import time

# Configuration des chemins pour les imports
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
cin_dir = os.path.join(project_root, "CinématiqueRobot")

if project_root not in sys.path:
    sys.path.append(project_root)

if cin_dir not in sys.path:
    sys.path.append(cin_dir)

# Import du module de génération des commandes moteurs
from MouvementConnecte import Motor_command_angles


def stream_commands(commands, dt_s: float = 0.01):
    """
    Version simple : envoie UNE commande à la fois,
    espacée de dt_s secondes.

    Pour l'instant: print d'une ligne CSV par commande.
    Tu pourras remplacer le print par un write() série plus tard.
    """
    for idx, cmd in enumerate(commands):
        if len(cmd) != 4:
            print(f"Commande #{idx} invalide (attendu 4 éléments) : {cmd}")
            continue

        theta1, theta2, theta3, pince_fermee = cmd
        # Une ligne CSV simple : theta1,theta2,theta3,pince(0/1)
        line = f"{float(theta1)},{float(theta2)},{float(theta3)},{1 if pince_fermee else 0}"
        print(line, flush=True)
        time.sleep(dt_s)


def main():
    # Stream \"lent\" des commandes vers la sortie standard
    stream_commands(Motor_command_angles, dt_s=0.01)


if __name__ == "__main__":
    main()

