import sys
import os
import time
import serial
import serial.tools.list_ports

# ===============================
# PARAMÈTRE — Change le port ici
# ===============================
COM_PORT = "COM3"
BAUDRATE = 115200

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


# ===============================
# Détection du port série
# ===============================

def find_openrb_port():
    """Cherche automatiquement le port série de l'OpenRB-150.
    Priorité : USB Serial > tout le reste. Ignore le Bluetooth."""
    ports = serial.tools.list_ports.comports()
    print("Ports série disponibles :")
    for p in ports:
        print(f"  {p.device} — {p.description}")
    
    # Argument en ligne de commande ? (ex: python PieToArduino.py COM3)
    if len(sys.argv) > 1:
        return sys.argv[1]
    
    # Chercher un port USB (pas Bluetooth)
    for p in ports:
        desc = p.description.lower()
        if "usb" in desc or "openrb" in desc:
            return p.device
    
    # Fallback : premier port non-Bluetooth
    for p in ports:
        if "bluetooth" not in p.description.lower():
            return p.device
    
    # Rien trouvé automatiquement : demander à l'utilisateur
    user_port = input("\nAucun port USB détecté. Entre le port manuellement (ex: COM3) : ").strip()
    if user_port:
        return user_port
    return None


# ===============================
# Envoi des commandes via série
# ===============================

def stream_commands(ser, commands, dt_s: float = 0.05):
    """
    Envoie les commandes sur le port série, une par une,
    espacées de dt_s secondes.
    
    Format CSV : theta1,theta2,theta3,pince(0/1)\n
    """
    print(f"\nEnvoi de {len(commands)} commandes (dt = {dt_s}s)...")
    
    for idx, cmd in enumerate(commands):
        if len(cmd) != 4:
            print(f"Commande #{idx} invalide (attendu 4 éléments) : {cmd}")
            continue

        theta1, theta2, theta3, pince_fermee = cmd
        # Ligne CSV identique au format attendu par l'Arduino
        line = f"{float(theta1)},{float(theta2)},{float(theta3)},{1 if pince_fermee else 0}\n"
        
        ser.write(line.encode('utf-8'))
        
        # Lire la réponse de debug de l'Arduino (optionnel)
        if ser.in_waiting > 0:
            response = ser.readline().decode('utf-8', errors='ignore').strip()
            if response:
                print(f"  Arduino: {response}")
        
        time.sleep(dt_s)
    
    print("Envoi terminé !")


# ===============================
# Main
# ===============================

def main():
    # 1) Trouver le port
    port = find_openrb_port()
    if port is None:
        print("ERREUR : Aucun port série trouvé. Vérifie que l'OpenRB est branché.")
        sys.exit(1)
    
    print(f"\nConnexion sur {port} à 115200 baud...")
    
    # 2) Ouvrir la connexion série
    ser = serial.Serial(port, baudrate=115200, timeout=1)
    time.sleep(2)  # Attendre le reset de l'Arduino après connexion
    
    # 3) Lire le message de bienvenue
    while ser.in_waiting > 0:
        print(f"  Arduino: {ser.readline().decode('utf-8', errors='ignore').strip()}")
    
    # 4) Envoyer les commandes
    if not Motor_command_angles:
        print("Aucune commande à envoyer (Motor_command_angles est vide).")
    else:
        stream_commands(ser, Motor_command_angles, dt_s=0.05)
    
    # 5) Fermer proprement
    ser.close()
    print("Port série fermé.")


if __name__ == "__main__":
    main()
