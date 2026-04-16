print(">>> Script de communication PieToArduino lancé")
import sys
import os
import time
import json
import argparse
import serial
import serial.tools.list_ports

# Configuration des chemins pour les imports
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
cin_dir = os.path.join(project_root, "CinématiqueRobot")

if project_root not in sys.path:
    sys.path.append(project_root)
if cin_dir not in sys.path:
    sys.path.append(cin_dir)

# Parse les arguments
parser = argparse.ArgumentParser(description="Envoi de commandes au robot via port série")
parser.add_argument("--manual", metavar="JSON_PATH",
                    help="Chemin vers un fichier JSON de commandes manuelles à envoyer directement")
parser.add_argument("port", nargs="?", help="Port série (ex: COM3). Si omis, recherche automatique.")
args, _ = parser.parse_known_args()

if args.manual:
    # Mode manuel : lire les angles depuis un fichier JSON
    print(f"[PIE] Mode MANUEL — chargement depuis {args.manual}")
    with open(args.manual, 'r') as f:
        Motor_command_angles = json.load(f)
else:
    # Mode auto : importer MouvementConnecte (planification complète)
    print(f"[DEBUG] Tentative d'import de MouvementConnecte...")
    print(f"[DEBUG] sys.path contains: {sys.path}")
    try:
        import MouvementConnecte
        Motor_command_angles = MouvementConnecte.Motor_command_angles
        print(f"[DEBUG] Import réussi. Motor_command_angles len: {len(Motor_command_angles)}")
    except ImportError as e:
        print(f"[PIE] ERREUR : Impossible d'importer MouvementConnecte : {e}")
        Motor_command_angles = []
    except Exception as e:
        print(f"[PIE] ERREUR INCONNUE lors de l'import : {e}")
        import traceback
        traceback.print_exc()
        Motor_command_angles = []


# ===============================
# Détection du port série
# ===============================

def find_openrb_port():
    """Cherche automatiquement le port série de l'OpenRB-150.
    Priorité : Port passé en argument > USB Serial > tout le reste. Ignore le Bluetooth."""
    
    # 1. Si un port a été passé explicitement via argparse
    if args.port:
        return args.port

    ports = list(serial.tools.list_ports.comports())
    print("[PIE] Ports série disponibles :")
    for p in ports:
        print(f"  {p.device} — {p.description}")
    
    # 2. Chercher un port USB (pas Bluetooth)
    for p in ports:
        desc = p.description.lower()
        # On cherche des mots clés typiques des cartes de contrôle
        if any(kw in desc for kw in ["usb", "openrb", "arduino", "serial"]):
            if "bluetooth" not in desc:
                return p.device
    
    # 3. Fallback : premier port non-Bluetooth
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

def stream_commands(ser, commands, dt_s: float = 0.02):
    """
    Envoie les commandes sur le port série, une par une,
    espacées de dt_s secondes.
    
    Format CSV : theta1,theta2,theta3,pince(0/1)\n
    """
    print(f"\nEnvoi de {len(commands)} commandes (dt = {dt_s}s)...")
    
    for idx, cmd in enumerate(commands):
        if len(cmd) != 5:
            print(f"Commande #{idx} invalide (attendu 5 éléments) : {cmd}")
            continue

        theta1, theta2, theta3, pince_fermee, angle_deg = cmd

        line = (
            f"{float(theta1)},"
            f"{float(theta2)},"
            f"{float(theta3)},"
            f"{1 if pince_fermee else 0},"
            f"{float(angle_deg)}\n"
        )

        
        ser.write(line.encode('utf-8'))
        
        # Lire la réponse de debug de l'Arduino (optionnel)
        if ser.in_waiting > 0:
            response = ser.readline().decode('utf-8', errors='ignore').strip()
            if response:
                if "ALARM" in response:
                    print(f"\n[!!!] ALERTE MATÉRIELLE : {response}")
                    print("L'alarme rouge s'est déclenchée. Vérifie l'alimentation et la position du robot.")
                else:
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
    time.sleep(0.5)  # Attendre le reset de l'Arduino après connexion (OpenRB ~500ms)
    
    # 3) Test de connexion (Ping/Pong)
    print(">>> Test de connexion (Ping)...")
    ser.write(b"?\n") # Envoi d'un ping
    time.sleep(0.2)  # Laisser le temps à l'Arduino de répondre au ping
    
    has_responded = False
    while ser.in_waiting > 0:
        msg = ser.readline().decode('utf-8', errors='ignore').strip()
        if msg:
            print(f"  Arduino: {msg}")
            if "PONG" in msg or "OpenRB" in msg:
                has_responded = True
    
    if has_responded:
        print(">>> CONNEXION BIDIRECTIONNELLE ÉTABLIE (Robot -> Python OK)")
    else:
        print(">>> ATTENTION : Le robot n'a pas encore répondu.")
        print("    Essayez d'appuyer sur le bouton RESET de l'OpenRB maintenant !")
        # Une deuxième chance après le message
        time.sleep(0.5)  # Deuxième chance réduite
        while ser.in_waiting > 0:
            msg = ser.readline().decode('utf-8', errors='ignore').strip()
            if msg:
                print(f"  Arduino (après reset): {msg}")
                has_responded = True
    
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
