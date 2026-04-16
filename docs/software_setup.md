# Guide d'Installation Logicielle

Suivez ces étapes pour configurer l'environnement de contrôle du Robot Delta.

## 1. Prérequis

- **Python 3.10+** installé sur votre système.
- **Arduino IDE** (ou VS Code + PlatformIO) pour flasher le firmware.

## 2. Environnement Python (PC ou Raspberry Pi)

Il est fortement recommandé d'utiliser un environnement virtuel :

```bash
# Créer l'environnement
python -m venv venv

# Activer l'environnement
# Sur Windows :
venv\Scripts\activate
# Sur Linux/macOS :
source venv/bin/activate

# Installer les dépendances
pip install -r requirements.txt
```

## 3. Configuration du Firmware Arduino

1.  Ouvrez `Arduino/OpenRB_DeltaRobot/OpenRB_DeltaRobot.ino` dans l'IDE Arduino.
2.  Installez les bibliothèques **DynamixelShield** et le gestionnaire de cartes **OpenRB** si ce n'est pas déjà fait.
3.  Connectez votre OpenRB-150 et téléversez le code.

## 4. Configuration de la Caméra (Linux/Raspberry Pi)

Si vous utilisez un Raspberry Pi avec la caméra officielle, assurez-vous que GStreamer est installé :

```bash
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav
```

L'application utilise un pipeline optimisé pour une détection à faible latence.

## 5. Lancer l'Application

Lancez l'interface principale depuis la racine du projet :

```bash
python UI/main.py
```
