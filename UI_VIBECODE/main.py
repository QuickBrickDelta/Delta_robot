import sys
import os
import subprocess
import threading
import numpy as np

# Configuration des chemins pour importer les modules du robot
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
cinematique_dir = os.path.join(project_root, "CinématiqueRobot")
if project_root not in sys.path:
    sys.path.append(project_root)
if cinematique_dir not in sys.path:
    sys.path.append(cinematique_dir)

from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QFrame)
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt6.QtGui import QFont, QPalette, QColor

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D

# Imports robot pour la simulation
try:
    from MouvementConnecte import Motor_command_xyz
    from Cinematique_delta3bras import rotZ, GetBrasComplet
except ImportError as e:
    print(f"Erreur d'importation des modules du robot: {e}")
    Motor_command_xyz = []

class WorkerThread(QThread):
    """ Thread pour lancer PieToArduino en arrière-plan sans geler l'UI """
    finished_signal = pyqtSignal()
    output_signal = pyqtSignal(str)

    def run(self):
        # Lancer le script PieToArduino
        script_path = os.path.join(project_root, "Communication", "PieToArduino.py")
        try:
            # Exécution silencieuse (les prints vont dans la console)
            process = subprocess.Popen([sys.executable, script_path], 
                                       stdout=subprocess.PIPE, 
                                       stderr=subprocess.STDOUT,
                                       text=True)
            for line in process.stdout:
                self.output_signal.emit(line.strip())
            process.wait()
        except Exception as e:
            self.output_signal.emit(f"Erreur: {e}")
        self.finished_signal.emit()

class VibeCodeUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VibeCode Control — Delta Robot")
        self.resize(1000, 700)
        self.setStyleSheet(self.get_dark_theme())

        # Widget central
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(20)

        # ====== GAUCHE : Panneau de contrôle ======
        control_panel = QFrame()
        control_panel.setStyleSheet("""
            QFrame {
                background-color: #1E1E2E;
                border-radius: 15px;
                border: 2px solid #89B4FA;
            }
        """)
        control_panel.setFixedWidth(350)
        control_layout = QVBoxLayout(control_panel)
        control_layout.setContentsMargins(20, 30, 20, 30)

        # Titre
        title = QLabel("D E L T A\nV I B E")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("color: #89B4FA; font-size: 36px; font-weight: bold; border: none;")
        control_layout.addWidget(title)

        control_layout.addStretch()

        # Status
        self.status_label = QLabel("STATUT : PRÊT")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet("color: #A6E3A1; font-size: 18px; font-weight: bold; border: none;")
        control_layout.addWidget(self.status_label)

        control_layout.addSpacing(30)

        # Bouton Démarrer
        self.btn_start = QPushButton("▶ DÉMARRER")
        self.btn_start.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_start.setStyleSheet("""
            QPushButton {
                background-color: #89B4FA;
                color: #11111B;
                font-size: 24px;
                font-weight: bold;
                border-radius: 10px;
                padding: 15px;
                border: none;
            }
            QPushButton:hover {
                background-color: #74C7EC;
            }
            QPushButton:pressed {
                background-color: #89DCEB;
            }
            QPushButton:disabled {
                background-color: #585B70;
                color: #A6ADC8;
            }
        """)
        self.btn_start.clicked.connect(self.start_robot)
        control_layout.addWidget(self.btn_start)

        # Bouton Quitter
        self.btn_quit = QPushButton("Quitter")
        self.btn_quit.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_quit.setStyleSheet("""
            QPushButton {
                background-color: transparent;
                color: #F38BA8;
                font-size: 16px;
                border: 1px solid #F38BA8;
                border-radius: 5px;
                padding: 8px;
            }
            QPushButton:hover {
                background-color: rgba(243, 139, 168, 0.1);
            }
        """)
        self.btn_quit.clicked.connect(self.close)
        control_layout.addWidget(self.btn_quit)

        control_layout.addStretch()

        main_layout.addWidget(control_panel)

        # ====== DROITE : Matplotlib 3D ======
        plot_frame = QFrame()
        plot_frame.setStyleSheet("""
            QFrame {
                background-color: #11111B;
                border-radius: 15px;
                border: 2px solid #585B70;
            }
        """)
        plot_layout = QVBoxLayout(plot_frame)
        plot_layout.setContentsMargins(10, 10, 10, 10)

        self.fig = plt.figure(facecolor='#11111B')
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor('#11111B')
        
        # Désactiver les axes pour un look plus "vibe"
        self.ax.set_axis_off()
        
        plot_layout.addWidget(self.canvas)
        main_layout.addWidget(plot_frame)

        # ====== ANIMATION 3D ======
        self.current_frame = 0
        self.traj_points = []
        if Motor_command_xyz:
            self.traj_points = Motor_command_xyz
        
        # Initial draw
        if self.traj_points:
            self.draw_robot(self.traj_points[0][:3])

        self.anim_timer = QTimer()
        self.anim_timer.timeout.connect(self.update_animation)
        self.is_animating = False

        self.worker = None

    def get_dark_theme(self):
        return """
            QMainWindow {
                background-color: #11111B;
            }
        """

    def draw_robot(self, pos):
        self.ax.cla()
        self.ax.set_axis_off()
        lim = 40.0
        self.ax.set_xlim(-lim, lim)
        self.ax.set_ylim(-lim, lim)
        self.ax.set_zlim(-80.0, 10.0)

        angles_phi = [0, np.radians(120), np.radians(240)]
        colors = ['#F38BA8', '#A6E3A1', '#89B4FA'] # Vibe colors (Red, Green, Blue)

        # Base
        base_pts = [rotZ(np.array([0, 10.0, 0]), a) for a in angles_phi]
        base_pts.append(base_pts[0])
        self.ax.plot(np.array(base_pts)[:,0], np.array(base_pts)[:,1], np.array(base_pts)[:,2], color='#CDD6F4', lw=2)

        # Bras
        for k, phi in enumerate(angles_phi):
            res = GetBrasComplet(pos[0], pos[1], pos[2], phi)
            if res is None: continue
            c = colors[k]
            # Moteur -> Coude
            self.ax.plot([res["M"][0], res["B"][0]], [res["M"][1], res["B"][1]], [res["M"][2], res["B"][2]], color=c, lw=4)
            # Parallélogramme
            self.ax.plot([res["BG"][0], res["EG"][0]], [res["BG"][1], res["EG"][1]], [res["BG"][2], res["EG"][2]], color='#F9E2AF', lw=2)
            self.ax.plot([res["BD"][0], res["ED"][0]], [res["BD"][1], res["ED"][1]], [res["BD"][2], res["ED"][2]], color='#94E2D5', lw=2)
            # Entretoises
            self.ax.plot([res["BG"][0], res["BD"][0]], [res["BG"][1], res["BD"][1]], [res["BG"][2], res["BD"][2]], color='#585B70', lw=2)
            self.ax.plot([res["EG"][0], res["ED"][0]], [res["EG"][1], res["ED"][1]], [res["EG"][2], res["ED"][2]], color='#CBA6F7', lw=2)
            
        self.canvas.draw()

    def update_animation(self):
        if not self.traj_points or self.current_frame >= len(self.traj_points):
            self.anim_timer.stop()
            self.is_animating = False
            return
            
        # Simplification: juste sauter de point en point pour l'instant (la vraie interpo prend trop de frames pour un PyQt timer de base)
        pos = self.traj_points[self.current_frame][:3]
        self.draw_robot(pos)
        self.current_frame += 1

    def start_robot(self):
        self.btn_start.setDisabled(True)
        self.btn_quit.setDisabled(True)
        self.status_label.setText("STATUT : EN MOUVEMENT")
        self.status_label.setStyleSheet("color: #F9E2AF; font-size: 18px; font-weight: bold; border: none;")

        # Redémarrer l'animation de 0
        self.current_frame = 0
        if self.traj_points:
            self.is_animating = True
            self.anim_timer.start(50) # 50ms par waypoint

        # Lancer PieToArduino en parallèle
        self.worker = WorkerThread()
        self.worker.output_signal.connect(self.log_output)
        self.worker.finished_signal.connect(self.robot_finished)
        self.worker.start()

    def log_output(self, text):
        print(f"[PIE] {text}")

    def robot_finished(self):
        self.status_label.setText("STATUT : TERMINÉ")
        self.status_label.setStyleSheet("color: #A6E3A1; font-size: 18px; font-weight: bold; border: none;")
        self.btn_start.setDisabled(False)
        self.btn_quit.setDisabled(False)
        self.anim_timer.stop()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = VibeCodeUI()
    window.show()
    sys.exit(app.exec())
