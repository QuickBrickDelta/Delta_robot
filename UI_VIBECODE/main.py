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
from PyQt6.QtGui import QFont, QPalette, QColor, QImage, QPixmap

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D

import cv2
try:
    import mediapipe as mp
    HAS_MEDIAPIPE = True
except ImportError:
    HAS_MEDIAPIPE = False

# Imports robot pour la simulation
try:
    from MouvementConnecte import Motor_command_xyz
    from Cinematique_delta3bras import rotZ, GetBrasComplet
    from MouvementRobot import interpolate_linear, interpolate_joint
except ImportError as e:
    print(f"Erreur d'importation des modules robot : {e}. Assurez-vous d'exécuter depuis la racine du projet.")
    sys.exit(1)
    Motor_command_xyz = [] # Keep this line from original code
    interpolate_linear = None # Keep this line from original code
    interpolate_joint = None # Keep this line from original code

# Import module Vision
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from VisionNumerique.HSV.bloc_detection import (
        COLOR_RANGES, load_homography, pix_to_world_cm,
        MIN_AREA_PX, COLOR_STD_THRESH, DOMINANT_FRAC_THRESH,
        RECT_ANGLE_TOL_DEG, RECT_AREA_RATIO_MIN
    )
    from VisionNumerique.HSV.bloc_detection_w_filter import detect_blocks
    HAS_VISION = True
except ImportError as e:
    print(f"Module VisionNumerique non trouvé : la vision des blocs sera désactivée. Erreur: {e}")
    HAS_VISION = False

class WorkerThread(QThread):
    """ Thread pour lancer PieToArduino en arrière-plan sans geler l'UI """
    finished_signal = pyqtSignal()
    output_signal = pyqtSignal(str)

    def run(self):
        # Lancer le script PieToArduino
        script_path = os.path.join(project_root, "Communication", "PieToArduino.py")
        try:
            # Exécution silencieuse (les prints vont dans la console)
            self.process = subprocess.Popen([sys.executable, script_path], 
                                       stdout=subprocess.PIPE, 
                                       stderr=subprocess.STDOUT,
                                       text=True)
            for line in self.process.stdout:
                self.output_signal.emit(line.strip())
            self.process.wait()
        except Exception as e:
            self.output_signal.emit(f"Erreur: {e}")
        self.finished_signal.emit()

    def stop(self):
        if hasattr(self, 'process') and self.process.poll() is None:
            self.process.terminate()

class CameraThread(QThread):
    """ Thread pour capturer la caméra OpenCV + Mediapipe et envoyer les images à l'UI """
    change_pixmap_signal = pyqtSignal(QImage)
    status_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._run_flag = True
        self.latest_blocks = []
        
        # Charger z_table pour les coordonnées physiques des blocs
        self.z_table = -38.0
        try:
            import config_traj
            self.z_table = config_traj.z_table
        except:
            pass

    def run(self):
        if os.name == 'nt':
            # Environnement local Windows: Webcam standard
            cap = cv2.VideoCapture(0)
        else:
            # Environnement Raspberry Pi: Pipeline GStreamer (libcamera)
            pipeline = (
                "libcamerasrc ! "
                "video/x-raw,format=NV12,width=1280,height=720,framerate=15/1 ! "
                "videoconvert ! video/x-raw,format=BGR ! "
                "appsink drop=true max-buffers=1 sync=false"
            )
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not cap.isOpened():
            self.status_signal.emit("Caméra introuvable")
            return

        if HAS_MEDIAPIPE:
            mp_hands = mp.solutions.hands
            mp_drawing = mp.solutions.drawing_utils
            hands = mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=1,
                model_complexity=0,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5
            )
        else:
            hands = None

        while self._run_flag:
            ret, frame = cap.read()
            if not ret:
                continue

            # Inverser l'image pour un effet miroir plus naturel
            frame = cv2.flip(frame, 1)

            # Conversion pour Qt/Mediapipe (RGB)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # 1. Overlay Computer Vision (Blocs)
            if HAS_VISION:
                if not hasattr(self, 'H_cam'):
                    loaded = load_homography()
                    if loaded:
                        self.H_cam, _, _, self.cam_center = loaded
                        self.cam_center = self.cam_center.reshape(2)
                    else:
                        self.H_cam = None

                # Appel du NOUVEAU détecteur ("_w_filter")
                # Il utilise ses propres variables globales pour les seuils
                detections = detect_blocks(frame, COLOR_RANGES, h_data=None)
                
                current_blocks = []
                for det in detections:
                    box = np.array(det["box"], dtype=np.int32)
                    cx, cy = det["center"]
                    col = det["color"]
                    
                    # Couleurs en RGB pour dessiner sur l'image Qt
                    color_rgb = {
                        "red": (255, 0, 0), "green": (0, 255, 0),
                        "green_dark": (0, 150, 0), "green_light": (144, 238, 144),
                        "blue": (0, 0, 255), "yellow": (255, 255, 0),
                        "orange": (255, 165, 0), "purple": (255, 0, 255),
                        "white": (220, 220, 220),
                    }.get(col, (255, 255, 255))

                    cv2.drawContours(rgb, [box], 0, color_rgb, 2)
                    cv2.circle(rgb, (int(cx), int(cy)), 5, color_rgb, -1)
                    
                    label = f"{col}"
                    if self.H_cam is not None:
                        xy_world = pix_to_world_cm((cx, cy), self.H_cam)
                        if xy_world:
                            Xcm = float(xy_world[0] - self.cam_center[0])
                            Ycm = float(xy_world[1] - self.cam_center[1])
                            label += f" | X={Xcm:+.1f} Y={Ycm:+.1f}"
                            
                            # Normaliser la couleur pour le planificateur de trajectoire
                            # ex: "green_dark" -> "green"
                            base_color = col.split('_')[0] if '_' in col else col
                            
                            # Ajouter le bloc détecté à la liste partagée (format final MouvementConnecte)
                            current_blocks.append([base_color, "2x4", round(Xcm, 2), round(Ycm, 2), float(self.z_table)])
                            
                    cv2.putText(rgb, label, (int(cx) + 8, int(cy) - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_rgb, 2, cv2.LINE_AA)
                                
                self.latest_blocks = current_blocks

            # 2. Overlay Mediapipe (Mains)
            if hands:
                result = hands.process(rgb)
                # Dessin des points de repère de la main
                if result.multi_hand_landmarks:
                    for hand_landmarks in result.multi_hand_landmarks:
                        mp_drawing.draw_landmarks(
                            rgb, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
            # Convertir pour PyQt
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            
            # Redimensionner l'image pour l'UI
            p = qt_image.scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio)
            self.change_pixmap_signal.emit(p)

        if hands:
            hands.close()
        cap.release()

    def stop(self):
        self._run_flag = False
        self.wait()

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

        # ====== DROITE : Matplotlib 3D + Caméra ======
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(10)

        # -- Haut : Plot 3D
        plot_frame = QFrame()
        plot_frame.setStyleSheet("""
            QFrame {
                background-color: #11111B;
                border-radius: 15px;
                border: 2px solid #585B70;
            }
        """)
        plot_layout = QVBoxLayout(plot_frame)
        plot_layout.setContentsMargins(0, 0, 0, 0) # Supprime les marges du widget

        self.fig = plt.figure(facecolor='#11111B')
        self.fig.subplots_adjust(left=-0.05, right=1.05, bottom=-0.05, top=1.05) # "Zoom" extrême en enlevant les marges matplotlib
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor('#11111B')
        self.ax.set_axis_off()
        
        plot_layout.addWidget(self.canvas)
        right_layout.addWidget(plot_frame, stretch=2) # Le plot prend 2/3 de l'espace

        # -- Bas : Caméra
        cam_frame = QFrame()
        cam_frame.setStyleSheet("""
            QFrame {
                background-color: #1E1E2E;
                border-radius: 15px;
                border: 2px solid #A6E3A1;
            }
        """)
        cam_layout = QVBoxLayout(cam_frame)
        cam_layout.setContentsMargins(5, 5, 5, 5)
        
        self.cam_label = QLabel("INITIALISATION DE LA CAMÉRA...")
        self.cam_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.cam_label.setStyleSheet("color: #A6ADC8; font-size: 16px; font-weight: bold; border: none;")
        cam_layout.addWidget(self.cam_label)
        
        right_layout.addWidget(cam_frame, stretch=1) # La caméra prend 1/3 de l'espace

        main_layout.addWidget(right_panel)

        # Démarrage direct du Thread de Caméra
        self.camera_thread = CameraThread()
        self.camera_thread.change_pixmap_signal.connect(self.update_camera_image)
        self.camera_thread.status_signal.connect(self.update_camera_status)
        self.camera_thread.start()

        # ====== ANIMATION 3D ======
        self.anim_timer = QTimer()
        self.anim_timer.timeout.connect(self.update_animation)
        self.is_animating = False

        self.worker = None

    def rebuild_trajectory(self):
        """ Recharge la trajectoire et l'animation basées sur les derniers blocs détectés """
        global Motor_command_xyz
        
        # 1. Sauvegarder les blocs vus par la caméra
        import json
        import os
        detected_path = os.path.join(os.path.dirname(__file__), "detected_blocks.json")
        with open(detected_path, "w") as f:
            if hasattr(self, 'camera_thread'):
                json.dump(self.camera_thread.latest_blocks, f)
            else:
                json.dump([], f)
                
        # 2. Recharger le script qui planifie la trajectoire physiquement (MouvementConnecte)
        import importlib
        import MouvementConnecte
        importlib.reload(MouvementConnecte)
        Motor_command_xyz = MouvementConnecte.Motor_command_xyz

        # 3. Recalculer l'animation matplotlib 3D pour l'interface
        self.traj_points = []
        if Motor_command_xyz and interpolate_linear and interpolate_joint:
            steps_per_move = 30
            current_pos = Motor_command_xyz[0][:3]
            for i in range(1, len(Motor_command_xyz)):
                target_pt = Motor_command_xyz[i]
                pos_xyz = target_pt[:3]
                mode = target_pt[3] if len(target_pt) > 3 else 'L'
                
                if mode == 'G':
                    segment = [current_pos] * 20
                elif mode == 'J':
                    segment = interpolate_joint(current_pos, pos_xyz, steps_per_move)
                else:
                    segment = interpolate_linear(current_pos, pos_xyz, steps_per_move)
                
                self.traj_points.extend(segment)
                current_pos = pos_xyz

        # Dessiner le robot à la position initiale
        if self.traj_points:
            self.draw_robot(self.traj_points[0][:3])
        else:
            # Fallback
            self.draw_robot([0, 0, -25])

    def get_dark_theme(self):
        return """
            QMainWindow {
                background-color: #11111B;
            }
        """

    def draw_robot(self, pos):
        self.ax.cla()
        self.ax.set_axis_off()
        lim = 25.0  # Limite réduite pour "zoom in" horizontal
        self.ax.set_xlim(-lim, lim)
        self.ax.set_ylim(-lim, lim)
        self.ax.set_zlim(-45.0, 15.0)  # Limite réduite pour "zoom in" vertical

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

    def update_camera_image(self, qt_img):
        self.cam_label.setPixmap(QPixmap.fromImage(qt_img))
        
    def update_camera_status(self, text):
        self.cam_label.setText(text)

    def update_animation(self):
        if not self.traj_points or self.current_frame >= len(self.traj_points):
            self.anim_timer.stop()
            self.is_animating = False
            return
            
        pos = self.traj_points[self.current_frame][:3]
        self.draw_robot(pos)
        self.current_frame += 1

    def start_robot(self):
        self.btn_start.setDisabled(True)
        # on ne désactive plus le bouton Quitter pour permettre d'arrêter à tout moment
        self.status_label.setText("STATUT : RECALCUL...")
        self.status_label.setStyleSheet("color: #F9E2AF; font-size: 18px; font-weight: bold; border: none;")

        # Recalcule la trajectoire avec les blocs détectés présentement par la caméra
        QApplication.processEvents() # MàJ de l'UI
        self.rebuild_trajectory()

        self.status_label.setText("STATUT : EN MOUVEMENT")
        
        # Redémarrer l'animation de 0
        self.current_frame = 0
        if self.traj_points:
            self.is_animating = True
            # Vitesse réaliste : 50ms par point interpolé (même timing que le robot)
            self.anim_timer.start(50) 

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
        self.anim_timer.stop()

    def closeEvent(self, event):
        # Kill OpenCV Camera thread
        if hasattr(self, 'camera_thread') and self.camera_thread.isRunning():
            self.camera_thread.stop()
            
        # Kill the background process if it is running when the window is closed
        if hasattr(self, 'worker') and self.worker and self.worker.isRunning():
            self.worker.stop()
            self.worker.wait(1000) # give it a second to terminate
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = VibeCodeUI()
    window.show()
    sys.exit(app.exec())
