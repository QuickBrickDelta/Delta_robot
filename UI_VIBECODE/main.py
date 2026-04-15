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

if sys.platform != "win32":
    os.environ["QT_QPA_PLATFORM"] = "xcb"

from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QLabel, QFrame,
                             QDoubleSpinBox, QSizePolicy, QPlainTextEdit)
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt6.QtGui import QFont, QPalette, QColor, QImage, QPixmap

import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d import Axes3D
from config_traj import bacs

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
    from MouvementRobot import interpolate_linear, interpolate_joint, get_all_thetas
    import config_traj
    ROBOT_ROTATION_OFFSET_DEG = config_traj.ROBOT_ROTATION_OFFSET_DEG
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
        COLOR_RANGES, load_homography, pix_to_world_cm, detect_blocks,
        MIN_AREA_PX, COLOR_STD_THRESH, DOMINANT_FRAC_THRESH,
        RECT_ANGLE_TOL_DEG, RECT_AREA_RATIO_MIN
    )
    HAS_VISION = True
except ImportError as e:
    print(f"Module VisionNumerique non trouvé : la vision des blocs sera désactivée. Erreur: {e}")
    HAS_VISION = False

class WorkerThread(QThread):
    """ Thread pour lancer PieToArduino en arrière-plan sans geler l'UI """
    finished_signal = pyqtSignal()
    output_signal = pyqtSignal(str)

    def __init__(self, manual_path=None):
        super().__init__()
        self.manual_path = manual_path  # None = mode auto (MouvementConnecte), sinon JSON manuel

    def run(self):
        script_path = os.path.join(project_root, "Communication", "PieToArduino.py")
        # -u force le mode "unbuffered" pour voir les logs en temps réel
        cmd = [sys.executable, "-u", script_path]
        if self.manual_path:
            cmd += ["--manual", self.manual_path]
        self.output_signal.emit(f">>> Commande: {' '.join(cmd)}")
        try:
            self.process = subprocess.Popen(cmd,
                                       stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT,
                                       text=True,
                                       bufsize=1) # Unbuffered line reading
            if self.process.stdout:
                for line in self.process.stdout:
                    self.output_signal.emit(line.strip())
            self.process.wait()
        except Exception as e:
            self.output_signal.emit(f"!!! ERREUR CRITIQUE SUBPROCESS : {e}")
            import traceback
            self.output_signal.emit(traceback.format_exc())
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
        self.pause_detection = False  # Désactive la vision pendant le mouvement du robot

        # Pour l'auto-calibration avec le bloc jaune
        self.calibr_offset_x = 0.0
        self.calibr_offset_y = 0.0
        
        # Charger z_table pour les coordonnées physiques des blocs
        self.z_table = -38.0
        try:
            import config_traj
            self.z_table = config_traj.z_table
        except:
            pass

    def run(self):
        print("[CAM] Thread démarré (OS:", os.name, ")")

        if os.name == 'nt':
            cap = cv2.VideoCapture(0)
        else:
            pipeline = (
                'libcamerasrc af-mode=manual lens-position=3.4 ! '
                'video/x-raw,format=NV12,width=4608,height=2592,framerate=2/1 ! '
                'videoscale ! video/x-raw,width=1920,height=1080 ! '
                'videoconvert ! video/x-raw,format=BGR ! '
                'appsink drop=true max-buffers=1 sync=false'
            )

            print("[CAM] Pipeline GStreamer utilisé :")
            print("      ", pipeline)
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if not cap.isOpened():
            print("[CAM] ÉCHEC ouverture caméra")
            self.status_signal.emit("Caméra introuvable")
            return

        print("[CAM] Caméra ouverte avec succès")

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
                print("[CAM] Frame non lue (ret=False)")
                continue

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            if HAS_VISION and not self.pause_detection:
                if not hasattr(self, 'H_cam'):
                    loaded = load_homography()
                    if loaded:
                        self.H_cam, _, _, self.cam_center = loaded
                        self.cam_center = self.cam_center.reshape(2)
                    else:
                        self.H_cam = None

                detections = detect_blocks(frame, COLOR_RANGES, h_data=None)
                current_blocks = []
                for det in detections:
                    box = np.array(det["box"], dtype=np.int32)
                    cx, cy = det["center"]
                    col = det["color"]

                    color_rgb = {
                        "red": (255, 0, 0),
                        "green_dark": (0, 150, 0), "green_light": (144, 238, 144),
                        "blue": (0, 0, 255), "yellow": (255, 255, 0),
                        "orange": (255, 165, 0),
                    }.get(col, (255, 255, 255))

                    cv2.drawContours(rgb, [box], 0, color_rgb, 2)
                    cv2.circle(rgb, (int(cx), int(cy)), 5, color_rgb, -1)

                    label = f"{col}"
                    if self.H_cam is not None:
                        xy_world = pix_to_world_cm((cx, cy), self.H_cam)
                        if xy_world:
                            raw_Xcm = float(xy_world[0] - self.cam_center[0])
                            raw_Ycm = float(xy_world[1] - self.cam_center[1])

                            Xcm = raw_Xcm - self.calibr_offset_x
                            Ycm = raw_Ycm - self.calibr_offset_y

                            R_MAX = 22.0
                            r_current = (Xcm**2 + Ycm**2)**0.5
                            if r_current > R_MAX:
                                Xcm = Xcm * (R_MAX / r_current)
                                Ycm = Ycm * (R_MAX / r_current)
                                label += " [LIMITE]"

                            label += f" | X={Xcm:+.1f} Y={Ycm:+.1f}"
                            angle_deg = float(det.get("angle", 0.0))
                            current_blocks.append([col, "2x4", float(round(-Xcm, 2)), float(round(Ycm, 2)), float(self.z_table), angle_deg])


                    cv2.putText(rgb, label, (int(cx) + 8, int(cy) - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_rgb, 2, cv2.LINE_AA)

                self.latest_blocks = current_blocks

            if hands:
                result = hands.process(rgb)
                if result.multi_hand_landmarks:
                    for hand_landmarks in result.multi_hand_landmarks:
                        mp_drawing.draw_landmarks(
                            rgb, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
            p = qt_image.scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio)
            self.change_pixmap_signal.emit(p)

        if hands:
            hands.close()
        cap.release()
        print("[CAM] Thread caméra terminé proprement")

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
        self.control_panel = QFrame()
        self.control_panel.setStyleSheet("""
            QFrame {
                background-color: #1E1E2E;
                border-radius: 15px;
                border: 2px solid #89B4FA;
            }
        """)
        self.control_panel.setFixedWidth(350)
        control_layout = QVBoxLayout(self.control_panel)
        control_layout.setContentsMargins(20, 30, 20, 30)

        # Titre
        self.title_label = QLabel("D E L T A\nV I B E")
        self.title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.title_label.setStyleSheet("color: #89B4FA; font-size: 36px; font-weight: bold; border: none;")
        control_layout.addWidget(self.title_label)

        control_layout.addStretch()

        # Status
        self.status_label = QLabel("STATUT : PRÊT")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet("color: #A6E3A1; font-size: 18px; font-weight: bold; border: none;")
        control_layout.addWidget(self.status_label)

        control_layout.addSpacing(20)

        # Liste des blocs à ramasser
        self.sequence_title = QLabel("ORDRE DE RAMASSAGE :")
        self.sequence_title.setStyleSheet("color: #A6ADC8; font-size: 14px; font-weight: bold; border: none;")
        control_layout.addWidget(self.sequence_title)

        self.sequence_label = QLabel("Aucun mouvement prévu")
        self.sequence_label.setWordWrap(True)
        self.sequence_label.setStyleSheet("color: #CDD6F4; font-size: 16px; font-weight: bold; border: none;")
        control_layout.addWidget(self.sequence_label)

        control_layout.addSpacing(20)

        # ====== MODE MANUEL ======
        manual_title = QLabel("POSITION MANUELLE :")
        manual_title.setStyleSheet("color: #A6ADC8; font-size: 14px; font-weight: bold; border: none;")
        control_layout.addWidget(manual_title)

        spin_style = """
            QDoubleSpinBox {
                background-color: #313244;
                color: #CDD6F4;
                border: 1px solid #585B70;
                border-radius: 6px;
                padding: 4px 30px 4px 6px;
                font-size: 13px;
                font-family: monospace;
            }
            QDoubleSpinBox:focus { border: 1px solid #89B4FA; }
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
                width: 16px;
                background-color: #45475A;
                border-radius: 3px;
            }
            QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover {
                background-color: #585B70;
            }
        """
        xyz_row = QHBoxLayout()
        for label_txt, attr, lo, hi, default in [
            ("X", "spin_x", -22.0, 22.0, 0.0),
            ("Y", "spin_y", -22.0, 22.0, 0.0),
            ("Z", "spin_z", -45.0, -18.0, -20.0),
        ]:
            col_widget = QWidget()
            col_layout = QVBoxLayout(col_widget)
            col_layout.setContentsMargins(0, 0, 0, 0)
            col_layout.setSpacing(2)
            lbl = QLabel(label_txt)
            lbl.setStyleSheet("color: #89B4FA; font-size: 13px; font-weight: bold; border: none;")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            spin = QDoubleSpinBox()
            spin.setRange(lo, hi)
            spin.setValue(default)
            spin.setSingleStep(0.5)
            spin.setDecimals(1)
            spin.setStyleSheet(spin_style)
            setattr(self, attr, spin)
            col_layout.addWidget(lbl)
            col_layout.addWidget(spin)
            xyz_row.addWidget(col_widget)
        control_layout.addLayout(xyz_row)

        self.btn_go_manual = QPushButton("→ ALLER")
        self.btn_go_manual.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_go_manual.setStyleSheet("""
            QPushButton {
                background-color: #CBA6F7;
                color: #11111B;
                font-size: 16px;
                font-weight: bold;
                border-radius: 8px;
                padding: 10px;
                border: none;
            }
            QPushButton:hover { background-color: #B4BEFE; }
            QPushButton:pressed { background-color: #89DCEB; }
            QPushButton:disabled { background-color: #585B70; color: #A6ADC8; }
        """)
        self.btn_go_manual.clicked.connect(self.go_manual)
        control_layout.addWidget(self.btn_go_manual)

        self.manual_status = QLabel("")
        self.manual_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.manual_status.setWordWrap(True)
        self.manual_status.setStyleSheet("color: #A6ADC8; font-size: 13px; border: none; padding: 2px;")
        control_layout.addWidget(self.manual_status)

        control_layout.addSpacing(10)

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

        control_layout.addSpacing(20)

        # Console de logs
        log_title = QLabel("LOGS SYSTÈME :")
        log_title.setStyleSheet("color: #A6ADC8; font-size: 14px; font-weight: bold; border: none;")
        control_layout.addWidget(log_title)

        self.log_console = QPlainTextEdit()
        self.log_console.setReadOnly(True)
        self.log_console.setStyleSheet("""
            QPlainTextEdit {
                background-color: #11111B;
                color: #CDD6F4;
                border: 1px solid #585B70;
                border-radius: 8px;
                font-family: 'Consolas', 'Monaco', monospace;
                font-size: 11px;
            }
        """)
        self.log_console.setMaximumHeight(150)
        control_layout.addWidget(self.log_console)

        control_layout.addStretch()

        main_layout.addWidget(self.control_panel)

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

        # Label position XYZ temps réel
        self.xyz_label = QLabel("X: —     Y: —     Z: —")
        self.xyz_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.xyz_label.setStyleSheet(
            "color: #CBA6F7; font-size: 16px; font-family: monospace; "
            "font-weight: bold; border: none; padding: 4px;"
        )
        plot_layout.addWidget(self.xyz_label)
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

        # Timer pour le fade de couleur RGB (Visual Candy)
        self.hue = 0.0
        self.color_timer = QTimer()
        self.color_timer.timeout.connect(self.update_colors)
        self.color_timer.start(50)

        # État du robot pour les déplacements relatifs
        self.current_robot_pos = [0.0, 0.0, -20.0]

        # Message initial
        self.log_output("--- Système Initialisé ---")
        self.log_output("Prêt à démarrer la communication...")

    def update_colors(self):
        self.hue += 0.005
        if self.hue >= 1.0:
            self.hue = 0.0
        color = QColor.fromHsvF(self.hue, 0.7, 0.9).name()
        
        self.title_label.setStyleSheet(f"color: {color}; font-size: 36px; font-weight: bold; border: none;")
        self.control_panel.setStyleSheet(f"QFrame {{ background-color: #1E1E2E; border-radius: 15px; border: 2px solid {color}; }}")

    def update_pick_sequence_ui(self, current_idx):
        if not hasattr(self, 'pick_sequence') or not self.pick_sequence:
            self.sequence_label.setText("Vidé")
            return
            
        text = ""
        for i, col in enumerate(self.pick_sequence):
            # Formater joliment (ex: [1] vert, [2] rouge, etc.)
            emoji = {"red": "🔴", "green_dark": "🟢", "green_light": "🟢", "blue": "🔵", "yellow": "🟡", "orange": "🟠"}.get(col, "⚫")
            item = f"{emoji} {col.upper()}"
            if i == current_idx:
                text += f"<span style='color: #F9E2AF; font-size: 20px;'>➜ {item}</span><br>"
            elif i < current_idx:
                text += f"<span style='color: #585B70; text-decoration: line-through;'>{item}</span><br>"
            else:
                text += f"<span style='color: #CDD6F4;'>{item}</span><br>"
                
        self.sequence_label.setText(text)

    def rebuild_trajectory(self):
        """ Recharge la trajectoire et l'animation basées sur les derniers blocs détectés """
        global Motor_command_xyz
        
        # 1. Sauvegarder les blocs vus par la caméra
        import json
        import os
        detected_path = os.path.join(os.path.dirname(__file__), "detected_blocks.json")
        with open(detected_path, "w") as f:
            raw_blocks = self.camera_thread.latest_blocks if hasattr(self, 'camera_thread') else []
            # Appliquer l'offset X de calibration (défini dans config_traj)
            x_offset = getattr(config_traj, 'DETECTION_X_OFFSET_CM', 0.0)
            x_offset_multiplicator = getattr(config_traj, 'OFFSET_MULTIPLICATOR', 0.0)
            adjusted_blocks = []
            for bloc in raw_blocks:
                # bloc = [couleur, type, x, y, z, angle]
                adjusted = list(bloc)
                adjusted[2] = round(float(adjusted[2])*x_offset_multiplicator + x_offset, 2)
                adjusted_blocks.append(adjusted)
            json.dump(adjusted_blocks, f)
                
        # 2. Recharger le script qui planifie la trajectoire physiquement (MouvementConnecte)
        import importlib
        import MouvementConnecte
        importlib.reload(MouvementConnecte)
        Motor_command_xyz = MouvementConnecte.Motor_command_xyz

        # 3. Recalculer l'animation matplotlib 3D pour l'interface
        self.traj_points = []
        self.frame_to_pick_idx = []
        
        # 4. Extraire la séquence de ramassage depuis la trajectoire sémantique
        self.pick_sequence = []
        if hasattr(MouvementConnecte, 'Trajectory'):
            for step in MouvementConnecte.Trajectory:
                if step[2] == "closeGripper":
                    self.pick_sequence.append(step[0]) # couleur du bloc
        self.update_pick_sequence_ui(-1)

        if Motor_command_xyz and interpolate_linear and interpolate_joint:
            steps_per_move = 30
            current_pos = Motor_command_xyz[0][:3]
            cmd_idx = 1
            current_pick_idx = -1
            
            # On navigue à travers la vraie trajectoire pour lier les indices d'animation au bloc courant
            for step in MouvementConnecte.Trajectory[1:]: # Ignorer le home initial
                move_type = step[2]
                if move_type in ["home", "joint", "linear", "closeGripper", "openGripper"]:
                    target_pt = Motor_command_xyz[cmd_idx]
                    cmd_idx += 1
                    pos_xyz = target_pt[:3]
                    mode = target_pt[3] if len(target_pt) > 3 else 'L'

                    if move_type == "closeGripper":
                        current_pick_idx += 1
                        
                    if mode == 'G':
                        segment = [current_pos] * 20
                    elif mode == 'J':
                        segment = interpolate_joint(current_pos, pos_xyz, steps_per_move)
                    else:
                        segment = interpolate_linear(current_pos, pos_xyz, steps_per_move)
                    
                    self.traj_points.extend(segment)
                    self.frame_to_pick_idx.extend([current_pick_idx] * len(segment))
                    current_pos = pos_xyz

        # Dessiner le robot à la position initiale
        if self.traj_points:
            self.draw_robot(self.traj_points[0][:3])
        else:
            # Fallback
            self.draw_robot([0, 0, -20])

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

        offset_rad = np.radians(ROBOT_ROTATION_OFFSET_DEG)
        angles_phi = [0 + offset_rad, np.radians(120) + offset_rad, np.radians(240) + offset_rad]
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
            self.xyz_label.setText("X: —     Y: —     Z: —")
            return
            
        pos = self.traj_points[self.current_frame][:3]
        idx = self.frame_to_pick_idx[self.current_frame] if self.current_frame < len(self.frame_to_pick_idx) else -1
        self.update_pick_sequence_ui(idx)
        
        # Mise à jour du label XYZ
        self.xyz_label.setText(f"X: {pos[0]:+.1f} cm     Y: {pos[1]:+.1f} cm     Z: {pos[2]:+.1f} cm")
        
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

        # PAUSE la détection lourde pendant le mouvement pour libérer le CPU
        self.camera_thread.pause_detection = True

        self.status_label.setText("STATUT : PRÊT DANS 2s...")
        self.log_output("Calcul de la trajectoire terminé.")
        self.log_output("Lancement de la communication avec l'OpenRB...")
        QApplication.processEvents()

        # Délai de 2 secondes pour laisser le temps au système de finir tous les calculs
        # avant d'envoyer la première commande aux moteurs
        QTimer.singleShot(2000, self._launch_robot_movement)

    def _launch_robot_movement(self):
        """Appelé après le délai de 2s — lance le mouvement physique + animation"""
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
        # Ajouter à la console UI
        self.log_console.appendPlainText(text)
        # Scroll automatique
        self.log_console.verticalScrollBar().setValue(
            self.log_console.verticalScrollBar().maximum()
        )

    def robot_finished(self):
        self.status_label.setText("STATUT : TERMINÉ")
        self.status_label.setStyleSheet("color: #A6E3A1; font-size: 18px; font-weight: bold; border: none;")
        self.btn_start.setDisabled(False)
        self.btn_go_manual.setDisabled(False)
        self.anim_timer.stop()

        # Si c'était un cycle AUTOMATIQUE (MouvementConnecte), le robot finit à Home [0,0,-25]
        if hasattr(self, 'worker') and self.worker and self.worker.manual_path is None:
            self.current_robot_pos = [0.0, 0.0, -20.0]

        # RÉACTIVE la détection vision maintenant que le robot est arrêté
        self.camera_thread.pause_detection = False

    def go_manual(self):
        """Mode manuel : vérifie la portée et envoie le robot à la position X,Y,Z saisie."""
        import json
        x = self.spin_x.value()
        y = self.spin_y.value()
        z = self.spin_z.value()
        target_pos = [x, y, z]

        # 1. Vérifier la cinématique inverse
        thetas = get_all_thetas(target_pos)
        if thetas is None:
            self.manual_status.setStyleSheet("color: #F38BA8; font-size: 13px; border: none; padding: 2px;")
            self.manual_status.setText(f"⚠️ Hors portée ({x:+.1f}, {y:+.1f}, {z:+.1f})")
            return

        self.manual_status.setStyleSheet("color: #A6E3A1; font-size: 13px; border: none; padding: 2px;")
        self.manual_status.setText(f"✓ Accessible — envoi en cours...")

        # 2. Générer la trajectoire
        steps = 40
        
        # --- CALCUL FLUIDE (Joint-Space) ---
        # On calcule les angles de départ et de fin une seule fois
        th_start = get_all_thetas(self.current_robot_pos)
        th_end = get_all_thetas(target_pos)
        
        if th_start is None or th_end is None:
            self.manual_status.setStyleSheet("color: #F38BA8; font-size: 13px; border: none; padding: 2px;")
            self.manual_status.setText(f"⚠️ Position hors limites")
            return

        # On interpole directement les ANGLES (linspace) : 100% fluide, zéro bruit
        th_trajectoire = np.linspace(th_start, th_end, num=steps)
        manual_commands = []
        for th in th_trajectoire:
            t1, t2, t3 = [float(v) for v in th]
            manual_commands.append([t1, t2, t3, False])  # pince ouverte

        # --- CALCUL PREVIEW (XYZ Curve) ---
        # On garde interpolate_joint uniquement pour l'animation 3D (visuel courbé)
        pts_preview = interpolate_joint(self.current_robot_pos, target_pos, steps)
        
        # Mise à jour de la position courante mémorisée
        self.current_robot_pos = target_pos

        # 3. Sauvegarder en JSON pour PieToArduino
        manual_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "manual_command_angles.json")
        with open(manual_path, "w") as f:
            json.dump(manual_commands, f)

        # 4. Animer le 3D
        self.traj_points = list(pts_preview)
        self.frame_to_pick_idx = [-1] * len(self.traj_points)
        self.current_frame = 0
        self.anim_timer.start(50)

        # 5. Lancer PieToArduino (mode manuel via arg)
        self.btn_go_manual.setDisabled(True)
        self.btn_start.setDisabled(True)
        self.status_label.setText("STATUT : MODE MANUEL")
        self.status_label.setStyleSheet("color: #CBA6F7; font-size: 18px; font-weight: bold; border: none;")
        self.worker = WorkerThread(manual_path=manual_path)
        self.worker.output_signal.connect(self.log_output)
        self.worker.finished_signal.connect(self.robot_finished)
        self.worker.start()

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
