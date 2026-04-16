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
                             QDoubleSpinBox, QSizePolicy, QPlainTextEdit,
                             QStackedLayout, QGridLayout)
from PyQt6.QtCore import Qt, QTimer, QThread, pyqtSignal, QMimeData, QPoint
from PyQt6.QtGui import QFont, QPalette, QColor, QImage, QPixmap, QDrag

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
                        self.h_data_tuple = loaded
                    else:
                        self.H_cam = None
                        self.h_data_tuple = None

                detections = detect_blocks(frame, COLOR_RANGES, h_data=getattr(self, 'h_data_tuple', None))
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

class ColorPill(QLabel):
    def __init__(self, color_id, color_name, bg_color, parent=None):
        super().__init__("", parent) # No text
        self.color_id = color_id
        self.setFixedSize(25, 25)
        self.setStyleSheet(f"""
            QLabel {{
                background-color: {bg_color};
                border: 2px solid rgba(255, 255, 255, 0.2);
                border-radius: 30px;
            }}
        """)
        self.setToolTip(color_name)
        self.setCursor(Qt.CursorShape.OpenHandCursor)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.drag_start_position = event.position().toPoint()

    def mouseMoveEvent(self, event):
        if not (event.buttons() & Qt.MouseButton.LeftButton):
            return
        if (event.position().toPoint() - self.drag_start_position).manhattanLength() < QApplication.startDragDistance():
            return
            
        drag = QDrag(self)
        mime_data = QMimeData()
        mime_data.setText(self.color_id)
        drag.setMimeData(mime_data)
        
        pixmap = self.grab()
        drag.setPixmap(pixmap)
        drag.setHotSpot(event.position().toPoint())
        
        self.hide()
        drop_action = drag.exec(Qt.DropAction.MoveAction)
        if drop_action == Qt.DropAction.IgnoreAction:
            self.show()  # Si annulé, réaffiche

class DropBin(QFrame):
    def __init__(self, bin_id, label_text, layout_dir='V'):
        super().__init__()
        self.bin_id = bin_id
        self.on_drop_callback = None  # Sera assigné par VibeCodeUI
        self.setAcceptDrops(True)
        self.setFixedSize(60, 60)
        
        # Style circulaire épuré comme le dessin
        self.setStyleSheet(f"""
            QFrame {{
                background-color: transparent;
                border: 2px solid #585B70;
                border-radius: 15px;
            }}
        """)
        
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # Label discret au centre (Numéro)
        self.name_label = QLabel(str(bin_id) if bin_id != 0 else "B")
        self.name_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.name_label.setStyleSheet("color: #585B70; font-size: 14px; font-weight: bold; border: none;")
        main_layout.addWidget(self.name_label)

        # Layout pour les pastilles (Overlay au centre)
        self.layout = QVBoxLayout() if layout_dir == 'V' else QHBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.setSpacing(2)
        self.layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        main_layout.addLayout(self.layout)
        
        # Espaceur pour garder le label centré quand vide
        self.layout.addStretch()

    def dragEnterEvent(self, event):
        if event.mimeData().hasText():
            # Limite de 3 couleurs par bac (sauf pour le bac Non-Assigné qui a l'ID 0)
            if self.bin_id != 0:
                pills_count = sum(1 for i in range(self.layout.count()) if isinstance(self.layout.itemAt(i).widget(), ColorPill))
                if pills_count >= 3:
                    event.ignore()
                    return
                    
            event.acceptProposedAction()
            self.setStyleSheet("""
                QFrame {
                    background-color: #313244;
                    border: 2px solid #89B4FA;
                    border-radius: 10px;
                }
            """)

    def dragLeaveEvent(self, event):
        self.setStyleSheet("""
            QFrame {
                background-color: #181825;
                border: 2px solid #585B70;
                border-radius: 10px;
            }
        """)

    def dropEvent(self, event):
        self.dragLeaveEvent(event)
        source_widget = event.source()
        if source_widget:
            source_widget.setParent(None)
            self.layout.insertWidget(self.layout.count() - 1, source_widget)
            source_widget.show()
            event.acceptProposedAction()
            # Sauvegarder le mapping immédiatement après chaque drop
            if self.on_drop_callback:
                self.on_drop_callback()

class VibeCodeUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("VibeCode Control — Delta Robot")
        self.setMinimumSize(1100, 850)
        self.resize(1200, 900)
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
        self.control_panel.setFixedWidth(380)
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
            ("A°", "spin_a", -90.0, 90.0, 0.0),
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

        # -- Haut : Plot 3D et Config Bacs (Divisé en 2 horizontalement)
        top_right_layout = QHBoxLayout()
        top_right_layout.setContentsMargins(0, 0, 0, 0)
        top_right_layout.setSpacing(10)

        # 1. Le Plot 3D
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
        self.fig.subplots_adjust(left=-0.05, right=1.05, bottom=-0.05, top=1.05)
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
        
        # 2. Le Panneau de configuration des Bacs
        config_bacs_frame = QFrame()
        config_bacs_frame.setStyleSheet("""
            QFrame {
                background-color: #11111B;
                border-radius: 10px;
                border: 2px solid #A6ADC8;
            }
        """)
        # Set a fixed width for the bins panel so it doesn't take 50% randomly
        config_bacs_frame.setFixedWidth(350)
        config_layout = QVBoxLayout(config_bacs_frame)
        config_layout.setContentsMargins(20, 20, 20, 20)

        bin_title = QLabel("CONFIGURATION DES BACS:")
        bin_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        bin_title.setStyleSheet("color: #A6ADC8; font-size: 14px; font-weight: bold; border: none;")
        config_layout.addWidget(bin_title)
        
        hint_lbl = QLabel("(Glisser-Déposer)")
        hint_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        hint_lbl.setStyleSheet("color: #585B70; font-size: 11px; border: none;")
        config_layout.addWidget(hint_lbl)
        
        config_layout.addSpacing(10)

        # Création de la banque (Non-assignées)
        self.bin_bank = DropBin(0, "BANQUE (Non-Assignées)", layout_dir='H')
        self.bin_bank.setMinimumHeight(45)

        # Création des 9 Bacs de tri (Plus petits)
        self.bins = {}
        for i in range(1, 10):
            self.bins[i] = DropBin(i, f"S{i}", layout_dir='V')
            self.bins[i].setFixedSize(50, 50)
            self.bins[i].setStyleSheet(self.bins[i].styleSheet() + "font-size: 8px;")

        # Création du Layout Triangulaire selon schéma utilisateur:
        triangle_layout = QGridLayout()
        triangle_layout.setSpacing(2)

        # Ligne du haut (Horizontal horizontal)
        triangle_layout.addWidget(self.bins[4], 1, 3)
        triangle_layout.addWidget(self.bins[5], 1, 5)
        triangle_layout.addWidget(self.bins[6], 1, 7)

        # Diagonale Gauche
        triangle_layout.addWidget(self.bins[3], 2, 2) # Décalé un peu vers l'extérieur
        triangle_layout.addWidget(self.bins[2], 3, 3)
        triangle_layout.addWidget(self.bins[1], 4, 4)

        # Diagonale Droite
        triangle_layout.addWidget(self.bins[7], 2, 8)
        triangle_layout.addWidget(self.bins[8], 3, 7)
        triangle_layout.addWidget(self.bins[9], 4, 6) 

        # Force chaque "unité" de ta grille à avoir une taille minimale
        for i in range(11): # Pour 11 colonnes
            triangle_layout.setColumnMinimumWidth(i, 20)
            triangle_layout.setColumnStretch(i, 1)
        for i in range(10): # Pour 10 lignes
            triangle_layout.setRowMinimumHeight(i, 20)
            triangle_layout.setRowStretch(i, 1)
        
        config_layout.addWidget(self.bin_bank)
        config_layout.addLayout(triangle_layout)
        config_layout.addStretch()
        
        import json
        import os
        mapping = {"red": 1, "blue": 2, "green_dark": 3, "green_light": 1, "yellow": 1, "orange": 2}
        map_file = os.path.join(os.path.dirname(__file__), "color_mapping.json")
        if os.path.exists(map_file):
            try:
                with open(map_file, "r") as f:
                    mapping.update(json.load(f))
            except: pass
            
        color_props = {
            "red": ("🔴 Roug", "#BE1142"),
            "blue": ("🔵 Bleu", "#1357C5"),
            "green_dark": ("🟢 VF.", "#24701E"),
            "green_light": ("🟢 VC.", "#5BF65B"),
            "yellow": ("🟡 Jaun", "#F8E813"),
            "orange": ("🟠 Oran", "#FD6D13"),
        }
        
        self.pills = []
        for c_id, (c_name, bg) in color_props.items():
            pill = ColorPill(c_id, c_name, bg)
            self.pills.append(pill)
            target_bin_id = mapping.get(c_id, 0)
            
            inserted = False
            if target_bin_id in self.bins:
                self.bins[target_bin_id].layout.insertWidget(self.bins[target_bin_id].layout.count() - 1, pill)
                inserted = True
            
            if not inserted:
                self.bin_bank.layout.insertWidget(self.bin_bank.layout.count() - 1, pill)

        # Connecter le callback de sauvegarde à tous les bacs
        for b in self.bins.values():
            b.on_drop_callback = self.save_color_mapping
        self.bin_bank.on_drop_callback = self.save_color_mapping

        # Ajout des deux moitiés (plot et config bacs) dans le layout HAUT
        top_right_layout.addWidget(plot_frame, stretch=1)
        top_right_layout.addWidget(config_bacs_frame)
        
        # Le haut prend 2/3 de l'espace global du panneau de droite
        right_layout.addLayout(top_right_layout, stretch=2)

        # -- Bas : Caméra ou 2D Plot (empilés)
        self.bottom_stack_widget = QWidget()
        self.bottom_stack = QStackedLayout(self.bottom_stack_widget)
        self.bottom_stack.setContentsMargins(0, 0, 0, 0)
        
        # Index 0 : Caméra
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
        self.bottom_stack.addWidget(cam_frame)

        # Index 1 : Matplotlib 2D Graph
        plot_2d_frame = QFrame()
        plot_2d_frame.setStyleSheet("""
            QFrame {
                background-color: #1E1E2E;
                border-radius: 15px;
                border: 2px solid #89B4FA;
            }
        """)
        plot_2d_layout = QVBoxLayout(plot_2d_frame)
        plot_2d_layout.setContentsMargins(0, 0, 0, 0)
        self.fig_2d = plt.figure(facecolor='#1E1E2E')
        self.fig_2d.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95)
        self.canvas_2d = FigureCanvas(self.fig_2d)
        self.ax_2d = self.fig_2d.add_subplot(111)
        self.ax_2d.set_facecolor('#1E1E2E')
        self.ax_2d.tick_params(colors='#CDD6F4')
        plot_2d_layout.addWidget(self.canvas_2d)
        self.bottom_stack.addWidget(plot_2d_frame)
        
        right_layout.addWidget(self.bottom_stack_widget, stretch=1) # Le bas prend 1/3 de l'espace

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
        self.log_output("--- Système VibeCode Initialisé ---")
        self.log_output("Prêt à démarrer la communication...")

    def save_color_mapping(self):
        """Sauvegarde color→slot_id dans color_mapping.json après chaque drag-and-drop."""
        import json, os
        current_mapping = {}
        for pill in self.pills:
            parent_widget = pill.parent()
            # Remonter jusqu'au DropBin parent
            while parent_widget is not None and not hasattr(parent_widget, 'bin_id'):
                parent_widget = parent_widget.parent()
            if parent_widget and parent_widget.bin_id != 0:
                current_mapping[pill.color_id] = parent_widget.bin_id
            else:
                current_mapping[pill.color_id] = 0  # Non-assigné

        map_file = os.path.join(os.path.dirname(__file__), "color_mapping.json")
        with open(map_file, "w") as f:
            json.dump(current_mapping, f, indent=2)
        self.log_output(f"[BACS] Mapping sauvegardé : {current_mapping}")

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
        import config_traj
        
        # Obtenir les assignations actuelles des bacs via les widgets Drag&Drop
        current_mapping = {}
        for pill in self.pills:
            parent_widget = pill.parent()
            while parent_widget is not None and not hasattr(parent_widget, 'bin_id'):
                parent_widget = parent_widget.parent()
            if parent_widget and parent_widget.bin_id != 0:
                current_mapping[pill.color_id] = parent_widget.bin_id
            else:
                current_mapping[pill.color_id] = 0 # banque/non-assigné
                
        map_file = os.path.join(os.path.dirname(__file__), "color_mapping.json")
        with open(map_file, "w") as f:
            json.dump(current_mapping, f)
            
        detected_path = os.path.join(os.path.dirname(__file__), "detected_blocks.json")
        with open(detected_path, "w") as f:
            raw_blocks = self.camera_thread.latest_blocks if hasattr(self, 'camera_thread') else []
            # Appliquer l'offset X/Y de calibration (défini dans config_traj)
            x_offset = getattr(config_traj, 'DETECTION_X_OFFSET_CM', 0.0)
            y_offset = getattr(config_traj, 'DETECTION_Y_OFFSET_CM', 0.0)
            x_mult = getattr(config_traj, 'OFFSET_X_MULTIPLICATOR', 1.0)
            y_mult = getattr(config_traj, 'OFFSET_Y_MULTIPLICATOR', 1.0)
            adjusted_blocks = []
            for bloc in raw_blocks:
                # bloc = [couleur, type, x, y, z, angle]
                adjusted = list(bloc)
                adjusted[2] = round(float(adjusted[2])*x_mult + x_offset, 2)
                adjusted[3] = round(float(adjusted[3])*y_mult + y_offset, 2)
                adjusted_blocks.append(adjusted)
            json.dump(adjusted_blocks, f)
                
        # 2. Recharger le script qui planifie la trajectoire physiquement (MouvementConnecte)
        import importlib
        import MouvementConnecte
        import animation_and_plot_traj
        
        # Important: Reload config_traj FIRST so it reads color_mapping.json
        importlib.reload(config_traj)
        import plannif_trajectoire
        import shortest_path_algorithms
        importlib.reload(shortest_path_algorithms) # Recharger avant plannif
        importlib.reload(plannif_trajectoire)
        importlib.reload(animation_and_plot_traj)
        importlib.reload(MouvementConnecte)
        Motor_command_xyz = MouvementConnecte.Motor_command_xyz
        
        # --- Dessin de la Route 2D ---
        self.ax_2d.clear() # On efface l'ancien graphe si existant
        self.ax_2d.set_facecolor('#1E1E2E')
        blocs_sorted = getattr(MouvementConnecte, 'blocs_sorted', [])
        ui_drop_zones = {}
        for color, bin_id in current_mapping.items():
            if bin_id != 0:
                attr_name = f"p{bin_id}"
                if hasattr(config_traj, attr_name):
                    ui_drop_zones[color] = getattr(config_traj, attr_name)
                else:
                    print(f"Attention: {attr_name} non trouvé dans config_traj")
        if len(blocs_sorted) > 0:
            animation_and_plot_traj.draw_route_2D_v2(self.ax_2d, blocs_sorted, config_traj.home_position, drop_positions=ui_drop_zones)
            # Rafraîchissement des ticks pour qu'ils restent visibles après le clear
            self.ax_2d.tick_params(colors='#CDD6F4')
        self.canvas_2d.draw()
        
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
        
        # Basculer l'interface de la caméra vers le graphe 2D
        self.bottom_stack.setCurrentIndex(1)
        
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

        # Revenir à la vue Caméra à la place du Graphe 2D
        self.bottom_stack.setCurrentIndex(0)

        # Si c'était un cycle AUTOMATIQUE (MouvementConnecte), le robot finit à Home [0,0,-25]
        if hasattr(self, 'worker') and self.worker and self.worker.manual_path is None:
            self.current_robot_pos = [0.0, 0.0, -20.0]
            
            # RÉACTIVE la détection vision maintenant que le robot est arrêté
            self.camera_thread.pause_detection = False
            
            # On laisse le temps à la caméra d'analyser les nouveaux blocs restants 
            self.status_label.setText("NOUVELLE VÉRIFICATION...")
            QTimer.singleShot(1500, self.check_and_restart)
        else:
            self.camera_thread.pause_detection = False

    def check_and_restart(self):
        # Si on a encore des blocs valides à l'écran, on relance automatiquement
        if hasattr(self, 'camera_thread') and len(self.camera_thread.latest_blocks) > 0:
            self.log_output("Blocs restants détectés : Reprise automatique de la séquence.")
            self.start_robot()
        else:
            self.log_output("Zone de tri totalement vide. Séquence terminée.")
            self.status_label.setText("STATUT : REPOS (ZONE VIDE)")
            self.status_label.setStyleSheet("color: #A6E3A1; font-size: 18px; font-weight: bold; border: none;")

    def go_manual(self):
        """Mode manuel : vérifie la portée et envoie le robot à la position X,Y,Z saisie."""
        import json
        x = self.spin_x.value()
        y = self.spin_y.value()
        z = self.spin_z.value()
        angle = self.spin_a.value()
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
            manual_commands.append([t1, t2, t3, False, float(angle)])  # pince ouverte, angle au poignet

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
