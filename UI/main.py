import sys
import os
from PyQt6 import QtWidgets, uic

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        base_dir = os.path.dirname(__file__)
        ui_path = os.path.join(base_dir, "untitled.ui")
        uic.loadUi(ui_path, self)

        # Connect buttons
        self.pushButton.clicked.connect(self.start_action)

    def start_action(self):
        print("Start button clicked!")

app = QtWidgets.QApplication(sys.argv)
window = MainWindow()
window.show()
app.exec()