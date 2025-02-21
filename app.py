from PyQt5 import QtGui, QtWidgets, QtCore
import sys
import home, main
import cv2
import function

so_hang_con = 0
so_hang_xanh = 0
so_hang_do = 0

class Application(QtWidgets.QApplication):
    def __init__(self, sys_argv):
        super().__init__(sys_argv)
        self.main_window = QtWidgets.QMainWindow()
        self.ui = None
        self.camera_thread = None 
        self.kinematic_thread = None 
        self.client = None
        while self.client == None:
            self.client = function.connect_to_plc(port='COM6', baudrate=9600, timeout=0.5)
        self.show_home()

    def show_home(self):
        self.ui = home.Ui_MainWindow()
        self.ui.setupUi(self.main_window)
        self.ui.start_btn.clicked.connect(self.show_main)
        self.main_window.show()

    def show_main(self):
        global so_hang_con
        global so_hang_xanh
        global so_hang_do
        self.ui = main.Ui_MainWindow()
        self.ui.setupUi(self.main_window)
        self.main_window.show()

        self.camera_thread = CameraThread(self.ui.video_label)
        self.camera_thread.start()
        
        self.kinematic_thread = KinematicThread(self.client)
        self.kinematic_thread.start()

        self.ui.start_btn.clicked.connect(self.call_start_btn)
        self.ui.stop_btn.clicked.connect(self.call_stop_btn)

        self.ui.hangcon_label.setText(str(so_hang_con))
        self.ui.hangxanh_label.setText(str(so_hang_xanh))
        self.ui.hangdo_label.setText(str(so_hang_do))

    def call_start_btn(self):
        pass

    def call_stop_btn(self):
        pass

class CameraThread(QtCore.QThread):
    frame_update = QtCore.pyqtSignal(QtGui.QImage)

    def __init__(self, video_label):
        super().__init__()
        self.video_label = video_label

    def run(self):
        cap = cv2.VideoCapture(0)
        while cap.isOpened():
            ret, frame = cap.read()
            if ret:
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_frame.shape
                bytes_per_line = ch * w
                qt_frame = QtGui.QImage(rgb_frame.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)

                pixmap = QtGui.QPixmap.fromImage(qt_frame)
                self.video_label.setPixmap(pixmap)
                self.video_label.setScaledContents(True)

        cap.release()

class KinematicThread(QtCore.QThread):
    def __init__(self, client):
        super().__init__()
        self.client = client
    
    def run(self):
        function.kinematics_thread(self.client)

if __name__ == "__main__":
    app = Application(sys.argv)
    sys.exit(app.exec_())