# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'main.ui'
#
# Created by: PyQt5 UI code generator 5.15.11
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(800, 600)
        MainWindow.setStyleSheet("background-color: rgb(217, 217, 217);")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.label_logo_khoa = QtWidgets.QLabel(self.centralwidget)
        self.label_logo_khoa.setGeometry(QtCore.QRect(699, 0, 101, 91))
        self.label_logo_khoa.setStyleSheet("border-image: url(:/logo/logo_khoa.png);\n"
"background-color: rgb(255, 255, 255);")
        self.label_logo_khoa.setText("")
        self.label_logo_khoa.setObjectName("label_logo_khoa")
        self.label_school = QtWidgets.QLabel(self.centralwidget)
        self.label_school.setGeometry(QtCore.QRect(92, 0, 611, 91))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setPointSize(18)
        font.setBold(True)
        self.label_school.setFont(font)
        self.label_school.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_school.setStyleSheet("background-color: rgb(85, 170, 255);")
        self.label_school.setAlignment(QtCore.Qt.AlignCenter)
        self.label_school.setObjectName("label_school")
        self.label_logo_truong = QtWidgets.QLabel(self.centralwidget)
        self.label_logo_truong.setGeometry(QtCore.QRect(0, 0, 91, 91))
        self.label_logo_truong.setStyleSheet("border-image: url(:/logo/logo_truong.png);\n"
"background-color: rgb(255, 255, 255);")
        self.label_logo_truong.setText("")
        self.label_logo_truong.setObjectName("label_logo_truong")
        self.camera_groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.camera_groupBox.setGeometry(QtCore.QRect(10, 100, 451, 431))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.camera_groupBox.setFont(font)
        self.camera_groupBox.setObjectName("camera_groupBox")
        self.video_label = QtWidgets.QLabel(self.camera_groupBox)
        self.video_label.setGeometry(QtCore.QRect(10, 40, 431, 381))
        self.video_label.setText("")
        self.video_label.setObjectName("video_label")
        self.button_groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.button_groupBox.setGeometry(QtCore.QRect(470, 100, 321, 161))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.button_groupBox.setFont(font)
        self.button_groupBox.setObjectName("button_groupBox")
        self.start_btn = QtWidgets.QPushButton(self.button_groupBox)
        self.start_btn.setGeometry(QtCore.QRect(40, 50, 81, 81))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setPointSize(14)
        font.setBold(True)
        self.start_btn.setFont(font)
        self.start_btn.setStyleSheet("background-color: rgb(0, 170, 0);")
        self.start_btn.setObjectName("start_btn")
        self.stop_btn = QtWidgets.QPushButton(self.button_groupBox)
        self.stop_btn.setGeometry(QtCore.QRect(202, 50, 81, 81))
        font = QtGui.QFont()
        font.setFamily("Times New Roman")
        font.setPointSize(14)
        font.setBold(True)
        self.stop_btn.setFont(font)
        self.stop_btn.setStyleSheet("background-color: rgb(255, 0, 0);")
        self.stop_btn.setObjectName("stop_btn")
        self.data_groupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.data_groupBox.setGeometry(QtCore.QRect(470, 260, 321, 271))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.data_groupBox.setFont(font)
        self.data_groupBox.setObjectName("data_groupBox")
        self.hang_label = QtWidgets.QLabel(self.data_groupBox)
        self.hang_label.setGeometry(QtCore.QRect(10, 75, 101, 21))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.hang_label.setFont(font)
        self.hang_label.setObjectName("hang_label")
        self.hang1_label = QtWidgets.QLabel(self.data_groupBox)
        self.hang1_label.setGeometry(QtCore.QRect(122, 76, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.hang1_label.setFont(font)
        self.hang1_label.setObjectName("hang1_label")
        self.hang2_label = QtWidgets.QLabel(self.data_groupBox)
        self.hang2_label.setGeometry(QtCore.QRect(230, 76, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.hang2_label.setFont(font)
        self.hang2_label.setObjectName("hang2_label")
        self.thung_label = QtWidgets.QLabel(self.data_groupBox)
        self.thung_label.setGeometry(QtCore.QRect(10, 105, 91, 91))
        self.thung_label.setStyleSheet("border-image: url(:/logo/thunghang.png);")
        self.thung_label.setText("")
        self.thung_label.setObjectName("thung_label")
        self.thung1_label = QtWidgets.QLabel(self.data_groupBox)
        self.thung1_label.setGeometry(QtCore.QRect(116, 106, 91, 91))
        self.thung1_label.setStyleSheet("border-image: url(:/logo/thunghang.png);")
        self.thung1_label.setText("")
        self.thung1_label.setObjectName("thung1_label")
        self.thung2_label = QtWidgets.QLabel(self.data_groupBox)
        self.thung2_label.setGeometry(QtCore.QRect(220, 108, 91, 91))
        self.thung2_label.setStyleSheet("border-image: url(:/logo/thunghang.png);")
        self.thung2_label.setText("")
        self.thung2_label.setObjectName("thung2_label")
        self.hangcon_label = QtWidgets.QLabel(self.data_groupBox)
        self.hangcon_label.setGeometry(QtCore.QRect(16, 209, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        self.hangcon_label.setFont(font)
        self.hangcon_label.setText("")
        self.hangcon_label.setAlignment(QtCore.Qt.AlignCenter)
        self.hangcon_label.setObjectName("hangcon_label")
        self.hangxanh_label = QtWidgets.QLabel(self.data_groupBox)
        self.hangxanh_label.setGeometry(QtCore.QRect(120, 210, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        self.hangxanh_label.setFont(font)
        self.hangxanh_label.setText("")
        self.hangxanh_label.setAlignment(QtCore.Qt.AlignCenter)
        self.hangxanh_label.setObjectName("hangxanh_label")
        self.hangdo_label = QtWidgets.QLabel(self.data_groupBox)
        self.hangdo_label.setGeometry(QtCore.QRect(230, 210, 81, 21))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        self.hangdo_label.setFont(font)
        self.hangdo_label.setText("")
        self.hangdo_label.setAlignment(QtCore.Qt.AlignCenter)
        self.hangdo_label.setObjectName("hangdo_label")
        self.button_groupBox.raise_()
        self.camera_groupBox.raise_()
        self.label_logo_khoa.raise_()
        self.label_school.raise_()
        self.label_logo_truong.raise_()
        self.data_groupBox.raise_()
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 33))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label_school.setText(_translate("MainWindow", "<html><head/><body><p>Trường Đại học Sư phạm Kỹ thuật </p><p>Thành phố Hồ Chí Minh</p></body></html>"))
        self.camera_groupBox.setTitle(_translate("MainWindow", "CAMERA"))
        self.button_groupBox.setTitle(_translate("MainWindow", "BUTTON"))
        self.start_btn.setText(_translate("MainWindow", "START"))
        self.stop_btn.setText(_translate("MainWindow", "STOP"))
        self.data_groupBox.setTitle(_translate("MainWindow", "DATA"))
        self.hang_label.setText(_translate("MainWindow", "Hàng còn lại"))
        self.hang1_label.setText(_translate("MainWindow", "Hàng loại 1"))
        self.hang2_label.setText(_translate("MainWindow", "Hàng loại 2"))
import image_rc


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
