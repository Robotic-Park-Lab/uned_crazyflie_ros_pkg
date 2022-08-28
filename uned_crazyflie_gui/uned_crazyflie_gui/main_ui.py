# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'main.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1192, 757)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setMinimumSize(QtCore.QSize(200, 300))
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setContentsMargins(10, 10, 10, 10)
        self.verticalLayout.setObjectName("verticalLayout")
        self.scrollArea = QtWidgets.QScrollArea(self.centralwidget)
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setObjectName("scrollArea")
        self.scrollAreaWidgetContents = QtWidgets.QWidget()
        self.scrollAreaWidgetContents.setGeometry(QtCore.QRect(0, 0, 1170, 674))
        self.scrollAreaWidgetContents.setObjectName("scrollAreaWidgetContents")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.scrollAreaWidgetContents)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.Logo_UNED = QtWidgets.QLabel(self.scrollAreaWidgetContents)
        self.Logo_UNED.setMaximumSize(QtCore.QSize(100, 100))
        self.Logo_UNED.setStyleSheet("border-image: url(:/logo/figs/UNED.jpg);")
        self.Logo_UNED.setText("")
        self.Logo_UNED.setObjectName("Logo_UNED")
        self.gridLayout_2.addWidget(self.Logo_UNED, 0, 2, 1, 1)
        self.MainWidget = QtWidgets.QWidget(self.scrollAreaWidgetContents)
        self.MainWidget.setMinimumSize(QtCore.QSize(1000, 500))
        self.MainWidget.setObjectName("MainWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.MainWidget)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.ControlWidget = QtWidgets.QTabWidget(self.MainWidget)
        self.ControlWidget.setMaximumSize(QtCore.QSize(350, 16777215))
        self.ControlWidget.setObjectName("ControlWidget")
        self.CL_tab = QtWidgets.QWidget()
        self.CL_tab.setObjectName("CL_tab")
        self.layoutWidget = QtWidgets.QWidget(self.CL_tab)
        self.layoutWidget.setGeometry(QtCore.QRect(0, 0, 341, 481))
        self.layoutWidget.setObjectName("layoutWidget")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.layoutWidget)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.GoalPose_label = QtWidgets.QLabel(self.layoutWidget)
        self.GoalPose_label.setObjectName("GoalPose_label")
        self.verticalLayout_3.addWidget(self.GoalPose_label)
        self.Position_widget = QtWidgets.QWidget(self.layoutWidget)
        self.Position_widget.setObjectName("Position_widget")
        self.gridLayout_5 = QtWidgets.QGridLayout(self.Position_widget)
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.Position_label_2 = QtWidgets.QLabel(self.Position_widget)
        self.Position_label_2.setObjectName("Position_label_2")
        self.gridLayout_5.addWidget(self.Position_label_2, 0, 2, 1, 2)
        self.Xp_label = QtWidgets.QLabel(self.Position_widget)
        self.Xp_label.setObjectName("Xp_label")
        self.gridLayout_5.addWidget(self.Xp_label, 1, 0, 1, 1)
        self.Xp_edit = QtWidgets.QLineEdit(self.Position_widget)
        self.Xp_edit.setObjectName("Xp_edit")
        self.gridLayout_5.addWidget(self.Xp_edit, 1, 1, 1, 1)
        self.Yp_label = QtWidgets.QLabel(self.Position_widget)
        self.Yp_label.setObjectName("Yp_label")
        self.gridLayout_5.addWidget(self.Yp_label, 1, 2, 1, 1)
        self.Yp_edit = QtWidgets.QLineEdit(self.Position_widget)
        self.Yp_edit.setObjectName("Yp_edit")
        self.gridLayout_5.addWidget(self.Yp_edit, 1, 3, 1, 1)
        self.Zp_label = QtWidgets.QLabel(self.Position_widget)
        self.Zp_label.setObjectName("Zp_label")
        self.gridLayout_5.addWidget(self.Zp_label, 1, 4, 1, 1)
        self.Zp_edit = QtWidgets.QLineEdit(self.Position_widget)
        self.Zp_edit.setObjectName("Zp_edit")
        self.gridLayout_5.addWidget(self.Zp_edit, 1, 5, 1, 1)
        self.verticalLayout_3.addWidget(self.Position_widget)
        self.Orientation_widget = QtWidgets.QWidget(self.layoutWidget)
        self.Orientation_widget.setObjectName("Orientation_widget")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.Orientation_widget)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.Orientation_label = QtWidgets.QLabel(self.Orientation_widget)
        self.Orientation_label.setObjectName("Orientation_label")
        self.gridLayout_4.addWidget(self.Orientation_label, 0, 3, 1, 3)
        self.Qx_label = QtWidgets.QLabel(self.Orientation_widget)
        self.Qx_label.setObjectName("Qx_label")
        self.gridLayout_4.addWidget(self.Qx_label, 1, 0, 1, 1)
        self.Qx_edit = QtWidgets.QLineEdit(self.Orientation_widget)
        self.Qx_edit.setObjectName("Qx_edit")
        self.gridLayout_4.addWidget(self.Qx_edit, 1, 1, 1, 1)
        self.Qy_label = QtWidgets.QLabel(self.Orientation_widget)
        self.Qy_label.setObjectName("Qy_label")
        self.gridLayout_4.addWidget(self.Qy_label, 1, 2, 1, 1)
        self.Qy_edit = QtWidgets.QLineEdit(self.Orientation_widget)
        self.Qy_edit.setObjectName("Qy_edit")
        self.gridLayout_4.addWidget(self.Qy_edit, 1, 3, 1, 1)
        self.Qz_label = QtWidgets.QLabel(self.Orientation_widget)
        self.Qz_label.setObjectName("Qz_label")
        self.gridLayout_4.addWidget(self.Qz_label, 1, 4, 1, 1)
        self.Qz_edit = QtWidgets.QLineEdit(self.Orientation_widget)
        self.Qz_edit.setObjectName("Qz_edit")
        self.gridLayout_4.addWidget(self.Qz_edit, 1, 5, 1, 1)
        self.Qw_label = QtWidgets.QLabel(self.Orientation_widget)
        self.Qw_label.setObjectName("Qw_label")
        self.gridLayout_4.addWidget(self.Qw_label, 1, 6, 1, 1)
        self.Qw_edit = QtWidgets.QLineEdit(self.Orientation_widget)
        self.Qw_edit.setObjectName("Qw_edit")
        self.gridLayout_4.addWidget(self.Qw_edit, 1, 7, 1, 1)
        self.verticalLayout_3.addWidget(self.Orientation_widget)
        self.GoalPose_button = QtWidgets.QPushButton(self.layoutWidget)
        self.GoalPose_button.setObjectName("GoalPose_button")
        self.verticalLayout_3.addWidget(self.GoalPose_button)
        self.line = QtWidgets.QFrame(self.layoutWidget)
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.line.setObjectName("line")
        self.verticalLayout_3.addWidget(self.line)
        self.Controlle_label = QtWidgets.QLabel(self.layoutWidget)
        self.Controlle_label.setObjectName("Controlle_label")
        self.verticalLayout_3.addWidget(self.Controlle_label)
        self.Controllers_widget = QtWidgets.QWidget(self.layoutWidget)
        self.Controllers_widget.setObjectName("Controllers_widget")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.Controllers_widget)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.XKp_label = QtWidgets.QLabel(self.Controllers_widget)
        self.XKp_label.setObjectName("XKp_label")
        self.gridLayout_3.addWidget(self.XKp_label, 1, 1, 1, 1)
        self.XKi_label = QtWidgets.QLabel(self.Controllers_widget)
        self.XKi_label.setObjectName("XKi_label")
        self.gridLayout_3.addWidget(self.XKi_label, 1, 3, 1, 1)
        self.VzKp_label = QtWidgets.QLabel(self.Controllers_widget)
        self.VzKp_label.setObjectName("VzKp_label")
        self.gridLayout_3.addWidget(self.VzKp_label, 7, 1, 1, 1)
        self.VzKd_label = QtWidgets.QLabel(self.Controllers_widget)
        self.VzKd_label.setObjectName("VzKd_label")
        self.gridLayout_3.addWidget(self.VzKd_label, 7, 5, 1, 1)
        self.XKd_label = QtWidgets.QLabel(self.Controllers_widget)
        self.XKd_label.setObjectName("XKd_label")
        self.gridLayout_3.addWidget(self.XKd_label, 1, 5, 1, 1)
        self.Z_label = QtWidgets.QLabel(self.Controllers_widget)
        self.Z_label.setObjectName("Z_label")
        self.gridLayout_3.addWidget(self.Z_label, 3, 0, 1, 1)
        self.YKp_label = QtWidgets.QLabel(self.Controllers_widget)
        self.YKp_label.setObjectName("YKp_label")
        self.gridLayout_3.addWidget(self.YKp_label, 2, 1, 1, 1)
        self.ZKp_label = QtWidgets.QLabel(self.Controllers_widget)
        self.ZKp_label.setObjectName("ZKp_label")
        self.gridLayout_3.addWidget(self.ZKp_label, 3, 1, 1, 1)
        self.ZKp_edit = QtWidgets.QLineEdit(self.Controllers_widget)
        self.ZKp_edit.setObjectName("ZKp_edit")
        self.gridLayout_3.addWidget(self.ZKp_edit, 3, 2, 1, 1)
        self.ZKd_edit = QtWidgets.QLineEdit(self.Controllers_widget)
        self.ZKd_edit.setObjectName("ZKd_edit")
        self.gridLayout_3.addWidget(self.ZKd_edit, 3, 6, 1, 1)
        self.XKd_edit = QtWidgets.QLineEdit(self.Controllers_widget)
        self.XKd_edit.setObjectName("XKd_edit")
        self.gridLayout_3.addWidget(self.XKd_edit, 1, 6, 1, 1)
        self.VxKp_label = QtWidgets.QLabel(self.Controllers_widget)
        self.VxKp_label.setObjectName("VxKp_label")
        self.gridLayout_3.addWidget(self.VxKp_label, 5, 1, 1, 1)
        self.Vy_label = QtWidgets.QLabel(self.Controllers_widget)
        self.Vy_label.setObjectName("Vy_label")
        self.gridLayout_3.addWidget(self.Vy_label, 6, 0, 1, 1)
        self.ZKd_label = QtWidgets.QLabel(self.Controllers_widget)
        self.ZKd_label.setObjectName("ZKd_label")
        self.gridLayout_3.addWidget(self.ZKd_label, 3, 5, 1, 1)
        self.VzKi_label = QtWidgets.QLabel(self.Controllers_widget)
        self.VzKi_label.setObjectName("VzKi_label")
        self.gridLayout_3.addWidget(self.VzKi_label, 7, 3, 1, 1)
        self.VyKd_label = QtWidgets.QLabel(self.Controllers_widget)
        self.VyKd_label.setObjectName("VyKd_label")
        self.gridLayout_3.addWidget(self.VyKd_label, 6, 5, 1, 1)
        self.VzKp_edit = QtWidgets.QLineEdit(self.Controllers_widget)
        self.VzKp_edit.setObjectName("VzKp_edit")
        self.gridLayout_3.addWidget(self.VzKp_edit, 7, 2, 1, 1)
        self.XKi_edit = QtWidgets.QLineEdit(self.Controllers_widget)
        self.XKi_edit.setObjectName("XKi_edit")
        self.gridLayout_3.addWidget(self.XKi_edit, 1, 4, 1, 1)
        self.VzKd_edit = QtWidgets.QLineEdit(self.Controllers_widget)
        self.VzKd_edit.setObjectName("VzKd_edit")
        self.gridLayout_3.addWidget(self.VzKd_edit, 7, 6, 1, 1)
        self.VxKi_edit = QtWidgets.QLineEdit(self.Controllers_widget)
        self.VxKi_edit.setObjectName("VxKi_edit")
        self.gridLayout_3.addWidget(self.VxKi_edit, 5, 4, 1, 1)
        self.XKp_edit = QtWidgets.QLineEdit(self.Controllers_widget)
        self.XKp_edit.setObjectName("XKp_edit")
        self.gridLayout_3.addWidget(self.XKp_edit, 1, 2, 1, 1)
        self.YKp_edit = QtWidgets.QLineEdit(self.Controllers_widget)
        self.YKp_edit.setObjectName("YKp_edit")
        self.gridLayout_3.addWidget(self.YKp_edit, 2, 2, 1, 1)
        self.VzKi_edit = QtWidgets.QLineEdit(self.Controllers_widget)
        self.VzKi_edit.setObjectName("VzKi_edit")
        self.gridLayout_3.addWidget(self.VzKi_edit, 7, 4, 1, 1)
        self.VxKd_edit = QtWidgets.QLineEdit(self.Controllers_widget)
        self.VxKd_edit.setObjectName("VxKd_edit")
        self.gridLayout_3.addWidget(self.VxKd_edit, 5, 6, 1, 1)
        self.Y_label = QtWidgets.QLabel(self.Controllers_widget)
        self.Y_label.setObjectName("Y_label")
        self.gridLayout_3.addWidget(self.Y_label, 2, 0, 1, 1)
        self.VxKi_label = QtWidgets.QLabel(self.Controllers_widget)
        self.VxKi_label.setObjectName("VxKi_label")
        self.gridLayout_3.addWidget(self.VxKi_label, 5, 3, 1, 1)
        self.YKi_label = QtWidgets.QLabel(self.Controllers_widget)
        self.YKi_label.setObjectName("YKi_label")
        self.gridLayout_3.addWidget(self.YKi_label, 2, 3, 1, 1)
        self.YKd_edit = QtWidgets.QLineEdit(self.Controllers_widget)
        self.YKd_edit.setObjectName("YKd_edit")
        self.gridLayout_3.addWidget(self.YKd_edit, 2, 6, 1, 1)
        self.VyKd_edit = QtWidgets.QLineEdit(self.Controllers_widget)
        self.VyKd_edit.setObjectName("VyKd_edit")
        self.gridLayout_3.addWidget(self.VyKd_edit, 6, 6, 1, 1)
        self.Vx_label = QtWidgets.QLabel(self.Controllers_widget)
        self.Vx_label.setObjectName("Vx_label")
        self.gridLayout_3.addWidget(self.Vx_label, 5, 0, 1, 1)
        self.VyKi_edit = QtWidgets.QLineEdit(self.Controllers_widget)
        self.VyKi_edit.setObjectName("VyKi_edit")
        self.gridLayout_3.addWidget(self.VyKi_edit, 6, 4, 1, 1)
        self.X_label = QtWidgets.QLabel(self.Controllers_widget)
        self.X_label.setObjectName("X_label")
        self.gridLayout_3.addWidget(self.X_label, 1, 0, 1, 1)
        self.VyKi_label = QtWidgets.QLabel(self.Controllers_widget)
        self.VyKi_label.setObjectName("VyKi_label")
        self.gridLayout_3.addWidget(self.VyKi_label, 6, 3, 1, 1)
        self.VxKp_edit = QtWidgets.QLineEdit(self.Controllers_widget)
        self.VxKp_edit.setObjectName("VxKp_edit")
        self.gridLayout_3.addWidget(self.VxKp_edit, 5, 2, 1, 1)
        self.VxKd_label = QtWidgets.QLabel(self.Controllers_widget)
        self.VxKd_label.setObjectName("VxKd_label")
        self.gridLayout_3.addWidget(self.VxKd_label, 5, 5, 1, 1)
        self.VyKp_label = QtWidgets.QLabel(self.Controllers_widget)
        self.VyKp_label.setObjectName("VyKp_label")
        self.gridLayout_3.addWidget(self.VyKp_label, 6, 1, 1, 1)
        self.VyKp_edit = QtWidgets.QLineEdit(self.Controllers_widget)
        self.VyKp_edit.setObjectName("VyKp_edit")
        self.gridLayout_3.addWidget(self.VyKp_edit, 6, 2, 1, 1)
        self.Vz_label = QtWidgets.QLabel(self.Controllers_widget)
        self.Vz_label.setObjectName("Vz_label")
        self.gridLayout_3.addWidget(self.Vz_label, 7, 0, 1, 1)
        self.YKi_edit = QtWidgets.QLineEdit(self.Controllers_widget)
        self.YKi_edit.setObjectName("YKi_edit")
        self.gridLayout_3.addWidget(self.YKi_edit, 2, 4, 1, 1)
        self.ZKi_edit = QtWidgets.QLineEdit(self.Controllers_widget)
        self.ZKi_edit.setObjectName("ZKi_edit")
        self.gridLayout_3.addWidget(self.ZKi_edit, 3, 4, 1, 1)
        self.YKd_label = QtWidgets.QLabel(self.Controllers_widget)
        self.YKd_label.setObjectName("YKd_label")
        self.gridLayout_3.addWidget(self.YKd_label, 2, 5, 1, 1)
        self.ZKi_label = QtWidgets.QLabel(self.Controllers_widget)
        self.ZKi_label.setObjectName("ZKi_label")
        self.gridLayout_3.addWidget(self.ZKi_label, 3, 3, 1, 1)
        self.V_label = QtWidgets.QLabel(self.Controllers_widget)
        self.V_label.setObjectName("V_label")
        self.gridLayout_3.addWidget(self.V_label, 4, 0, 1, 3)
        self.Position_label = QtWidgets.QLabel(self.Controllers_widget)
        self.Position_label.setObjectName("Position_label")
        self.gridLayout_3.addWidget(self.Position_label, 0, 0, 1, 3)
        self.verticalLayout_3.addWidget(self.Controllers_widget)
        self.Controllers_button = QtWidgets.QPushButton(self.layoutWidget)
        self.Controllers_button.setObjectName("Controllers_button")
        self.verticalLayout_3.addWidget(self.Controllers_button)
        self.ControlWidget.addTab(self.CL_tab, "")
        self.OL_tab = QtWidgets.QWidget()
        self.OL_tab.setObjectName("OL_tab")
        self.verticalSlider = QtWidgets.QSlider(self.OL_tab)
        self.verticalSlider.setGeometry(QtCore.QRect(20, 90, 51, 231))
        self.verticalSlider.setOrientation(QtCore.Qt.Vertical)
        self.verticalSlider.setObjectName("verticalSlider")
        self.label_39 = QtWidgets.QLabel(self.OL_tab)
        self.label_39.setGeometry(QtCore.QRect(20, 20, 67, 17))
        self.label_39.setObjectName("label_39")
        self.horizontalSlider = QtWidgets.QSlider(self.OL_tab)
        self.horizontalSlider.setGeometry(QtCore.QRect(80, 260, 160, 16))
        self.horizontalSlider.setMaximum(50)
        self.horizontalSlider.setProperty("value", 0)
        self.horizontalSlider.setSliderPosition(0)
        self.horizontalSlider.setTracking(True)
        self.horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider.setInvertedAppearance(False)
        self.horizontalSlider.setInvertedControls(False)
        self.horizontalSlider.setObjectName("horizontalSlider")
        self.label_41 = QtWidgets.QLabel(self.OL_tab)
        self.label_41.setGeometry(QtCore.QRect(80, 230, 31, 17))
        self.label_41.setObjectName("label_41")
        self.verticalSlider_2 = QtWidgets.QSlider(self.OL_tab)
        self.verticalSlider_2.setGeometry(QtCore.QRect(270, 100, 16, 221))
        self.verticalSlider_2.setMinimum(-50)
        self.verticalSlider_2.setMaximum(50)
        self.verticalSlider_2.setOrientation(QtCore.Qt.Vertical)
        self.verticalSlider_2.setObjectName("verticalSlider_2")
        self.label_42 = QtWidgets.QLabel(self.OL_tab)
        self.label_42.setGeometry(QtCore.QRect(260, 30, 67, 17))
        self.label_42.setObjectName("label_42")
        self.horizontalSlider_2 = QtWidgets.QSlider(self.OL_tab)
        self.horizontalSlider_2.setGeometry(QtCore.QRect(80, 190, 160, 16))
        self.horizontalSlider_2.setMinimum(-50)
        self.horizontalSlider_2.setMaximum(50)
        self.horizontalSlider_2.setTracking(True)
        self.horizontalSlider_2.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_2.setObjectName("horizontalSlider_2")
        self.label_43 = QtWidgets.QLabel(self.OL_tab)
        self.label_43.setGeometry(QtCore.QRect(80, 160, 41, 17))
        self.label_43.setObjectName("label_43")
        self.lineEdit_27 = QtWidgets.QLineEdit(self.OL_tab)
        self.lineEdit_27.setGeometry(QtCore.QRect(120, 160, 113, 25))
        self.lineEdit_27.setObjectName("lineEdit_27")
        self.lineEdit_28 = QtWidgets.QLineEdit(self.OL_tab)
        self.lineEdit_28.setGeometry(QtCore.QRect(120, 220, 113, 25))
        self.lineEdit_28.setObjectName("lineEdit_28")
        self.lineEdit_29 = QtWidgets.QLineEdit(self.OL_tab)
        self.lineEdit_29.setGeometry(QtCore.QRect(10, 50, 81, 25))
        self.lineEdit_29.setObjectName("lineEdit_29")
        self.lineEdit_30 = QtWidgets.QLineEdit(self.OL_tab)
        self.lineEdit_30.setGeometry(QtCore.QRect(230, 60, 91, 25))
        self.lineEdit_30.setObjectName("lineEdit_30")
        self.ControlWidget.addTab(self.OL_tab, "")
        self.horizontalLayout.addWidget(self.ControlWidget)
        self.GraphsWidget = QtWidgets.QTabWidget(self.MainWidget)
        self.GraphsWidget.setObjectName("GraphsWidget")
        self.sensors_tab = QtWidgets.QWidget()
        self.sensors_tab.setObjectName("sensors_tab")
        self.gridLayout_6 = QtWidgets.QGridLayout(self.sensors_tab)
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.widget = QtWidgets.QWidget(self.sensors_tab)
        self.widget.setMaximumSize(QtCore.QSize(300, 300))
        self.widget.setObjectName("widget")
        self.gridLayout = QtWidgets.QGridLayout(self.widget)
        self.gridLayout.setObjectName("gridLayout")
        self.comboBox_2 = QtWidgets.QComboBox(self.widget)
        self.comboBox_2.setObjectName("comboBox_2")
        self.gridLayout.addWidget(self.comboBox_2, 0, 1, 1, 2)
        self.lineEdit_9 = QtWidgets.QLineEdit(self.widget)
        self.lineEdit_9.setObjectName("lineEdit_9")
        self.gridLayout.addWidget(self.lineEdit_9, 7, 1, 1, 2)
        self.label_6 = QtWidgets.QLabel(self.widget)
        self.label_6.setObjectName("label_6")
        self.gridLayout.addWidget(self.label_6, 4, 0, 1, 1)
        spacerItem = QtWidgets.QSpacerItem(20, 28, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.gridLayout.addItem(spacerItem, 3, 2, 1, 1)
        self.label_8 = QtWidgets.QLabel(self.widget)
        self.label_8.setObjectName("label_8")
        self.gridLayout.addWidget(self.label_8, 6, 0, 1, 1)
        self.label_9 = QtWidgets.QLabel(self.widget)
        self.label_9.setObjectName("label_9")
        self.gridLayout.addWidget(self.label_9, 7, 0, 1, 1)
        self.lineEdit_7 = QtWidgets.QLineEdit(self.widget)
        self.lineEdit_7.setObjectName("lineEdit_7")
        self.gridLayout.addWidget(self.lineEdit_7, 4, 1, 1, 2)
        self.label_3 = QtWidgets.QLabel(self.widget)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 0, 0, 1, 1)
        self.label_5 = QtWidgets.QLabel(self.widget)
        self.label_5.setObjectName("label_5")
        self.gridLayout.addWidget(self.label_5, 2, 0, 2, 1)
        self.lineEdit_8 = QtWidgets.QLineEdit(self.widget)
        self.lineEdit_8.setObjectName("lineEdit_8")
        self.gridLayout.addWidget(self.lineEdit_8, 5, 1, 2, 2)
        self.label_7 = QtWidgets.QLabel(self.widget)
        self.label_7.setObjectName("label_7")
        self.gridLayout.addWidget(self.label_7, 5, 0, 1, 1)
        self.label_4 = QtWidgets.QLabel(self.widget)
        self.label_4.setObjectName("label_4")
        self.gridLayout.addWidget(self.label_4, 1, 0, 1, 1)
        self.comboBox = QtWidgets.QComboBox(self.widget)
        self.comboBox.setObjectName("comboBox")
        self.gridLayout.addWidget(self.comboBox, 1, 1, 1, 2)
        self.gridLayout_6.addWidget(self.widget, 0, 0, 1, 1)
        self.label = QtWidgets.QLabel(self.sensors_tab)
        self.label.setMinimumSize(QtCore.QSize(300, 225))
        self.label.setMaximumSize(QtCore.QSize(300, 250))
        self.label.setStyleSheet("border-image: url(:/logo/figs/Crazyflie.png);")
        self.label.setText("")
        self.label.setObjectName("label")
        self.gridLayout_6.addWidget(self.label, 0, 1, 1, 1)
        self.tabWidget = QtWidgets.QTabWidget(self.sensors_tab)
        self.tabWidget.setMinimumSize(QtCore.QSize(0, 250))
        self.tabWidget.setObjectName("tabWidget")
        self.tab = QtWidgets.QWidget()
        self.tab.setObjectName("tab")
        self.label_2 = QtWidgets.QLabel(self.tab)
        self.label_2.setGeometry(QtCore.QRect(150, 30, 251, 171))
        self.label_2.setStyleSheet("border-image: url(:/logo/figs/Multiranger.png);")
        self.label_2.setText("")
        self.label_2.setObjectName("label_2")
        self.checkBox = QtWidgets.QCheckBox(self.tab)
        self.checkBox.setGeometry(QtCore.QRect(10, 10, 92, 23))
        self.checkBox.setObjectName("checkBox")
        self.lineEdit = QtWidgets.QLineEdit(self.tab)
        self.lineEdit.setGeometry(QtCore.QRect(220, 10, 113, 25))
        self.lineEdit.setObjectName("lineEdit")
        self.lineEdit_2 = QtWidgets.QLineEdit(self.tab)
        self.lineEdit_2.setGeometry(QtCore.QRect(360, 100, 113, 25))
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.lineEdit_3 = QtWidgets.QLineEdit(self.tab)
        self.lineEdit_3.setGeometry(QtCore.QRect(220, 190, 113, 25))
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.lineEdit_4 = QtWidgets.QLineEdit(self.tab)
        self.lineEdit_4.setGeometry(QtCore.QRect(70, 100, 113, 25))
        self.lineEdit_4.setObjectName("lineEdit_4")
        self.tabWidget.addTab(self.tab, "")
        self.tab_4 = QtWidgets.QWidget()
        self.tab_4.setObjectName("tab_4")
        self.tabWidget.addTab(self.tab_4, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.tabWidget.addTab(self.tab_2, "")
        self.tab_3 = QtWidgets.QWidget()
        self.tab_3.setObjectName("tab_3")
        self.tabWidget.addTab(self.tab_3, "")
        self.tab_5 = QtWidgets.QWidget()
        self.tab_5.setObjectName("tab_5")
        self.tabWidget.addTab(self.tab_5, "")
        self.tab_6 = QtWidgets.QWidget()
        self.tab_6.setObjectName("tab_6")
        self.tabWidget.addTab(self.tab_6, "")
        self.gridLayout_6.addWidget(self.tabWidget, 1, 0, 1, 2)
        self.widget.raise_()
        self.label.raise_()
        self.tabWidget.raise_()
        self.GraphsWidget.addTab(self.sensors_tab, "")
        self.horizontalLayout.addWidget(self.GraphsWidget)
        self.gridLayout_2.addWidget(self.MainWidget, 3, 1, 2, 2)
        self.header_separator = QtWidgets.QFrame(self.scrollAreaWidgetContents)
        self.header_separator.setFrameShape(QtWidgets.QFrame.HLine)
        self.header_separator.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.header_separator.setObjectName("header_separator")
        self.gridLayout_2.addWidget(self.header_separator, 1, 0, 1, 3)
        self.InfoWidget = QtWidgets.QWidget(self.scrollAreaWidgetContents)
        self.InfoWidget.setMaximumSize(QtCore.QSize(150, 16777215))
        self.InfoWidget.setObjectName("InfoWidget")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.InfoWidget)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.Id_label = QtWidgets.QLabel(self.InfoWidget)
        self.Id_label.setObjectName("Id_label")
        self.verticalLayout_4.addWidget(self.Id_label)
        self.Id_edit = QtWidgets.QLineEdit(self.InfoWidget)
        self.Id_edit.setObjectName("Id_edit")
        self.verticalLayout_4.addWidget(self.Id_edit)
        self.Id_Button = QtWidgets.QPushButton(self.InfoWidget)
        self.Id_Button.setObjectName("Id_Button")
        self.verticalLayout_4.addWidget(self.Id_Button)
        self.Order_label = QtWidgets.QLabel(self.InfoWidget)
        self.Order_label.setObjectName("Order_label")
        self.verticalLayout_4.addWidget(self.Order_label)
        self.Order_box = QtWidgets.QComboBox(self.InfoWidget)
        self.Order_box.setObjectName("Order_box")
        self.Order_box.addItem("")
        self.Order_box.setItemText(0, "")
        self.Order_box.addItem("")
        self.Order_box.addItem("")
        self.verticalLayout_4.addWidget(self.Order_box)
        self.Info_browser = QtWidgets.QTextBrowser(self.InfoWidget)
        self.Info_browser.setObjectName("Info_browser")
        self.verticalLayout_4.addWidget(self.Info_browser)
        self.gridLayout_2.addWidget(self.InfoWidget, 3, 0, 2, 1)
        self.Logo = QtWidgets.QLabel(self.scrollAreaWidgetContents)
        self.Logo.setMinimumSize(QtCore.QSize(100, 100))
        self.Logo.setMaximumSize(QtCore.QSize(100, 100))
        self.Logo.setStyleSheet("border-image: url(:/logo/figs/LogoRoboticPark.png);")
        self.Logo.setText("")
        self.Logo.setObjectName("Logo")
        self.gridLayout_2.addWidget(self.Logo, 0, 0, 1, 1)
        self.Title = QtWidgets.QLabel(self.scrollAreaWidgetContents)
        self.Title.setMaximumSize(QtCore.QSize(16777215, 100))
        self.Title.setObjectName("Title")
        self.gridLayout_2.addWidget(self.Title, 0, 1, 1, 1)
        self.scrollArea.setWidget(self.scrollAreaWidgetContents)
        self.verticalLayout.addWidget(self.scrollArea)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1192, 22))
        self.menubar.setObjectName("menubar")
        self.menuFile = QtWidgets.QMenu(self.menubar)
        self.menuFile.setObjectName("menuFile")
        self.menuEdit = QtWidgets.QMenu(self.menubar)
        self.menuEdit.setObjectName("menuEdit")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.toolBar = QtWidgets.QToolBar(MainWindow)
        self.toolBar.setObjectName("toolBar")
        MainWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.toolBar)
        self.Close_action = QtWidgets.QAction(MainWindow)
        self.Close_action.setObjectName("Close_action")
        self.menuFile.addAction(self.Close_action)
        self.menubar.addAction(self.menuFile.menuAction())
        self.menubar.addAction(self.menuEdit.menuAction())

        self.retranslateUi(MainWindow)
        self.ControlWidget.setCurrentIndex(0)
        self.GraphsWidget.setCurrentIndex(0)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.GoalPose_label.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">Goal Pose</span></p></body></html>"))
        self.Position_label_2.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-style:italic;\">Position</span></p></body></html>"))
        self.Xp_label.setText(_translate("MainWindow", "X:"))
        self.Yp_label.setText(_translate("MainWindow", "Y:"))
        self.Zp_label.setText(_translate("MainWindow", "Z:"))
        self.Orientation_label.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-style:italic;\">Orientation</span></p></body></html>"))
        self.Qx_label.setText(_translate("MainWindow", "Qx:"))
        self.Qy_label.setText(_translate("MainWindow", "Qy:"))
        self.Qz_label.setText(_translate("MainWindow", "Qz:"))
        self.Qw_label.setText(_translate("MainWindow", "Qw:"))
        self.GoalPose_button.setText(_translate("MainWindow", "Send"))
        self.Controlle_label.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-weight:600;\">Controllers</span></p></body></html>"))
        self.XKp_label.setText(_translate("MainWindow", "Kp:"))
        self.XKi_label.setText(_translate("MainWindow", "Ki:"))
        self.VzKp_label.setText(_translate("MainWindow", "Kp:"))
        self.VzKd_label.setText(_translate("MainWindow", "Kd:"))
        self.XKd_label.setText(_translate("MainWindow", "Kd:"))
        self.Z_label.setText(_translate("MainWindow", "Z:"))
        self.YKp_label.setText(_translate("MainWindow", "Kp:"))
        self.ZKp_label.setText(_translate("MainWindow", "Kp:"))
        self.VxKp_label.setText(_translate("MainWindow", "Kp:"))
        self.Vy_label.setText(_translate("MainWindow", "Vy:"))
        self.ZKd_label.setText(_translate("MainWindow", "Kd:"))
        self.VzKi_label.setText(_translate("MainWindow", "Ki:"))
        self.VyKd_label.setText(_translate("MainWindow", "Kd:"))
        self.Y_label.setText(_translate("MainWindow", "Y:"))
        self.VxKi_label.setText(_translate("MainWindow", "Ki:"))
        self.YKi_label.setText(_translate("MainWindow", "Ki:"))
        self.Vx_label.setText(_translate("MainWindow", "Vx:"))
        self.X_label.setText(_translate("MainWindow", "X:"))
        self.VyKi_label.setText(_translate("MainWindow", "Ki:"))
        self.VxKd_label.setText(_translate("MainWindow", "Kd:"))
        self.VyKp_label.setText(_translate("MainWindow", "Kp:"))
        self.Vz_label.setText(_translate("MainWindow", "Vz:"))
        self.YKd_label.setText(_translate("MainWindow", "Kd:"))
        self.ZKi_label.setText(_translate("MainWindow", "Ki:"))
        self.V_label.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-style:italic;\">Velocity:</span></p></body></html>"))
        self.Position_label.setText(_translate("MainWindow", "<html><head/><body><p><span style=\" font-style:italic;\">Position:</span></p></body></html>"))
        self.Controllers_button.setText(_translate("MainWindow", "Update"))
        self.ControlWidget.setTabText(self.ControlWidget.indexOf(self.CL_tab), _translate("MainWindow", "Close Loop"))
        self.label_39.setText(_translate("MainWindow", "Thrust"))
        self.label_41.setText(_translate("MainWindow", "Roll"))
        self.label_42.setText(_translate("MainWindow", "Pitch"))
        self.label_43.setText(_translate("MainWindow", "Yaw"))
        self.ControlWidget.setTabText(self.ControlWidget.indexOf(self.OL_tab), _translate("MainWindow", "Open Loop"))
        self.label_6.setText(_translate("MainWindow", "<html><head/><body><p align=\"right\">X:</p></body></html>"))
        self.label_8.setText(_translate("MainWindow", "<html><head/><body><p align=\"right\">Z:</p></body></html>"))
        self.label_9.setText(_translate("MainWindow", "<html><head/><body><p align=\"right\">Yaw:</p></body></html>"))
        self.label_3.setText(_translate("MainWindow", "Kalman Filter:"))
        self.label_5.setText(_translate("MainWindow", "Pose:"))
        self.label_7.setText(_translate("MainWindow", "<html><head/><body><p align=\"right\">Y:</p></body></html>"))
        self.label_4.setText(_translate("MainWindow", "Controller Type:"))
        self.checkBox.setText(_translate("MainWindow", "Enable"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), _translate("MainWindow", "Multi-Ranger"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_4), _translate("MainWindow", "Flow v2"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("MainWindow", "LightHouse"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_3), _translate("MainWindow", "Loco Positioning"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_5), _translate("MainWindow", "Vicon"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_6), _translate("MainWindow", "LED Ring"))
        self.GraphsWidget.setTabText(self.GraphsWidget.indexOf(self.sensors_tab), _translate("MainWindow", "Sensors"))
        self.Id_label.setText(_translate("MainWindow", "Id:"))
        self.Id_Button.setText(_translate("MainWindow", "Connect"))
        self.Order_label.setText(_translate("MainWindow", "Order:"))
        self.Order_box.setItemText(1, _translate("MainWindow", "Take off"))
        self.Order_box.setItemText(2, _translate("MainWindow", "Land"))
        self.Title.setText(_translate("MainWindow", "<html><head/><body><p align=\"center\"><span style=\" font-size:28pt; font-weight:600;\">Crazyflie 2.X GUI</span></p></body></html>"))
        self.menuFile.setTitle(_translate("MainWindow", "File"))
        self.menuEdit.setTitle(_translate("MainWindow", "Edit"))
        self.toolBar.setWindowTitle(_translate("MainWindow", "toolBar"))
        self.Close_action.setText(_translate("MainWindow", "Close (Ctrl+w)"))
import logo_rc


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
