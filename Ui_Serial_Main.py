# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Serial_Qt.ui'
#
# Created by: PyQt5 UI code generator 5.5
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Serial_Main(object):
    def setupUi(self, Serial_Main):
        Serial_Main.setObjectName("Serial_Main")
        Serial_Main.resize(370, 300)
        Serial_Main.setMinimumSize(QtCore.QSize(370, 300))
        Serial_Main.setMaximumSize(QtCore.QSize(370, 300))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("touchscreen.ico"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        icon.addPixmap(QtGui.QPixmap("touchscreen.ico"), QtGui.QIcon.Normal, QtGui.QIcon.On)
        Serial_Main.setWindowIcon(icon)
        self.centralwidget = QtWidgets.QWidget(Serial_Main)
        self.centralwidget.setObjectName("centralwidget")
        self.pushButton_Choice_File = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_Choice_File.setEnabled(False)
        self.pushButton_Choice_File.setGeometry(QtCore.QRect(260, 55, 91, 23))
        self.pushButton_Choice_File.setObjectName("pushButton_Choice_File")
        self.label_File_Name = QtWidgets.QLabel(self.centralwidget)
        self.label_File_Name.setGeometry(QtCore.QRect(14, 80, 351, 16))
        self.label_File_Name.setObjectName("label_File_Name")
        self.pushButton_Send = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_Send.setEnabled(False)
        self.pushButton_Send.setGeometry(QtCore.QRect(260, 27, 91, 23))
        self.pushButton_Send.setObjectName("pushButton_Send")
        self.plainTextEdit = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.plainTextEdit.setGeometry(QtCore.QRect(9, 100, 341, 131))
        self.plainTextEdit.setMinimumSize(QtCore.QSize(10, 10))
        self.plainTextEdit.setObjectName("plainTextEdit")
        self.comboBox_Baudrate = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox_Baudrate.setGeometry(QtCore.QRect(89, 28, 74, 20))
        self.comboBox_Baudrate.setObjectName("comboBox_Baudrate")
        self.comboBox_COM = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox_COM.setGeometry(QtCore.QRect(9, 28, 74, 20))
        self.comboBox_COM.setObjectName("comboBox_COM")
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(9, 9, 65, 16))
        self.label_2.setObjectName("label_2")
        self.pushButton_open_COM = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_open_COM.setGeometry(QtCore.QRect(169, 27, 83, 23))
        self.pushButton_open_COM.setObjectName("pushButton_open_COM")
        self.progressBar = QtWidgets.QProgressBar(self.centralwidget)
        self.progressBar.setGeometry(QtCore.QRect(9, 241, 361, 21))
        self.progressBar.setMaximum(5)
        self.progressBar.setProperty("value", 0)
        self.progressBar.setObjectName("progressBar")
        self.pushButton_close_COM = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_close_COM.setEnabled(False)
        self.pushButton_close_COM.setGeometry(QtCore.QRect(169, 55, 83, 23))
        self.pushButton_close_COM.setObjectName("pushButton_close_COM")
        Serial_Main.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(Serial_Main)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 370, 18))
        self.menubar.setObjectName("menubar")
        Serial_Main.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(Serial_Main)
        self.statusbar.setObjectName("statusbar")
        Serial_Main.setStatusBar(self.statusbar)
        self.action = QtWidgets.QAction(Serial_Main)
        self.action.setObjectName("action")

        self.retranslateUi(Serial_Main)
        QtCore.QMetaObject.connectSlotsByName(Serial_Main)

    def retranslateUi(self, Serial_Main):
        _translate = QtCore.QCoreApplication.translate
        Serial_Main.setWindowTitle(_translate("Serial_Main", "Загрузчик GSM"))
        self.pushButton_Choice_File.setText(_translate("Serial_Main", "Выбрать файл"))
        self.label_File_Name.setText(_translate("Serial_Main", "Файл:"))
        self.pushButton_Send.setText(_translate("Serial_Main", "Прошить"))
        self.label_2.setText(_translate("Serial_Main", "Выбор порта"))
        self.pushButton_open_COM.setText(_translate("Serial_Main", "Открыть порт"))
        self.pushButton_close_COM.setText(_translate("Serial_Main", "Закрыть порт"))
        self.action.setText(_translate("Serial_Main", "Закрыть"))

