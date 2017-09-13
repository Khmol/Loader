#coding: utf8
import sys, serial, time
from Ui_Serial_Main import *
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QBasicTimer
from Config_Serial_Qt import *
from CRC16 import *

class Serial_Qt(QtWidgets.QMainWindow):
    #инициализация окна
    # pyuic5 Serial_Qt.ui -o Ui_Serial_Main.py
    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        #инициализация интерфейса
        self.ui = Ui_Serial_Main()
        self.ui.setupUi(self)
        #настройка действий по кнопкам
        self.ui.pushButton_Send.clicked.connect(self.Send_Stop_Handler)             #начинаем передачу файла в порт
        self.ui.pushButton_Choice_File.clicked.connect(self.showDialog_Open_File)   #выбрать файл
        self.ui.pushButton_open_COM.clicked.connect(self.open_COM_Handler)          #обрабатываем нажатие кнопки отсрыть порт
        self.ui.pushButton_close_COM.clicked.connect(self.close_COM_Handler)        #обрабатываем нажатие кнопки закрыть порт
        #инициализация RS
        self.init_RS()
        #инициализация таймера
        self.timer_Reset()
        #добавляем нужные скорости в comboBox_Baudrate
        self.ui.comboBox_Baudrate.addItems(BAUDRATES)
        self.ui.comboBox_Baudrate.setCurrentIndex(5)
        #вывод строки в statusbar
        self.ui.statusbar.showMessage('Загрузчик: версия 2.00')

    #*********************************************************************
    #первоначальная инициализация переменных
    #*********************************************************************
    def timer_Reset(self):
        #инициализация таймера приемника по RS
        self.timer_RX_RS = QBasicTimer()
        self.timer_RX_RS.stop()

    #*********************************************************************
    #первоначальная инициализация переменных
    #*********************************************************************
    def init_RS(self):
        list_com_ports = '' #перечень доступных портов
        #проверка какие порты свободны
        try:
            portList = self.scan_COM_ports()
            for s in portList:
                list_com_ports += s + ' '
        except:
            pass
        finally:
            #инициализация таймера
            self.timer_Reset()
            #настройка списка для выбора порта
            #добавляем свободные порты в comboBox_COM
            self.ui.comboBox_COM.clear()
            self.ui.comboBox_COM.addItems(list_com_ports.split())
            self.ui.comboBox_COM.setCurrentIndex(0)

    #*********************************************************************
    #первоначальная инициализация переменных
    #*********************************************************************
    def Init_Vars(self):
        self.test_mode = False   #флаг отладки - что бы мы не получили если сошлось CRC меняем данные на ОК
        #определяем переменные поскольку они еще не опеделены
        self.STATUS_NEW = 0 #текущее состояние
        self.STATUS_OLD = 0 #прошлое состояние
        self.Transmit_Off = True    #флаг выключеной передачи (файл закрыт)
        self.flag_RX_OK = False #флаг успешного приема
        #словарь для ID1
        self.ID1 = {
            "BOOTLODER_REQ": 1,
            "BOOTLODER_RESP": 2,
            "SETUP_REQ": 3,
            "SETUP_RESP": 4,
            "CPU_RESET": 254 #программный сброс
        }
        #словарь для ID2
        self.ID2 = {
            "IDLE": 0,
            "START": 1,
            "ERASE": 2,
            "WRITE": 3,
            "WRITE_END": 4,
            "END": 10,
            "RESET": 11
        }
        self.Position = {
            "PRE_H": 0,
            "PRE_L": 1,
            "SIZE_H": 2,
            "SIZE_L": 3,
            "SEQ": 4,
            "ID1": 5,
            "ID2": 6,
            "DATA_START": 7,
        }
        #начальные данные для передатчика
        self.rs_ID1_TX = self.ID1["CPU_RESET"]   #ID1 передаваемой команды - вначале сбрасываем устройство
        self.rs_ID2_TX = self.ID2["IDLE"]        #ID2 передаваемой команды
        self.RS_START = bytearray([0x55, 0xAA])   #стартовая последовательность для RS
        self.rs_DATA_TX = bytearray([0x00]) #данные для передачи
        self.rs_pack_size_TX = 0    #размер передаваемого пакета в байтах
        self.rs_pack_seq_TX = 0     #номер пакета в последовательности
        self.TIME_TO_CPU_RESET = 2500                   #время на перезагрузку - 2,5 с
        self.TIME_TO_ERASE = 2000                       #время на стирание FLASH - 2 с
        #начальные данные для приемника
        self.rs_pack_size_RX = 0    #размер принятого пакета в байтах
        self.rs_pack_seq_RX = 0     #номер принятого пакета в последовательности
        self.rs_ID_RX = 0           #ID принятой команды
        #определяем переменную если она еще не определена
        try:
            if self.filename:
                return
        except:
            self.filename = '' #файл не выбран

    #*********************************************************************
    #активация кнопок после выбора порта и скорости
    #*********************************************************************
    def enable_Buttons_to_Work(self):
        self.ui.pushButton_Choice_File.setEnabled(SET)      #активируем кнопку выбор файла
        self.ui.pushButton_open_COM.setDisabled(SET)        #де-активируем кнопку открытие порта
        self.ui.pushButton_close_COM.setEnabled(SET)        #активируем кнопку закрытие порта
        self.ui.comboBox_COM.setDisabled(SET)               #де-активируем выбор порта
        self.ui.comboBox_Baudrate.setDisabled(SET)          #де-активируем выбор скорости

    #*********************************************************************
    #активация кнопок после выбора порта и скорости
    #*********************************************************************
    def Disable_All_Buttons(self):
        self.ui.pushButton_Choice_File.setDisabled(SET)        #де-активируем кнопку выбор файла
        self.ui.pushButton_open_COM.setDisabled(SET)           #де-активируем кнопку открытие порта
        self.ui.pushButton_close_COM.setDisabled(SET)          #де-активируем кнопку закрытие порта
        self.ui.comboBox_COM.setDisabled(SET)                  #де-активируем выбор порта
        self.ui.comboBox_Baudrate.setDisabled(SET)             #де-активируем выбор скорости
        self.ui.pushButton_Send.setDisabled(SET)               #де-активируем кнопку "Прошить"

    #*********************************************************************
    #активация кнопок после выбора порта и скорости
    #*********************************************************************
    def Disable_Buttons_to_Select_COM(self):
        self.ui.pushButton_Choice_File.setDisabled(SET)        #де-активируем кнопку выбор файла
        self.ui.pushButton_open_COM.setEnabled(SET)            #активируем кнопку открытие порта
        self.ui.pushButton_close_COM.setDisabled(SET)          #де-активируем кнопку закрытие порта
        self.ui.comboBox_COM.setEnabled(SET)                   #активируем выбор порта
        self.ui.comboBox_Baudrate.setEnabled(SET)              #активируем выбор скорости
        self.ui.pushButton_Send.setDisabled(SET)               #де-активируем кнопку "Прошить"

    #*********************************************************************
    #активация кнопок после выбора порта и скорости
    #*********************************************************************
    def open_COM_Handler(self):
        self.Init_Vars()        #первоначальная инициализация переменных
        self.Serial_Config()    #инициализируем порт
        self.enable_Buttons_to_Work()   #активируем кнопки
        if self.filename!='':
            self.ui.pushButton_Send.setEnabled(SET)           #активируем кнопку "Прошить"

    #*********************************************************************
    #обработчик кнопки Прошить/Прервать
    #*********************************************************************
    def Send_Stop_Handler(self):
        #если нужно запустить прошивку
        if self.STATUS_NEW == self.ID2["IDLE"]:
            #меняем состояние программы
            self.STATUS_OLD = self.STATUS_NEW
            self.STATUS_NEW = self.ID2["START"]
            self.Change_Buttons_to_Send()           # отправить стартовый пакет
        #если уже запущена прошивка программы
        elif self.STATUS_NEW > self.ID2["IDLE"]:
            #изменяем назначение кнопки Прервать
            self.Change_Button_Stop_to_Send()
            # активировать кнопки
            self.enable_Buttons_to_Work()
            self.STATUS_OLD = self.STATUS_NEW
            self.STATUS_NEW = self.ID2["IDLE"]

    #*********************************************************************
    #де-активация кнопок после закрытия порта
    #*********************************************************************
    def close_COM_Handler(self):
        self.ser.close()                                  #закрываем порт
        self.Change_Button_Stop_to_Send()                 #изменяем надпись на кнопке - Прервать на Прошить
        self.Disable_All_Buttons()                        #выключаем все кнопки и элементы выбора
        self.init_RS()                                    #проводим переопрос портов
        self.Disable_Buttons_to_Select_COM()              #изменяем кнопки для выбора порта и скорости
        #переходим в состояние IDLE если были в другом состоянии
        if self.STATUS_NEW == self.ID2["END"]:
            self.STATUS_OLD = self.STATUS_NEW
            self.STATUS_NEW = self.ID2["RESET"]
            #обнуляем полосу прогресса
            self.ui.progressBar.setValue(0)   #установка нового значения для progressBar
            self.timer_RX_RS.start(self.TIME_TO_CPU_RESET, self) #ждем пока устройство перезагрузится

    #*********************************************************************
    #определение свободных COM портов
    #*********************************************************************
    def scan_COM_ports(self):
        """scan for available ports. return a list of tuples (num, name)"""
        available = []
        for i in range(NUMBER_SCAN_PORTS):
            try:
                s = serial.Serial(i)
                available.append((s.portstr))
                s.close()   # explicit close 'cause of delayed GC in java
            except serial.SerialException:
                pass
        return available

    #*********************************************************************
    #перевод данных из byte в bytearray
    #*********************************************************************
    def Byte_to_Bytearray(self, RX_Data):
        #переводим принятые данные в bytearray для удобства дальнейшей работы с ними
        RX_Data_return = bytearray(len(RX_Data))
        for i in range(len(RX_Data)):
            RX_Data_return[i] = RX_Data[i]
        return RX_Data_return

    #*********************************************************************
    #настройка порта со значением выбранным в comboBox_COM
    #*********************************************************************
    #pyuic5 Serial_Qt.ui -o Serial_Qt_Form.py - для обновления главной формы
    def Serial_Config(self):
            # configure the serial connections (the parameters differs on the device you are connecting to)
            baudrate = int(self.ui.comboBox_Baudrate.currentText())
            try:
                #проверяем есть ли порт
                if self.ser.isOpen() == False:#
                    self.ser = serial.Serial(self.ui.comboBox_COM.currentText(),
                        baudrate=baudrate,#9600,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        timeout=0,
                        bytesize=serial.EIGHTBITS,
                        xonxoff=0)
            except:
                try:
                    self.ser = serial.Serial(self.ui.comboBox_COM.currentText(),
                            baudrate=baudrate,#9600,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            timeout=0,
                            bytesize=serial.EIGHTBITS,
                            xonxoff=0)
                    if baudrate == 115200:
                        self.TIME_TO_RX = 40#было 40
                    elif baudrate == 57600:
                        self.TIME_TO_RX = 100#было 60
                    elif baudrate == 38400:
                        self.TIME_TO_RX = 100#было 80
                    elif baudrate == 19200:
                        self.TIME_TO_RX = 100#
                    elif baudrate == 9600:
                        self.TIME_TO_RX = 150#
                    elif baudrate == 1200:
                        self.TIME_TO_RX = 1200#
                except:
                    out_str = "Порт будет закрыт, повторите прошивку заново."
                    QtWidgets.QMessageBox.warning(self, 'Ошибка работы с портом №2',out_str , QtWidgets.QMessageBox.Ok)
                    #Закрытие порта и выключение записи - переход в исходное состояние
                    self.STATUS_OLD = self.STATUS_NEW
                    self.STATUS_NEW = self.ID2["IDLE"]
                    self.close_COM_Handler()
                    return

    #*********************************************************************
    #проверка наличия данных в буфере RS
    #*********************************************************************
    def Recieve_RS_Data (self):
        RX_Data = ''  #данных нет
        while self.ser.inWaiting() > 0:
            RX_Data = self.ser.read(MAX_WAIT_BYTES)
        self.string_Data = '<< ' + str(len(RX_Data)) + ' байт '
        for i in range (len(RX_Data)):
            self.string_Data = self.string_Data + ' ' + str(hex(RX_Data[i]))
        print (self.string_Data)
        return RX_Data

    #*********************************************************************
    # анализ принятых данных из RS
    #*********************************************************************
    def analyze_pack(self):
        #проверка на стартовую посылку
        if self.rs_receive_pack[0:2] == self.RS_START:
            #производим рассчет CRC16 для self.rs_send_pack без последних двух байт
            crc = INITIAL_MODBUS
            pack_length = len(self.rs_receive_pack)
            crc_rx = self.rs_receive_pack[pack_length-2:]   #выделяем CRC
            crc_rx_int = int.from_bytes(crc_rx, byteorder='little') #преобразуем в int
            #вычисляем непосредственно CRC
            for ch in self.rs_receive_pack[:pack_length-2]:
                crc = calcByte(ch,crc)
            #сравниваем полученное CRC с расчитанным
            if crc == crc_rx_int:
                # если мы в режиме отладки - и CRC сошлось, всегда принимаем 'OK'
                if self.test_mode == True:
                    self.rs_receive_pack = self.Byte_to_Bytearray(self.rs_receive_pack)
                    self.rs_receive_pack[self.Position["DATA_START"]] = ord('O')
                    self.rs_receive_pack[(self.Position["DATA_START"]+1)] = ord('K')
                #CRC сошлось, проводим проверку тела данных
                #проверяем правильная ли длина

                rx_length = self.rs_receive_pack[self.Position["SIZE_H"]:(self.Position["SIZE_L"]+1)]
                print(int.from_bytes(rx_length, byteorder='little'))
                if pack_length != int.from_bytes(rx_length, byteorder='little'):
                    return  #выходим если принятая длина не совпадает с реальной
                if self.rs_receive_pack[self.Position["DATA_START"]:(self.Position["DATA_START"]+2)] == OK_ANSWER[0:2]:#получено OK ?
                    if self.STATUS_NEW == self.ID2["START"]:
                        if self.rs_receive_pack[self.Position["ID2"]] == self.ID2["START"]:#проверка ответного ID2 - было [4]
                            self.STATUS_OLD = self.STATUS_NEW
                            self.STATUS_NEW = self.ID2["ERASE"]
                            self.flag_RX_OK = True
                    elif self.STATUS_NEW == self.ID2["ERASE"]:
                        if self.rs_receive_pack[self.Position["ID2"]] == self.ID2["ERASE"]: #проверка ответного ID2 - было [4]
                            self.STATUS_OLD = self.STATUS_NEW
                            self.STATUS_NEW = self.ID2["WRITE"]
                            #устанавливаем ID2 состояние WRITE
                            self.rs_ID2_TX = self.ID2["WRITE"]
                            self.flag_RX_OK = True
                    elif self.STATUS_NEW == self.ID2["WRITE"]:
                        if self.rs_receive_pack[self.Position["ID2"]] == self.ID2["WRITE"]: #проверка ответного ID2 - было [4]
                            #получен ответ
                            self.pos_last = self.pos_new
                            self.flag_RX_OK = True
                    elif self.STATUS_NEW == self.ID2["WRITE_END"]:
                        if self.rs_receive_pack[self.Position["ID2"]] == self.ID2["WRITE_END"]: #проверка ответного ID2 - было [4]
                            self.STATUS_OLD = self.STATUS_NEW
                            self.STATUS_NEW = self.ID2["END"]
                            #устанавливаем ID2 состояние WRITE
                            self.rs_ID2_TX = self.ID2["END"]
                            self.flag_RX_OK = True
                    elif self.STATUS_NEW == self.ID2["END"]:
                        #проверка на Err
                        if self.rs_receive_pack[self.Position["ID2"]] == self.ID2["END"]:    #проверка ответного ID2 - было [4]
                            #изменяем назначение кнопки Прервать
                            self.Change_Button_Stop_to_Send()
                            # активировать кнопки
                            #self.enable_Buttons()
                            self.STATUS_OLD = self.STATUS_NEW
                            self.STATUS_NEW = self.ID2["IDLE"]
                            #устанавливаем ID2 состояние IDLE
                            self.rs_ID2_TX = self.ID2["IDLE"]
                            self.flag_RX_OK = True
                    return
                else:
                    if self.STATUS_NEW == self.ID2["END"]:
                        self.flag_RX_OK = False
        else:
            #если хоть что то было не так, не будет установлен флаг self.flag_RX_OK
            # и нужно повторить передачу последнего пакета
            if self.STATUS_NEW == self.ID2["WRITE"]:
                #данные были получены с ошибкой, отправляем предыдущий пакет повторно
                self.Send_Previous_Pack()
                self.flag_RX_OK = False
            elif self.STATUS_NEW == self.ID2["START"]:
                #повторяем попытку отправить  запрос на начало прошивки
                self.Send_Start_Pack()
                self.flag_RX_OK = False
            elif self.STATUS_NEW == self.ID2["ERASE"]:
                #повторяем попытку отправить запрос на стирание
                self.Send_Erase_Pack()
                self.flag_RX_OK = False
            elif self.STATUS_NEW == self.ID2["END"]:
                #повторяем попытку отправить запрос на стирание
                self.Send_End_Pack()
                self.flag_RX_OK = False
        return

    #*********************************************************************
    #передача пакета в RS
    #*********************************************************************
    def Transmit_RS_Data (self):
        #установка начальног значения CRC16
        crc = INITIAL_MODBUS
        #проверка от
        self.ser.isOpen()
        #полезные данные для передачи
        useful_data = self.rs_pack_seq_TX.to_bytes(1,'little') + \
                      self.rs_ID1_TX.to_bytes(1,'little') + \
                      self.rs_ID2_TX.to_bytes(1,'little') + \
                      self.rs_DATA_TX
        #определяем длину полезных данных
        self.rs_pack_size_TX = len(useful_data) + 6 # 2 - стартовых, 2 - длина, 2 - CRC = 6
        self.rs_pack_seq_TX += 1   #увеличиваем счетчик последовательности
        #обнуляем счетчик последовательности
        if self.rs_pack_seq_TX == 256:
            self.rs_pack_seq_TX = 0
        self.rs_send_pack = self.RS_START + \
                            self.rs_pack_size_TX.to_bytes(2,'little') + \
                            useful_data
        #производим рассчет CRC16 для self.rs_send_pack
        for ch in self.rs_send_pack:
            crc = calcByte(ch,crc)
        self.rs_send_pack = self.rs_send_pack + crc.to_bytes(2, byteorder='little')
        #выводим в терминал для отладки
        self.string_Data = '>> ' + str(len(self.rs_send_pack)) + ' байт '
        for i in range (len(self.rs_send_pack)):
            self.string_Data = self.string_Data + ' ' + str(hex(self.rs_send_pack[i]))
        print (self.string_Data)
        #отправляем в порт
        self.ser.write(self.rs_send_pack)
        return
        #ps_pack_int = int.from_bytes(rs_pack_bytes, byteorder='big')
        #self.ser.write(self.rs_tx_buffer.encode('utf-8'))
        #rs_pack_size_TX = len(rs_pack_seq_TX.to_bytes(1,'big') + rs_ID.to_bytes(1,'big') + rs_DATA)

    #*********************************************************************
    #открытие файла
    #*********************************************************************
    def showDialog_Open_File(self):
        self.filename = QtWidgets.QFileDialog.getOpenFileName(self, 'Открыть файл прошивки', '/dir')
        if self.filename[0]!='':
            self.ui.pushButton_Send.setEnabled(SET)   #активируем кнопку "Прошить"
            self.file_hex = open(self.filename[0],'rb')#открываем файл если он не открыт
            self.ui.label_File_Name.setText("Файл: " + self.filename[0])  #выводим название файла
            self.file_hex.seek(0, 2)    #переходим на конец файла
            self.length_file = self.file_hex.tell()  #сохраняем текущую позицию в файле
            self.ui.progressBar.setMaximum(self.length_file)    #установка максимального значения для progressBar
            self.ui.progressBar.setValue(0)   #установка начального значения для progressBar
        return

    #*********************************************************************
    #отправка файла в порт
    #*********************************************************************
    def send_file(self):
        if self.STATUS_NEW == self.ID2["WRITE"]:
            self.Transmit_Off = self.file_hex.closed
            #если файл закрыт - открываем его
            if self.Transmit_Off == True:
                self.file_hex = open(self.filename[0],'rb')#открываем файл если он не открыт
                self.Transmit_Off = self.file_hex.closed
            self.pos_new = 0
            self.pos_last = 0
            self.timer_RX_RS.start(self.TIME_TO_RX, self) #ждем ответ в течении self.TIME_TO_RX мс
        return

    #*********************************************************************
    #передача пакета в RS
    #*********************************************************************
    def Read_File(self):
        try:
            self.Transmit_Off = self.file_hex.closed
            #проверка оттрыт ли файл
            if self.Transmit_Off == False:
                #файл открыт
                self.file_hex.seek(self.pos_new)
                #читаем данные из файла
                self.rs_DATA_TX_temp = self.file_hex.read(READ_BYTES) #читает из файла 100 байт
                self.pos_new = self.file_hex.tell()  #сохраняем текущую позицию в файле
                self.ui.progressBar.setValue(self.pos_new)   #установка текущего значения для progressBar
                if not self.rs_DATA_TX_temp: # возвращается пустая строка в случае конца файла
                    self.file_hex.close()   #закрываем файл
                    self.pos_new = 0
                    self.pos_last = 0
                    #переходим в режим окончания записи
                    self.STATUS_OLD = self.STATUS_NEW
                    self.STATUS_NEW = self.ID2["WRITE_END"]
                    self.Transmit_Off = self.file_hex.closed
                    self.Send_Write_End_Pack()  #отправляем окончание передачи
                    return #возвращаем принятые данные
                #формируем аднные для передачи
                self.rs_DATA_TX = self.pos_new.to_bytes(4,'little') + self.rs_DATA_TX_temp
                self.Transmit_RS_Data() #передаем данные в порт
                delay_time = self.TIME_TO_RX;      #задержка в 100 мс
                self.timer_RX_RS.start(delay_time, self) #ждем ответ в течении self.TIME_TO_RX мс
            return
        except:
            out_str = "Ошибка работы с фойлом. Откройте файл заново."
            QtWidgets.QMessageBox.warning(self, 'Сообщение',out_str , QtWidgets.QMessageBox.Ok)
            return


    #*********************************************************************
    # изменить назначение кнопки Прошить на Прервать
    #*********************************************************************
    def Change_Button_Send_to_Stop(self):
        _translate = QtCore.QCoreApplication.translate
        self.ui.pushButton_Send.setText(_translate("Serial_Main", "Прервать"))

    #*********************************************************************
    # изменить назначение кнопки Прервать на Прошить
    #*********************************************************************
    def Change_Button_Stop_to_Send(self):
        _translate = QtCore.QCoreApplication.translate
        self.ui.pushButton_Send.setText(_translate("Serial_Main", "Прошить"))

    #*********************************************************************
    # изменить значение кнопок для передачи
    #*********************************************************************
    def Change_Buttons_to_Send(self):
        #изменяем назначение кнопки прошить
        self.Change_Button_Send_to_Stop()
        #настраиваем видимость кнопок
        self.ui.pushButton_Choice_File.setDisabled(SET)   #де-активируем кнопку выбор файла
        self.ui.pushButton_open_COM.setDisabled(SET)      #де-активируем кнопку открытие порта
        self.ui.pushButton_close_COM.setDisabled(SET)     #де-активируем кнопку закрытие порта
        self.ui.comboBox_COM.setDisabled(SET)             #де-активируем выбор порта
        self.ui.comboBox_Baudrate.setDisabled(SET)        #де-активируем выбор скорости
        self.timer_RX_RS.start(self.TIME_TO_RX, self) #ждем ответ в течении self.TIME_TO_RX мс

    #*********************************************************************
    # отправить пакет сброса
    #*********************************************************************
    def Send_CPU_Reset_Pack(self):
        #формируем данные для передачи
        self.rs_ID2_TX = self.ID2["START"]
        self.rs_DATA_TX = bytearray([0x00]) #данных для передечи нет
        self.Transmit_RS_Data() #передаем данные в порт
        #изменияем значение rs_ID1_TЧ для следующей передачи, должна быть BOOTLODER_REQ
        self.rs_ID1_TX = self.ID1["BOOTLODER_REQ"]
        # закрываем порт и ожидаем self.TIME_TO_CPU_RESET
        self.ser.close()  #закрываем порт
        self.timer_RX_RS.start(self.TIME_TO_CPU_RESET, self) #ждем ответ в течении self.TIME_TO_CPU_RESET мс
        return

    #*********************************************************************
    #отправить стартовый пакет
    #*********************************************************************
    def Send_Start_Pack(self):
        #изменяем назначение кнопки прошить
        self.Change_Button_Send_to_Stop()
        #меняем состояние программы
        self.STATUS_OLD = self.STATUS_NEW
        self.STATUS_NEW = self.ID2["START"]
        #настраиваем видимость кнопок
        self.ui.pushButton_Choice_File.setDisabled(SET)   #де-активируем кнопку выбор файла
        self.ui.pushButton_open_COM.setDisabled(SET)      #де-активируем кнопку открытие порта
        self.ui.pushButton_close_COM.setDisabled(SET)     #де-активируем кнопку закрытие порта
        self.ui.comboBox_COM.setDisabled(SET)             #де-активируем выбор порта
        self.ui.comboBox_Baudrate.setDisabled(SET)        #де-активируем выбор скорости
        #формируем данные для передачи
        self.rs_ID1_TX = self.ID1["BOOTLODER_REQ"]
        self.rs_ID2_TX = self.ID2["START"]
        self.rs_DATA_TX = bytearray([0x00]) #данных для передечи нет
        self.Transmit_RS_Data() #передаем данные в порт
        self.timer_RX_RS.start(self.TIME_TO_RX, self) #ждем ответ в течении self.TIME_TO_RX мс
        return

    #*********************************************************************
    #отправить  пакет стирания
    #*********************************************************************
    def Send_Erase_Pack(self):
        #устанавливаем ID2 состояние ERASE
        self.rs_ID2_TX = self.ID2["ERASE"]
        #данных для передечи нет
        self.rs_DATA_TX = self.length_file.to_bytes(4,'little') #данных для передечи нет
        self.Transmit_RS_Data() #передаем данные в порт
        self.timer_RX_RS.start(self.TIME_TO_ERASE, self) #ждем ответ в течении 2 с
        return

    #*********************************************************************
    #отправить окончания передачи
    #*********************************************************************
    def Send_Write_End_Pack(self):
        #устанавливаем ID2 состояние ERASE
        self.rs_ID2_TX = self.ID2["WRITE_END"]
        #данных для передечи нет
        self.rs_DATA_TX = bytearray([0x00]) #данных для передечи нет
        self.Transmit_RS_Data() #передаем данные в порт
        self.timer_RX_RS.start(self.TIME_TO_RX, self) #ждем ответ в течении self.TIME_TO_RX мс
        return

    #*********************************************************************
    #отправить окончания передачи
    #*********************************************************************
    def Send_End_Pack(self):
        #устанавливаем ID2 состояние ERASE
        self.rs_ID2_TX = self.ID2["END"]
        #данных для передечи нет
        self.rs_DATA_TX = bytearray([0x00]) #данных для передечи нет
        self.Transmit_RS_Data() #передаем данные в порт
        self.timer_RX_RS.start(self.TIME_TO_RX, self) #ждем ответ в течении self.TIME_TO_RX мс
        return

    #*********************************************************************
    #отправить предыдущий пакет
    #*********************************************************************
    def Send_Previous_Pack(self):
        #прошлый пакет не был принят, нужно повторить попытку передачи
        #уменьшаем счетчик если это не первый пакет который передается первый раз
        if self.pos_new != self.pos_last:
            self.pos_new = self.pos_last    #возвращаемя к прошлому пакету
            if self.rs_pack_seq_TX == 0: #номер пакета возвращаем прошлый
                self.rs_pack_seq_TX == 255
            else:
                self.rs_pack_seq_TX -= 1
        self.Read_File()
        return

    #*********************************************************************
    #инициализация таймера
    #*********************************************************************
    def timerEvent(self, e):
            self.timer_RX_RS.stop() #выключаем таймер
            try:
                if self.rs_ID1_TX == self.ID1["CPU_RESET"]:
                    #вывод "RESET"
                    self.ui.plainTextEdit.appendPlainText("Перезагрузка устройства")
                    #отпаравляем пакет перезагрузки
                    self.Send_CPU_Reset_Pack()
                    return
                else:
                    if self.STATUS_NEW == self.ID2["IDLE"]:
                        #ничего не делаем в состоянии IDLE
                        self.flag_RX_OK = False
                    elif self.STATUS_NEW == self.ID2["WRITE"]:
                        #проверка есть ли данные в буфере RS
                        self.rs_receive_pack = self.Recieve_RS_Data()
                        if self.rs_receive_pack != '':
                            #анализзируем принятые данные
                            self.analyze_pack()
                        if self.flag_RX_OK == True:
                            #пакет был принят без ошибок
                            #вывод принятых данных в окно
                            self.ui.plainTextEdit.appendPlainText("Записано "+str(self.pos_new)+" байт")
                            #отправляем слудующий пакет
                            self.Read_File()
                        else:
                            # в принятом пакете была ошибка, отправляем предыдущий пакет повторно
                            self.Send_Previous_Pack()
                            #вывод "ошибка обмена данными"
                            self.ui.plainTextEdit.appendPlainText("Ошибка обмена данными")
                        self.flag_RX_OK = False
                        return
                    elif self.STATUS_NEW == self.ID2["START"]:
                        self.Serial_Config() # открываем порт
                        self.rs_receive_pack = self.Recieve_RS_Data()
                        if self.rs_receive_pack != '':
                            #анализзируем принятые данные
                            self.analyze_pack()
                        if self.flag_RX_OK == True:
                            #вывод "ERASE"
                            self.ui.plainTextEdit.appendPlainText("Очистка памяти")
                            #переходим к передаче пакето очистки
                            self.Send_Erase_Pack()
                        else:
                            #пакет был принят с ошибкой, отправляем его повторно
                            self.Send_Start_Pack()  #отправляем стартовый пакет
                            #вывод "ошибка обмена данными"
                            self.ui.plainTextEdit.appendPlainText("Ошибка обмена данными")
                            self.timer_RX_RS.start(self.TIME_TO_RX, self) #ждем ответ в течении self.TIME_TO_RX мс
                        self.flag_RX_OK = False
                        return
                    elif self.STATUS_NEW == self.ID2["ERASE"]:
                        self.rs_receive_pack = self.Recieve_RS_Data()
                        if self.rs_receive_pack != '':
                            #анализзируем принятые данные
                            self.analyze_pack()
                        if self.flag_RX_OK == True:
                            #переходим к передаче файла
                            self.send_file()
                        else:
                            #пакет был принят с ошибкой, отправляем его повторно
                            self.Send_Erase_Pack()
                            #вывод "ошибка обмена данными"
                            self.ui.plainTextEdit.appendPlainText("Ошибка обмена данными")
                        self.flag_RX_OK = False
                        return
                    elif self.STATUS_NEW == self.ID2["WRITE_END"]:
                        self.rs_receive_pack = self.Recieve_RS_Data()
                        if self.rs_receive_pack != '':
                            #анализзируем принятые данные
                            self.analyze_pack()
                        if self.flag_RX_OK == True:
                            #переходим к передаче завершающего пакета
                            self.Send_End_Pack()
                        else:
                            #пакет был принят с ошибкой, отправляем его повторно
                            self.Send_Write_End_Pack()
                            #вывод "ошибка обмена данными"
                            self.ui.plainTextEdit.appendPlainText("Ошибка обмена данными")
                        self.flag_RX_OK = False
                        return
                    elif self.STATUS_NEW == self.ID2["END"]:
                        self.rs_receive_pack = self.Recieve_RS_Data()
                        if self.rs_receive_pack != '':
                            #анализзируем принятые данные
                            self.analyze_pack()
                            if self.flag_RX_OK == True:
                                #закрываем порт и изменяем надписи на кнопках
                                self.close_COM_Handler()
                            else:
                                #пакет был принят c ошибкой, отправляем его повторно
                                self.Send_End_Pack()
                                #вывод "ошибка обмена данными"
                                self.ui.plainTextEdit.appendPlainText("Ошибка обмена данными")
                        else:
                            #вывод "END RESET"
                            self.ui.plainTextEdit.appendPlainText("Завершение записи")
                            #закрываем порт и изменяем надписи на кнопках
                            self.close_COM_Handler()
                            #переходим в состояние IDLE

                        self.flag_RX_OK = False
                        return
                    elif self.STATUS_NEW == self.ID2["RESET"]:
                        #проводим повторный опрос портов
                        self.init_RS()
                        self.STATUS_OLD = self.STATUS_NEW
                        self.STATUS_NEW = self.ID2["IDLE"]
                        #ID1 слдующей передаваемой команды - вначале сбрасываем устройство
                        self.rs_ID1_TX = self.ID1["CPU_RESET"]
            except:
                    out_str = "Порт будет закрыт, повторите прошивку заново."
                    QtWidgets.QMessageBox.warning(self, 'Ошибка работы с портом №1.',out_str , QtWidgets.QMessageBox.Ok)
                    #Закрытие порта и выключение записи - переход в исходное состояние
                    self.STATUS_OLD = self.STATUS_NEW
                    self.STATUS_NEW = self.ID2["IDLE"]
                    self.close_COM_Handler()
                    return

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    myapp = Serial_Qt()
    myapp.show()
    sys.exit(app.exec_())
