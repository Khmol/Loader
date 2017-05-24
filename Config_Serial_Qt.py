#coding: utf8

#class Config_Serial_Qt():
BAUDRATES = ['1200', '9600', '19200', '38400', '57600', '115200']    #возможные значения скоростей для RS-232
READ_BYTES = 100
OK_ANSWER = bytearray('OK'.encode('latin-1')) #OK
ERR_ANSWER = bytearray('Err'.encode('latin-1')) #Err
MAX_WAIT_BYTES = 200    #максимальное количество байт в буфере порта на прием
NUMBER_SCAN_PORTS = 10  #количество портов для сканирования 10
SET = 1
RESET = 0

