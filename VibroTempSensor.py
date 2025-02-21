from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.uic import loadUi
# from PyQt5.QtWebEngineWidgets import QWebEngineView
import os, sys, time, json, struct
import minimalmodbus, serial
import serial.tools.list_ports

import pandas as pd

class MainUI(QMainWindow):
    def __init__(self):
        super(MainUI, self).__init__()
        loadUi("VibroTempSensor.ui", self)
        self.sensorRegisters =[
                '01_MB_REG_STATUS',
                '02_MB_REG_DEVICE_CODE',
                '03_MB_REG_DEVICE_ID',
                '04_MB_REG_SW_VER',
                '05_MB_REG_SW_BUILD',
                '06_MB_REG_TEMPERATURE_BOTTOM',
                '07_MB_REG_TEMPERATURE_TOP',
                '08_MB_REG_ACC_TEMPERATURE',
                '09_MB_REG_DEVICE_RANGE',
                '10_MB_REG_SAMPLE_FREQ',
                '11_MB_REG_DATA_UPDATE_COUNTER',
                '12_MB_REG_ACCELERATION_RMS_X_LO',
                '13_MB_REG_ACCELERATION_RMS_X_HI',
                '14_MB_REG_ACCELERATION_RMS_Y_LO',
                '15_MB_REG_ACCELERATION_RMS_Y_HI',
                '16_MB_REG_ACCELERATION_RMS_Z_LO',
                '17_MB_REG_ACCELERATION_RMS_Z_HI',
                '18_MB_REG_VELOCITY_RMS_X_LO',
                '19_MB_REG_VELOCITY_RMS_X_HI',
                '20_MB_REG_VELOCITY_RMS_Y_LO',
                '21_MB_REG_VELOCITY_RMS_Y_HI',
                '22_MB_REG_VELOCITY_RMS_Z_LO',
                '23_MB_REG_VELOCITY_RMS_Z_HI',
                '24_MB_REG_DISPLACEMENT_RMS_X_LO',
                '25_MB_REG_DISPLACEMENT_RMS_X_HI',
                '26_MB_REG_DISPLACEMENT_RMS_Y_LO',
                '27_MB_REG_DISPLACEMENT_RMS_Y_HI',
                '28_MB_REG_DISPLACEMENT_RMS_Z_LO',
                '29_MB_REG_DISPLACEMENT_RMS_Z_HI',
                '30_MB_REG_PEAK_TO_PEAK_X_LO',
                '31_MB_REG_PEAK_TO_PEAK_X_HI',
                '32_MB_REG_PEAK_TO_PEAK_Y_LO',
                '33_MB_REG_PEAK_TO_PEAK_Y_HI',
                '34_MB_REG_PEAK_TO_PEAK_Z_LO',
                '35_MB_REG_PEAK_TO_PEAK_Z_HI',
                '36_MB_REG_PEAK_FACTOR_X_LO',
                '37_MB_REG_PEAK_FACTOR_X_HI',
                '38_MB_REG_PEAK_FACTOR_Y_LO',
                '39_MB_REG_PEAK_FACTOR_Y_HI',
                '40_MB_REG_PEAK_FACTOR_Z_LO',
                '41_MB_REG_PEAK_FACTOR_Z_HI',
                '42_',
                '43_',
                '44_',
                '45_MB_REG_MAX_AMPLITUDE_FREQUENCY_X',
                '46_MB_REG_MAX_AMPLITUDE_FREQUENCY_Y',
                '47_MB_REG_MAX_AMPLITUDE_FREQUENCY_Z']

        self.sensorDataFields =[
                    '00_TIME',
                    '01_STATUS',
                    '02_DEVICE_CODE',
                    '03_DEVICE_ID',
                    '04_SW_VER',
                    '05_SW_BUILD',
                    '06_TEMPERATURE_BOTTOM',
                    '07_TEMPERATURE_TOP',
                    '08_ACC_TEMPERATURE',
                    '09_DEVICE_RANGE',
                    '10_SAMPLE_FREQ',
                    '11_DATA_UPDATE_COUNTER',
                    '12_ACCELERATION_RMS_X',
                    '14_ACCELERATION_RMS_Y',
                    '16_ACCELERATION_RMS_Z',
                    '18_VELOCITY_RMS_X',
                    '20_VELOCITY_RMS_Y',
                    '22_VELOCITY_RMS_Z',
                    '24_DISPLACEMENT_RMS_X',
                    '26_DISPLACEMENT_RMS_Y',
                    '28_DISPLACEMENT_RMS_Z',
                    '30_PEAK_TO_PEAK_X',
                    '32_PEAK_TO_PEAK_Y',
                    '34_PEAK_TO_PEAK_Z',
                    '36_PEAK_FACTOR_X',
                    '38_PEAK_FACTOR_Y',
                    '40_PEAK_FACTOR_Z',
                    '45_MAX_AMPLITUDE_FREQUENCY_X',
                    '46_MAX_AMPLITUDE_FREQUENCY_Y',
                    '47_MAX_AMPLITUDE_FREQUENCY_Z'
                    ]

        # ---------- Serial ports ----------
        portList = serial.tools.list_ports.comports(include_links=False)
        self.COMPorts = []
        for item in portList:
            self.COMPorts.append(item.device)
        message = "Доступные COM-порты: " + str(self.COMPorts)
        print(message)
        self.statusbar.showMessage(message)
        self.cBox_COMPort.addItems(self.COMPorts)

        self.Baudrates = ['2400', '4800', '9600', '19200', '38400', '57600', '115200']
        self.cBox_Baudrate.addItems(self.Baudrates)
        self.cBox_Baudrate.setCurrentIndex(6)

        self.Frequency = ['0.03', '0.1','0.5','1.0','5.0']
        self.cBox_Freq.addItems(self.Frequency)
        self.cBox_Freq.setCurrentIndex(2)

        self.Time = ['1', '5', '10', '30', '60', '300', '600']
        self.cBox_Time.addItems(self.Time)
        self.cBox_Time.setCurrentIndex(2)


        self.pBtn_Connect.clicked.connect(self.connectSensor)
        
        self.pBtn_Start.clicked.connect(self.collectData)
        self.pBtn_File.clicked.connect(self.dumpData)

    def IEEE754_to_float(self, int_lo: int, int_hi: int) -> float:
        # Забираем 4 символа 16-ричной записи каждого регистра
        hex_lo = hex(int_lo)[2:]
        hex_hi = hex(int_hi)[2:]
        # собираем шестнадцатиричное значение из регистров
        hex_value = f"{hex_lo:0>4}{hex_hi:0>4}"
        # Преобразуем hex в байты
        bytes_value = bytes.fromhex(hex_value)
        # Используем struct.unpack для получения float из байт
        float_value = struct.unpack('!f', bytes_value)[0]
        
        return float_value

    def connectSensor(self) -> None:
        try: # Инициализация датчика
            # Modbus-адрес драйвера по умолчанию - 1
            self.sensor = minimalmodbus.Instrument(self.cBox_COMPort.currentText(), 7) #COM5, 7 - адрес датчика
                    # Настройка порта: скорость - 115200 бод/с, четность - нет, кол-во стоп-бит - 1.
            if 1:
                self.sensor.mode = minimalmodbus.MODE_RTU
            else:
                self.sensor.mode = minimalmodbus.MODE_TCP
            # print(self.sensor.serial.port)
            # print(self.sensor.address)
            self.sensor.serial.baudrate = self.cBox_Baudrate.currentText()
            self.sensor.serial.parity = minimalmodbus.serial.PARITY_NONE
            self.sensor.serial.stopbits = 1
            # self.sensor.serial.databits = 8
            # self.sensor.serial.timeout  = 0.05          # seconds
            self.sensor.close_port_after_each_call = True
            message = ''.join(['Датчик подключен: ', str(self.sensor)])
            print(message)
            self.statusbar.showMessage(message)
            self.tab_Measure.setEnabled(True)
            self.pBtn_Connect.setStyleSheet('QPushButton {background-color : #45a049;}'
                                         'QPushButton:hover { background-color: forestgreen;}')
        except (IOError, AttributeError, ValueError) as error: # minimalmodbus.serial.serialutil.SerialException:
            message = "Датчик не виден"
            print(message)
            self.statusbar.showMessage(message)
            print(error)
            self.tab_Measure.setEnabled(True)

    def getDataFromSensor(self) -> list:
        try:
            dataline = self.sensor.read_registers(0, 50)
        except (IOError, AttributeError) as err:
            message = "Не удалось считать данные с датчика"
            print(message)
            self.statusbar.showMessage(message)
            print(err)
            dataline = []
        return dataline

    def collectData(self) -> None:
        freq = float(self.cBox_Freq.currentText())
        duration = int(self.cBox_Time.currentText())
        numberMeasurements = round(duration/freq)
        timestamp = 0.0
        self.pBtn_Start.setEnabled(False)
        self.pBtn_Start.setStyleSheet('QPushButton {background-color : red;}')
                                        # 'QPushButton:hover { background-color: forestgreen;}')

        self.sensorData = pd.DataFrame(columns=self.sensorDataFields)

        for _ in range(numberMeasurements):
            checktime = time.time()
            dataline = self.getDataFromSensor()

            line = []
            try:
                line.append(timestamp)
                line.append(dataline[0])
                line.append(dataline[1])
                line.append(dataline[2])
                line.append(dataline[3])
                line.append(dataline[4])
                line.append(dataline[5])
                line.append(dataline[6])
                line.append(dataline[7])
                line.append(dataline[8])
                line.append(dataline[9])
                line.append(dataline[10])
                line.append(self.IEEE754_to_float(dataline[11], dataline[12]))
                line.append(self.IEEE754_to_float(dataline[13], dataline[14]))
                line.append(self.IEEE754_to_float(dataline[15], dataline[16]))
                line.append(self.IEEE754_to_float(dataline[17], dataline[18]))
                line.append(self.IEEE754_to_float(dataline[19], dataline[20]))
                line.append(self.IEEE754_to_float(dataline[21], dataline[22]))
                line.append(self.IEEE754_to_float(dataline[23], dataline[24]))
                line.append(self.IEEE754_to_float(dataline[25], dataline[26]))
                line.append(self.IEEE754_to_float(dataline[27], dataline[28]))
                line.append(self.IEEE754_to_float(dataline[29], dataline[30]))
                line.append(self.IEEE754_to_float(dataline[31], dataline[32]))
                line.append(self.IEEE754_to_float(dataline[33], dataline[34]))
                line.append(self.IEEE754_to_float(dataline[35], dataline[36]))
                line.append(self.IEEE754_to_float(dataline[37], dataline[38]))
                line.append(self.IEEE754_to_float(dataline[39], dataline[40]))
                line.append(dataline[44])
                line.append(dataline[45])
                line.append(dataline[46])
            except IndexError as err:
                message = "Не удалось считать данные"
                print(message)
                self.statusbar.showMessage(message)
                print(err)

            try:
                self.sensorData.loc[len(self.sensorData)] = line
            except ValueError as err:
                message = "Не удалось считать данные с датчика"
                print(message)
                self.statusbar.showMessage(message)
                print(err)
            if freq >= 0.02:
                time.sleep(freq-0.2)
            timestamp = time.time()-checktime

        message = "Измерение завершено"
        print(message)
        self.statusbar.showMessage(message)
        self.pBtn_Start.setEnabled(True)
        self.pBtn_Start.setStyleSheet('QPushButton {background-color : green;}')
        try:
            self.sensor.serial.close()
        except AttributeError as err:
            message = "Не удалось считать данные с датчика"
            print(message)
            self.statusbar.showMessage(message)
            print(err)

    def dumpData(self) -> None:
        dataDir = 'data'
        os.makedirs(dataDir, exist_ok = True)
        comment = self.lEd_FileComment.text()
        filename = time.strftime(f"%Y-%m-%d_%H-%M-%S_{comment}")
        self.sensorData.to_csv(os.path.join(dataDir, f"sensorData_{filename}.csv"))

if __name__ == '__main__':
    app = QApplication([])
    
    rotor = MainUI()
    rotor.show()
    
    sys.exit(app.exec_())