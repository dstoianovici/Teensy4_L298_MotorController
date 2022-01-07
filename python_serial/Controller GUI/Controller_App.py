from os import remove
import sys, time
from PyQt5.uic.uiparser import QtCore
from numpy.lib.function_base import append
from pyqtgraph.widgets.PlotWidget import PlotWidget
import serial.tools.list_ports as listPorts
sys.path.append("../")
from open_motor_serial import open_motor
from PyQt5.QtWidgets import (
    QApplication, QDialog, QMainWindow, QMessageBox,
)
from PyQt5.uic import loadUi
from PyQt5 import QtCore

import re
import subprocess

# from Controller_UI import Ui_MainWindow


class Main_Window(QMainWindow):
    
    def __init__(self, parent=None):
        super().__init__(parent)
        loadUi("ui/Controller_GUI.ui",self)

        #Members
        self._port = None
        self._baudrate = None
        self._timeout = 0.5
        self.motors = open_motor()

        self.open_ports = self.find_connected_ports()
        self.fillPortMenu(self.open_ports)
        self.connectBaudrateMenu()
        self.connectButtons()
        

        self.pos0_data = []
        self.pos1_data = []
        self.pos2_data = []
        self.pos3_data = []

        self.vel0_data = []
        self.vel1_data = []
        self.vel2_data = []
        self.vel3_data = []

        self.data_array = [self.pos0_data,self.pos1_data,self.pos2_data,self.pos3_data,self.vel0_data,self.vel1_data, self.vel2_data, self.vel3_data]

        # pos0_pen = self.responsePlot.mkPen(color='r')
        # pos1_pen = PlotWidget.mkPen(color='g')
        # pos2_pen = self.responsePlot.mkPen(color='b')
        # pos3_pen = self.responsePlot.mkPen(color='y')

        self.responsePlot.addLegend()


        self.pos0_line = self.responsePlot.plot(name="pos0", pen=(1,4))
        self.pos1_line = self.responsePlot.plot(name="pos1", pen=(2,4))
        self.pos2_line = self.responsePlot.plot(name="pos2", pen=(3,4))
        self.pos3_line = self.responsePlot.plot(name="pos3", pen=(4,4))


        self.timer = QtCore.QTimer()
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()


      

    def fillPortMenu(self,ports):
        for port in ports:
            self.menuPort.addAction(port).triggered.connect(lambda:self.select_port(port))


    def connectBaudrateMenu(self):
        self.action115200.triggered.connect(lambda:self.select_baudrate(115200))
        self.action57600.triggered.connect(lambda:self.select_baudrate(57600))
        self.action38400.triggered.connect(lambda:self.select_baudrate(38400))
        self.action19200.triggered.connect(lambda:self.select_baudrate(19200))
        self.action9600.triggered.connect(lambda:self.select_baudrate(9600))
        self.action4800.triggered.connect(lambda:self.select_baudrate(4800))
        self.action2400.triggered.connect(lambda:self.select_baudrate(2400))
        self.action1200.triggered.connect(lambda:self.select_baudrate(1200))

    def connectButtons(self):
        self.portConnect.clicked.connect(self.serial_connect)
        self.send_commandButton.clicked.connect(self.motors.get_response_json)

    def find_connected_ports(self):
        ports = []
        devices = listPorts.comports()
        for dev in devices:
            ports.append(dev.device)
        return ports

    def select_port(self,port):
        self._port = port
        print(self._port)

    def select_baudrate(self,rate):
        self._baudrate = rate
        print(self._baudrate)

    def serial_connect(self):
        self.motors.init_serial_port(self._port,self._baudrate,self._timeout)
        print(self.motors.ser.isOpen())

    # def connect_signals_slots(self):

    def update_plot_data(self):
        # if(self.motors.ser.is_open()):
            rx_data = self.motors.get_response_json()
            self.pos0_data.append(rx_data['pos0'])
            self.pos1_data.append(rx_data['pos1'])
            self.pos2_data.append(rx_data['pos2'])
            self.pos3_data.append(rx_data['pos3'])

            self.vel0_data.append(rx_data['vel0'])
            self.vel1_data.append(rx_data['vel1'])
            self.vel2_data.append(rx_data['vel2'])
            self.vel3_data.append(rx_data['vel3'])


            if(len(self.pos0_data) > 100):
                for data in self.data_array:
                    data.pop(0)


            

            # self.timestamp_data.append("t")

            self.pos0_line.setData(self.pos0_data)
            self.pos1_line.setData(self.pos1_data)
            self.pos2_line.setData(self.pos2_data)
            self.pos3_line.setData(self.pos3_data)

        

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = Main_Window()
    win.show()
    sys.exit(app.exec())

