from os import remove
import sys, time
import serial.tools.list_ports as listPorts
sys.path.append("../")
import open_motor_serial
from PyQt5.QtWidgets import (
    QApplication, QDialog, QMainWindow, QMessageBox
)
from PyQt5.uic import loadUi

import re
import subprocess

# from Controller_UI import Ui_MainWindow


class Main_Window(QMainWindow):
    
    def __init__(self, parent=None):
        super().__init__(parent)
        loadUi("ui/Controller_GUI.ui",self)
        
        self.open_ports = self.find_connected_ports()
        self.fillPortMenu(self.open_ports)
        self.connectBaudrateMenu()
        self.connectButtons()


        #Members
        self._port = None
        self._baudrate = None
        self._timeout = 0.5
        self.serial_comms = None


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
        self.serial_comms = open_motor_serial.serial_communicator(self._port,self._baudrate,self._timeout)
        print(self.serial_comms.isOpen())

    # def connect_signals_slots(self):
        

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = Main_Window()
    win.show()
    sys.exit(app.exec())

