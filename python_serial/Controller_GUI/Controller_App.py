from os import remove
import sys, time
from PyQt5.uic.uiparser import QtCore
from numpy.lib.function_base import append
from pyqtgraph.widgets.PlotWidget import PlotWidget
import serial.tools.list_ports as listPorts
sys.path.append("../src/open_motor/")
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
        self._command_type = None

        self._graph_type = "Position"

        self.open_ports = self.find_connected_ports()
        self.fillPort_comboBox(self.open_ports)

        self.commandType_comboBox.currentIndexChanged.connect(self.set_command_type)
        self.graph_comboBox.currentIndexChanged.connect(self.set_graph_type)

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

        self.vel0_line = self.responsePlot.plot(name="vel0", pen=(1,4))
        self.vel1_line = self.responsePlot.plot(name="vel1", pen=(2,4))
        self.vel2_line = self.responsePlot.plot(name="vel2", pen=(3,4))
        self.vel3_line = self.responsePlot.plot(name="vel3", pen=(4,4))

        self.timer = QtCore.QTimer()
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()


      

    def fillPortMenu(self,ports):
        for port in ports:
            self.menuPort.addAction(port).triggered.connect(lambda:self.select_port(port))

    def fillPort_comboBox(self,ports):
        for port in ports:
            self.port_comboBox.addItem(port)

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
        self.portConnect_Button.clicked.connect(self.serial_connect)
        self.sendCommand_Button.clicked.connect(self.send_command)
        self.stop_Button.clicked.connect(self.stop_motors_command)

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
        self._port = self.port_comboBox.currentText()
        self._baudrate = self.baud_comboBox.currentText()
        if((self._port != "Port" and self._baudrate != "Baudrate")):
            self.motors.init_serial_port(self._port,self._baudrate,self._timeout)
        else:
            print("Incorrect Port or Baudrate")

        self.update_plot_data()
        # print(self.motors.ser.isOpen())

    # def connect_signals_slots(self):

    def update_plot_data(self):
        if(self.motors.ser.isOpen()):
            rx_data = self.motors.get_response_json()
            self.pos0_data.append(rx_data['pos0'])
            self.pos1_data.append(rx_data['pos1'])
            self.pos2_data.append(rx_data['pos2'])
            self.pos3_data.append(rx_data['pos3'])

            self.vel0_data.append(rx_data['vel0'])
            self.vel1_data.append(rx_data['vel1'])
            self.vel2_data.append(rx_data['vel2'])
            self.vel3_data.append(rx_data['vel3'])

        if(self._graph_type == "Position Graph"):
            self.pos0_line.setData(self.pos0_data)
            self.pos1_line.setData(self.pos1_data)
            self.pos2_line.setData(self.pos2_data)
            self.pos3_line.setData(self.pos3_data)
        
        elif(self._graph_type == "Velocity Graph"):

            self.vel0_line.setData(self.vel0_data)
            self.vel1_line.setData(self.vel1_data)
            self.vel2_line.setData(self.vel2_data)
            self.vel3_line.setData(self.vel3_data)


        if(len(self.pos0_data) > 100):
            for data in self.data_array:
                data.pop(0)

    def set_command_type(self):
        self._command_type = self.commandType_comboBox.currentText()
        
        if  self._command_type == "Select Command Type":
            self.param_stackedWidget.setCurrentIndex(0)
        
        elif  self._command_type == "PWM":
            self.param_stackedWidget.setCurrentIndex(1)

        elif self._command_type == "PID Position":
            self.param_stackedWidget.setCurrentIndex(2)

        elif  self._command_type == "PID Velocity":
            self.param_stackedWidget.setCurrentIndex(3)
        
        else:
            self.param_stackedWidget.setCurrentIndex(0)

    def set_graph_type(self):
        self._graph_type = self.graph_comboBox.currentText()
        
        if(self._graph_type == "Position Graph"):
            self.vel0_line.clear()
            self.vel1_line.clear()
            self.vel2_line.clear()
            self.vel3_line.clear()

        elif(self._graph_type == "Velocity Graph"):
            self.pos0_line.clear()
            self.pos1_line.clear()
            self.pos2_line.clear()
            self.pos3_line.clear()

    def send_command(self):
        if  self._command_type == "Select Command Type":
            print("No Command")
        
        elif  self._command_type == "PWM":
            pwm0 = self.pwm_spinBox_0.value()
            pwm1 = self.pwm_spinBox_1.value()
            pwm2 = self.pwm_spinBox_2.value()
            pwm3 = self.pwm_spinBox_3.value()
            self.motors.send_pwm_goal(pwm0,pwm1,pwm2,pwm3)

        elif self._command_type == "PID Position":
            pos0 = self.pid_pos_spinBox_0.value()
            pos1 = self.pid_pos_spinBox_1.value()
            pos2 = self.pid_pos_spinBox_2.value()
            pos3 = self.pid_pos_spinBox_3.value()
            self.motors.send_pos_goal(pos0,pos1,pos2,pos3)


        elif  self._command_type == "PID Velocity":
            vel0 = self.pid_vel_spinBox_0.value()
            vel1 = self.pid_vel_spinBox_1.value()
            vel2 = self.pid_vel_spinBox_2.value()
            vel3 = self.pid_vel_spinBox_3.value()
            self.motors.send_vel_goal(vel0,vel1,vel2,vel3)
        
        else:
            print("No Command")           
        
    def stop_motors_command(self):
        self.motors.send_pwm_goal(0,0,0,0)

        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = Main_Window()
    win.show()
    sys.exit(app.exec())

