import serial
import json
import time

class serial_communicator:

    
    def __init__(port, baudRate, timeout):
        arduino = serial.Serial(port, baudRate, timeout)

    def send_pwm_goal(pwm0,pwm1,pwm2,pwm3):



