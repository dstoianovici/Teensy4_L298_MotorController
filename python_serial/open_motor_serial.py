import serial
import json
import time

class serial_communicator:

    def __init__(port, baudRate, timeout):
        arduino = serial.Serial(port, baudRate, timeout=2.5)

