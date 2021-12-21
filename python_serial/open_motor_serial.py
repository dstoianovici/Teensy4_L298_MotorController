import serial
import json
import time

class serial_communicator:

    def __init__(self,port, baudRate, _timeout):
        self.ser = serial.Serial(port, baudRate, timeout = _timeout)
        self.pos_pid_msg = {}
        self.vel_pid_msg = {}
        self.pid_config_msg = {}
        self.pwm_msg = {}

    def send_pwm_goal(self,pwm0,pwm1,pwm2,pwm3):
        self.pwm_msg["command"] = "pwm_direct"
        self.pwm_msg["pwm0"] = pwm0
        self.pwm_msg["pwm1"] = pwm1
        self.pwm_msg["pwm2"] = pwm2
        self.pwm_msg["pwm3"] = pwm3

        msg = json.dumps(self.pwm_msg).encode("ascii")

        self.ser.write(msg)

    def get_response(self):
        self.ser.flushInput()
        data = self.ser.readline().decode("utf-8")
        return data

    # def wait_for_response(self):
    #     response = "string"
    #     while(response != "msg_recieved"):
    #         response = self.get_response()
        

    






