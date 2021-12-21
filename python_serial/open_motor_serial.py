import serial
import json
import time

class serial_communicator:

    def __init__(self,port, baudRate, _timeout):
        self.ser = serial.Serial(port, baudRate, timeout = _timeout)
        self.pos_pid_msg = {}
        self.vel_pid_msg = {}
        self.pid_config_msg = {}
        self.msg_data = {}

    def send_pwm_goal(self,pwm0,pwm1,pwm2,pwm3):
        self.msg_data["command"] = "pwm_direct"
        self.msg_data["pwm0"] = pwm0
        self.msg_data["pwm1"] = pwm1
        self.msg_data["pwm2"] = pwm2
        self.msg_data["pwm3"] = pwm3

        msg = json.dumps(self.msg_data).encode("ascii")

        self.ser.write(msg)


    def send_vel_goal(self,vel0,vel1,vel2,vel3):
        self.msg_data["command"] = "vel_pid"
        self.msg_data["vel0"] = vel0
        self.msg_data["vel1"] = vel1
        self.msg_data["vel2"] = vel2
        self.msg_data["vel3"] = vel3

        msg = json.dumps(self.msg_data).encode("ascii")

        self.ser.write(msg)


    def get_response(self):
        self.ser.flushInput()
        data = self.ser.readline().decode("utf-8")
        return data

    # def wait_for_response(self):
    #     response = "string"
    #     while(response != "msg_recieved"):
    #         response = self.get_response()
        

    






