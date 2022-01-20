#! /usr/bin/python3

#Testing serial comms to teensy


# import serial as s
# from time import sleep



baudRate = 115200
port = "/dev/ttyACM0"

from open_motor_serial import open_motor
import json
import serial
import time

comms = open_motor()
comms.init_serial_port(port,baudRate,0.5)

def main():
    while True:
        # comms.send_vel_goal(100,0,100,100)
        # print("Response:" + comms.get_response())
        # time.sleep(0.5)

        # comms.send_pwm_goal(0,0,0,0)
        # print("Response:" + comms.get_response())
        # time.sleep(0.5)

        # comms.send_vel_goal(-100,0,-100,-100)
        # print("Response:" + comms.get_response())
        # time.sleep(0.5)

        # comms.send_pos_goal(100,100,100,100)
        comms.send_pwm_goal(100,100,-100,-100)
        # data = comms.get_response()

        # print(data)

        # print("Response:" + data)
        time.sleep(0.5)

        comms.send_pwm_goal(-100,-100,100,100)
        data = comms.get_response_json()

        print(data['pos1'])
        print(data['vel2'])

        # print("Response:" + data)
        time.sleep(0.5)

        # comms.send_pos_goal(0,0,0,0)
        # print("Response:" + comms.get_response())
        # time.sleep(1.5)

        # comms.send_pos_goal(-100,-100,-100,-100)
        # print("Response:" + comms.get_response())
        # time.sleep(1.5)

        # comms.send_pos_goal(0,0,0,0)
        # print("Response:" + comms.get_response())
        # time.sleep(1.5)

        


if __name__ == "__main__":
    main()

