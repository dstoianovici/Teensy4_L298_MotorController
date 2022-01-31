#! /usr/bin/python3

baudRate = 115200
port = "/dev/ttyACM0"

import sys
sys.path.append("../src/open_motor/")
# This line points the python path to the open_motor module.
# This will need to be changed based on the location of your code.

from open_motor_serial import open_motor
import time

comms = open_motor()
comms.init_serial_port(port,baudRate,0.5)

def main():
    while True:

        comms.send_pwm_goal(0,0,0,0)
        data = comms.get_response_json()
        print(data)
        

        print("Position motor 0: " + str(data['pos0']))
        print("Position motor 1: " + str(data['pos1']))
        print("Position motor 2: " + str(data['pos2']))
        print("Position motor 3: " + str(data['pos3']))

        print("Velocity motor 0: " + str(data['vel0']))
        print("Velocity motor 0: " + str(data['vel1']))
        print("Velocity motor 0: " + str(data['vel2']))
        print("Velocity motor 0: " + str(data['vel3']))

        time.sleep(0.5)
        



if __name__ == "__main__":
    main()

