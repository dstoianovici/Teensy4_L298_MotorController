#! /usr/bin/python3

#Testing serial comms to teensy


# import serial as s
# from time import sleep



baudRate = 15200
port = "/dev/ttyACM0"

import open_motor_serial
import json
import serial
import time

jtest = {}
jtest["update"] = "true"
jtest["mot0_speed"] = 0
jtest["mot2_speed"] = 0
jtest["mot3_speed"] = 0

jtest = json.dumps(jtest,skipkeys = True).encode("ascii")

print(jtest)

arduino = serial.Serial(port, baudRate, timeout=2.5)


def write_read():
    arduino.write(jtest)
    arduino.flush()
    # time.sleep(0.5)

    try:
        data = arduino.readline().decode("utf-8")
        print("hello nerd")
        print(data)

    except Exception as e:
        print(e)
        pass

def main():
    while True:
        write_read()


if __name__ == "__main__":
    main()

