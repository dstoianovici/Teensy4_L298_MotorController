#! /usr/bin/python3

#Testing serial comms to teensy


# import serial as s
# from time import sleep



baudRate = 9600
port = "/dev/ttyACM0"
    
import serial
import json
import time


jtest = {}
jtest["mot_num"] = 0;

jtest = json.dumps(jtest,skipkeys = True).encode("ascii")

print(jtest)

arduino = serial.Serial(port, baudRate, timeout=1.0)

def write_read():

    arduino.write(jtest)
    # arduino.flush()
    time.sleep(0.5)

    try:
        data = arduino.readline().decode('utf-8')
        print(data)

    except Exception as e:
        print(e)
        pass

def main():
    while True:
        write_read()


if __name__ == "__main__":
    main()

