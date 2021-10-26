#! /usr/bin/python3

#Testing serial comms to teensy


# import serial as s
# from time import sleep



baudRate = 115200
port = "/dev/ttyACM0"

# # ser = s.Serial(port,baudRate)

# i = 0

# while(1):

#     i=i+1

#     ser = s.Serial(port,baudRate)

#     ser.write(i)
#     ser.write(bytes('\n','ascii'))




#     print("Sent: " + str(i))

#     # ser.flush

#     # ser.close()

#     sleep(0.5)


##From ArduinoCC
    
import serial
import time
arduino = serial.Serial(port, baudRate, timeout=.1)
def write_read(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    return data
while True:
    num = input("Enter a number: ") # Taking input from user
    value = write_read(num)
    print(value) # printing the value

