# open_motor
This repo provides an open source motor controller library with control inputs over ROS adaptable to many hardware control schemes. In current development, the library is implemented on a teensy4.0 using a Teensy4 Motor Shield. 

This Teensy4 Motor shield incoroporates 2x L298N motor driver ICs, 4 encoder inputs, and 4 current measurement circuits allowing for control of 4 DC motors. The firmware provided here allows for direct integration into robotics projects to get protoypes running quickly.

This repo serves as documentation for the motor controller library as well as documentation for the hardware.

![MotorController](https://user-images.githubusercontent.com/26233185/141203471-50df6b40-a233-4334-bb6f-87546810c80e.jpg)

### Features
- ROS Control with custom messages using rosserial on ROS-Melodic
- Dynamic PID tuning for each controller 
- Position and Velocity Setpoints for each motor
- Abstracted Motor Controller Library
- Custom UDEV rules and USB parameters for automatic port connection (Linux)



## Installation
This board is supplied with a firmware allowing for 4 motor control out of the box. This motor controller library uses ROS for PID parameter and setpoint communication as well as feedback. 

### Setting up the host computer
#### ROS
These instructions assume you already are familiar with ROS and Linux based OS. Furthermore, this system has been developed for ROS1 Melodic, any other distros are not yet tested.

First, ensure that rosserial is installed on you system, if not install using

        sudo apt-get install ros-melodic-rosserial
        
or to test another distro please use

        sudo apt-get install ros-<your distro>-rosserial

#### Python
  TO BE ADDED

## Board Dimensions
![Board_Dimensions](https://user-images.githubusercontent.com/26233185/141202799-c8fdb869-865a-4dba-a2c7-790d3b03d4e7.JPG)
All board dimensions are taken from Eagle CAD, feel free to round dimension numbers when convenient. It should be noted that the heat sinks on the L298N IC stick up 30mm from the top of the PCB.
