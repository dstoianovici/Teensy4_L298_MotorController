# open_motor
This repo provides an open source motor controller library, with control inputs available via  Python 3 Serial as well as a ROS Serial Interface. Although optimized for the Teensy4.0 Motor Controller Shield, this software is adaptable to many hardware control schemes.

**This repo is under rapid development, so check back for updates frequently, and submit all issues!**

This Teensy4 Motor shield incoroporates 2x L298N motor driver ICs, 4 encoder inputs, and 4 current measurement circuits allowing for control of 4 DC motors. The firmware and accompanying command code provided here allows for direct integration into robotics projects to get prototypes running quickly!

This repo serves as documentation for the motor controller library as well as documentation for the hardware.

![MotorController](https://user-images.githubusercontent.com/26233185/141203471-50df6b40-a233-4334-bb6f-87546810c80e.jpg)

### Features
- Control interface with ROS using custom messages and rosserial
- Control interface through Python3
- Dynamic PID tuning for each motor
- PID Tuning GUI (python only at the moment)
- Position and Velocity Control
- Abstracted Motor Controller Library
- Custom UDEV rules and USB parameters for automatic port connection (Linux)

## Wiring
The Teensy4.0 Motor Controller v1.1 provides 4 motor outputs, 4 encoder inputs, power input, and headers to plug in the Teensy4.0.
Each motor block, 0-3, is outlined on the screen print of the board and contains the motor output screw terminals, the encoder pin inputs, and the 4 flyback diodes that enable motor direction. Motor leads can be connected to the boards using the screw terminals and the corresponding encoder is plugged into the vertical pins behind it. Please use the boards screen print and the diagram shown in the **Board Dimensions** section for reference.

The next step is to plug in the Teensy4.0 into the female headers, and then the power input, VS and GND to the right of the Teensy Headers, can be wired. It is important to have an independent power switch for the power input as the power to the motor controller must be turned on after the microcontroller has been powered up, this power switch can even be a relay connected to the host computer allowing programatic switching of motor power. **It is imperative that power be provided to the Teensy4.0 via USB before power is given to VS or the level shifters will burn** this is because the Teensy's 3.3V output is used as a reference for the level shifters that allow 5V encoders to be read by the 3.3V logic Teensy.

## Installation and Configuration
<!-- This board is supplied with a firmware allowing for 4 motor control out of the box. Using the available serial package and Teensy Board firmware, motor control and communication is available of out the box. -->

### Setting up the Teensy
Open the '/open_motor/Teensy' directory in VS Code and switch to the ROS or PYTHON respectively to upload the correct firmware for your needs. Next, use PlatformIO Build to flash the Teensy4.0 with the motor controller firmware. If this is your first time flashing your teensy, then the board may need to be flashed from the Arduino IDE for the first time, as described [here](https://www.pjrc.com/teensy/troubleshoot.html), under the **No Serial Port While Programming** heading. A simple blink program from Arduino's IDE will be fine. After this is complete your teensy should flash the motor controller firmware correctly.

### ROS
#### Requirements
These instructions assume you already are familiar with ROS and Ubuntu Bash. Furthermore, this system has been developed for ROS1 Melodic, any other distros are not yet tested.
To use ROS, please clone this repo from the ROS branch, or clone the whole project and checkout the ROS branch. 

#### Setting up ROS
First, ensure that rosserial is installed on you system, if not install using

        sudo apt-get install ros-melodic-rosserial
        
Next, copy the directory `open_motor/ROS/open_motor_msgs` from this repo into `your_catkin_ws/src`. From here we can run `catkin_make` to compile the ROS messages for 'open_motor'.

With these steps the ROS setup is complete and these messages can now be used to communicate with the ROS node on the Teensy.

To test your setup start a roscore and run:

        rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200

this should result in launching 3 topics: `open_motor_feedback`, `open_motor_setpoints`, and `open_motor_pid_config`. This can also be verified using a `rostopic list` command.
      

### Python
#### Requirements
- python3 to send commands
- VS Code with PlatformIO extension installed to compile and upload teensy code
- A working directory for your project to establish absolute file paths within the project. This is currently necessary as the open_motor library is not yet released as a pip package
- Clone this github repo in your working directory 

#### Setting up Python 
To test the setup navigate to your 'open_motor/python_serial' and run the command

      source venv/bin/activate
  
which will load a python venv configured for the open_motor package. From here you may inspect and run sample code from the '/examples' directory as well running a beta version of the motor tuning software in '/Controller_GUI'. 


The open_motor python library is located in 'open_motor/python_serial/src/open_motor'. To use this library in your projects please copy 'open_motor_serial.py' into your working directory, or link back to the original install location as done with 'sys.path' in the examples using an absolute path from the working directory of your code. This is a temporary solution as the package is not yet prepared for pip distribution.



## Board Dimensions
![Board_Dimensions](https://user-images.githubusercontent.com/26233185/141202799-c8fdb869-865a-4dba-a2c7-790d3b03d4e7.JPG)
All board dimensions are taken from Eagle CAD, feel free to round dimension numbers when convenient. It should be noted that the heat sinks on the L298N IC stick up 30mm from the top of the PCB.