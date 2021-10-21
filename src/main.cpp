/**
 * Author: Dan Stoianovici
 * Email: stoianovici.dan.s@gmail.com
 * License: MIT
 * 
 *  ROS Enabled Motor Controller for Teensy4.0 Motor Controller Board
 * 

 REQUIREMENTS
 - Use FreeRTOS
 - Schedule Task for each of following functions
    - Position Measurement
    - Velocity Calculation
    - Acceleration Calculation
    - Current Measurement and Calculation 
    - Recieve Custom ROS Message to 

**/


// #include <Arduino.h>
#include <MotorController_Pins.h>
#include <MotorController.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#define GEAR_RATIO 131.0
#define COUNT_PER_ROT_ENC 16.0
#define COUNT_PER_ROT GEAR_RATIO * COUNT_PER_ROT_ENC

Motor mot0(MOT0_EN,MOT0_PWM1,MOT0_PWM2,SENSE0,ENC0_A,ENC0_B,COUNT_PER_ROT);  





// MotorController motor_controller;

// static const int LED_PIN = LED_BUILTIN;


volatile int goal_pos;

ros::NodeHandle nh;
std_msgs::Int32 pos_fb;
std_msgs::Float32 velocity_msg;

void set_position_goal(const std_msgs::Int32& goal_msg){
  goal_pos = goal_msg.data;
  // mot0.setSetpoint((int)goal_msg.data);
}


ros::Subscriber<std_msgs::Int32> position_goal_sub("goal_pos", &set_position_goal);
ros::Publisher enc_feedback_pub("pos_fb", &pos_fb);
ros::Publisher velocity_pub("velocity_fb", &velocity_msg);

void setup() {
  // Serial.begin(115200);



  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(position_goal_sub);
  nh.advertise(enc_feedback_pub);
  nh.advertise(velocity_pub);



  mot0.init_motor();
  mot0.enable_motor();

  mot0.setPIDUpdateRate(10);
  mot0.setPID_vars(0.05, 0.25, -0.1);

  
}

void loop() {

  nh.spinOnce();

  
  // mot0.drive_motor(goal_pos);
  // error_msg =  mot0.pid_position(goal_pos);
  velocity_msg.data = mot0.pid_velocity(goal_pos);


  pos_fb.data = mot0.read_enc();
  enc_feedback_pub.publish(&pos_fb);
  velocity_pub.publish(&velocity_msg);

  delay(10);

}