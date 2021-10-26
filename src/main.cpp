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
// #include <std_msgs/Int32.h>
// #include <std_msgs/Float32.h>
// #include <std_msgs/String.h>

#define SERIAL

#define GEAR_RATIO 131
#define COUNT_PER_ROT_ENC 16
#define COUNT_PER_ROT GEAR_RATIO*COUNTPER_ROT_ENC

// Motor mot0(MOT0_EN,MOT0_PWM1,MOT0_PWM2,SENSE0,ENC0_A,ENC0_B); 



// #ifdef ROS
   
// #ifdef PYTHON


// MotorController motor_controller;

// static const int LED_PIN = LED_BUILTIN;


volatile int goal_pos;

#ifdef ROS
ros::NodeHandle nh;
std_msgs::Int32 pos_fb;
std_msgs::Float32 error_msg;

void set_position_goal(const std_msgs::Int32& goal_msg){
  goal_pos = goal_msg.data;
  // mot0.setSetpoint((int)goal_msg.data);
}

ros::Subscriber<std_msgs::Int32> position_goal_sub("goal_pos", &set_position_goal);
ros::Publisher enc_feedback_pub("enc_reading", &pos_fb);
ros::Publisher error_string_pub("motor_output", &error_msg);
#endif

#ifdef SERIAL

#endif


void setup() {

  #ifdef SERIAL

    Serial.begin(115200);
    Serial.setTimeout(1);
    // pinMode(13, OUTPUT);

    
  #endif


  #ifdef ROS

    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(position_goal_sub);
    nh.advertise(enc_feedback_pub);
    nh.advertise(error_string_pub);
  
  #endif


  // mot0.init_motor();
  // mot0.enable_motor();

  // mot0.setPIDUpdateRate(10);

  
}

void loop() {


  
  // // mot0.drive_motor(goal_pos);
  // error_msg =  mot0.pid_position(goal_pos);


  #ifdef ROS
    nh.spinOnce();
    pos_fb.data = mot0.read_enc();
    enc_feedback_pub.publish(&pos_fb);
    error_string_pub.publish(&error_msg);
  #endif

  #ifdef SERIAL

  // if(Serial.available() > 0){
  //   int val = Serial.parseInt();

  //   if(val%2 == 0) digitalWrite(13,HIGH);
  //   else digitalWrite(13, LOW);
  // }

 int x;
 while (!Serial.available());
 x = Serial.readString().toInt();
 Serial.print(x + 1);



  #endif



  delay(50);

}