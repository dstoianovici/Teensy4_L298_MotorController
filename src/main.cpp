/**
 * Author: Dan Stoianovici
 * Email: stoianovici.dan.s@gmail.com
 * License: MIT
 * 
 *  ROS Enabled Motor Controller for Teensy4.0 Motor Controller Board
 * 

 REQUIREMENTS
 - Use update functiions to separate tasks
 - Schedule Task for each of following functions
    - Position Measurement
    - Velocity Calculation
    - Serial Comms
    - ROS Comms (Custom Message)

    --------Future Features----------
    - Acceleration Calculation
    - Current Measurement and Calculation 

**/


// #include <Arduino.h>
#include <MotorController_Pins.h>
#include <MotorController.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <open_motor_msgs/feedback.h>
#include <open_motor_msgs/pid_config.h>
#include <open_motor_msgs/setpoints.h>





#define GEAR_RATIO 131.0
#define COUNT_PER_ROT_ENC 16.0
#define COUNT_PER_ROT GEAR_RATIO * COUNT_PER_ROT_ENC

// Communicator::Comm_Data comm_msg;


Motor mot0(MOT0_EN,MOT0_PWM1,MOT0_PWM2,SENSE0,ENC0_A,ENC0_B,COUNT_PER_ROT);  
Motor mot1(MOT1_EN,MOT1_PWM1,MOT1_PWM2,SENSE1,ENC1_A,ENC1_B,COUNT_PER_ROT);  
Motor mot2(MOT2_EN,MOT2_PWM1,MOT2_PWM2,SENSE2,ENC2_A,ENC2_B,COUNT_PER_ROT);  
Motor mot3(MOT3_EN,MOT3_PWM1,MOT3_PWM2,SENSE3,ENC3_A,ENC3_B,COUNT_PER_ROT);  

MotorController motor_controller(mot0, mot1, mot2, mot3);






// MotorController motor_controller;

// static const int LED_PIN = LED_BUILTIN;


int goal_pos;
float goal_vel;

ros::NodeHandle nh;
open_motor_msgs::feedback feedback;


void setpoint_callback(const open_motor_msgs::setpoints setpoint_msg){
  int set_pos[4];
  float set_vel[4];

  for(int i=0; i < motor_controller.numMotors(); i++){
    set_pos[i] = setpoint_msg.position_setpoint[i];
    set_vel[i] = setpoint_msg.velocity_setpoint[i];
  }
  motor_controller.updateSetpoints_pos(set_pos);
  motor_controller.updateSetpoints_vel(set_vel);
}

void pid_config_callback(const open_motor_msgs::pid_config pid_vars){
  if(pid_vars.update == true){
    mot0.setPID_vars_pos(pid_vars.kP_pos,pid_vars.kI_pos,pid_vars.kD_pos);
    mot0.setPID_vars_vel(pid_vars.kP_vel,pid_vars.kI_vel,pid_vars.kD_vel);
    mot0.setPIDUpdateRate(pid_vars.pid_update_position);
  }
}

ros::Subscriber<open_motor_msgs::setpoints> setpoints_sub("open_motor_setpoints",&setpoint_callback);
ros::Subscriber<open_motor_msgs::pid_config> pid_config_sub("open_motor_pid_config",&pid_config_callback);
ros::Publisher feedback_pub("open_motor_feedback", &feedback);


void setup() {

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(pid_config_sub);
  nh.subscribe(setpoints_sub);
  nh.advertise(feedback_pub);



  mot0.init_motor();
  mot0.enable_motor();

  // mot1.init_motor();
  // mot1.enable_motor();

  mot2.init_motor();
  mot2.enable_motor();

  mot3.init_motor();
  mot3.enable_motor();


  // mot0.setPIDUpdateRate(15);
  mot3.setPID_vars_pos(1.25, 0.03, 0.0);  
  mot3.setPID_vars_vel(1.0, 0.15, 0.0075);

  motor_controller.assignPIDvars_all_pos(1.25, 0.03, 0.0);
  motor_controller.assignPIDvars_all_vel(1.0, 0.15, 0.0075);


  for(int i = 0; i<4; i++){
    feedback.position[i] = 0;
    feedback.velocity[i] = 0;
  }


}

void loop() {

  nh.spinOnce();

  feedback.position[0] = mot0.read_enc();
  feedback.position[1] = mot1.read_enc();
  feedback.position[2] = mot2.read_enc();
  feedback.position[3] = mot3.read_enc();

  feedback.velocity[0] = mot0.getVelocity();
  feedback.velocity[1] = mot1.getVelocity();
  feedback.velocity[2] = mot2.getVelocity();
  feedback.velocity[3] = mot3.getVelocity();

  feedback_pub.publish(&feedback);

  motor_controller.update_pid_vel_all();

  delay(5);
}