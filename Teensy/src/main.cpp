/**
 * Author: Dan Stoianovici
 * Email: stoianovici.dan.s@gmail.com
 * License: MIT
 * 
 *  ROS Enabled Motor Controller for Teensy4.0 Motor Controller Board
 *  Publishes feedback and accepts commands over ros to emulate ros actions
 * 
**/


#include <MotorController_Pins.h>
#include <MotorController.h>


#define GEAR_RATIO 131.0
#define COUNT_PER_ROT_ENC 16.0
#define COUNT_PER_ROT GEAR_RATIO * COUNT_PER_ROT_ENC
#define NUM_MOTORS 4


/////  Global Vars for Motor Control  /////
Message_Parser::Comm_Data comm_msg;

Motor::motor_struct mot0{MOT0_EN,MOT0_PWM1,MOT0_PWM2,SENSE0,ENC0_A,ENC0_B,COUNT_PER_ROT};
Motor::motor_struct mot1{MOT1_EN,MOT1_PWM1,MOT1_PWM2,SENSE1,ENC1_A,ENC1_B,COUNT_PER_ROT};
Motor::motor_struct mot2{MOT2_EN,MOT2_PWM1,MOT2_PWM2,SENSE2,ENC2_A,ENC2_B,COUNT_PER_ROT};
Motor::motor_struct mot3{MOT3_EN,MOT3_PWM1,MOT3_PWM2,SENSE3,ENC3_A,ENC3_B,COUNT_PER_ROT};

MotorController motor_controller;


/////  Global Callback Prototypes and Variables for ROS  /////
open_motor_msgs::feedback feedback;
void setpoint_callback(const open_motor_msgs::setpoints& setpoint_msg);
void pid_config_callback(const open_motor_msgs::pid_config& pid_vars);

ros::NodeHandle nh;
ros::Subscriber<open_motor_msgs::setpoints> setpoints_sub("open_motor_setpoints", &setpoint_callback);
ros::Subscriber<open_motor_msgs::pid_config> pid_config_sub("open_motor_pid_config", &pid_config_callback);
ros::Publisher feedback_pub("open_motor_feedback", &feedback);


void setup() {
  //Add  motors to motor controller
  motor_controller.addMotor(mot0);
  motor_controller.addMotor(mot1);
  motor_controller.addMotor(mot2);
  motor_controller.addMotor(mot3);

  //Initialize ROS Node
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(pid_config_sub);
  nh.subscribe(setpoints_sub);
  nh.advertise(feedback_pub);

  //Initialize to default PID variables and enable motors
  motor_controller.assignPIDupdate_all(50);
  motor_controller.assignPIDvars_all_pos(1.0,0.0,0.0);
  motor_controller.assignPIDvars_all_vel(1.0,0.0,0.0);
  motor_controller.initAllMotors();
  motor_controller.enableAllMotors();
}

void loop() {
  nh.spinOnce();

  motor_controller.parse_data_ros(&comm_msg);
  motor_controller.run_controller(&comm_msg);  
  motor_controller.prepare_feedback_data_ros(&feedback);
  feedback_pub.publish(&feedback);
  
  delay(5);
}



/////  Callback Functions /////
void setpoint_callback(const open_motor_msgs::setpoints& setpoint_msg){
  comm_msg.command = setpoint_msg.command;
  for(uint8_t i = 0; i < NUM_MOTORS; i++){
    comm_msg.goal_position[i] = setpoint_msg.position_setpoint[i];
    comm_msg.goal_velocity[i] = setpoint_msg.velocity_setpoint[i];
  }
}

void pid_config_callback(const open_motor_msgs::pid_config& pid_vars){
  if(pid_vars.update == true){

    comm_msg.pos_update = pid_vars.pid_update_position;
    comm_msg.vel_update = pid_vars.pid_update_velocity;

    for(uint8_t i = 0; i < NUM_MOTORS; i++){
      comm_msg.kP_pos[i] = pid_vars.kP_pos[i];
      comm_msg.kI_pos[i] = pid_vars.kI_pos[i];
      comm_msg.kD_pos[i] = pid_vars.kD_pos[i];

      comm_msg.kP_vel[i] = pid_vars.kP_vel[i];
      comm_msg.kI_vel[i] = pid_vars.kI_vel[i];
      comm_msg.kD_vel[i] = pid_vars.kD_vel[i];
    }
  }
}