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


#include <MotorController_Pins.h>
#include <MotorController.h>

#define SERIAL_COM

#define GEAR_RATIO 131
#define COUNT_PER_ROT_ENC 16
#define COUNT_PER_ROT GEAR_RATIO*COUNT_PER_ROT_ENC

#define BAUD 115200
#define TIMEOUT 1.0

Message_Parser::Comm_Data comm_data;


Motor_Struct mot0 = {MOT0_EN,MOT0_PWM1,MOT0_PWM2,SENSE0,ENC0_A,ENC0_B, COUNT_PER_ROT};
Motor_Struct mot1 = {MOT1_EN,MOT1_PWM1,MOT1_PWM2,SENSE1,ENC1_A,ENC1_B, COUNT_PER_ROT}; 
Motor_Struct mot2 = {MOT2_EN,MOT2_PWM1,MOT2_PWM2,SENSE2,ENC2_A,ENC2_B, COUNT_PER_ROT}; 
Motor_Struct mot3 = {MOT3_EN,MOT3_PWM1,MOT3_PWM2,SENSE3,ENC3_A,ENC3_B, COUNT_PER_ROT};

MotorController motors(mot0,mot1,mot2,mot3);






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

#ifdef SERIAL_COM
Serial_Comms serial_comms(BAUD,TIMEOUT);
#endif



void setup() {

  #ifdef SERIAL_COM

    serial_comms.init();

    
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
  // mot0.setPID_vars_pos(1.0,0.0,0.0);
  // mot0.setPIDUpdateRate(50);

  // // mot1.init_motor();
  // // mot1.enable_motor();

  // mot2.init_motor();
  // mot2.enable_motor();
  // mot2.setPID_vars_vel(1.0,0.0,0.0);
  // mot2.setPIDUpdateRate(50);
  
  // mot3.init_motor();
  // mot3.enable_motor();
  // mot3.setPID_vars_vel(1.0,0.0,0.0);
  // mot3.setPIDUpdateRate(50);

  motors.initAllMotors();
  motors.enableAllMotors();
  motors.assignPIDupdate_all(10.0);
  motors.assignPIDvars_all_vel(1.0,0.0,0.0);
  motors.assignPIDvars_all_pos(1.0,0.0,0.0);

  motors.drive_all(100);
  delay(250);
  motors.drive_all(0);


  
}

void loop() {

  #ifdef ROS
    nh.spinOnce();
    pos_fb.data = mot0.read_enc();
    enc_feedback_pub.publish(&pos_fb);
    error_string_pub.publish(&error_msg);
  #endif

  #ifdef SERIAL_COM

  serial_comms.check_for_data(comm_data);

  // motors.run_controller(comm_data);
  // motors.prepare_feedback_data(comm_data);


  // Serial.print(comm_data.goal_pwm[0]);
  // Serial.print(",");
  // Serial.print(comm_data.goal_pwm[1]);
  // Serial.print(",");
  // Serial.print(comm_data.goal_pwm[2]);
  // Serial.print(",");
  // Serial.println(comm_data.goal_pwm[3]);

  motors.printEncoder_All();


  Serial.println(mot3.read_enc());


  Serial.println();
 
  delay(15);
}



  #endif
