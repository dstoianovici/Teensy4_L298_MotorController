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
#define COUNT_PER_ROT GEAR_RATIO*COUNTPER_ROT_ENC

Motor mot0(MOT0_EN,MOT0_PWM1,MOT0_PWM2,SENSE0,ENC0_A,ENC0_B); 



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

#ifdef SERIAL_COM

#endif


void setup() {

  #ifdef SERIAL_COM

    Serial.begin(9600);
    Serial.setTimeout(1.0);
    // pinMode(13, OUTPUT);

    
  #endif


  #ifdef ROS

    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(position_goal_sub);
    nh.advertise(enc_feedback_pub);
    nh.advertise(error_string_pub);
  
  #endif


  mot0.init_motor();
  mot0.enable_motor();

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

  #ifdef SERIAL_COM

 String msg;
 StaticJsonDocument<512> msg_doc;
 int speed = 0;

 if (Serial.available() > 0){ 
   msg = Serial.readStringUntil('\n');
 }

 DeserializationError   error = deserializeJson(msg_doc, msg);

  if (error) {
    Serial.println(error.c_str()); 
    return;
  }

  speed = msg_doc["Speed"];

  

  mot0.drive_motor(speed);
//   if (msg_doc["This"] == "1") {
//      Serial.println("{\"Success\":\"True\"}");
//   }
//   else {
//       Serial.println("{\"Success\":\"False\"}");
//    }

  delay(50);

}



  #endif
