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
Motor mot1(MOT1_EN,MOT1_PWM1,MOT1_PWM2,SENSE0,ENC1_A,ENC1_B); 
Motor mot2(MOT2_EN,MOT2_PWM1,MOT2_PWM2,SENSE0,ENC2_A,ENC2_B); 
Motor mot3(MOT3_EN,MOT3_PWM1,MOT3_PWM2,SENSE0,ENC3_A,ENC3_B); 




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


 int speed0 = 0;
 int speed1 = 0;
 int speed2 = 0;
 int speed3 = 0;

void setup() {

  #ifdef SERIAL_COM

    Serial.begin(115200);
    Serial.setTimeout(0.5);
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

  // mot1.init_motor();
  // mot1.enable_motor();

  mot2.init_motor();
  mot2.enable_motor();
  
  mot3.init_motor();
  mot3.enable_motor();
  

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

 String rx_msg_str;
 StaticJsonDocument<512> rx_msg;

 StaticJsonDocument<512> tx_doc;
 String tx_msg;

 if (Serial.available() > 0){
   rx_msg_str = Serial.readStringUntil('\n');

   DeserializationError   error = deserializeJson(rx_msg, rx_msg_str);
   if (error) Serial.println(error.c_str()); 

   if(rx_msg["command"] == "pwm_direct"){
     speed0 = rx_msg["pwm0"];
     speed2 = rx_msg["pwm2"];
     speed3 = rx_msg["pwm3"];
   }

 }


  mot0.drive_motor(speed0);
  mot2.drive_motor(speed2);
  mot3.drive_motor(speed3);

  tx_doc["msg_type"] = "encoder_pos";
  tx_doc["enc0"] = mot0.read_enc();
  tx_doc["enc1"] = mot1.read_enc();
  tx_doc["enc2"] = mot2.read_enc();
  tx_doc["end3"] = mot3.read_enc();
  serializeJson(tx_doc,tx_msg);
  Serial.println(tx_msg);
  Serial.flush();
  tx_doc.clear();


  delay(15);

}



  #endif
