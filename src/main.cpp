/**
 * Author: Dan Stoianovici
 * Email: stoianovici.dan.s@gmail.com
 * License: MIT
 * 
 *  ROS Enabled Motor Controller for Teensy4.0 Motor Controller Board

**/


#include <Arduino.h>
#include <MotorController_Pins.h>
#include <MotorController.h>
#include <ros.h>
#include <FreeRTOS_TEENSY4.h>

#define GEAR_RATIO 131
#define COUNT_PER_ROT_ENC 16
#define COUNT_PER_ROT GEAR_RATIO*COUNTPER_ROT_ENC

Motor mot0(MOT0_EN,MOT0_PWM1,MOT0_PWM2,SENSE0,ENC0_A,ENC0_B); 

// MotorController motor_controller;

// static const int LED_PIN = LED_BUILTIN;


// void test_task(void *param){
//   while(1){
//     Serial.println("test");
//     vTaskDelay(200/portTICK_PERIOD_MS);
//  }
// }

// void get_setpoint(void *param){
//   while(1){
//     mot0._oldSet = mot0.getSetpoint();
//     Serial.print("Setpoint Old: ");
//     Serial.print(mot0._oldSet);
//     Serial.println();

//     if(Serial.available() > 0){
//       mot0.setSetpoint(Serial.parseInt());
//     }
//     else{
//       mot0.setSetpoint(mot0._oldSet);
//     }

//     Serial.print("Setpoint: ");
//     Serial.print(mot0.getSetpoint());
//     Serial.println();

//     vTaskDelay(150/portTICK_PERIOD_MS);
//   }
// }

// void motor_controller_task(void *param){
//   while(1){
//     Serial.println("motor task");
//     //mot0.pid_position();
//     vTaskDelay(100/portTICK_PERIOD_MS);
//  }
// }

void setup() {
  Serial.begin(115200);

  mot0.init_motor();
  mot0.enable_motor();
  // mot0.setPID_vars(1.0,0.0,0.0);

  
  // //Task to run forever
  // xTaskCreate( //create task
  //   motor_controller_task, //Function to be called
  //   "motor_controller_task", //Task Name
  //   1000, //Stack size (Words in freeRTOS)
  //   NULL, //Param to pass to function
  //   1, // Task Priority
  //   NULL //Task Handle
  //   );

  // xTaskCreate( //create task
  //   get_setpoint, //Function to be called
  //   "setpoint_task", //Task Name
  //   1000, //Stack size (Words in freeRTOS)
  //   NULL, //Param to pass to function
  //   2, // Task Priority
  //   NULL //Task Handle
  //   );



  // xTaskCreate( //create task
  //   test_task, //Function to be called
  //   "test_task", //Task Name
  //   500, //Stack size (Words (bytes) in freeRTOS)
  //   NULL, //Param to pass to function
  //   0, // Task Priority
  //   NULL //Task Handle
  //   );


  // vTaskStartScheduler();

}

void loop() {
 
 mot0.drive_motor(-100);
 delay(2000);
 Serial.println(mot0.read_enc());

 mot0.drive_motor(100);
 delay(2000);
 Serial.println(mot0.read_enc());

// motor_controller.run_motor(0,100);

}