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

#define GEAR_RATIO 131
#define COUNT_PER_ROT_ENC 16
#define COUNT_PER_ROT GEAR_RATIO*COUNTPER_ROT_ENC

Motor mot0(MOT0_EN,MOT0_PWM1,MOT0_PWM2,SENSE0,ENC0_A,ENC0_B); 


// MotorController motor_controller;

// static const int LED_PIN = LED_BUILTIN;

TaskHandle_t get_setpoint_handle;
TaskHandle_t motor_controller_handle;


void test_task(void *param){
  while(1){
    Serial.println("test");
    vTaskDelay(200/portTICK_PERIOD_MS);
 }
}

void get_setpoint(void *param){
  while(1){
    // vTaskSuspend(motor_controller_handle);
    // for(int i = 0; i < 10; i++){
    //   mot0._setpoint = (i*25);
    //   vTaskDelay(1000/portTICK_PERIOD_MS);
    //   Serial.println(mot0.getSetpoint());
    // }

    if(Serial.available() > 0){
      mot0.setSetpoint(Serial.parseInt());
      vTaskDelay(150/portTICK_PERIOD_MS);
    }

    else; // vTaskDelay(150/portTICK_PERIOD_MS);
    // vTaskResume(motor_controller_handle);

  }

}

void motor_controller_task(void *param){
  while(1){
    // Serial.println("motor task");
    // mot0.pid_position();

    vTaskSuspend(get_setpoint_handle);

    mot0.drive_motor_setpoint();

    // Serial.println("hey fuck you");

    // vTaskDelay(10/portTICK_PERIOD_MS);
    // Serial.println(mot0.read_enc());
    
    vTaskDelay(100/portTICK_PERIOD_MS);

    vTaskResume(get_setpoint_handle);
 }
}

void setup() {
  Serial.begin(115200);

  mot0.init_motor();
  mot0.enable_motor();
  // mot0.setPID_vars(1.0,0.0,0.0);

  
  //Task to run forever
  xTaskCreate( //create task
    motor_controller_task, //Function to be called
    "motor_controller_task", //Task Name
    1000, //Stack size (Words in freeRTOS)
    NULL, //Param to pass to function
    1, // Task Priority
    &motor_controller_handle   //Task Handle
  );


xTaskCreate( //create task
  get_setpoint, //Function to be called
  "setpoint_task", //Task Name
  1000, //Stack size (Words in freeRTOS)
  NULL, //Param to pass to function
  2, // Task Priority
  &get_setpoint_handle //Task Handle
  );


// xTaskCreate( //create task
//   test_task, //Function to be called
//   "test_task", //Task Name
//   500, //Stack size (Words (bytes) in freeRTOS)
//   NULL, //Param to pass to function
//   0, // Task Priority
//   NULL //Task Handle
// );


  vTaskStartScheduler();

}

void loop() {

}