#include <Arduino.h>
#include <MotorController_Pins.h>
#include <MotorController.h>

#define GEAR_RATIO 131
#define COUNT_PER_ROT_ENC 16
#define COUNT_PER_ROT GEAR_RATIO*COUNTPER_ROT_ENC

Motor mot0(MOT0_EN,MOT0_PWM1,MOT0_PWM2,SENSE0,ENC0_A,ENC0_B); 

MotorController motor_controller;

void setup() {
  Serial.begin(115200);

  mot0.init_motor();
  mot0.enable_motor();

  motor_controller.addMotor(&mot0);

}

void loop() {
 
//  mot0.drive_motor(-100);
//  delay(2000);
//  Serial.println(mot0.read_enc());

//  mot0.drive_motor(100);
//  delay(2000);
//  Serial.println(mot0.read_enc());

motor_controller.run_motor(0,100);


}