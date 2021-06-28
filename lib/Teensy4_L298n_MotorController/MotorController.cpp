#include <MotorController.h>


//Explicit Constructor
Motor::Motor(int EN, int PWM1, int PWM2, int SENSE, int encA, int encB) : encoder(encA,encB){
    _EN = EN;
    _PWM1 = PWM1;
    _PWM2 = PWM2;
    _SENSE = SENSE;
   

    enc_count = 0;
}

void Motor::init_motor(){
    pinMode(_EN,OUTPUT);
    pinMode(_PWM1,OUTPUT);
    pinMode(_PWM2,OUTPUT);
    //pinMode(_SENSE,INPUT);
}

void Motor::drive_motor(int pwm_duty_cycle){
    if(pwm_duty_cycle < 0){
        analogWrite(_PWM2,abs(pwm_duty_cycle));
        analogWrite(_PWM1,0);
    }

    else{
        analogWrite(_PWM1,pwm_duty_cycle);
        analogWrite(_PWM2,0); 
    }
}

int Motor::read_enc(){
    enc_count = encoder.read();
    return enc_count;
}

void Motor::enable_motor(){
    digitalWrite(_EN,HIGH);

}

void Motor::disable_motor(){
    digitalWrite(_EN,LOW);
}

void Motor::brake_motor(int brake_power){
    analogWrite(_PWM1,brake_power);
    analogWrite(_PWM2,brake_power);
}




///////////////////Motor Controller///////////////////
MotorController::MotorController(){};

void MotorController::addMotor(Motor* motor){
    motor_array[motor_count] = motor;
    motor_count++;
}

void MotorController::run_motor(int motor, int speed){
    motor_array[motor]->drive_motor(speed);
}