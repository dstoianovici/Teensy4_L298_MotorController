#include <MotorController.h>


//Explicit Constructor
Motor::Motor(int EN, int PWM1, int PWM2, int SENSE, int encA, int encB) : encoder(encA,encB){
    _EN = EN;
    _PWM1 = PWM1;
    _PWM2 = PWM2;
    _SENSE = SENSE;

    //default PID for testing

    _kP = 1.0;
    _kI = 0.0;
    _kD = 0.0;
   

    _enc_count = 0;
    _previousTimeUpdate = 0;
}

void Motor::init_motor(){
    pinMode(_EN,OUTPUT);
    pinMode(_PWM1,OUTPUT);
    pinMode(_PWM2,OUTPUT);
    //pinMode(_SENSE,INPUT); //V1.0 has incorrectly setup OP-AMP circuit
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

void Motor::drive_motor_setpoint(){
    int pwm_duty_cycle = _setpoint;
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
    _enc_count = encoder.read();
    return _enc_count;
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

void Motor::setSetpoint(int setpoint){
    _setpoint = setpoint;
}

int Motor::getSetpoint(){
    return _setpoint;
}

void Motor::setPID_vars(float kP, float kI, float kD){
    _kP = kP;
    _kI = kI;
    _kD = kD;
}

std_msgs::Float32 Motor::pid_position(int setpoint){
  _currentTime = millis();
  _elapsedTime = (_currentTime - _previousTime)/1000.0;
  _error = float(setpoint - read_enc());


  std_msgs::Float32  error_msg;
//   error_msg.data = abs(_error);


  float output = 0.0;

  if(abs(_error) > 0.0){
    _cumError += _error*_elapsedTime;
    _rateError = (_error-_lastError)/_elapsedTime;

    output = _kP*_error + _kI*_cumError + _kD*_rateError;

    error_msg.data = output;
    
    if(output > 255.0) output = 255.0;

    drive_motor((int)output); 

    _previousTime = _currentTime;
    _lastError = _error;
  }

  else{
    drive_motor(0);
  }

return error_msg;

}


void Motor::update_PID(int goal){
    _currentTime = millis();

    if(_currentTimeUpdate - _previousTimeUpdate >= _updateTime_PID){
        pid_position(goal);
        _previousTimeUpdate = millis();
    }

    else;

}

void Motor::setPIDUpdateRate(float millis){
    _updateTime_PID = millis;
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

