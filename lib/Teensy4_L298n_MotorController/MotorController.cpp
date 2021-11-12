#include <MotorController.h>


//Explicit Constructor
Motor::Motor(int EN, int PWM1, int PWM2, int SENSE, int encA, int encB, float ticks_per_rot) : encoder(encA,encB){
    _EN = EN;
    _PWM1 = PWM1;
    _PWM2 = PWM2;
    _SENSE = SENSE;

    _ticks_per_rot = ticks_per_rot;   

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

void Motor::assignSetpoint(int setpoint){
    _setpoint = setpoint;
}

int Motor::getSetpoint(){
    return _setpoint;
}

void Motor::setPID_vars_pos(float kP, float kI, float kD){
    _kP_pos = kP;
    _kI_pos = kI;
    _kD_pos = kD;
}

void Motor::setPID_vars_vel(float kP, float kI, float kD){
    _kP_vel = kP;
    _kI_vel = kI;
    _kD_vel = kD;
}

float Motor::pid_position(int setpoint){
  _currentTime = millis();
  int position = read_enc();
  _elapsedTime = (_currentTime - _previousTime)/1000.0;
  _error = float(setpoint - position);


  float output = 0.0;

  if(abs(_error) > 0.0){
    _cumError += _error*_elapsedTime;
    _rateError = (_error-_lastError)/_elapsedTime;

    output = _kP_pos*_error + _kI_pos*_cumError + _kD_pos*_rateError;
    
    if(output > 255.0) output = 255.0;

    drive_motor((int)output); 

    _previousTime = _currentTime;
    _lastError = _error;
  }

  else{
    drive_motor(0);
  }

return position;
}

float Motor::getVelocity(){
    _currentTime_vel = millis();
    _encoderCurrent_vel = read_enc();

    float deltaTime = (_currentTime_vel - _previousTime_vel)/MILLIS_PER_MIN;
    float deltaEncoder = (_encoderCurrent_vel - _encoderPast_vel)/_ticks_per_rot;

    _encoderPast_vel = _encoderCurrent_vel;
    _previousTime_vel = _currentTime_vel;

    return deltaEncoder/deltaTime; //RPM
}

float Motor::pid_velocity(float setpoint){
  _currentTime = millis();
  float vel = getVelocity();
  _elapsedTime = (_currentTime - _previousTime)/1000.0;
  _error = setpoint - vel;
  if(setpoint != _setpoint_old) _cumError = 0;


  float output = 0.0;

  if(setpoint == 0) drive_motor(0);

  else if(abs(_error) > 0.0){
    _cumError += _error*_elapsedTime;
    _rateError = (_error-_lastError);

    output = _kP_vel*_error + _kI_vel*_cumError + _kD_vel*_rateError;
    
    if(output > 255.0) output = 255.0;

    drive_motor((int)output); 

    _previousTime = _currentTime;
    _lastError = _error;
    _setpoint_old = setpoint;
  }

  else drive_motor(0);


  return vel;
}

void Motor::update_PID_Pos(int goal){
    _currentTime = millis();

    if(_currentTimeUpdate - _previousTimeUpdate >= _updateTime_PID){
        pid_position(goal);
        _previousTimeUpdate = millis();
    }

    else;

}

float Motor::update_PID_Vel(float setpoint){
    float vel = getVelocity();
    _currentTime = millis();;
    if(_currentTimeUpdate - _previousTimeUpdate >= _updateTime_PID){
        vel = pid_velocity(setpoint);
        _previousTimeUpdate = millis();
    }

    return vel;
}

void Motor::setPIDUpdateRate(float millis){
    _updateTime_PID = millis;
}

void Motor::assignSetpoint_pos(int setpoint){
    _setpoint_pos = setpoint;
}

void Motor::assignSetpoint_vel(float setpoint){
    _setpoint_vel = setpoint;
}


float Motor::pid_velocity_setpoint(){
  _currentTime = millis();
  float vel = getVelocity();
  _elapsedTime = (_currentTime - _previousTime)/1000.0;
  _error = _setpoint_vel - vel;
  if(_setpoint_vel != _setpoint_vel_old) _cumError = 0;


  float output = 0.0;

  if(_setpoint_vel == 0) drive_motor(0);

  else if(abs(_error) > 0.0){
    _cumError += _error*_elapsedTime;
    _rateError = (_error-_lastError);

    output = _kP_vel*_error + _kI_vel*_cumError + _kD_vel*_rateError;
    
    if(output > 255.0) output = 255.0;

    drive_motor((int)output); 

    _previousTime = _currentTime;
    _lastError = _error;
    _setpoint_vel_old = _setpoint_vel;
  }

  else drive_motor(0);


  return vel;
}




///////////////////Motor Controller///////////////////
MotorController::MotorController(Motor &_mot0, Motor &_mot1, Motor &_mot2, Motor &_mot3){
    addMotor(_mot0);
    addMotor(_mot1);
    addMotor(_mot2);
    addMotor(_mot3);
}

size_t MotorController::addMotor(Motor &motor){
    motors.push_back(motor);
    return motors.size();
}

void MotorController::run_motor(int motor_num, int pwm){
    motors[motor_num].drive_motor(pwm);
}

void MotorController::run_motors_setpoint(){
    for(uint8_t i = 0; i<motors.size(); i++){
        motors[i].drive_motor_setpoint();
    }
}

void MotorController::update_pid_vel_all(){
    for(uint8_t i = 0; i < numMotors(); i++){
        motors[i].pid_velocity_setpoint();
    }
}

void MotorController::updateSetpoints(float setpoints[4]){
    for(uint8_t i = 0; i<motors.size(); i++){
        motors[i].assignSetpoint(setpoints[i]);
    }
}

void MotorController::updateSetpoints_vel(float setpoints[4]){
    for(uint8_t i = 0; i<numMotors(); i++){
        motors[i].assignSetpoint_vel(setpoints[i]);
    }
}

void MotorController::updateSetpoints_pos(int setpoints[4]){
    for(uint8_t i = 0; i<numMotors(); i++){
        motors[i].assignSetpoint_pos(setpoints[i]);
    }
}

int MotorController::numMotors(){
    return motors.size();
}

void MotorController::assignPIDvars_all_pos(float kP, float kI, float kD){
    for(uint8_t i = 0; i < numMotors(); i++){
        motors[i].setPID_vars_pos(kP,kI,kD);
    }
}

void MotorController::assignPIDvars_all_vel(float kP, float kI, float kD){
    for(uint8_t i = 0; i < numMotors(); i++){
        motors[i].setPID_vars_vel(kP,kI,kD);
    }
}









// ///////////////Communicator///////////////////////////
// ROS_Comms::ROS_Comms(ros::NodeHandle &nh,int baudrate){
//     _baudrate = baudrate;
//     _nh = nh;
    
//     _nh.getHardware()->setBaud(115200);
//     _nh.initNode();
//     _nh.subscribe(_pid_config_sub);
//     _nh.subscribe(_setpoints_sub);
//     _nh.advertise(_feedback_pub);

// }