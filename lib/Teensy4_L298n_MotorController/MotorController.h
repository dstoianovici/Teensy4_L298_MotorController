#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <MotorController_Pins.h>
#include <Arduino.h>
#include <Encoder.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <vector>

#define MILLIS_PER_MIN 60000.0 //1000 millis/sec * 60 sec/min



class Motor{
    public:
        Motor(int EN, int PWM1, int PWM2, int SENSE, int encA, int encB, float ticks_per_rotation);

        void init_motor();
        void drive_motor(int pwm_duty_cycle);
        void drive_motor_setpoint();
        int read_enc();
        void enable_motor();
        void disable_motor();
        void brake_motor(int brake_power);
        void setSetpoint(int setpoint);
        int getSetpoint();
        void getSetpoint_ROS();
        std_msgs::Float32 pid_position(int setpoint);


        float getVelocity();
        float pid_velocity(float setpoint);

        void setPID_vars(float kP, float kI, float kD);

        void update_PID(int goal);
        float update_PID_Vel(float setpoint);
        void setPIDUpdateRate(float millis);



        int _enc_count;

        int _newSet;
        int _oldSet;

        int _setpoint;



        Encoder encoder;


    private:

        //Motor Vars
        int _EN;
        int _PWM1;
        int _PWM2;
        int _SENSE;
        int _encA;
        int _encB;
        float  _ticks_per_rot;

        //PID Vars
        volatile float _currentTime, _previousTime, _elapsedTime, _error, _cumError, _rateError, _lastError;
        float _kP, _kI, _kD;


        //Velocity calc Vars
        volatile float _currentTime_vel, _previousTime_vel;
        volatile int _encoderPast_vel, _encoderCurrent_vel;  


        //Vars for PID update Rate
        volatile float _updateTime_PID,_currentTimeUpdate,_previousTimeUpdate;

        

        
};

class MotorController{
    public:
        MotorController();

        void addMotor(Motor* motor);
        void run_motor(int motor, int speed);

    private:
        int motor_count = 0;
        Motor** motor_array;
};


class Communicator{
    public:
        Communicator(int baudrate);
        virtual void init();

        struct Comm_Data{
            float kP_pos[4]; //For each motor
            float kI_pos[4];
            float kD_pos[4];
            int goal_position[4] = {0,0,0,0};
            int pos_error[4];

            float kP_vel[4]; //For each motor
            float kI_vel[4];
            float kD_vel[4];
            float goal_velocity[4];
            float velocity_error[4];             
        };

    private:
        int _baudrate;

};

class Serial_Comms : private Communicator , public Motor{
    public:

    private:

};

class ROS_Comms : private Communicator, public Motor{
    public:

    private:

};

#endif