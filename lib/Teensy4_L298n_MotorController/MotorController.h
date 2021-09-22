#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

    #include <MotorController_Pins.h>
    #include <Arduino.h>
    #include <Encoder.h>
    #include <ros.h>
    #include <FreeRTOS_TEENSY4.h>



    class Motor{
        public:
            Motor(int EN, int PWM1, int PWM2, int SENSE, int encA, int encB);

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
            void pid_position();

            void setPID_vars(float kP, float kI, float kD);

            int _enc_count;

            int _newSet;
            int _oldSet;

            int _setpoint;



            Encoder encoder;

            
        

        private:
            int _EN;
            int _PWM1;
            int _PWM2;
            int _SENSE;
            int _encA;
            int _encB;

            //PID Vars
            float volatile _currentTime, _previousTime, _elapsedTime, _error, _cumError, _rateError, _lastError;
            float _kP, _kI, _kD;

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

#endif