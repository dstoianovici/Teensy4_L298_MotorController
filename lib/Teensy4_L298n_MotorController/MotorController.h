#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <MotorController_Pins.h>
#include <Arduino.h>
#include <Encoder.h>
#include <vector>
#include <ArduinoJson.h>


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
        void assignSetpoint(int setpoint);
        void assignSetpoint_pos(int setpoint);
        void assignSetpoint_vel(float setpoint);
        int getSetpoint();
        void getSetpoint_ROS();

        float getVelocity();


        float pid_position(int setpoint);
        float pid_velocity(float setpoint);

        float pid_position_setpoint();
        float pid_velocity_setpoint();

        void setPID_vars_pos(float kP, float kI, float kD);
        void setPID_vars_vel(float kP, float kI, float kD);


        void update_PID_Pos(int goal);
        float update_PID_Vel(float setpoint);
        void setPIDUpdateRate(float millis);

        void update_PID_Pos_setpoint(); //uses internal setpoint
        void update_PID_Vel_setpoint(); //Uses internal setpoint



        int _enc_count;

        int _newSet;
        int _oldSet;

        int _setpoint;
        int _setpoint_old;

        int _setpoint_pos;
        int _setpoint_pos_old;

        float _setpoint_vel;
        float _setpoint_vel_old;





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
        float _kP_pos, _kI_pos, _kD_pos;
        float _kP_vel, _kI_vel, _kD_vel;



        //Velocity calc Vars
        volatile float _currentTime_vel, _previousTime_vel;
        volatile int _encoderPast_vel, _encoderCurrent_vel;  


        //Vars for PID update Rate
        volatile float _updateTime_PID,_currentTimeUpdate,_previousTimeUpdate;

        

        
};

class MotorController{
    public:
        MotorController(Motor &mot0, Motor &mot1, Motor &mot2, Motor &mot3); //Motor &mot0, Motor &mot1, Motor &mot2, Motor &mot3

        size_t addMotor(Motor &motor); //Adds motor to motors vector returns size of motors vector
        void run_motor(int motor_num, int pwm); //runs a specfic motor in the motors array at given pwm
        void run_motors_setpoint();
        void update_pid_vel_all(); //uses internal setpoint update for velocity goals on motors
        void updateSetpoints(float setpoints[4]);
        void updateSetpoints_vel(float setpoints[4]);
        void updateSetpoints_pos(int setpoints[4]);
        int numMotors();
        void assignPIDvars_all_pos(float kP, float kI, float kD);
        void assignPIDvars_all_vel(float kP, float kI, float kD);



    private:
        int motor_count = 0;
        std::vector<Motor> motors;
};

class Message_Parser{
    public:
        Message_Parser();
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

};

class Serial_Comms : private Message_Parser , public Motor{
    public:

    private:

};

// class ROS_Comms : private Message_Parser{
//     public:
//         ROS_Comms(ros::NodeHandle &nh,int baudrate);
//         void setpoint_callback(const open_motor_msgs::setpoints setpoint_msg);
//         void pid_config_callback(const open_motor_msgs::pid_config pid_vars);



//     private:
//         int _baudrate;
//         ros::NodeHandle _nh;
//         // open_motor_msgs::feedback feedback;

//         ros::Subscriber<open_motor_msgs::setpoints> _setpoints_sub;
//         ros::Subscriber<open_motor_msgs::pid_config> _pid_config_sub;
//         ros::Publisher _feedback_pub;

// };

#endif