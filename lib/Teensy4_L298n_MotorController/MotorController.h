#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <MotorController_Pins.h>
#include <Arduino.h>
#include <Encoder.h>
#include <vector>
#include <ArduinoJson.h>


#define MILLIS_PER_MIN 60000.0 //1000 millis/sec * 60 sec/min

enum Command{
        NONE,
        PWM_DIRECT,
        POS_PID,
        VEL_PID,
        PID_VARS_POS_ALL,
        PID_VARS_VEL_ALL
};

struct Motor_Struct{
    int EN;
    int PWM1;
    int PWM2;
    int SENSE;
    int encA;
    int encB;
    float ticks_per_rotation;
};




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
        void assignSetpoint_pwm(int setpoint);
        void assignSetpoint_pos(int setpoint);
        void assignSetpoint_vel(float setpoint);

        int getSetpoint_pwm();
        int getSetpoint_pos();
        float getSetpoint_vel();
        
        void getSetpoint_ROS();

        float getVelocity();


        float pid_position(int setpoint);
        float pid_velocity(float setpoint);

        void pid_position_setpoint();
        float pid_velocity_setpoint();

        void setPID_vars_pos(float kP, float kI, float kD);
        void setPID_vars_vel(float kP, float kI, float kD);


        void update_PID_Pos(int goal);
        float update_PID_Vel(float setpoint);
        void setPIDUpdateRate(float millis);

        void update_PID_Pos_setpoint(); //uses internal setpoint
        void update_PID_Vel_setpoint(); //Uses internal setpoint

    private:

        //Encoder Objects
        Encoder encoder;

        //Motor Vars
        int _EN;
        int _PWM1;
        int _PWM2;
        int _SENSE;
        int _encA;
        int _encB;
        float  _ticks_per_rot;
        int _enc_count;

        //Setpoint Vars
        int _setpoint_pwm;
        int _setpoint_pwm_old;

        int _setpoint_pos;
        int _setpoint_pos_old;

        float _setpoint_vel;
        float _setpoint_vel_old;


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

class Message_Parser{
    public:

        struct Comm_Data{
            Command command = NONE;

            String rx_str;
            StaticJsonDocument<1024> rx_json;
            String tx_str;
            StaticJsonDocument<1024> tx_json;

            float kP_pos[4]; //For each motor
            float kI_pos[4];
            float kD_pos[4];
            int goal_position[4] = {0,0,0,0};
            int pos_feedback[4];

            float kP_vel[4]; //For each motor
            float kI_vel[4];
            float kD_vel[4];
            float goal_velocity[4] = {0,0,0,0};
            float velocity_feedback[4];

            int goal_pwm[4] = {0,0,0,0};             
        };
};

class MotorController{
    public:
        MotorController(const Motor &mot0, const Motor &mot1, const Motor &mot2, const Motor &mot3);
        size_t addMotor(const Motor &motor); //Adds motor to motors vector returns size of motors vector
        void initAllMotors();
        void enableAllMotors();
        void disableAllMotors();
        void run_motor(int motor_num, int pwm); //runs a specfic motor in the motors array at given pwm
        void run_motors_setpoint_pwm(); //Run all motors at given PWM setpoints
        void run_pid_pos_all();
        void run_pid_vel_all(); //uses internal setpoint update for velocity goals on motors
        void drive_all(int pwm);

        void assignSetpoints_pwm(float setpoints[4]);
        void assignSetpoints_vel(float setpoints[4]);
        void assignSetpoints_pos(int setpoints[4]);

        void updatePID_pos();
        void updatePID_vel();

        int numMotors();
        void assignPIDvars_all_pos(float kP, float kI, float kD);
        void assignPIDvars_all_vel(float kP, float kI, float kD);
        void assignPIDupdate_all(float freq);
        void parse_data(Message_Parser::Comm_Data& data);
        void prepare_feedback_data(Message_Parser::Comm_Data& data);
        void run_controller(Message_Parser::Comm_Data& data);

        int* getEncoder_all();
        void printEncoder_All();

        std::vector<Motor> motors;


    private:
        int motor_count = 0;
        volatile float _pid_update_freq;
        float _last_update_time;
};

class Serial_Comms{
    public:
            Serial_Comms(int baudrate, float timeout);
            void init();
            void check_for_data(Message_Parser::Comm_Data& data);
            void send_feedback_data(Message_Parser::Comm_Data& data);

    private:
            int _baudrate;
            float _timeout;

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