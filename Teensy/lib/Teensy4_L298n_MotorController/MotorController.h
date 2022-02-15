#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <MotorController_Pins.h>
#include <Arduino.h>
#include <Encoder.h>
#include <vector>
#include <ArduinoJson.h>

#define ROS
// #define PYTHON_SERIAL

#ifdef ROS
  #include <open_motor_msgs/feedback.h>
  #include <open_motor_msgs/pid_config.h>
  #include <open_motor_msgs/setpoints.h>
#endif

#define MILLIS_PER_MIN 60000.0 //1000 millis/sec * 60 sec/min

enum Command{
    NONE,
    PWM_DIRECT,
    POS_PID,
    VEL_PID,
    PID_VARS_POS_ALL,
    PID_VARS_VEL_ALL
};


//This class defines the parameters and behaviors of a single motor. 
class Motor{ 
    public:

        /**
         * Motor constructor for initialization with direct params.
         @param EN enable hw pin for motor.
         @param PWM1 pwm1 hw pin for motor.
         @param PWM2 pwm2 hw pin for motor.
         @param SENSE current sense hw pin for motor.
         @param encA encoder channel A for quadrature encoder.
         @param encB encoder channel B for quadrature encoder.
         @param ticks_per_rotation how many encoder ticks per one output shaft rotation for the motor.
        */
        Motor(int EN, int PWM1, int PWM2, int SENSE, int encA, int encB, float ticks_per_rotation);

        /**
         * Motor constructor for initialization with motor_struct.
         @param motor motor_struct containing motor details
        */
        Motor(motor_struct motor);


        /**
         Destructor
        */
        ~Motor::Motor();

        /**
          Setup microcontroller pins.
        */
        void init_motor();

        /**
          Drive motor at specified PWM, negative PWM will change direction.
          @param pwm_duty_cycle PWM to write to motor.
        */
        void drive_motor(int pwm_duty_cycle);

        /**
          Drive motor with internal PWM Setpoint.
        */
        void drive_motor_setpoint();

        /**
          Return the current encoder position.
        */
        int read_enc();

        /**
          return the current motor velocity.
        */
        float getVelocity();

        /**
          Set motor enable pin to on.
        */
        void enable_motor();

        /**
          Set motor enable pin to off.
        */
        void disable_motor();

        /**
          Write pwm to both motor channels resulting in braking.
          @param brake_power 0-255 PWM braking power. 
        */
        void brake_motor(int brake_power);

        /**
          Change motor pwm setpoint
          @param setpoint motor pwm setpoint
        */
        void assignSetpoint_pwm(int setpoint);

        /**
          Change motor position setpoint in encoder ticks
          @param setpoint motor position setpoint in encoder ticks
        */
        void assignSetpoint_pos(int setpoint);

        /**
          Change motor velocity setpoint in output shaft RPM
          @param setpoint motor vel setpoint in rpm
        */
        void assignSetpoint_vel(float setpoint);

        /**
          Return internal PWM setpoint
        */
        int getSetpoint_pwm();

        /**
          Return internal position setpoint
        */
        int getSetpoint_pos();

        /**
          Return internal velocity setpoint
        */
        float getSetpoint_vel();

        /**
          Run one loop of position PID with specified setpoint
          @param setpoint encoder tick position setpoint
        */
        int pid_position(int setpoint);

        /**
          Run one loop of velocity PID with specified setpoint
          @param setpoint RPM velocity setpoint
        */
        float pid_velocity(float setpoint);

        /**
          Run one loop of position PID with internal setpoint
        */
        void pid_position_setpoint();

        /**
          Run one loop of velocity PID with internal setpoint
        */
        float pid_velocity_setpoint();

        /**
          Set PID vars for position PID
          @param kP Proportional Error Constant
          @param kI Integral Error Constant
          @param kD Derivative Error Constant
        */
        void setPID_vars_pos(float kP, float kI, float kD);

        /**
          Set PID vars for velocity PID
          @param kP Proportional Error Constant
          @param kI Integral Error Constant
          @param kD Derivative Error Constant
        */
        void setPID_vars_vel(float kP, float kI, float kD);

        /**
          Run position PID at time interval specified by setPIDUpdateRate
          @param goal Position goal in encoder ticks
        */
        void update_PID_Pos(int goal);

        /**
          Run position PID at time interval specified by setPIDUpdateRate
          @param setpoint Velocity setpoint in encoder ticks
        */
        float update_PID_Vel(float setpoint);

        /**
          Change the update delay for the update PID functions
          @param millis PID update rate specified in millis, Hz coming soon
        */
        void setPIDUpdateRate(float millis);

        /**
          Run position PID with internal setpoint at time interval specified by setPIDUpdateRate
        */
        void update_PID_Pos_setpoint();

        /**
          Run velocity PID with internal setpoint at time interval specified by setPIDUpdateRate
        */
        void update_PID_Vel_setpoint();

        /**
         Struct for passing motor params to motor param constructor
         @param EN enable hw pin for motor.
         @param PWM1 pwm1 hw pin for motor.
         @param PWM2 pwm2 hw pin for motor.
         @param SENSE current sense hw pin for motor.
         @param encA encoder channel A for quadrature encoder.
         @param encB encoder channel B for quadrature encoder.
         @param ticks_per_rotation how many encoder ticks per one output shaft rotation for the motor.
        */
        typedef struct motor_struct{
          int EN; 
          int PWM1;
          int PWM2;
          int SENSE;
          int encA;
          int encB;
          float ticks_per_rotation;
        }motor_struct;
        
       

    private:

        //Encoder Instance
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
        int _newSet;
        int _oldSet;
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


/**
  Message Parser Class to store Comm_Data struct and future parsers
*/
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


/**
  Motor Controller class manages all 4 motors on the board
*/
class MotorController : private Message_Parser, Motor{
    public:

        /**
          Class constructor is passed already created motors by reference
          @param mot0 Motor 0
          @param mot1 Motor 1
          @param mot2 Motor 2
          @param mot3 Motor 3
        */
        MotorController(Motor &mot0, Motor &mot1, Motor &mot2, Motor &mot3);
        
        /**
          Class constructor is used when instantiating motors within the class
          @param num_motors number of motors to be added to the controller
        */
        MotorController();

        /**
          Destructor
        */
        ~MotorController::MotorController();

        /**
         Adds a motor to the motors vector
         @param motor Motor to add.
         @return size of motor vector
        */
        size_t addMotor(Motor &motor); 

        /**
         instantiates a motor in the motor controller and adds it to motors vector
         @param motor Motor to add.
         @return size of motor vector
        */
        size_t addMotor(motor_struct motor);

        /**
          Initialize all motors in motor controller
        */
        void initAllMotors();

         /**
          Enable all motors in motor controller
        */
        void enableAllMotors();

        /**
          Disable all motors in motor controller
        */
        void disableAllMotors();

        /**
          Run specific motor at given PWM
          @param motor_num Motor number (0-3)
          @param pwm Goal PWM 
        */
        void run_motor(int motor_num, int pwm); //runs a specfic motor in the motors array at given pwm

        /**
          Run all motors at internally set PWM setpoint
        */
        void run_motors_setpoint_pwm();

        /**
          Run one position PID loop for all motors with internal setpoints
        */
        void run_pid_pos_all();

        /**
          Run one velocity PID loop for all motors with internal setpoints
        */
        void run_pid_vel_all();

        /**
          Assign pwm setpoints for all motors
          @param setpoints array of motor pwm setpoints, index corresponding to motor number
        */
        void assignSetpoints_pwm(float setpoints[4]);

        /**
          Assign velocity setpoints for all motors in RPM
          @param setpoints array of motor pwm setpoints, index corresponding to motor number
        */
        void assignSetpoints_vel(float setpoints[4]);

        /**
          Assign position setpoints for all motors in encoder ticks
          @param setpoints array of motor pwm setpoints, index corresponding to motor number
        */
        void assignSetpoints_pos(int setpoints[4]);


        /**
          Run position PID with internal setpoint at time interval specified by setPIDupdate_all for all motors
        */
        void updatePID_pos();

        /**
          Run velocity PID with internal setpoint at time interval specified by setPIDupdate_all for all motors
        */
        void updatePID_vel();

        /**
          Return number of motors in motor controller
        */
        int numMotors();

        /**
          Assign the same position PID variables to all motors
          @param kP Proportional Error Constant
          @param kI Integral Error Constant
          @param kD Derivative Error Constant
        */
        void assignPIDvars_all_pos(float kP, float kI, float kD);

        /**
          Assign the same velocity PID variables to all motors
          @param kP Proportional Error Constant
          @param kI Integral Error Constant
          @param kD Derivative Error Constant
        */
        void assignPIDvars_all_vel(float kP, float kI, float kD);

        /**
          Assign the same PID update rate to all motors specified as delay in millis
          @param millis
        */
        void assignPIDupdate_all(float millis);

        /**
          Parse incoming serial data into motor parameters
        */
        void parse_data();

        /**
          Prepare feedback data for serial comms
        */
        void prepare_feedback_data();

        /**
          Run the state machine to control all motors and switch between pwm, position, and velocity control
        */
        void run_controller();


    private:
        int motor_count = 0;
        std::vector<Motor> motors;
        Message_Parser::Comm_Data _data;
        volatile float _pid_update_freq;
        float _last_update_time;
};

class Serial_Comms : private Message_Parser{
    public:
            Serial_Comms(int baudrate, float timeout);
            void init();
            void check_for_data(Serial_Comms::Comm_Data& data);
            void send_feedback_data(Message_Parser::Comm_Data& data);

    private:
            Message_Parser::Comm_Data _data;
            int _baudrate;
            float _timeout;

};

class ROS_Comms : private Message_Parser{
    public:
        ros::NodeHandle nh;
        ros::Subscriber<open_motor_msgs::setpoints, ROS_Comms> _setpoints_sub;
        ros::Subscriber<open_motor_msgs::pid_config, ROS_Comms> _pid_config_sub;
        open_motor_msgs::feedback feedback;

        ROS_Comms(int baudrate);
        
        void setpoint_callback(const open_motor_msgs::setpoints setpoint_msg);
        void pid_config_callback(const open_motor_msgs::pid_config pid_vars);



    private:
        int _baudrate;
        ros::NodeHandle _nh;
        // open_motor_msgs::feedback feedback;

        ros::Subscriber<open_motor_msgs::setpoints> _setpoints_sub;
        ros::Subscriber<open_motor_msgs::pid_config> _pid_config_sub;
        ros::Publisher _feedback_pub;

};

#endif