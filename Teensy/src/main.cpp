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


#include <MotorController_Pins.h>
#include <MotorController.h>

#define SERIAL_COM

#define GEAR_RATIO 20
#define COUNT_PER_ROT_ENC 12
#define COUNT_PER_ROT GEAR_RATIO*COUNT_PER_ROT_ENC

#define BAUD 115200
#define TIMEOUT 0.5

Message_Parser::Comm_Data comm_data;


Motor mot0(MOT0_EN,MOT0_PWM1,MOT0_PWM2,SENSE0,ENC0_A,ENC0_B, COUNT_PER_ROT);
Motor mot1(MOT1_EN,MOT1_PWM1,MOT1_PWM2,SENSE0,ENC1_A,ENC1_B, COUNT_PER_ROT); 
Motor mot2(MOT2_EN,MOT2_PWM1,MOT2_PWM2,SENSE0,ENC2_A,ENC2_B, COUNT_PER_ROT); 
Motor mot3(MOT3_EN,MOT3_PWM1,MOT3_PWM2,SENSE0,ENC3_A,ENC3_B, COUNT_PER_ROT);

MotorController motors(mot0,mot1,mot2,mot3);







#ifdef SERIAL_COM
Serial_Comms serial_comms(BAUD,TIMEOUT);
#endif



void setup() {

  #ifdef SERIAL_COM

    serial_comms.init();

    
  #endif


  mot0.init_motor();
  mot0.enable_motor();
  mot0.setPID_vars_pos(1.0,0.00,0.0);
  mot0.setPID_vars_vel(1.0,0.00,0.0);

  mot1.init_motor();
  mot1.enable_motor();
  mot1.setPID_vars_pos(1.0,0.00,0.0);
  mot1.setPID_vars_vel(1.0,0.00,0.0);

  mot2.init_motor();
  mot2.enable_motor();
  mot2.setPID_vars_pos(1.0,0.0,0.0);
  mot2.setPID_vars_vel(1.0,0.0,0.0);

  mot3.init_motor();
  mot3.enable_motor();
  mot3.setPID_vars_pos(1.0,0.0,0.0);
  mot3.setPID_vars_vel(1.0,0.0,0.0);


  
}

void loop() {


  #ifdef SERIAL_COM

  serial_comms.check_for_data(comm_data);
 
  switch(comm_data.command){
      default:
        break;

      case NONE:

        mot0.drive_motor(0);
        mot1.drive_motor(0);
        mot2.drive_motor(0);
        mot3.drive_motor(0);

        comm_data.pos_feedback[0] = mot0.read_enc();
        comm_data.pos_feedback[1] = mot1.read_enc();
        comm_data.pos_feedback[2] = mot2.read_enc();
        comm_data.pos_feedback[3] = mot3.read_enc();

        comm_data.velocity_feedback[0] = mot0.getVelocity();
        comm_data.velocity_feedback[1] = mot1.getVelocity();
        comm_data.velocity_feedback[2] = mot2.getVelocity();
        comm_data.velocity_feedback[3] = mot3.getVelocity();

        break;

      case PWM_DIRECT:

        mot0.drive_motor(comm_data.goal_pwm[0]);
        mot1.drive_motor(comm_data.goal_pwm[1]);
        mot2.drive_motor(comm_data.goal_pwm[2]);
        mot3.drive_motor(comm_data.goal_pwm[3]);

        comm_data.pos_feedback[0] = mot0.read_enc();
        comm_data.pos_feedback[1] = mot1.read_enc();
        comm_data.pos_feedback[2] = mot2.read_enc();
        comm_data.pos_feedback[3] = mot3.read_enc();

        comm_data.velocity_feedback[0] = mot0.getVelocity();
        comm_data.velocity_feedback[1] = mot1.getVelocity();
        comm_data.velocity_feedback[2] = mot2.getVelocity();
        comm_data.velocity_feedback[3] = mot3.getVelocity();

        break;

      case POS_PID:

        comm_data.velocity_feedback[0] = mot0.getVelocity();
        comm_data.velocity_feedback[1] = mot1.getVelocity();
        comm_data.velocity_feedback[2] = mot2.getVelocity();
        comm_data.velocity_feedback[3] = mot3.getVelocity();



        comm_data.pos_feedback[0] = mot0.pid_position(comm_data.goal_position[0]);
        comm_data.pos_feedback[1] = mot1.pid_position(comm_data.goal_position[1]);
        comm_data.pos_feedback[2] = mot2.pid_position(comm_data.goal_position[2]);
        comm_data.pos_feedback[3] = mot3.pid_position(comm_data.goal_position[3]);

        break;
      
      case VEL_PID:

        comm_data.pos_feedback[0] = mot0.read_enc();
        comm_data.pos_feedback[1] = mot1.read_enc();
        comm_data.pos_feedback[2] = mot2.read_enc();
        comm_data.pos_feedback[3] = mot3.read_enc();

        comm_data.velocity_feedback[0] = mot0.pid_velocity(comm_data.goal_velocity[0]);
        comm_data.velocity_feedback[1] = mot1.pid_velocity(comm_data.goal_velocity[1]);
        comm_data.velocity_feedback[2] = mot2.pid_velocity(comm_data.goal_velocity[2]);
        comm_data.velocity_feedback[3] = mot3.pid_velocity(comm_data.goal_velocity[3]);

        break;
  }

  serial_comms.send_feedback_data(comm_data);



  delay(150);
}



  #endif
