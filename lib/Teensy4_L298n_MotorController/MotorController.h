#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

    #include <MotorController_Pins.h>
    #include <Arduino.h>
    #include <Encoder.h>



    class Motor{
        public:
            Motor(int EN, int PWM1, int PWM2, int SENSE, int encA, int encB);

            void init_motor();
            void drive_motor(int pwm_duty_cycle);
            int read_enc();
            void enable_motor();
            void disable_motor();
            void brake_motor(int brake_power);

            int enc_count;

            Encoder encoder;
        

        private:
            int _EN;
            int _PWM1;
            int _PWM2;
            int _SENSE;
            int _encA;
            int _encB;
    };

    class MotorController{
        public:
            MotorController();

            void MotorControllerISR();

            void addMotor(Motor* motor);
            void run_motor(int motor, int speed);

        private:
            int motor_count = 0;
            Motor** motor_array;



    };

#endif