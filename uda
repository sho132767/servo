#ifndef ARRC_arm_H_
#define ARRC_arm_H_
#include"mbed.h"
#include"Servo.hpp"
#include"config.hpp"
#include "robomaster_can.hpp"
#include"pid.hpp"
#include "motor.hpp"

class ARM_uda{
private:
    double kp = 0.5;
    double ki = 0.0;
    double kd = 0.0;
    double MAX_CURRENT = 8000;
    Timer timer;
    

    int16_t clamp_current(double value, int16_t max_val) {
        if (value > max_val) return max_val;
        if (value < -max_val) return -max_val;
        return static_cast<int16_t>(value);
    }
    Serial pc;
    robomaster::Robomaster_Array array;
    robomaster::Robomaster_ESC ESC1;
    DigitalIn lim;
    PID pid;
    Servo servos[2];
    public:


    ARM_uda(PinName servo1,PinName servo2,PinName pin_lim,PinName robomas1,PinName robomas2,uint8_t id)
    : pc(USBTX, USBRX, 115200),
    array(robomas1, robomas2, 1000000),
    ESC1(id),
          lim(pin_lim, PullUp),
          pid(kp, ki, kd),
           servos{ Servo(servo1), Servo(servo2) }
    {
        array.add_ESC(&ESC1);
        timer.start();
    }
    
    void take(){
         servos[0].set_angle(90);
    }
    void hanase(){
         servos[0].set_angle(0);
    }
    void up(){
      servos[1].set_angle(0);
    
    }
    void down(){
        servos[1].set_angle(180);
    }
    void robomascatch(){
        int16_t initial_raw = ESC1.get_continuous_angle();
        double initial_angle = (initial_raw / (8191.0 * 36)) * 360.0;
        double target = initial_angle + 90.0;

        double t = timer.read();

        
            double angle = (ESC1.get_continuous_angle() / (8191.0 * 36)) * 360.0;
            pid.Input(target, angle);
            double output = pid.Output();
            int16_t current = clamp_current(output, MAX_CURRENT);

            ESC1.set_current(current);

           

        array.send();
        wait_us(10);

        //pc.printf("angle,%lf current,%d\n", angle, current);

        while (timer.read() - t < dt);
        t = timer.read();
    
    }
    void robomasback(){
        int16_t initial_raw = ESC1.get_continuous_angle();
        double initial_angle = (initial_raw / (8191.0 * 36)) * 360.0;
        double target = initial_angle + -90.0;

        double t = timer.read();

        
            double angle = (ESC1.get_continuous_angle() / (8191.0 * 36)) * 360.0;
            pid.Input(target, angle);
            double output = pid.Output();
            int16_t current = clamp_current(output, MAX_CURRENT);

            ESC1.set_current(current);

           

        array.send();
        wait_us(10);

        //pc.printf("angle,%lf current,%d\n", angle, current);

        while (timer.read() - t < dt);
        t = timer.read();
    
    }
};
#endif
