#include"mbed.h"
#include"Servo.hpp"
#include"config.hpp"
#include "motor.hpp"
#include "encoder.hpp"


Timer timer;
int main(){
    Servo servo[2]{
        {(PB_10)},
        {(PB_2)}};
    double t=timer.read();
    
    servo[0].set_angle(180-114);
    servo[1].set_angle(180-85);
    while(true){
    while(timer.read() - t < dt){
        t=timer.read();
    }
    }
    }

