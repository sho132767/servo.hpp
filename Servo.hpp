#ifndef ARRC_MOTOR_H_
#define ARRC_MOTOR_H_
#include "mbed.h"
#include "motor.hpp"

class Servo {
private:
    PwmOut servo;
    int min;   
    int max;  
    int now_angle;
public:
    
    Servo(PinName pin, int min = 500, int max = 2500) 
        : servo(pin), min(min), max(max), now_angle(0) {
        servo.period_ms(20); 
        set_angle(0);         
    }

   
    void set_angle(int d) {
        if (d < 0) d = 0;
        if (d > 180) d = 180;
        now_angle = d;
        int target_angle = min + (max - min) * d / 180;
        int step = (target_angle > now_angle) ? 100 : -100;
     for (int p = now_angle; (step > 0 ? p <= target_angle : p >= target_angle);  p += step) {
        servo.pulsewidth_us(p);
        ThisThread::sleep_for(10);
    }
    }
    };

#endif
