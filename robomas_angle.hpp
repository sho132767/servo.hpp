#ifndef ARRC_anglerobomas_H_
#define ARRC_anglerobomas_H_

#include "mbed.h"
#include "pid.hpp"
#include "robomaster_can.hpp"
#include "encoder.hpp"
#include "config.h"
#include <cstdint>

class angle_robomas_enc {
    private:
    Encoder* encoder;
    robomaster::Robomaster_ESC* esc;
    PID pid;
    double gia_mot;
    double gia_dt;
    double target_angle;
public:
    angle_robomas_enc(Encoder* enc, robomaster::Robomaster_ESC* esc,
                      double kp, double ki, double kd,double gia_mot=1.0,double gia_dt=1.0)
        : encoder(enc), esc(esc), pid(kp, ki, kd), gia_mot(gia_mot),gia_dt(gia_dt),target_angle(0) {}

    void setTarget(double angle) {
        target_angle = (angle*M_PI/180)*(gia_mot/gia_dt);
    }

    void update() {
        double now_angle = encoder->get_theta();  
        pid.Input(target_angle, now_angle);        
        int16_t current = static_cast<int16_t>(pid.Output());
        esc->set_current(current);                     
    }

};

class angle_robomas {
    private:
    robomaster::Robomaster_ESC* esc;
    PID pid;
    double gia_mot;
    double gia_dt;
    double target_angle;
    const int CURRENT_MAX = 8000;
    int16_t current;
public:

    angle_robomas(robomaster::Robomaster_ESC* esc,
                  double kp, double ki, double kd,double gia_mot=1.0,double gia_dt=1.0)
        : esc(esc), pid(kp, ki, kd), gia_mot(gia_mot),gia_dt(gia_dt),target_angle(0.0) {}

    void setTarget(double angle) {
        target_angle = angle;
    }

    void update() {
        double now_angle = esc->get_continuous_angle() / 36.0;       

        pid.Input(target_angle, now_angle);        
        current = static_cast<int16_t>(pid.Output());

        if(abs(current) > CURRENT_MAX)
            current = current < 0 ? -CURRENT_MAX : CURRENT_MAX;
        
        esc->set_current(current);
    }

    int get_current(){
        return current;
    }
};

#endif
