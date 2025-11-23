#ifndef ARRC_anglerobomas_H_
#define ARRC_anglerobomas_H_

#include "mbed.h"
#include "pid.hpp"
#include "robomaster_can.hpp"
#include "encoder.hpp"
#include "config.h"

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
                      double kp, double ki, double kd, double gia_mot=1.0, double gia_dt=1.0)
        : encoder(enc), esc(esc), pid(kp, ki, kd), gia_mot(gia_mot), gia_dt(gia_dt), target_angle(0) {}

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
    double accumulated_angle;  // 累積角度[rad]
    int16_t prev_raw_count;    // 前回の生カウント値(0~8191)
    bool first_update;         // 初回判定
    
    // カウント値をラジアンに変換
    double count_to_radian(int16_t count) {
        return (count / 8192.0) * 2.0 * M_PI;
    }
    
public:
    angle_robomas(robomaster::Robomaster_ESC* esc,
                  double kp, double ki, double kd, double gia_mot=1.0, double gia_dt=1.0)
        : esc(esc), pid(kp, ki, kd), gia_mot(gia_mot), gia_dt(gia_dt), 
          target_angle(0.0), accumulated_angle(0.0), prev_raw_count(0), first_update(true) {}

    void setTarget(double angle) {
        target_angle = (angle*M_PI/180)*(gia_mot/gia_dt);
    }

    void update() {
        int16_t raw_count = esc->get_angle();  // 0~8191のカウント値
        
        if (first_update) {
            // 初回は現在位置を基準とする
            accumulated_angle = count_to_radian(raw_count);
            prev_raw_count = raw_count;
            first_update = false;
        } else {
            // カウント値の差分を計算
            int16_t delta_count = raw_count - prev_raw_count;
            
            // 巻き戻り検出と補正
            // 正方向の巻き戻り: 8191 → 0 (delta_count が大きな負の値)
            if (delta_count < -4096) {
                delta_count += 8192;
            }
            // 逆方向の巻き戻り: 0 → 8191 (delta_count が大きな正の値)
            else if (delta_count > 4096) {
                delta_count -= 8192;
            }
            
            // 累積角度を更新
            accumulated_angle += count_to_radian(delta_count);
            prev_raw_count = raw_count;
        }
        
        // PID制御
        pid.Input(target_angle, accumulated_angle);        
        int16_t current = static_cast<int16_t>(pid.Output());
        esc->set_current(current);                     
    }
    
    // 現在の累積角度を取得(度数法)
    double getCurrentAngle() {
        return accumulated_angle * 180.0 / M_PI * (gia_dt/gia_mot);
    }
    
    // 角度をリセット(現在位置を0度とする)
    void resetAngle() {
        first_update = true;
        accumulated_angle = 0.0;
    }
};

#endif
