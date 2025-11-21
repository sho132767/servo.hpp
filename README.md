# 使い方
#include "mbed.h"
#include "servo.hpp"

int main() {
    
    Servo servo(pin);

    // 滑らかに90度へ動かす（推奨）
    myservo.set_angle(90);
    ThisThread::sleep_for(10);

    // 一気に180度へ動かす
    myservo.fast_angle(180);
    ThisThread::sleep_for(20);
    myservo.set_angle(0);

    while (true) {
        // 現在角度を表示
        printf("Current angle: %d\n", myservo.fast_angle);
        ThisThread::sleep_for(10);
    }
}
