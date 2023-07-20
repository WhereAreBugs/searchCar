//
// Created by 神奇bug在哪里 on 7/15/23.
//

#include "Motor.h"
#include "logger.h"
#include "Arduino.h"
#include "AsyncTCP.h"

extern AsyncClient tcp;
Motor::Motor(uint8_t pin_m1_A, uint8_t pin_m1_B, uint8_t pin_m2_A, uint8_t pin_m2_B) {
    this->pin_m1_A = pin_m1_A;
    this->pin_m1_B = pin_m1_B;
    this->pin_m2_A = pin_m2_A;
    this->pin_m2_B = pin_m2_B;

}

void Motor::setup() {
    pinMode(pin_m1_A, OUTPUT);
    pinMode(pin_m1_B, OUTPUT);
    pinMode(pin_m2_A, OUTPUT);
    pinMode(pin_m2_B, OUTPUT);
#ifndef SOFTPWM
    ledcSetup(1, 1000, 8);
    ledcSetup(2, 1000, 8);
    ledcSetup(3, 1000, 8);
    ledcSetup(4, 1000, 8);
    ledcAttachPin(pin_m1_A, 1);
    ledcAttachPin(pin_m1_B, 2);
    ledcAttachPin(pin_m2_A, 3);
    ledcAttachPin(pin_m2_B, 4);
#endif
}

void Motor::setSpeed(uint8_t motor, int32_t speed) {
    if (motor == 0)
    {
        motor1Speed = speed;
    } else if (motor == 1)
    {
        motor2Speed = speed;
    } else
    {
        LOGE("motor number error")
    }
}

void Motor::setPins(uint8_t pin_m1_A, uint8_t pin_m1_B, uint8_t pin_m2_A, uint8_t pin_m2_B) {
    this->pin_m1_A = pin_m1_A;
    this->pin_m1_B = pin_m1_B;
    this->pin_m2_A = pin_m2_A;
    this->pin_m2_B = pin_m2_B;
    pwmCounter = 0;

}

void Motor::update() const {
#ifndef SOFTPWM
    if (motor1Speed >= 0)
    {
        ledcWrite(1, motor1Speed/255*500);
        ledcWrite(2, 0);
    } else
    {
        ledcWrite(1, 0);
        ledcWrite(2, -motor1Speed/255*500);
    }
    if(motor2Speed >= 0)
    {
        ledcWrite(3, motor2Speed/255*500);
        ledcWrite(4, 0);
    } else
    {
        ledcWrite(3, 0);
        ledcWrite(4, -motor2Speed/255*500);
    }
#endif
    return;
}

void Motor::IQRHandler() {

    if (pwmCounter++>255)
    {
        pwmCounter = 0;
    }
    if (motor1Speed>0) {
        if (pwmCounter < motor1Speed) {
            digitalWrite(pin_m1_A, HIGH);
            digitalWrite(pin_m1_B, LOW);
        } else {
            digitalWrite(pin_m1_A, LOW);
            digitalWrite(pin_m1_B, LOW);
        }
    } else
    {
        if (pwmCounter < -motor1Speed) {
            digitalWrite(pin_m1_A, LOW);
            digitalWrite(pin_m1_B, HIGH);
        } else {
            digitalWrite(pin_m1_A, LOW);
            digitalWrite(pin_m1_B, LOW);
        }
    }
    if (motor2Speed >0) {
        if (pwmCounter < motor2Speed) {
            digitalWrite(pin_m2_A, HIGH);
            digitalWrite(pin_m2_B, LOW);
        } else {
            digitalWrite(pin_m2_A, LOW);
            digitalWrite(pin_m2_B, LOW);
        }
    } else
    {
        if (pwmCounter < -motor2Speed) {
            digitalWrite(pin_m2_A, LOW);
            digitalWrite(pin_m2_B, HIGH);
        } else {
            digitalWrite(pin_m2_A, LOW);
            digitalWrite(pin_m2_B, LOW);
        }
    }
}
