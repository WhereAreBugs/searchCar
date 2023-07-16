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
}

void Motor::setSpeed(uint8_t motor, uint8_t speed) {
    if (motor == 1)
    {
        motor1Speed = speed;
    } else if (motor == 2)
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

}

void Motor::update() const {
    if (motor1Speed >= 0)
    {
        analogWrite(pin_m1_A, motor1Speed);
        analogWrite(pin_m1_B, 0);
    } else
    {
        analogWrite(pin_m1_A, 0);
        analogWrite(pin_m1_B, -motor1Speed);
    }
    if(motor2Speed >= 0)
    {
        analogWrite(pin_m2_A, motor2Speed);
        analogWrite(pin_m2_B, 0);
    } else
    {
        analogWrite(pin_m2_A, 0);
        analogWrite(pin_m2_B, -motor2Speed);
    }
}
