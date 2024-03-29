//
// Created by 神奇bug在哪里 on 7/15/23.
//

#ifndef SEARCHCAR_MOTOR_H
#define SEARCHCAR_MOTOR_H
#include <cstdint>

class Motor {
private:
    uint8_t pin_m1_A;
    uint8_t pin_m1_B;
    uint8_t pin_m2_A;
    uint8_t pin_m2_B;
    int32_t motor1Speed;
    int32_t motor2Speed;
    uint32_t pwmCounter;
public:
    Motor(uint8_t pin_m1_A, uint8_t pin_m1_B, uint8_t pin_m2_A, uint8_t pin_m2_B);
    void setup() const;
    void setSpeed(uint8_t motor, int32_t speed);
    void setPins(uint8_t newPin_m1_A, uint8_t newPin_m1_B, uint8_t newPin_m2_A, uint8_t newPin_m2_B);
    void update() const;
    void IQRHandler();
};


#endif //SEARCHCAR_MOTOR_H
