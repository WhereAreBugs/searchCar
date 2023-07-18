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

public:
    Motor(uint8_t pin_m1_A, uint8_t pin_m1_B, uint8_t pin_m2_A, uint8_t pin_m2_B);
    void setup();
    void setSpeed(uint8_t motor, int32_t speed);
    void setPins(uint8_t pin_m1_A, uint8_t pin_m1_B, uint8_t pin_m2_A, uint8_t pin_m2_B);
    void update() const;
};


#endif //SEARCHCAR_MOTOR_H
