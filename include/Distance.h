//
// Created by 神奇bug在哪里 on 7/17/23.
//

#ifndef SEARCHCAR_DISTANCE_H
#define SEARCHCAR_DISTANCE_H
#include "Arduino.h"
#include "VL53L0X.h"

class Distance {
private:
    VL53L0X * left;
    VL53L0X * right;
    VL53L0X * front;

public:
    Distance(VL53L0X *Left, VL53L0X *Right, VL53L0X *Front);
//    void update();
    uint16_t getLeftDistance() const;
    uint16_t getRightDistance() const;
    uint16_t getFrontDistance() const;
    void setup();
};


#endif //SEARCHCAR_DISTANCE_H
