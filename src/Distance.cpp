//
// Created by 神奇bug在哪里 on 7/17/23.
//

#include "Distance.h"

Distance::Distance(VL53L0X *Left, VL53L0X *Right, VL53L0X *Front) {
    left = Left;
    right = Right;
    front = Front;
}

uint16_t Distance::getLeftDistance() const {
    return left->readRangeContinuousMillimeters();
}

uint16_t Distance::getRightDistance() const {
    return right->readRangeContinuousMillimeters();
}

uint16_t Distance::getFrontDistance() const {
    return front->readRangeContinuousMillimeters();
}

void Distance::setup() {
    left->setTimeout(500);
    right->setTimeout(500);
    front->setTimeout(500);
    left->startContinuous();
    right->startContinuous();
    front->startContinuous();
}


