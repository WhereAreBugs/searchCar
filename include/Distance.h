//
// Created by 神奇bug在哪里 on 7/17/23.
//

#ifndef SEARCHCAR_DISTANCE_H
#define SEARCHCAR_DISTANCE_H
#include "Arduino.h"
#include "Adafruit_VL53L0X.h"
#include "atomic"


class Distance {
private:
    Adafruit_VL53L0X *left;
    Adafruit_VL53L0X *right;
    Adafruit_VL53L0X *front;
    Adafruit_VL53L0X * back;
    VL53L0X_RangingMeasurementData_t measure1;
    VL53L0X_RangingMeasurementData_t measure2;
    VL53L0X_RangingMeasurementData_t measure3;
    VL53L0X_RangingMeasurementData_t measure4;
    std::atomic<uint16_t> leftData;
    std::atomic<uint16_t> rightData;
    std::atomic<uint16_t> frontData;
    std::atomic<uint16_t> backData;
public:
    Distance(Adafruit_VL53L0X *Left, Adafruit_VL53L0X *Right, Adafruit_VL53L0X *Front,
             Adafruit_VL53L0X *Back);
//    void update();
    uint16_t getLeftDistance() ;
    uint16_t getRightDistance() ;
    uint16_t getFrontDistance() ;
    void setup();
    void update();

};


#endif //SEARCHCAR_DISTANCE_H
