//
// Created by 神奇bug在哪里 on 7/17/23.
//

#include "Distance.h"
#include "logger.h"
#include "AsyncTCP.h"

extern AsyncClient tcp;
Distance::Distance(Adafruit_VL53L0X *Left, Adafruit_VL53L0X *Right, Adafruit_VL53L0X *Front,
                   Adafruit_VL53L0X *Back) {
    left = Left;
    front = Right;
    right = Front;
    back = Back;

}

uint16_t Distance::getLeftDistance()  {

    return leftData;
}

uint16_t Distance::getRightDistance()  {
    return rightData;
}

uint16_t Distance::getFrontDistance()  {
    return frontData;

}

void Distance::setup() {

}

void Distance::update() {
    if (left->Status != VL53L0X_ERROR_NONE) {
        LOGE("Failed to detect and initialize left VL53L0X sensor!")
        measure1.RangeStatus = VL53L0X_ERROR_NOT_IMPLEMENTED;
        measure1.RangeMilliMeter = 65535;
    } else{
    left->getSingleRangingMeasurement(&measure1, false);
    }
    if (front->Status != VL53L0X_ERROR_NONE) {
        LOGE("Failed to detect and initialize front VL53L0X sensor!")
        measure2.RangeStatus = VL53L0X_ERROR_NOT_IMPLEMENTED;
        measure2.RangeMilliMeter = 65535;
    } else {
        front->getSingleRangingMeasurement(&measure2, false);
    }
    if (right->Status != VL53L0X_ERROR_NONE) {
        LOGE("Failed to detect and initialize right VL53L0X sensor!")
        measure3.RangeStatus = VL53L0X_ERROR_NOT_IMPLEMENTED;
        measure3.RangeMilliMeter = 65535;
    } else {
        right->getSingleRangingMeasurement(&measure3, false);
    }
    if (1) {
        measure4.RangeStatus = VL53L0X_ERROR_NOT_IMPLEMENTED;
        measure4.RangeMilliMeter = -1;
    } else {
        back->getSingleRangingMeasurement(&measure4, false);
    }

//    if (back->timeoutOccurred())
//    {
//        LOGE("back VL53L0X sensor timeout!")
//        measure4.RangeStatus = VL53L0X_ERROR_TIME_OUT;
//        measure4.RangeMilliMeter = 0;
//    }
    leftData = measure1.RangeMilliMeter;
    frontData = measure2.RangeMilliMeter;
    rightData  = measure3.RangeMilliMeter;
    backData = measure4.RangeMilliMeter;
//    LOGI("left:"+String(leftData)+" right:"+String(rightData)+" front:"+String(frontData)+" back:"+String(backData))
}


