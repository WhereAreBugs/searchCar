//
// Created by 神奇bug在哪里 on 7/15/23.
//
#include "Control.h"
#include "Status.h"
#include <MPU9250.h>


extern MPU9250 mpu;
///外部对象
float pitch,yaw, roll;
///状态量

void getGlobalBaseInfo() {
    pitch = mpu.getPitch();
    yaw = mpu.getYaw();
    roll = mpu.getRoll();
}

Control::Control(PIDConfig config) {
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            map[i][j] = 0;
        }
    }
    pid.setConfig(config);
}

Control::Control() {
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j <8 ; ++j) {
            map[i][j] = 0;
        }
    }
}

void Control::update() {
    getGlobalBaseInfo();
    if (!status&STATUS_RUNNING_MASK)
    {
        return;
    }

}

void Control::IQRHandler() {

}
