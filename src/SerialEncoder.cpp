//
// Created by 神奇bug在哪里 on 7/22/23.
//

#include <HardwareSerial.h>
#include "SerialEncoder.h"
#include "Status.h"
#include "logger.h"
#include "AsyncTCP.h"
extern AsyncClient tcp;
SerialEncoder::SerialEncoder() {
}

void SerialEncoder::setup() {

    serial->setPins(35, 13);
    serial->setRxTimeout(10);
    serial->onReceive([this]() {
        IQRHandler();
    });
}
#define K 0.09430604982
void SerialEncoder::IQRHandler() {
    auto str = Serial1.readStringUntil('\n');
    int32_t motor1Speed, motor2Speed;
    sscanf(str.c_str(), "%d,%d", &motor1Speed, &motor2Speed);
    if ((status & STATUS_DIRECTION_MASK >> STATUS_DIRECTION_OFFSET) == 0) {
        offset_x += (motor1Speed + motor2Speed) / 2.0 * K;
    } else if ((status & STATUS_DIRECTION_MASK >> STATUS_DIRECTION_OFFSET) == 1) {
        LOGI("y_offset,current dir:" + String(status & STATUS_DIRECTION_MASK >> STATUS_DIRECTION_OFFSET) + ",speed:" + String((motor1Speed + motor2Speed) / 2.0 * K) + ",offset:" + String(offset_y))
        offset_y += (motor1Speed + motor2Speed) / 2.0 * K;
    } else if ((status & STATUS_DIRECTION_MASK >> STATUS_DIRECTION_OFFSET) == 2) {
        offset_x -= (motor1Speed + motor2Speed) / 2.0 * K;
    } else if ((status & STATUS_DIRECTION_MASK >> STATUS_DIRECTION_OFFSET) == 3) {
        LOGI("y_offset,current dir:" + String(status & STATUS_DIRECTION_MASK >> STATUS_DIRECTION_OFFSET) + ",speed:" + String((motor1Speed + motor2Speed) / 2.0 * K) + ",offset:" + String(offset_y))
        offset_y -= (motor1Speed + motor2Speed) / 2.0 * K;
    }
}

double SerialEncoder::getX_offset() const {
    return offset_x;
}
double SerialEncoder::getY_offset() const {
    return offset_y;
}

void SerialEncoder::attchSerial(HardwareSerial *serial) {
    this->serial = serial;
}

void SerialEncoder::reset() {
    offset_x = 0;
    offset_y = 0;
}
