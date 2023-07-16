//
// Created by 神奇bug在哪里 on 7/15/23.
//

#include "PID.h"

PID::PID(PIDConfig config) {
    this->config = config;
}

float PID::update(float input) {
    float error = config.target - input;
    integral += error;
    float derivative = error - lastError;
    lastError = error;
    float output = config.kp * error + config.ki * integral + config.kd * derivative;
    if (output > config.max)
    {
        output = config.max;
    } else if (output < config.min)
    {
        output = config.min;
    }
    return output;
}

void PID::setTarget(float target) {
    this->config.target = target;
}

void PID::setConfig(PIDConfig config) {
    this->config = config;
}

PIDConfig PID::getConfig() {
    return config;
}

PIDConfig::PIDConfig(float kp, float ki, float kd, float min, float target, float max) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->min = min;
    this->max = max;
    this->target = target;
}

PIDConfig::PIDConfig() {
    this->kp = 0;
    this->ki = 0;
    this->kd = 0;
    this->min = 0;
    this->max = 0;
    this->target = 0;
}
