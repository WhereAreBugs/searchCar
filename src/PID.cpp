//
// Created by 神奇bug在哪里 on 7/15/23.
//

#include "PID.h"
#include <cmath>
PID::PID(const PIDConfig config) {
    this->config = config;
}

float PID::update(float input) {
    float error;
    error = config.target - input;
    float integral = config.ki * error + integral;
    if (integral > config.max)
    {
        integral = config.max;
    } else if (integral < config.min)
    {
        integral = config.min;
    }
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
    this->min = 100;
    this->max = 100;
    this->target = 0;
}
