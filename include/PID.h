//
// Created by 神奇bug在哪里 on 7/15/23.
//

#ifndef SEARCHCAR_PID_H
#define SEARCHCAR_PID_H

struct PIDConfig {
    PIDConfig(float kp, float ki, float kd, float min, float target, float max);

    PIDConfig();

    float kp;
    float ki;
    float kd;
    float target;
    float max;
    float min;
};
class PID {
private:
    PIDConfig config;
    float lastError;
    float integral;
public:
    explicit PID(PIDConfig config);
    float update(float input);
    void setTarget(float target);
    void setConfig(PIDConfig config);
    PIDConfig getConfig();
};


#endif //SEARCHCAR_PID_H
