//
// Created by 神奇bug在哪里 on 7/15/23.
//

#ifndef SEARCHCAR_CONTROL_H
#define SEARCHCAR_CONTROL_H


#include <cstdint>
#include <vector>
#include "PID.h"

inline void getGlobalBaseInfo();



/**
 * @brief 维护一个可达性矩阵，并给出下一步的方向，用于完成路径规划，并且建立平面图。
 *        同时建立一个PID控制器，用于控制小车的转向，保证小车转向正确。
 *        并且建立一个坐标系，用于记录小车的位置，以及小车的朝向。
 *        通过这些信息，可以完成小车的自动探索迷宫和自动寻找目标。
 * @details 该类的主要功能是维护一个可达性矩阵，该矩阵的大小为地图的大小，每个元素的值为0或1，0表示该点不可达，1表示该点可达。
 * @details 将采用最小生成树算法，来建立一个平面图，该平面图的顶点为可达的点，边为可达的点之间的连线，权值为两点之间的距离。
 */
class Control {
private:
    /**
     * @brief 用于记录小车的位置，以及小车的朝向。
     * @details 该坐标系的原点为小车的初始位置，x轴为小车的初始朝向，y轴为x轴的逆时针旋转90度。
     */
    PID pid = PID(PIDConfig(0, 0, 0, 0, 0, 0)); //PID控制器
    int map[6][6]; //可达性矩阵
    inline void turnLeft();
    inline void turnRight();
    inline void goStraight();
    inline void turnBack();
    inline void stop();
    inline void start();
    inline static std::vector<bool> getNearbyInfo();
    inline void updateMap(int x,int y,bool isReachable);

public:
    /**
     * @brief 构造函数，用于初始化小车的位置，以及地图的大小。
     * @param start 小车的初始位置
     * @param end 小车的终点位置
     */
    explicit Control(PIDConfig config);
    Control();
    /**
     * @brief 用于更新小车的位置，以及小车的朝向。
     * @details 该函数会调用mpu.getPitch()、mpu.getYaw()、mpu.getRoll()函数，来获取小车的朝向。
     */
    void update();
    void IQRHandler();

};



#endif //SEARCHCAR_CONTROL_H
