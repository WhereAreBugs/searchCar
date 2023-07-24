//
// Created by 神奇bug在哪里 on 7/15/23.
//

#ifndef SEARCHCAR_CONTROL_H
#define SEARCHCAR_CONTROL_H


#include <cstdint>
#include <vector>
#include "PID.h"
#include "settings.h"
#include <stack>

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
    PID pid = PID(PIDConfig(TRUN_KP, 0, 0, 0, 0, 0)); //PID控制器
    int speedLeft = 0; //左轮速度
    int speedRight = 0; //右轮速度
    int map[26][26] = {0}; //可达性矩阵,默认全是0，比实际值大一，避免多余的-1。
    std::stack<int8_t> path; //路径栈,用于存储存在未完成遍历的节点
    std::vector<int8_t> visited; //已经访问过的节点
    /// 用于计算小车当前的坐标

    float current_distance_left = 0;
    float current_distance_right = 0;
    float current_distance_front = 0;
    const int  mapInfo[5][5]   {1,6,11,16,21,
                             2,7,12,17,22,
                             3,8,13,18,23,
                             4,9,14,19,24,
                             5,10,15,20,25};
private:
    inline void turnLeft();
    inline void turnRight();
    inline void goStraight();
    inline void turnBack();
    inline void stop();
    inline void start();
    inline static std::vector<bool> getNearbyInfo();
    inline void updateMap(int start, int end, bool isReachable);

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
    /**
     * @brief 定时器中断回调函数，用于一些重要的事件的处理，目前暂时没有想到这种事件。
     * @attention 该函数的执行时间必须尽可能的短，否则会影响小车的运行。
     */
    void IQRHandler();
    /**
     * @brief 下面两个函数用于外部调用，用于控制小车的运动。
     * @return 返回计算出的左右轮应该有的速度。
     */
    int getSpeedLeft() const{return speedLeft;}
    int getSpeedRight() const{return speedRight;}
private:  //不同状态下的行为
    /**
     * @brief 用于邻接矩阵（地图）建立完成之后，小车自动规划到目标点的路径，并且自动前往该地方
     * @details 该函数会调用findTargetWithMap()、moveToTarget()函数，来完成自动探索迷宫和自动寻找目标。
     * @attention 该函数需要标志位的支持，保证下次循环也会调用该函数。并且该函数会覆写STATUS_TARGET
     */
    inline void findTargetWithMap();
    /**
     * @brief 用于手动控制小车的运动，数据来源于远程服务器(串口/TCP)
     * @details 该函数需要使用到全局变量speed和angle，这些值来自于外部的串口/TCP通信。
     * @attention 该函数需要标志位的支持，保证下次循环也会调用该函数。目前检查的标志位为STATUS_MANUAL, 并且标志位匹配上之后其他大部分标志位都会失效。
     * @todo 该函数需要增加对更多的手动控制模式，比如控制小车走向某一个点等，方便调试。
     */
    inline void ManualMode();
    /**
     * @brief 在程序的最后，小车找到了目标数字，并且完成了地图的建立，现在需要小车自动找到出口并离开。
     * @details 该函数会用到之前建立的地图，以及小车当前的位置，来自动规划到出口的路径。
     * @attention 该函数需要根据多个标志位进行判断，理论上当小车地图建立完成且找到目标数字之后，该函数才会被调用。
     */
    inline void moveToExit();
    /**
     * @brief 用于小车在迷宫中确认当前所在区块是否存在数字，并且在这里左右移动，保证上位机能识别到数字。
     * @return 返回当前区块是否存在数字
     * @attention 该函数需要上位机的支持，需要和上位机之间存在通信。
     */
    inline bool checkIfHasTreasure();
    /**
     *  @brief 在小车地图完全建立之前，小车会调用该函数，使用DFS的方式来探索迷宫。
     *  @details 该函数仅会在status设置下一个目标的标志位，需要处理的函数自动解析该标志位，并判断出需要的移动方向，并且移动一个格子
     *  @attention 该函数包含边界检查。
     */
    inline void selectNextTarget();
    /**
     * @brief 让小车解析标志位中的目标点，并且调用相关的函数来移动到目标点。
     * @details 该函数会调用turnLeft()、turnRight()、goStraight()、turnBack()等函数，来完成小车的移动。
     * @attention 该函数也需要标志位的支持
     * @attention 需要注意，当小车尚未到达标志位时，该函数会被频繁调用。因此该函数中需要添加小车是否到位的判断
     */
    inline void moveToTarget();
    /**
     * @brief 在小车最开始的阶段，判断小车是否已经进入了迷宫。若没有，则调用该函数，让小车进入迷宫。
     */
    inline void enterTheMaze();

private: //状态判断用的辅助函数
    /**
     * @brief 用于判断小车是否已经进入了迷宫
     * @return  返回小车是否已经进入了迷宫
     * @attention 该函数依赖与小车三个距离传感器的数据，需要注意更新传感器的数据。
     */
    inline bool checkEnter();
    /**
     * @brief 用于规划路径的辅助函数，用于寻找小车移动到目标点所需要的最短路径。
     * @details 该函数会使用小车建立的邻接矩阵作为数据源，使用Dijkstra算法来寻找最短路径。
     * @param start 用于指定起始点
     * @param end 用于指定目标点
     * @param n 邻接矩阵的大小
     * @param targetPath  指定一个vector的指针，执行后该vector将会包含小车移动到目标点所需要的最短路径。
     */
    void dijkstra(int start, int end, int n, std::vector<int>& targetPath);
public:
    /**
     * @brief 用于指定小车的运动方向，小车会自动运动到下一个区块。
     * @param direction 用于指定小车的运动方向
     *                  0 -- 前进
     *                  1 -- 左转
     *                  2 -- 右转
     *                  3 -- 后退
     *
     */
    void ManualGo(uint8_t direction);
    void setKP(float kp);
    void setKI(float ki);
    void setKD(float kd);
    std::vector<int> getDistance();
public:
    /**
     * @brief 依据对距离传感器的累加，来计算小车当前所在的位置。
     * @return  返回小车当前所在的位置(区块编号)
     */
    int getCurrentPosition();

    /**
     * @brief 用于获取小车当前建立的邻接矩阵
     * @return 返回小车当前建立的邻接矩阵(二维向量类型)
     */


    std::vector<std::vector<int>> getMap();
    std::vector<float> getX_Y();
    /**
     * @brief 用于获取一个节点是否已经访问过
     * @return 返回一个节点是否已经访问过
     */
    bool notEsxited(int i);

private:
    /**
     * @brief 用于processMultiStep函数中，多步行走的标志。
     * @attention 以下的变量不应该在任何其他地方使用，否则会导致小车的运动出现问题。
     */
    int8_t currentDest = 0;
    std::vector<int> moveTargetPath;
    int8_t stepMultiCount = 0;
private:
    /**
     * @brief 仅用于findTargetWithMap函数中，用于记录小车路径规划的相关信息
     * @attention 以下的变量不应该在任何其他地方使用，否则会导致小车的运动出现问题。
     */
    bool isFindTargetWithMapInitialed = false;
    std::vector<int > mapTargetPath;
    int stepCount = 0;
    /**
     * @brief 以下变量用于全局记录两个数字点是否有记录
     * @attention 以下变量应在小车完成了对应位置的识别之后置为true。
     */
    bool isPointAChecked = false;
    bool isPointBChecked = false;

    /**
     * @brief 用于全局的扫描数据的存储
     * @attention 注意必须要先保证扫描完成才能进行访问
     */
     std::vector<bool> scan_result;
    void processMultiStep(bool inlineCall);

private:
    /**
     * @brief 用于处理传感器数据
     * @attention 以下变量不能用作其他用途！
     */
    bool isFrontSwitched = false;
/**
 * @brief 用于小车最后离开迷宫时候的路径规划
 * @attention  不要在其他地方使用以下变量
 */
    bool isMoveToExitInitialed{};
    std::vector<int> mapExitPath;

    int getAngleOffset();

    float last_distance_left;
    float last_distance_right;
    float last_distance_front;
};


/// 将会建立的图:
/// 1   6   11  16  21
/// 2   7   12  17  22
/// 3   8   13  18  23
/// 4   9   14  19  24
/// 5   10  15  20  25


#endif //SEARCHCAR_CONTROL_H
