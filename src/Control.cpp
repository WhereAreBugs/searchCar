//
// Created by 神奇bug在哪里 on 7/15/23.
//
#include "Control.h"
#include "Status.h"
#include <MPU9250.h>
#include <queue>
#include "logger.h"
#include "AsyncTCP.h"
#include "Distance.h"

extern Distance allDistance;

extern AsyncClient tcp;
extern MPU9250 mpu;

///外部对象
float pitch, yaw, roll;
///速度控制，角度控制
int speed = 0;
float angle = 0;
/// 仅用于自动寻路模式的变量
int auto_speed = 30;


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
        for (int j = 0; j < 8; ++j) {
            map[i][j] = 0;
        }
    }
}

#pragma clang diagnostic push
#pragma ide diagnostic ignored "cppcoreguidelines-narrowing-conversions"

void Control::update() {
    getGlobalBaseInfo();
    current_distance_left = allDistance.getLeftDistance();
    current_distance_right = allDistance.getRightDistance();
    if (!isFrontSwitched) {
        current_distance_front = allDistance.getFrontDistance();
    }
    if ((!isFrontSwitched)&&current_distance_front == 65535) {
        // 传感器数据溢出，此时应该更换传感器
        //TODO: 更换数据源到后方传感器
        //请注意，每次更换传感器后，都需要复位传感器的上次计数
        isFrontSwitched = true;
    }
    else if (isFrontSwitched && current_distance_front == 65535)
    {
        isFrontSwitched = false;
    }
    if (last_distance_front == 0 && last_distance_left == 0 && last_distance_right == 0) {
        last_distance_front = current_distance_front;
        last_distance_left = current_distance_left;
        last_distance_right = current_distance_right;
    } //初始化，防止后续计算出错
    updateX_Y(); //更新坐标系
    if (!(status & STATUS_RUNNING_MASK)) {
        // 系统等待启动
        speedLeft = 0, speedRight = 0;
        return;
    }  //启动判读
    if (status & STATUS_RUNNING_MASK && !(status & STATUS_ENTERED)) {
        // 系统已启动，但未进入迷宫
        static bool initialized = false;
        if (!initialized) {
            enterTheMaze();
            initialized = true;
        } else {
            if (checkEnter()) {
                status = status | STATUS_ENTERED;
                initialized = false;
                return;
            } else {
                auto pidResult = pid.update(yaw);
                speedLeft = auto_speed + pidResult;
                speedRight = auto_speed - pidResult;
                return;
            }
        }
    } //进入判断
    if (status & STATUS_MANUAL_CONTROL_MASK) {
        // 手动模式启用
        ManualMode();
        return;
    }  //手动模式
    if (status & STATUS_TURNING_MASK) {
        // 系统正在转弯, 检查转弯是否完成。
        auto pidResult = pid.update(yaw);
        speedLeft = pidResult;
        speedRight = -pidResult;
        if (pidResult < 5) //认为转弯已经完成
        {
            last_distance_front = 0, last_distance_right = 0, last_distance_left = 0;
            status &= ~STATUS_TURNING_MASK;
        }
        return;
    } //转弯判断
    if (status & STATUS_MAP_BUILD_MASK) {
        ///  地图已完成建立
        if (!(status & STATUS_ARRIVED_MASK)) {
            /// 当前系统有下一个点的目标，但是仍然没有到达下一个点
            moveToTarget();
            auto pid_result =  pid.update(yaw);
            speedLeft = auto_speed + pid_result;
            speedRight = auto_speed - pid_result;
        }
        if ((status & STATUS_AUTO_FIND_TARGET_MASK) && !(status & STATUS_FIND_TREASURE_MASK)) {
            /// 自动寻找目标启用且系统仍未找到目标
            findTargetWithMap(); //类似于selectTarget函数
            return;
        } else if (status & STATUS_FIND_TREASURE_MASK) {
            /// 系统已找到目标,且地图已经建立
            moveToExit(); //一种新的selectTarget函数，有着指定的位置
            return;
        } else if (status & STATUS_ARRIVED_EXIT_MASK) {
            /// 系统已到达出口
            stop();
            status = status & (~STATUS_RUNNING_MASK);// 系统停止运行
            return;
        } else {
            LOGE("status error, status can't be recognized。Current at map build finished")
            return;
        }
    }
    /// 剩下的情况应该是地图未建立

    if (status & STATUS_ARRIVED_MASK) {
        /// 系统到达了上次行动要求的目标点
        if (!(status & STATUS_SCAN_MASK)) {
            scan_result = getNearbyInfo();
            //TODO: 当前坐标判断
            status = status | STATUS_SCAN_MASK;//标志系统扫描完成
            status = status & (~STATUS_TARGET_SELECTED_MASK);
            LOGI("Current position scan finished")
        }
        if (!(status & STATUS_FIND_TREASURE_MASK)) {
            /// 检查当前位置是否检测到目标数字
            if (checkIfHasTreasure()) {
                //TODO: 当前处于目标数字处，接收信号左右移动以保证视觉系统能够识别到数字
                delay(100);
            }
        }
        /// 系统目标需要更多步才能完成
        if (status&STATUS_MULTI_STEP_MASK)
        {   try {
                processMultiStep(false);
            }
            catch (std::exception & all) //防止数组越界等异常发生造成系统复位
            {
                LOGE("Exception caught in processMultiStep" + String(all.what()))
                status = status & (~STATUS_MULTI_STEP_MASK); //清除标志位，让系统重新计算
            }
            return; //等待下一次循环
        }
        if (!(status & STATUS_TARGET_SELECTED_MASK)) {
            /// 根据当前矩阵选择下一个点
            selectNextTarget();
            LOGI("Next target selected")
            status = status | STATUS_TARGET_SELECTED_MASK;
            status = status & (~STATUS_ARRIVED_MASK) & (~STATUS_SCAN_MASK) & (~STATUS_FIND_TREASURE_MASK);
        }
    } //系统到达目标点
    else if (!((status) & STATUS_ARRIVED_MASK)) {
        /// 系统尚未未到达上次行动要求的目标点
        moveToTarget();
        auto pidResult = pid.update(yaw);
        speedLeft = auto_speed + pidResult;
        speedRight = auto_speed - pidResult;
        return;
    }
    else {
        LOGE("status error, status can't be recognized。Current at map build finished")
        return;
    }


}

#pragma clang diagnostic pop

void Control::IQRHandler() {


}

#pragma clang diagnostic push
#pragma ide diagnostic ignored "cppcoreguidelines-narrowing-conversions"

void Control::ManualMode() {
    pid.setTarget(angle);
    auto pidResult = pid.update(yaw);
    speedLeft = speed + pidResult;
    speedRight = speed - pidResult;
}

void Control::turnLeft() {
    LOGI("turn left")
    if ((status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET == 0)  //若当前方向为0，则向左转向后应该为3
    {
        status |= STATUS_DIRECTION(3);
    } else //否则就直接减1
    {
        status |= STATUS_DIRECTION(((status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET) - 1);
    }
    status |= STATUS_TURNING_MASK;
    //TODO: 添加PID的目标值(不知道MPU9250的表示情况)
}

void Control::turnRight() {
    LOGI("turn right")
    if (((status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET == 3))  //若当前方向为3，则向右转向后应该为0
    {
        status |= STATUS_DIRECTION(0);
    } else //否则就直接加1
    {
        status |= STATUS_DIRECTION(((status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET) + 1);
    }
    status |= STATUS_TURNING_MASK;
    //TODO: 添加PID的目标值(不知道MPU9250的表示情况)
}

void Control::goStraight() {
    LOGI("go straight")
    auto_speed = 100;
}

void Control::turnBack() {
    LOGI("turn back")
    if (((status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET == 0))  //若当前方向为0，则向后转向后应该为2
    {
        status |= STATUS_DIRECTION(2);
    } else if (((status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET == 1)) {
        status |= STATUS_DIRECTION(3);
    } else //否则就直接减2
    {
        status |= STATUS_DIRECTION(((status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET) - 2);
    }
    status |= STATUS_TURNING_MASK;
}

void Control::stop() {
    LOGW("stop")
    auto_speed = 0;
    speedLeft = 0;
    speedRight = 0;
}

void Control::start() {
    LOGW("start")
    status = status | STATUS_RUNNING_MASK;

}

/**
 * @brief 获取周围的可达性
 * @return result[3], 0:左侧可达，1:前方可达，2:右侧可达
 */
std::vector<bool> Control::getNearbyInfo() {
    std::vector<bool> result(3);
    result[0] = (allDistance.getLeftDistance() / 10) > 45;
    result[1] = (allDistance.getFrontDistance() / 10) > 45;
    result[2] = (allDistance.getRightDistance() / 10) > 45;
    return result;
}

void Control::findTargetWithMap() {
    LOGI("find target with map start")
    auto currentPos = getCurrentPosition();
    auto targetPos = (status &= STATUS_TARGET_SELECTED_MASK) >> STATUS_TARGET_OFFSET;

    if (!isFindTargetWithMapInitialed) {
        dijkstra(currentPos, targetPos, 26, mapTargetPath);
        isFindTargetWithMapInitialed = true;
    } else {
        status = status & STATUS_TARGET(mapTargetPath[stepCount++]);
        status = status & (~STATUS_ARRIVED_MASK);
        status = status & (~STATUS_SCAN_MASK);
        status = status | STATUS_TARGET_SELECTED_MASK;
        return;
    }
}

void Control::moveToExit() {
    LOGI("move to exit start")
    auto currentPos = getCurrentPosition();
    auto targetPos = 25;
    if (!isMoveToExitInitialed) {
        dijkstra(currentPos, targetPos, 26, mapExitPath);
        isMoveToExitInitialed = true;
    } else {
        status = status & STATUS_TARGET(mapExitPath[stepCount++]);
        status = status & (~STATUS_ARRIVED_MASK);
        status = status & (~STATUS_SCAN_MASK);
        status = status | STATUS_TARGET_SELECTED_MASK;
        return;
    }
}

bool Control::checkIfHasTreasure() {
    LOGI("check if has treasure start")
    auto currentPos = getCurrentPosition();
    if (currentPos == 15 || currentPos == 17) {
        return true;
    }
    return false;
}

void Control::selectNextTarget() {
    LOGV("select next target start")
    auto currentPos = getCurrentPosition();
    auto currentPosInfo = scan_result; //优化IDE的上下文搜索
    LOGD("current position is " + String(currentPos))
    switch ((status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET) {
        case 0: //方向：将入口方向作为上方，该方向为入口的下方
        {
            if (currentPosInfo[0] == 1 && (currentPos - 5) > 0 && notEsxited(currentPos - 5) && currentPos != 25) {
                // 左侧可达，且左侧未到达过
                path.push(currentPos - 5);
                updateMap(currentPos, currentPos - 5, true);
                updateMap(currentPos - 5, currentPos, true);
            }
            if (currentPosInfo[1] == 1 && (currentPos + 1) > 0 && notEsxited(currentPos + 1)) {
                // 前方可达，且前方未到达过
                path.push(currentPos + 1);
                updateMap(currentPos, currentPos + 1, true);
                updateMap(currentPos + 1, currentPos, true);
            }
            if (currentPosInfo[2] == 1 && (currentPos + 5) > 0 && notEsxited(currentPos + 5)) {
                // 右侧可达，且右侧未到达过
                path.push(currentPos + 5);
                updateMap(currentPos, currentPos + 5, true);
                updateMap(currentPos + 5, currentPos, true);
            }
            break;
        }
        case 1: //方向：将入口方向作为上方，该方向为入口的右方
        {
            if (currentPosInfo[0] == 1 && currentPos != 1 && notEsxited(currentPos - 1)) {
                // 左侧可达，且左侧未到达过
                path.push(currentPos - 1);
                updateMap(currentPos, currentPos - 1, true);
                updateMap(currentPos - 1, currentPos, true);
            }
            if (currentPosInfo[1] == 1 && (currentPos + 5) > 0 && notEsxited(currentPos + 5) && currentPos != 25) {
                // 前方可达，且前方未到达过
                path.push(currentPos - 5);
                updateMap(currentPos, currentPos - 5, true);
                updateMap(currentPos - 5, currentPos, true);
            }
            if (currentPosInfo[2] == 1 && (currentPos + 1) > 0 && notEsxited(currentPos + 1)) {
                // 右侧可达，且右侧未到达过
                path.push(currentPos + 1);
                updateMap(currentPos, currentPos + 1, true);
                updateMap(currentPos + 1, currentPos, true);
            }
            break;

        }
        case 2: //方向：将入口作为上方，该方向为上方
        {
            if (currentPosInfo[0] == 1 && (currentPos - 5) > 0 && notEsxited(currentPos - 5)) {
                // 左侧可达，且左侧未到达过
                path.push(currentPos + 5);
                updateMap(currentPos, currentPos + 5, true);
                updateMap(currentPos + 5, currentPos, true);
            }
            if (currentPosInfo[1] == 1 && (currentPos - 1) > 0 && notEsxited(currentPos - 1)) {
                // 前方可达，且前方未到达过
                path.push(currentPos - 1);
                updateMap(currentPos, currentPos - 1, true);
                updateMap(currentPos - 1, currentPos, true);
            }
            if (currentPosInfo[2] == 1 && (currentPos + 5) > 0 && notEsxited(currentPos + 5) && currentPos != 25) {
                // 右侧可达，且右侧未到达过
                path.push(currentPos + 5);
                updateMap(currentPos, currentPos + 5, true);
                updateMap(currentPos + 5, currentPos, true);
            }
            break;
        }
        case 3: {
            if (currentPosInfo[0] == 1 && (currentPos + 1) > 0 && notEsxited(currentPos + 1)) {
                // 左侧可达，且左侧未到达过
                path.push(currentPos + 1);
                updateMap(currentPos, currentPos + 1, true);
                updateMap(currentPos + 1, currentPos, true);
            }
            if (currentPosInfo[1] == 1 && (currentPos - 5) > 0 && notEsxited(currentPos - 5) && currentPos != 25) {
                // 前方可达，且前方未到达过
                path.push(currentPos - 5);
                updateMap(currentPos, currentPos - 5, true);
                updateMap(currentPos - 5, currentPos, true);
            }
            if (currentPosInfo[2] == 1 && (currentPos - 1) > 0 && notEsxited(currentPos - 1) && currentPos != 1) {
                // 右侧可达，且右侧未到达过
                path.push(currentPos - 1);
                updateMap(currentPos, currentPos - 1, true);
                updateMap(currentPos - 1, currentPos, true);
            }
            break;
        }
        default: {
            LOGE("error direction")
            break;
        }
    }
    visited.emplace_back(currentPos); //将当前位置加入已访问列表
    if (!path.empty()) {
        auto target = path.top();
        processMultiStep(true);
        path.pop();
    } else {
        status |= STATUS_MAP_BUILD_MASK;
    }
    LOGD("select next target end, next point: " + String((status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET))

}

void Control::moveToTarget() {
    if ((getCurrentPosition() == (status & STATUS_DIRECTION_MASK))) {
        //到达目标点
        LOGD("arrive target point")
        status &= ~STATUS_DIRECTION_MASK; //清除目标点
        status |= STATUS_ARRIVED_MASK;
        return;
    }
    // 未到达目标点
    //TODO: 处理这种情况

}

void Control::enterTheMaze() {
    LOGI("Let's enter the maze")
    speedLeft = auto_speed;
    speedRight = auto_speed;
}

bool Control::checkEnter() {
    LOGI("check enter")
    if (current_distance_left < 20 || current_distance_right < 20) {
        return true;
    } else {
        return false;
    }
}

void Control::ManualGo(uint8_t direction) {
    if (!(status & STATUS_MANUAL_CONTROL_MASK)) {
        LOGE("Manual control is not enabled!")
        return;
    }
    switch (direction) {
        case 0:
            goStraight();
            break;
        case 1:
            turnLeft();
            break;
        case 2:
            turnRight();
            break;
        case 3:
            turnBack();
            break;
        default:
            LOGE("direction error")
            break;
    }
    last_distance_front = 0;
    last_distance_left = 0;
    last_distance_right = 0;

}

void Control::setKP(float kp) {
    auto i = pid.getConfig();
    i.kp = kp;
    pid.setConfig(i);
}

void Control::setKI(float ki) {
    auto i = pid.getConfig();
    i.ki = ki;
    pid.setConfig(i);

}

void Control::setKD(float kd) {
    auto i = pid.getConfig();
    i.kd = kd;
    pid.setConfig(i);
}

int Control::getCurrentPosition() {
    return mapInfo[static_cast<int>(x_offset + 10) / 40][static_cast<int>(y_offset + 10) / 40];
}

void Control::updateMap(int start, int end, bool isReachable) {
    mapInfo[start][end] = isReachable;
}

std::vector<std::vector<int>> Control::getMap() {
    std::vector<std::vector<int>> result;
    for (int i = 0; i < 25; ++i) {
        for (int j = 0; j < 25; ++j) {
            result[i][j] = map[i][j];
        }
    }
    return result;
}

bool Control::notEsxited(int i) {
    try {
        visited.at(i);
        return false;
    }
    catch (const std::out_of_range &oor) {
        return true;
    }
}

void Control::updateX_Y() {
    if (status & STATUS_TURNING_MASK) {
        return;
    }
    //TODO: 参考系统当前倾角作适当校准
    switch (status & STATUS_DIRECTION_MASK >> STATUS_DIRECTION_OFFSET) {
        case 0: //方向：将入口作为上方，该方向为下方
        {
            x_offset += last_distance_front - current_distance_front;
            break;
        }
        case 1: {
            y_offset += last_distance_front - current_distance_front;
            break;
        }
        case 2: {
            x_offset -= last_distance_front - current_distance_front;
            break;
        }
        case 3: {
            y_offset -= last_distance_front - current_distance_front;
            break;
        }

    }
}

using namespace std;
#define INF 0x3f3f3f3f

void Control::dijkstra(int start, int end, int n, vector<int> &targetPath) {

    vector<int> dist(n, INF);
    vector<int> prev(n, -1);
    vector<bool> visit(n, false);

    dist[start] = 0;
    for (int i = 0; i < n - 1; ++i) {
        int u = -1;
        for (int j = 0; j < n; ++j) {
            if (!visit[j] && (u == -1 || dist[j] < dist[u])) {
                u = j;
            }
        }
        if (u == -1) break;
        visit[u] = true;
        for (int v = 0; v < n; ++v) {
            if (map[u][v] && !visit[v]) {
                int new_dist = dist[u] + map[u][v];
                if (new_dist < dist[v]) {
                    dist[v] = new_dist;
                    prev[v] = u;
                }
            }
        }
    }

    if (dist[end] == INF) {
        LOGE("No available path")
        return;
    }
    for (int v = end; v != -1; v = prev[v]) {
        targetPath.push_back(v);
    }
}

void Control::processMultiStep(bool inlineCall) {
    auto currentPos = getCurrentPosition();
    if (inlineCall) {
        auto targetPos = ((status & STATUS_TARGET_SELECTED_MASK) >> STATUS_TARGET_OFFSET); //从内联调用时，检查是否能单步完成
        switch ((status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET) {
            case 0: //当前朝向为下方
            {
                if (targetPos == currentPos + 1) {
                    //目标点在前方
                    goStraight();
                } else if (targetPos == currentPos + 5) {
                    //目标点在右方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnRight();
                } else if (targetPos == currentPos - 5) {
                    //目标点在左方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnLeft();
                } else if (targetPos == currentPos - 1) {
                    //目标点在后方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnBack();
                } else {
                    status |= STATUS_MULTI_STEP_MASK;
                    currentDest = targetPos;
                }
                break;
            }
            case 1: //当前方向为右方
            {
                if (targetPos == currentPos + 5) {
                    //目标点在前方
                    goStraight();
                } else if (targetPos == currentPos + 1) {
                    //目标点在右方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnRight();
                } else if (targetPos == currentPos - 1) {
                    //目标点在左方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnLeft();
                } else if (targetPos == currentPos - 5) {
                    //目标点在后方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnBack();
                } else {
                    status |= STATUS_MULTI_STEP_MASK;
                    currentDest = targetPos;
                }
                break;
            }
            case 2: //当前方向为上方
            {
                if (targetPos == currentPos - 1) {
                    //目标点在前方
                    goStraight();
                } else if (targetPos == currentPos + 5) {
                    //目标点在右方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnRight();
                } else if (targetPos == currentPos - 5) {
                    //目标点在左方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnLeft();
                } else if (targetPos == currentPos + 1) {
                    //目标点在后方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnBack();
                } else {
                    status |= STATUS_MULTI_STEP_MASK;
                    currentDest = targetPos;
                }
                break;
            }
            case 3: //当前方向为左方
            {
                if (targetPos == currentPos - 5) {
                    //目标点在前方
                    goStraight();
                } else if (targetPos == currentPos - 1) {
                    //目标点在右方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnRight();
                } else if (targetPos == currentPos + 1) {
                    //目标点在左方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnLeft();
                } else if (targetPos == currentPos + 5) {
                    //目标点在后方
                    status |= STATUS_TURN_FIRST_MASK;
                    turnBack();
                } else {
                    status |= STATUS_MULTI_STEP_MASK;
                    currentDest = targetPos;
                }
                break;
            }
        } //检查系统的目标能否一步到达，以及应该怎么走
        if (status & STATUS_MULTI_STEP_MASK) {
            dijkstra(currentPos, targetPos, 26, moveTargetPath); // 规划路径
            status &= ~STATUS_TARGET_SELECTED_MASK; //清空这个标志位
            status |= STATUS_TARGET(moveTargetPath[currentDest++]);
            status &= STATUS_ARRIVED_MASK;
            status &= STATUS_SCAN_MASK;
        }
    }
    else {
        if (status & STATUS_TURN_FIRST_MASK) {
            //转弯后的第一步
            goStraight();
            status &= ~STATUS_TURN_FIRST_MASK; // 清除标志位
        } else if (status & STATUS_MULTI_STEP_MASK) {
            //多步行走
            if (currentPos==currentDest) //如果当前已到达目的地
            {
                status &= ~STATUS_MULTI_STEP_MASK;
                status &= ~STATUS_TARGET_SELECTED_MASK;
            }
            else {
                status &= ~STATUS_TARGET_SELECTED_MASK;
                status |= STATUS_TARGET(moveTargetPath[currentDest++]);
            }
            }
        }
    }



#pragma clang diagnostic pop
