//
// Created by 神奇bug在哪里 on 7/15/23.
//
#include "Control.h"
#include "Status.h"
#include <queue>
#include "logger.h"
#include "AsyncTCP.h"
#include "Distance.h"
#include "Ticker.h"
#include "SerialEncoder.h"
extern Distance allDistance;
extern SerialEncoder encoder;
extern AsyncClient tcp;

Ticker goOutTicker;
///外部对象
float pitch, yaw, roll,accx;
///速度控制，角度控制
int speed = 0;
float angle = 0;
/// 仅用于自动寻路模式的变量
int auto_speed = 100;


void getGlobalBaseInfo() {
#ifndef DISABLE_I2C
#else
    pitch = 0;
    yaw = 0;
    roll = 0;
#endif

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
#ifndef DISABLE_I2C
    /**
     * @brief 由于VL53L0X传感器测距不准，因此舍弃掉mm级别的精度，只保留cm级别的精度。
     */
    current_distance_left = allDistance.getLeftDistance()/10; // NOLINT(bugprone-integer-division)
    current_distance_right = allDistance.getRightDistance()/10; // NOLINT(bugprone-integer-division)
    current_distance_front = allDistance.getFrontDistance()/10; // NOLINT(bugprone-integer-division)
    if (!(status & STATUS_RUNNING_MASK)) {
        // 系统等待启动
        speedLeft = 0, speedRight = 0;
        return;
    }  //启动判断
#else
    current_distance_left = 0;
    current_distance_right = 0;
    current_distance_front = 0;
#endif
    if (status & STATUS_TURNING_MASK) {
        // 系统正在转弯, 检查转弯是否完成。
        LOGI("Turning")
        auto pidResult = pid.update(yaw);
        speedLeft = pidResult;
        speedRight = -pidResult;
        if (pidResult < 5) //认为转弯已经完成
        {
            LOGI("Turn Finished")
            status &= ~STATUS_TURNING_MASK;
        }
        return;
    } //转弯判断
    if (status & STATUS_MANUAL_CONTROL_MASK) {
        // 手动模式启用
        ManualMode();
        return;
    }  //手动模式
    if (status & STATUS_RUNNING_MASK && !(status & STATUS_ENTERED)) {
        // 系统已启动，但未进入迷宫
        static bool initialized = false;
        if (!initialized) {
            enterTheMaze();
            initialized = true;
        } else {
            if (checkEnter()) {
                static int count = 0;
                if (count == 0)
                {
                    encoder.reset();
                }
                if (count++ <300) {
                    pid.setTarget(0);
                    auto pidResult = pid.update(yaw);
                    speedLeft = auto_speed + pidResult;
                    speedRight = auto_speed - pidResult;
                    return;
                }
                status = status | STATUS_ENTERED;
                status = status | STATUS_ARRIVED_MASK;
                LOGI("Entered the maze")
                initialized = false;
                return;
            } else {
                    pid.setTarget(0);
                    auto pidResult = pid.update(yaw);
                    speedLeft = auto_speed + pidResult;
                    speedRight = auto_speed - pidResult;
                return;
            }
        }
    } //进入判断
    if (status & STATUS_TURNING_MASK) {
        // 系统正在转弯, 检查转弯是否完成。
        LOGV("Turning")
        auto pidResult = pid.update(yaw);
        speedLeft = pidResult;
        speedRight = -pidResult;
        if (getAngleOffset() < 5) //认为转弯已经完成
        {
            status &= ~STATUS_TURNING_MASK;
        }
        return;
    } //转弯判断
    if (status & STATUS_MAP_BUILD_MASK) {
        LOGI("Building Map finished")
        ///  地图已完成建立
        if (status & STATUS_EXIT_WALK_OUT_MASK) {
            /// 到达出口，只差最后一步，走出迷宫
            LOGI("Walking out")
            auto pidResult = pid.update(yaw);
            speedLeft = auto_speed + pidResult;
            speedRight = auto_speed - pidResult;
            if (!goOutTicker.active()) {
                goOutTicker.attach_ms(1000, []() {
                    status = status | STATUS_ARRIVED_EXIT_MASK;
                    status = status & (~STATUS_RUNNING_MASK);
                    LOGW("Exited the maze")
                });
            }
            if (!(status & STATUS_ARRIVED_MASK)) {
                /// 当前系统有下一个点的目标，但是仍然没有到达下一个点
                moveToTarget();
                return;
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
                LOGW("Exited the maze,system stopped")
                return;
            } else {
                LOGE("status error, status can't be recognized。Current at map build finished")
                return;
            }
        }
    }
    /// 剩下的情况应该是地图未建立
    if (status & STATUS_ARRIVED_MASK) {
        /// 系统到达了上次行动要求的目标点
        if (!(status & STATUS_SCAN_MASK)) {
            scan_result = getNearbyInfo();
            LOGI("Scan result:"+String(scan_result[0])+","+String(scan_result[1])+","+String(scan_result[2]))
            status = status | STATUS_SCAN_MASK;//标志系统扫描完成
            status = status & (~STATUS_TARGET_SELECTED_MASK);
            LOGI("Current position scan finished")
        }
        if (!(status & STATUS_FIND_TREASURE_MASK)) {
            /// 检查当前位置是否检测到目标数字
            if (checkIfHasTreasure()) {
                //TODO: 当前处于目标数字处，接收信号左右移动以保证视觉系统能够识别到数字
                LOGI("Treasure found")
                delay(100);
            }
            LOGI("Current position check finished")
        }
        /// 系统目标需要更多步才能完成
        if (status&STATUS_MULTI_STEP_MASK)
        {
            LOGI("Multi step!")
            try {
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
            return;
        }
    } //系统到达目标点
    else  {
        /// 系统尚未未到达上次行动要求的目标点
        LOGI("Moving to target")
        moveToTarget();
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
    if (pid.getConfig().target < -90)
    {
        pid.setTarget(180);
    } else
    pid.setTarget(pid.getConfig().target - 90);
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
    if (pid.getConfig().target > 90)
    {
        pid.setTarget(-180);
    } else
    pid.setTarget(pid.getConfig().target + 90);
}

void Control::goStraight() {
    LOGI("go straight")
    auto_speed = 100;
    status &= ~STATUS_ARRIVED_MASK;
    status &= ~STATUS_SCAN_MASK;
    status &= ~STATUS_FIND_TREASURE_MASK;

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
    if (pid.getConfig().target == 0)
    {
        pid.setTarget(180);
    } else if (pid.getConfig().target == 90)
    {
        pid.setTarget(-90);
    } else if (pid.getConfig().target == -90)
    {
        pid.setTarget(90);
    } else
    {
        pid.setTarget(0);
    }

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
    if (currentPos == 25)
    {
        status = status | STATUS_EXIT_WALK_OUT_MASK;
        return;
    }
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
        status &= ~STATUS_TARGET_OFFSET;
        status = status | STATUS_TARGET(target);
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
        auto pidResult = pid.update(yaw);
        speedLeft = 0 + pidResult;
        speedRight = 0 - pidResult;
        return;
    } else
    {
        auto pidResult = pid.update(yaw);
        speedLeft = auto_speed + pidResult;
        speedRight = auto_speed - pidResult;
        return;
    }
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
    auto x_offset = encoder.getX_offset();
    auto y_offset = encoder.getY_offset();
    if (y_offset>200)
    {
        y_offset = 200;
        LOGE("y_offset overflow")
    }
    if (x_offset>200)
    {
        x_offset = 200;
        LOGE("x_offset overflow")
    }
    return mapInfo[(static_cast<int>(x_offset + 20) / 40)][(static_cast<int>(y_offset + 20) / 40)];
}

void Control::updateMap(int start, int end, bool isReachable) {
    map[start][end] = isReachable;
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
/**
 * @brief 处理多步到达目标点的情况
 * @param inlineCall 是否在规划路径时调用
 * @attention 该函数会涉及到数组越界的问题，因此需要在调用时进行异常处理
 */
void Control::processMultiStep(bool inlineCall) {
    auto currentPos = getCurrentPosition();
    if (inlineCall) { //从内联调用时，检查是否能单步完成
        auto targetPos = ((status & STATUS_TARGET_MASK) >> STATUS_TARGET_OFFSET);
        switch ((status & STATUS_DIRECTION_MASK) >> STATUS_DIRECTION_OFFSET) {
            case 0: //当前朝向为下方
            {
                LOGI("Current direction: down")
                LOGI("Target position:"+  String(targetPos))
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
                LOGI("Current direction: right")
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
                LOGI("Current direction: up")
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
                LOGI("Current direction: left")
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
    else { //从外部调用时，检查是否已经到达目标点
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

std::vector<int> Control::getDistance() {
    std::vector<int> dist;
    dist.resize(3);
    dist[0] = current_distance_left;
    dist[1] = current_distance_front;
    dist[2] = current_distance_right;
    return dist;
}

std::vector<float> Control::getX_Y() {
    vector<float> result = {static_cast<float>(encoder.getX_offset()), static_cast<float>(encoder.getY_offset())};
    return result;
}

int Control::getAngleOffset() {
    switch (status & STATUS_DIRECTION_MASK >> STATUS_DIRECTION_OFFSET) {
        case 0:
            return 0;
        case 1:
            return 90;
        case 2:
            return 180;
        case 3:
            return 270;
        default:
            return -1;
    }
}


#pragma clang diagnostic pop
