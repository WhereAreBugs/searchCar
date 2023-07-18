//
// Created by 神奇bug在哪里 on 7/17/23.
//

//#include <cstdint>
#include "Serial.h"
#include "Control.h"
#include "Status.h"
#include "logger.h"
#include "AsyncTCP.h"
#include <iostream>
extern int  speed;
extern int  angle;
extern uint64_t status;
extern Control globalControl;
extern AsyncClient tcp;

void globalCommandCallback(String command)
{
    if (strstr(command.c_str(),"go")!= nullptr)
    {
        int target;
        sscanf(command.c_str(),"go%d",&target);
        globalControl.ManualGo(target);
        sLOGI("go %d",target)
        tLOGI("go: " + String(target))
    } else if (strstr(command.c_str(),"setManual")!= nullptr)
    {
        int  target;
        sscanf(command.c_str(),"setManual%d",&target);
        if (target)
        {
            //启用手动控制
            status|=STATUS_MANUAL_CONTROL_MASK;
            status&=~STATUS_TARGET_SELECTED_MASK;
            LOGI("Manual control enabled")
        } else
        {
            status&=~STATUS_MANUAL_CONTROL_MASK;
            LOGI("Manual control disabled")
        }
    } else if (strstr(command.c_str(),"setKP")!= nullptr)
    {
        float target;
        sscanf(command.c_str(),"setKP%f",&target);
        globalControl.setKP(target);
        tLOGI("set KP to " + String(target))
    } else if (strstr(command.c_str(),"setKI")!= nullptr)
    {
        float target;
        sscanf(command.c_str(),"setKI%f",&target);
        globalControl.setKI(target);
        tLOGI("set KI to " + String(target))
    } else if (strstr(command.c_str(),"setKD")!= nullptr)
    {
        float target;
        sscanf(command.c_str(),"setKD%f",&target);
        globalControl.setKD(target);
        tLOGI("set KD to " + String(target))
    } else if (strstr(command.c_str(),"setSpeed")!= nullptr)
    {
        float target;
        sscanf(command.c_str(),"setSpeed%f",&target);
        speed = target;
        tLOGI("set speed to " + String(target))
    } else if (strstr(command.c_str(),"setAngle")!= nullptr){
        float target;
        sscanf(command.c_str(),"setAngle%f",&target);
        angle = target;
        tLOGI("set angle to " + String(target))
    } else if (strstr(command.c_str(),"setRunning")!= nullptr)
    {
        float target;
        sscanf(command.c_str(),"setRunning%d",&target);
        if (target)
        {
            status|=STATUS_RUNNING_MASK;
            tLOGI("set running to true")
        } else
        {
            status&=~STATUS_RUNNING_MASK;
            tLOGI("set running to false")
        }
    } else if (strstr(command.c_str(),"getCurrentMap")!= nullptr)
    {
        auto map = globalControl.getMap();
        String mapString;
        for (int i = 0; i < 8; ++i) {
            for (int j = 0; j <8 ; ++j) {
                mapString+=String(map[i][j])+" ";
            }
            mapString+="\n";
        }
        LOGI("Map:  "+mapString)
    }
    else if(strstr(command.c_str(),"getCurrentPositon")!= nullptr)
    {
        auto i = globalControl.getCurrentPosition();
        LOGI("Current position: "+String(i))
    }
    else
    {
        tLOGE("Unknown command: "+command)
    }
}