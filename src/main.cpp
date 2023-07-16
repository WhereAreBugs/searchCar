#include <Arduino.h>
#include <MPU9250.h>
#include "extEEPROM.h"
#include <VL53L0X.h>
#include "WiFi.h"
#include "AsyncTCP.h"
#include "logger.h"
#include "Motor.h"
#include "Control.h"
#include "settings.h"

inline MPU9250Setting getMPUsetthings()
{
    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;
    return setting;
}
inline void readEEProm()
{

}
extEEPROM eep(kbits_2, 1, 8,0x50);
MPU9250 mpu;
VL53L0X tofLeft;
VL53L0X tofRight;
VL53L0X tofFront;
AsyncClient tcp;
Motor globalMotor(32, 33, 25, 26);
Control globalControl(PIDConfig(TRUN_KP, TRUN_KI, TRUN_KD, -255, 0, 255));
/// 对象列表
inline bool SetupAllTof(){
    bool flag = true;
    tofLeft.setTimeout(500);
    tofLeft.setAddress(0x30);
    tofLeft.setSignalRateLimit(0.15);
    tofLeft.setMeasurementTimingBudget(200000);
    tofRight.setTimeout(500);
    tofRight.setAddress(0x31);
    tofRight.setSignalRateLimit(0.15);
    tofRight.setMeasurementTimingBudget(200000);
    tofFront.setTimeout(500);
    tofFront.setAddress(0x32);
    tofFront.setSignalRateLimit(0.15);
    tofFront.setMeasurementTimingBudget(33000); // 由于前方在快速移动，所以测量时间要短一些
    if (tofLeft.init()){
        Serial.println("[E] tofLeft init failed");
        flag = false;
    }
    if (tofRight.init()){
        Serial.println("[E] tofRight init failed");
        flag = false;
    }
    if (tofFront.init()){
        Serial.println("[E] tofFront init failed");
        flag = false;
    }
    return flag;
}
/// promotes



void setup() {
    ///板载资源初始化
    Serial.begin(115200);
    Wire.begin(21,22); //MPU9250连接的I2C总线
    Wire1.begin(23,19); //4个激光测距模块连接的I2C总线
    ///WIFI初始化
    WiFi.begin("exp","cuit1234");
    uint8_t wifiCount = 0;
    while (WiFiClass::status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(500);
        if (wifiCount<20)
        {
            wifiCount++;
        }else{
            Serial.println("WIFI connect failed");
            break;
        }
    }
    ///配置MPU9250
    if (!mpu.setup(0x68, getMPUsetthings(),Wire)) {//MPU9250的I2C地址为0x68
        LOGE("MPU9250 setup failed")
    }
    ///MPU9250校准
    mpu.calibrateAccelGyro();
    mpu.calibrateMag();
    ///配置EEPROM
    if (uint8_t i = eep.begin(extEEPROM::twiClock100kHz, &Wire)){
        LOGE("EEPROM setup failed")
    }
    ///配置激光测距模块
    if (!SetupAllTof()){
        LOGE("TOF setup failed")
    }
    ///配置电机


}

void loop() {
    mpu.update();
    globalMotor.update();
}