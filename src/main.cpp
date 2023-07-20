#include <Arduino.h>

#include "Adafruit_VL53L0X.h"
#include "WiFi.h"
#include "AsyncTCP.h"
#include "logger.h"
#include "Motor.h"
#include "Control.h"
#include "settings.h"
#include "Distance.h"
#include "Serial.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "esp32-hal-timer.h"

#ifndef DUSE_OTA

#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <esp32-hal-timer.h>
#include <hal/timer_types.h>

#endif



extern float yaw,pitch, roll,accx;
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();
MPU6050 mpu;
AsyncClient tcp;
Motor globalMotor(33, 32, 25, 26);
Control globalControl(PIDConfig(TRUN_KP, TRUN_KI, TRUN_KD, -255, 0, 255));
/// 对象列表
Distance allDistance(&lox1, &lox2, &lox3, &lox4);
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
hw_timer_t *timer = nullptr;
// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31


// set the pins to shut down
#define SHT_LOX1 16
#define SHT_LOX2 18
#define SHT_LOX3 17
// objects for the vl53l0x


// this holds the measurement


/// DMP
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

#define QUICKDEBUG
#define INTERRUPT_PIN 35
/// promotes

void I2C1_start(void *pVoid)
{
    mpu.initialize();
    int devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println();
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    while (1)
    {
        // if programming failed, don't try to do anything
        if (!dmpReady) break;
        // read a packet from FIFO
        vTaskDelay(1);
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yaw = (ypr[0] * 180 / M_PI);
            pitch = (ypr[1] * 180 / M_PI);
            roll = (ypr[2] * 180 / M_PI);
        }
    }
}
void I2C2_start(void *pVoid){
    pinMode(SHT_LOX1, OUTPUT);
    pinMode(SHT_LOX2, OUTPUT);
    pinMode(SHT_LOX3, OUTPUT);
    delay(10);
    // all reset
    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    digitalWrite(SHT_LOX3, LOW);
    delay(10);
    // all unreset
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, HIGH);
    digitalWrite(SHT_LOX3, HIGH);
    delay(10);

    // activating LOX1 and resetting LOX2
    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, LOW);
    digitalWrite(SHT_LOX3, LOW);

    delay(10);
    // initing LOX1
    if(!lox1.begin(LOX1_ADDRESS, false,&Wire1)) {
        Serial.println(F("Failed to boot first VL53L0X"));
    }
    delay(10);

    // activating LOX2
    digitalWrite(SHT_LOX2, HIGH);
    delay(10);

    //initing LOX2
    if(!lox2.begin(LOX2_ADDRESS, false,&Wire1,Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE)) {
        Serial.println(F("Failed to boot second VL53L0X"));

    }
    delay(10);
    digitalWrite(SHT_LOX3, HIGH);
    delay(10);
    Serial.println("lox2 ok!");
    if (!lox3.begin(0x32, false, &Wire1)) {
        LOGE("Failed to boot third VL53L0X")
        Serial.println("Failed to boot third VL53L0X");
    }

    Serial.println("Summary: lox1: " + String(lox1.Status) + " lox2: " + String(lox2.Status)+"lox3: " + String(lox3.Status)+"lox4: " + String(lox4.Status));

    while (1) {
        allDistance.update();
        vTaskDelay(500);
    }

}


void setup() {
    ///板载资源初始化
    Serial.begin(115200);
    ///配置电机

#ifndef QUICKDEBUG
    Serial.setPins(27, 14);
#endif

#ifndef DISABLE_I2C
    Wire.begin(21, 22); //MPU9250连接的I2C总线
    delay(2000);///保证I2C初始化
    Wire1.begin(23, 19); //4个激光测距模块连接的I2C总线
#endif
    ///WIFI初始化
#ifdef USE_ROUTER_WIFI
    WiFi.begin("exp", "Cuit1234");
#else
    WiFi.begin("esp01s","Cuit1234");
#endif
    uint8_t wifiCount = 0;
    while (WiFiClass::status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
        if (wifiCount < 20) {
            wifiCount++;
        } else {
            Serial.println("WIFI connect failed");
            break;
        }
    }
    Serial.println("WIFI connect success"+WiFi.localIP().toString());
    delay(2000);//保证I2C初始化
    ///配置TCP
#ifdef USE_ROUTER_WIFI
    tcp.connect(IPAddress(192, 168, 1, 131), 2333);
#else
    tcp.connect(WiFi.gatewayIP(), 1234);
#endif
    tcp.onConnect([](void *arg, AsyncClient *client) {
        LOGI("TCP connect success")
    }, nullptr);
    tcp.onData([](void *arg, AsyncClient *client, void *data, size_t len) {
        globalCommandCallback((char *) data);
    }, nullptr);
    delay(500);//保证TCP初始化
    ///配置MPU9250
#ifndef DISABLE_I2C
    xTaskCreate(I2C1_start, "I2C1_start", 4096, nullptr, 1, nullptr);
    ///配置激光测距模块
    xTaskCreate(I2C2_start, "I2C2_start", 4096, nullptr, 1, nullptr);
#endif
#ifndef DUSE_OTA
    /// OTA初始化
    ArduinoOTA.onStart([]() {
                String type;
                if (ArduinoOTA.getCommand() == U_FLASH)
                    type = "sketch";
                else // U_SPIFFS
                    type = "filesystem";

                // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                LOGI("Start updating " + type)
            })
            .onEnd([]() {
                LOGI("\nEnd")
            })
            .onProgress([](unsigned int progress, unsigned int total) {
                LOGI("Progress:" + String((progress / (total / 100))) + "\r")
            })
            .onError([](ota_error_t error) {
                LOGI("Error" + String(error) + " : ")
                if (error == OTA_AUTH_ERROR) { LOGI("Auth Failed")}
                else if (error == OTA_BEGIN_ERROR) { LOGI("Begin Failed")}
                else if (error == OTA_CONNECT_ERROR) { LOGI("Connect Failed")}
                else if (error == OTA_RECEIVE_ERROR) { LOGI("Receive Failed")}
                else if (error == OTA_END_ERROR) { LOGI("End Failed")}
            });
    ArduinoOTA.setPassword("Cuit1234");
    ArduinoOTA.setPort(3232);
    ArduinoOTA.begin();
    if (!MDNS.begin("esp32")) {
        LOGE("MDNS setup failed")
    }
    LOGI("OTA setup complete")
    LOGI("IP address: " + WiFi.localIP().toString())
#endif

    ///配置按键中断
#ifndef QUICKDEBUG
    pinMode(1, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);
    attachInterrupt(1, []() {
        if (status & STATUS_RUNNING_MASK)
            status = status & (~STATUS_RUNNING_MASK);
        else
            status = status | STATUS_RUNNING_MASK;
    }, RISING);
    attachInterrupt(3, []() {
        ///MPU9250校准
        mpu.calibrateAccelGyro();
        mpu.calibrateMag();
        LOGI("MPU9250 calibrate complete")
    }, RISING);
#endif
    globalMotor.setup();
    ///配置定时器
#ifndef SOFTPWM
#else
    hw_timer_t *timer__ = timerBegin(1, 80, true);
    timerAttachInterrupt((hw_timer_t *) timer__, [](){
        globalMotor.IQRHandler();
        }, true);
    timerAlarmWrite(timer__, 50, true);
    timerAlarmEnable(timer__);
#endif
    Serial.println("Motor setup complete");
    Serial.setRxTimeout(50);
    Serial.setTimeout(50);
    Serial.onReceive([]() {
        String data = Serial.readStringUntil(0x68);
        globalOpenMVCallback(data);
    });
}


void loop() {
   globalControl.update();
    /// 同步速度到电机
    globalMotor.setSpeed(0, globalControl.getSpeedLeft());
    globalMotor.setSpeed(1, globalControl.getSpeedRight());
#ifndef SOFTPWM
    globalMotor.update();
#endif
#ifndef DUSE_OTA
    ArduinoOTA.handle();
#endif
}
