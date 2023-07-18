//
// Created by 神奇bug在哪里 on 7/10/23.
//

#ifndef ESP32_MAIN_LOGGER_H
#define ESP32_MAIN_LOGGER_H
#define DISABLE_SERIAL_LOG

#ifndef DISABLE_SERIAL_LOG
#define sLOGI(...) Serial.printf("[I]");Serial.printf(__VA_ARGS__);Serial.println();
#define sLOGW(...) Serial.printf("[W]");Serial.printf(__VA_ARGS__);Serial.println();
#define sLOGE(...) Serial.printf("[E]");Serial.printf(__VA_ARGS__);Serial.println();
#define sLOGD(...) Serial.printf("[D]");Serial.printf(__VA_ARGS__);Serial.println();
#define sLOGV(...) Serial.printf("[V]");Serial.printf(__VA_ARGS__);Serial.println();
#else
#define sLOGI(...)
#define sLOGW(...)
#define sLOGE(...)
#define sLOGD(...)
#define sLOGV(...)
#endif
#ifndef DISABLE_TCP_LOG
#define tLOGV(...) String temp; temp += "[V]";temp += __VA_ARGS__;temp += "\n";tcp.write(temp.c_str(),temp.length());
#define tLOGD(...) String temp; temp += "[D]";temp += __VA_ARGS__;temp += "\n";tcp.write(temp.c_str(),temp.length());
#define tLOGI(...) String temp; temp += "[I]";temp += __VA_ARGS__;temp += "\n";tcp.write(temp.c_str(),temp.length());
#define tLOGW(...) String temp; temp += "[W]";temp += __VA_ARGS__;temp += "\n";tcp.write(temp.c_str(),temp.length());
#define tLOGE(...) String temp; temp += "[E]";temp += __VA_ARGS__;temp += "\n";tcp.write(temp.c_str(),temp.length());
#else
#define tLOGV(...)
#define tLOGD(...)
#define tLOGI(...)
#define tLOGW(...)
#define tLOGE(...)
#endif

#define LOGV(...) sLOGV(__VA_ARGS__) if(tcp.connected()) {tLOGV(__VA_ARGS__)}
#define LOGD(...) sLOGD(__VA_ARGS__) if(tcp.connected()) {tLOGD(__VA_ARGS__)}
#define LOGI(...) sLOGI(__VA_ARGS__) if(tcp.connected()) {tLOGI(__VA_ARGS__)}
#define LOGW(...) sLOGW(__VA_ARGS__) if(tcp.connected()) {tLOGW(__VA_ARGS__)}
#define LOGE(...) sLOGE(__VA_ARGS__) if(tcp.connected()) {tLOGE(__VA_ARGS__)}

#endif //ESP32_MAIN_LOGGER_H
