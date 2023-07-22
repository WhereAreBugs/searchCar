//
// Created by 神奇bug在哪里 on 7/22/23.
//

#ifndef SEARCHCAR_SERIALENCODER_H
#define SEARCHCAR_SERIALENCODER_H

class SerialEncoder {
private:
    HardwareSerial * serial = nullptr;
    double offset_x{};
    double offset_y =0 ;
    float yaw{};
public:
    explicit SerialEncoder();
    void setup();
    void IQRHandler();
    void attchSerial(HardwareSerial * serial);
    double getX_offset() const;
    double getY_offset() const;
    void reset();
};


#endif //SEARCHCAR_SERIALENCODER_H
