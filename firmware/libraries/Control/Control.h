#ifndef CONTROL_H
#define CONTROL_H

#include "Arduino.h"
const uint8_t MAGIC_NUMBER = 213;
const int PACKET_SIZE = 32;

struct Control {
    uint8_t magic = MAGIC_NUMBER;

    //armed
    uint8_t armed = 0;

    //gimbals
    uint8_t throttle;
    uint8_t yaw;
    uint8_t roll;
    uint8_t pitch;

    //dpad
    uint8_t up;
    uint8_t down;
    uint8_t left;
    uint8_t right;
    uint8_t center;

    //misc
    uint8_t btn1;
    uint8_t btn2;

    uint8_t calibrate_flag = 0;
};

void updateValues(Control *temp);
void printValues(Control *temp);
void calibrate(Control *temp);
void writeIntIntoEEPROM(int address, int number);
int readIntFromEEPROM(int address);
void loadGimbalValues();
void savePIDtoEEPROM();

#endif

