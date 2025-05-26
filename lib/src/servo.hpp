#ifndef SERVO_HPP
#define SERVO_HPP
#include "mbed.h"

class SERVO
{
    public:
        SERVO(CAN &can,int servo_canID);
        void set_angle(uint8_t id, uint8_t angle);
        uint8_t get_angle(uint8_t id);
        bool send_angle();
    private:
        CAN &can;
        int servo_canID;
        uint8_t angle[8];
        int servo_angles[8];
        bool success;
};
int8_t  MAX_SERVO_COUNT = 8;
inline SERVO::SERVO(CAN &can, int servo_canID) : can(can) {}

#endif