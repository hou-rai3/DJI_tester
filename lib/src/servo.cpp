#include "servo.hpp"
#include "mbed.h"

void SERVO::set_angle(uint8_t id, uint8_t angle)
{
    if (id >= 8) {
        printf("Error: Invalid servo ID %d. Program will terminate.\n", id);
    }
    if (id < MAX_SERVO_COUNT) {
        servo_angles[id] = angle;
    }
}

uint8_t SERVO::get_angle(uint8_t id)
{
    if (id >= 8) {
        printf("Error: Invalid servo ID %d. Program will terminate.\n", id);
    }
    if (id < MAX_SERVO_COUNT) {
        return servo_angles[id];
    }
    return 0; // Default return value if id is out of range
}

bool SERVO::send_angle()
{
    CANMessage servo_msg(servo_canID, reinterpret_cast<uint8_t *>(servo_angles), 8);
    if(can.write(servo_msg)){
        success = true;
    } else {
        success = false;
    }
    return success;
}