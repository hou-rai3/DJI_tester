#include <cmath>
#include <cstdio>
#include <mbed.h>
#include <lib.hpp>

BufferedSerial pc(USBTX, USBRX, 115200);
CAN can2(PB_12, PB_13, (int)1e6);
uint8_t DATA[8] = {};
DigitalIn Userbutton(BUTTON1);
bool Userbutton_f = false;
int DJI_ID = 0x200;
int16_t target_speed = 1000;

int main() {
    Userbutton.mode(PullUp);
    while (1) {
        Userbutton_f = Userbutton.read();
        if (Userbutton_f == 0) {
            int16_t target416 = static_cast<int16_t>(target_speed);
            DATA[0] = target416 >> 8;
            DATA[1] = target416 & 0xFF;
            DATA[2] = 0;
            DATA[3] = 0;
            DATA[4] = 0;
            DATA[5] = 0;
            DATA[6] = 0;
            DATA[7] = 0;
            CANMessage DJI_msg(DJI_ID, DATA, 8);
            if (!can2.write(DJI_msg)) {
                printf("can not can\n");
            }
        } else {
            DATA[0] = 0;
            DATA[1] = 0;
            DATA[2] = 0;
            DATA[3] = 0;
            DATA[4] = 0;
            DATA[5] = 0;
            DATA[6] = 0;
            DATA[7] = 0;
            CANMessage DJI_msg(DJI_ID, DATA, 8);
            if (!can2.write(DJI_msg)) {
                printf("can not can\n");
            }
        }
    }
}

// #include "C620.hpp"
// #include "VelPid.hpp"
// #include "mbed.h"
// #include <cmath>
// #include <cstdio>
// using namespace std::chrono_literals;

// InterruptIn left_encoderA(PC_6);
// InterruptIn left_encoderB(PC_5);
// InterruptIn right_encoderA(PC_6);
// InterruptIn right_encoderB(PC_8);
// volatile int right_enc = 0;
// volatile int left_enc = 0;

// BufferedSerial pc(USBTX, USBRX, 115200);
// CAN can1(PA_11, PA_12, (int)1e6);
// uint8_t DATA[8] = {};
// DigitalIn Userbutton(BUTTON1);
// bool Userbutton_f = false;
// int DJI_ID = 0x200;
// int16_t target_speed = 1000;
// auto pre_right = HighResClock::now();
// auto pre_left = HighResClock::now();
// Const variable C620Array c620;

// constexpr float target_rpm = 15000;
// constexpr float acc_time = 1.0;
// constexpr PidGain pid_gain{0.3, P 0.0, I 0.1, D};
// constexpr float limit = 1.0;
// constexpr auto dt = 0.05s;
// VelPid pid{{pid_gain, -limit, limit}};
// volatile float left_actual_rpm = 0;
// float now_left_actual_rpm = 0;
// float pre_left_actual_rpm = 0;

// Ticker can_read_ticker;

// float right_target = 0;
// float left_target = 0;

// float left_actual_enc = 0;
// float left_actual_enc1 = 0;

// float kp = 1.0;
// float ki = 0.0;
// float kd = 0.1;

// float integral = 0.0;
// float integral1 = 0.0;

// float pre_error = 0.0;
// float pre_error_right = 0.0;
// float dt = 0.1;エンコーダの位置（カウント）
//         left_enc--;
//     }
// }
// void right_enc_read() {
//     if (right_encoderA.read() == right_encoderB.read()) {
//         right_enc++;
//     } else {
//         right_enc--;
//     }
// }

// void can_send() {
//     CANMessage msg0(0x1FF, DATA, 8);
//     can1.write(msg0);
// }

// void pid_calculation() {
//     float right_actual = right_enc;
//     float left_actual = left_enc;
//     auto now_left = HighResClock::now();
//     auto now_right = HighResClock::now();
//     if (now_right - pre_right > 10ms) {
//         float error_right = right_target - right_actual;
//         integral += error_right * 0.1;
//         float deriv = (error_right - pre_error_right) / 0.1;
//         float output_right = kp * error_right + ki * integral + kd * deriv;
//         output_right = clamp(output_right, -10000.0f, 10000.0f);
//         printf("output_right*=%f  right_actual*=%f\n", output_right,
//                right_actual);
//         int16_t out_c610 = static_cast<int16_t>(output_right);
//         DATA[0] = out_c610 >> 8;
//         DATA[1] = out_c610 & 0xFF;
//         pre_right = now_right;
//         pre_error_right = error_right;
//     }
//     if (now_left - pre_left > 10ms) {
//         float error = left_target - left_actual;
//         integral += error * 0.1;
//         float deriv = (error - pre_error) / 0.1;
//         float output_left = kp * error + ki * integral + kd * deriv;
//         output_left = clamp(output_left, -10000.0f, 10000.0f);
//         printf("output_left*=%f  left_actual*=%f\n", output_left,
//         left_actual); int16_t out_c610 = static_cast<int16_t>(output_left);
//         DATA[2] = out_c610 >> 8;
//         DATA[3] = out_c610 & 0xFF;
//         pre_left = now_left;
//         pre_error = error;
//     }
// }

// void update_encoders() {
//     left_enc_read();
//     right_enc_read();
// }

// int main() {
//     Userbutton.mode(PullUp);
//     Ticker encoder_ticker;
//     encoder_ticker.attach(&update_encoders, 10ms);
//     Ticker pid_ticker;
//     pid_ticker.attach(&pid_calculation, 10ms);

//     while (1) {
//         Userbutton_f = Userbutton.read();
//         if (Userbutton_f == 0) {
//             right_target = target_speed;
//             left_target = target_speed;
//         } else {
//             right_target = 0;
//             left_target = 0;
//         }

//         can_send();
//         ThisThread::sleep_for(10ms);
//     }
// }