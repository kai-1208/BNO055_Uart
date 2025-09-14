#include "mbed.h"
#include "BNO055Uart.hpp"

static UnbufferedSerial pc(USBTX, USBRX, 115200);
BNO055Uart imu(PA_9, PA_10);
float yaw_offset = 0.0f;

int main() {
    printf("BNO055 UART Standard Project\n");
    if (imu.begin()) {
        printf("BNO055 Initialized!\n");
        ThisThread::sleep_for(100ms); // imu安定するまで待つ
        imu.update();
        yaw_offset = imu.getEuler().yaw; // 最初のヨー角
        printf("Yaw offset set to: %.2f\n", yaw_offset);
    } else {
        printf("Failed to initialize BNO055.\n");
        while(1);
    }

    while (1) {
        if (imu.update()) {
            BNO055Uart::EulerAngles angles = imu.getEuler();
            float relative_yaw = angles.yaw - yaw_offset;
            // 角度を-180～180度に
            if (relative_yaw > 180.0f) relative_yaw -= 360.0f;
            if (relative_yaw < -180.0f) relative_yaw += 360.0f;

            printf("Yaw: %7.2f, Pitch: %7.2f, Roll: %7.2f\n",
                relative_yaw, angles.pitch, angles.roll);
        } else {
            printf("Failed to update sensor data.\n");
        }
        ThisThread::sleep_for(100ms);
    }
}