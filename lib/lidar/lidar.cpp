#include <Arduino.h>
#include <RPLidar.h>
#include "pwmgen.h"
#include "lidar.h"

lidar rlidar;

void lidar::begin()
{
    Serial1.begin(115200);
    lidar_device.begin(Serial1);
    // lidar_device.begin(Serial2);
    lidar_motor.begin(3, 1000, 50);
}

void lidar::lidar_run(void)
{
    lidar_motor.start_pwm();
}

void lidar::lidar_stop(void)
{
    lidar_motor.stop_pwm();
}

void lidar::speed_set(int duty, int freq)
{
    lidar_motor.set_duty(duty);
    lidar_motor.set_frequency(freq);
}

coordinate lidar::coordinate_take(void)
{
    coordinate value;
    value.distance = 0;
    value.angle = 0;

    if (IS_OK(lidar_device.waitPoint()))
    {
        if (lidar_device.getCurrentPoint().quality < 20 && lidar_device.getCurrentPoint().quality > 5)
        {
            value.distance = lidar_device.getCurrentPoint().distance; // distance value in mm unit
            value.angle = lidar_device.getCurrentPoint().angle;       // angle value in degree
        }
    }
    else
    {
        this->lidar_stop();
        rplidar_response_device_info_t info;
        if (IS_OK(lidar_device.getDeviceInfo(info, 100)))
        {
            lidar_device.startScan();
            this->lidar_run();
            vTaskDelay(pdMS_TO_TICKS(10)); // Wait for LIDAR to start
        }
    }

    return value;
}
