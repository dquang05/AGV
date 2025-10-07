#pragma once
#ifndef LIDAR_H
#define LIDAR_H
#include <RPLidar.h>
#include "pwmgen.h"

typedef struct
{
    float distance;
    float angle;
} coordinate;

class lidar
{
public:
    void begin(void);
    void lidar_run(void);
    void lidar_stop(void);
    void speed_set(int duty, int freq);
    coordinate coordinate_take(void);
    RPLidar lidar_device;
    pwm_gen lidar_motor;
};

extern lidar rlidar;
#endif