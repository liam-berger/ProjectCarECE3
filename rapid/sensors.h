#pragma once
#include <Arduino.h>

// Multi-sensor data fusion
class Sensors
{
public:
    static constexpr int N = 8; // number of sensors

    uint16_t *values();

    float error();

    bool solid();

private:
    uint16_t raw[N];

    // Computed from spreadsheet
    const uint16_t min_vals[N] = {669, 734, 618, 757, 614, 711, 734, 805};
    const uint16_t max_vals[N] = {1652, 1635, 1039, 1565, 853, 1515, 1754, 1695};

    float last_error = 0;
    float last_sum = 0;

    float normalize(int sensor, uint16_t value);
    // Positions of sensors
    // Calculated via weighted average from spreadsheet
    // right -> left, same order as sensors 0 -> 7
    const float pos[8] = {
        -33.893f,
        -24.29f,
        -14.351f,
        -5.069f,
        5.307f,
        13.907f,
        23.785f,
        33.369f,
    };
};
