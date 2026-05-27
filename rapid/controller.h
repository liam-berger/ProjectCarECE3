#pragma once
#include "command.h"

class Controller
{
public:
    MotorCommand update(float raw_error);

    void reset();

private:
    // State
    float e_filt = 0.0f;      // Filtered (weighted sum of current and past error)
    float prev_e_filt = 0.0f; // Last step's filtered error to to calculate
    float d_filt = 0.0f;
    unsigned long last_ms = 0;
};
