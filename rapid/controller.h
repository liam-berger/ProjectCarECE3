#pragma once
#include "command.h"

class Controller
{
public:
    MotorCommand update(float raw_error);
    MotorCommand update(float raw_error, float curvature);

    void reset();

private:
    int slew_limit(int target, int previous, float dt);
    MotorCommand command_from_signed(int left, int right);

    // State
    float e_filt = 0.0f;      // Filtered (weighted sum of current and past error)
    float prev_e_filt = 0.0f; // Last step's filtered error to to calculate
    float d_filt = 0.0f;
    int last_left = 0;
    int last_right = 0;
    unsigned long last_ms = 0;
};
