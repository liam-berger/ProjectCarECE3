#include "sensors.h"
#include "constants.h"
#include "helper.h"

uint16_t *Sensors::values()
{
    return raw;
}

float Sensors::error()
{
    float sum = 0.0f;
    float weighted = 0.0f;

    for (int i = 0; i < N; ++i)
    {
        float v = normalize(i, raw[i]);

        v = v * v; // Sharpen peaks
        // v = v * v * v; // Even sharper

        sum += v;
        weighted += pos[i] * v;
    }

    if (sum < Tuning::lost_threshold)
    {
        // Use last line if no line is detected
        return last_error;
    }

    if (sum > Tuning::trust_threshold)
    {
        last_error = weighted / sum;
    }
    else
    {
        // if (Timers::get().ready(2))
        // {
        //   Serial.print("Untrusted sum: ");
        //   Serial.println(sum);
        //   Timers::get().reset(2);
        // }
    }

    return last_error;
}

bool Sensors::solid()
{
    float sum = 0;

    for (int i = 0; i < N; ++i)
    {
        sum += normalize(i, raw[i]);
    }

    if (last_sum >= Tuning::solid_threshold && sum >= Tuning::solid_threshold)
    {
        last_sum = 0.0f; // Forget crossbar if solid (reduce chance of immediate double response to crossbar)
        return true;
    }

    // Store last sum normally (prevent phantom crossbars)
    last_sum = sum;
    return false;
}

float Sensors::normalize(int sensor, uint16_t value)
{
    float n = (float(value) - float(min_vals[sensor])) / (float(max_vals[sensor]) - float(min_vals[sensor]));

    // Clamp away values outside of previously measure min and max
    n = clamp(n, 0.0f, 1.0f);

    // Deadband with small epsilon like the sheet formula
    const float eps = 0.05f;
    n = n * (1.0f + eps) - eps;

    // Clamp away values in the bottom epsilon to 0
    n = clamp(n, 0.0f, 1.0f);

    return n;
}
