#include <Arduino.h>
#include "controller.h"
#include "tuning.h"
#include "helper.h"

MotorCommand Controller::update(float raw_error, int base_speed)
{
    float dt = static_cast<float>(millis() - last_ms) / 1000.0;
    dt = clamp<float>(dt, 0.001, 0.1);
    last_ms = millis();

    // float big_error = raw_error + 1.4f * sinf(PI * raw_error / 8.0f);
    // Filtered error as linear combination of current and past error
    if (-1.0f < raw_error && raw_error < 1.0f)
        raw_error = 0.0f;
    e_filt = Tuning::alpha_e * raw_error + (1.0f - Tuning::alpha_e) * e_filt;
    // Note: e_filt is roughly between -8.0 and 8.0 max, usually more like -2.5 to 2.5

    float raw_d = (e_filt - prev_e_filt) / dt;
    prev_e_filt = e_filt;

    d_filt = Tuning::alpha_d * raw_d + (1.0f - Tuning::alpha_d) * d_filt;

    // // e_filt * dt is roughly between -0.05 and 0.05 max, usually more like -0.015 to 0.015
    // sum_e += e_filt * dt;
    // sum_e = clamp(sum_e, -Tuning::max_i, Tuning::max_i);
    // if (-1.0f < e_filt && e_filt < 1.0f)
    //     sum_e *= 0.90;

    float turn = (Tuning::kp * e_filt + Tuning::kd * d_filt + Tuning::ki * sum_e) * base_speed * Tuning::turn_mult * 2;
    turn = clamp<float>(turn, -base_speed * Tuning::max_turn, base_speed * Tuning::max_turn);

    // if (Timers::get().ready(0))
    // {
    //   Serial.print("Position: ");
    //   Serial.println(Tuning::kp * e_filt * base_speed * Tuning::turn_mult);
    //   Serial.print("Derivative: ");
    //   Serial.println(Tuning::kd * d_filt * base_speed * Tuning::turn_mult);
    //   Timers::get().reset(0);
    // }

    return {
        clamp<int>(int(base_speed - turn), -255, 255),
        clamp<int>(int(base_speed + turn), -255, 255),
        clamp<int>(abs(int(base_speed - turn)), 0, 255),
        clamp<int>(abs(int(base_speed + turn)), 0, 255),
        direction(base_speed - turn),
        direction(base_speed + turn),
    };
}

void Controller::reset()
{
    e_filt = 0;
    prev_e_filt = 0;
    d_filt = 0;
    sum_e = 0;
    last_ms = 0;
}
