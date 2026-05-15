#include <Arduino.h>
#include "controller.h"
#include "tuning.h"
#include "helper.h"

MotorCommand Controller::update(float raw_error)
{
    float dt = static_cast<float>(millis() - last_ms) / 1000.0;
    dt = clamp<float>(dt, 0.001, 0.1);
    last_ms = millis();

    // Filtered error as linear combination of current and past error
    e_filt = Tuning::alpha_e * raw_error + (1.0f - Tuning::alpha_e) * e_filt;

    float raw_d = (e_filt - prev_e_filt) / dt;
    prev_e_filt = e_filt;

    d_filt = Tuning::alpha_d * raw_d + (1.0f - Tuning::alpha_d) * d_filt;

    float turn = (Tuning::kp * e_filt + Tuning::kd * d_filt) * Tuning::base_speed * Tuning::turn_mult * 2;
    turn = clamp<float>(turn, -Tuning::base_speed * Tuning::max_turn, Tuning::base_speed * Tuning::max_turn);

    // if (Timers::get().ready(0))
    // {
    //   Serial.print("Position: ");
    //   Serial.println(Tuning::kp * e_filt * Tuning::base_speed * Tuning::turn_mult);
    //   Serial.print("Derivative: ");
    //   Serial.println(Tuning::kd * d_filt * Tuning::base_speed * Tuning::turn_mult);
    //   Timers::get().reset(0);
    // }

    return {
        clamp<int>(int(Tuning::base_speed - turn), -255, 255),
        clamp<int>(int(Tuning::base_speed + turn), -255, 255),
        clamp<int>(abs(int(Tuning::base_speed - turn)), 0, 255),
        clamp<int>(abs(int(Tuning::base_speed + turn)), 0, 255),
        direction(Tuning::base_speed - turn),
        direction(Tuning::base_speed + turn),
    };
}

void Controller::reset()
{
    e_filt = 0;
    d_filt = 0;
    last_ms = 0;
}
