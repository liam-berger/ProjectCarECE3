#include <Arduino.h>
#include "controller.h"
#include "tuning.h"
#include "helper.h"

MotorCommand Controller::update(float raw_error)
{
    unsigned long now = millis();
    float dt = static_cast<float>(now - last_ms) / 1000.0;
    dt = clamp<float>(dt, 0.001, 0.1);
    last_ms = now;

    // Filtered error as linear combination of current and past error
    e_filt = Tuning::alpha_e * raw_error + (1.0f - Tuning::alpha_e) * e_filt;

    float raw_d = (e_filt - prev_e_filt) / dt;
    prev_e_filt = e_filt;

    d_filt = Tuning::alpha_d * raw_d + (1.0f - Tuning::alpha_d) * d_filt;

    float turn = (Tuning::kp * e_filt + Tuning::kd * d_filt) * Tuning::turn_mult;
    turn = clamp<float>(turn, -Tuning::base_speed * Tuning::max_turn, Tuning::base_speed * Tuning::max_turn);

    // if (Timers::get().ready(0))
    // {
    //   Serial.print("Position: ");
    //   Serial.println(Tuning::kp * e_filt * Tuning::base_speed * Tuning::turn_mult);
    //   Serial.print("Derivative: ");
    //   Serial.println(Tuning::kd * d_filt * Tuning::base_speed * Tuning::turn_mult);
    //   Timers::get().reset(0);
    // }

    int target_left = clamp<int>(int(Tuning::base_speed - turn), -255, 255);
    int target_right = clamp<int>(int(Tuning::base_speed + turn), -255, 255);

    last_left = slew_limit(target_left, last_left, dt);
    last_right = slew_limit(target_right, last_right, dt);

    return command_from_signed(last_left, last_right);
}

void Controller::reset()
{
    e_filt = 0;
    prev_e_filt = 0;
    d_filt = 0;
    last_left = 0;
    last_right = 0;
    last_ms = 0;
}

int Controller::slew_limit(int target, int previous, float dt)
{
    return target;
    // int max_step = int(Tuning::slew_rate * dt);
    // if (max_step < 1)
    //     max_step = 1;

    // return clamp<int>(target, previous - max_step, previous + max_step);
}

MotorCommand Controller::command_from_signed(int left, int right)
{
    return {
        left,
        right,
        clamp<int>(abs(left), 0, 255),
        clamp<int>(abs(right), 0, 255),
        direction(left),
        direction(right),
    };
}
