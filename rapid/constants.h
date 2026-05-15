#pragma once

struct Tuning
{
    // Controller tuning
    // How much do we change steering in response to error?
    static constexpr float kp = 0.7f;  // Proportional
    static constexpr float kd = 0.14f; // Derivative

    // How much do we carry over past error vs new error?
    // Higher alpha is more responsive, lower alpha is more smooth
    // Effective memory length is roughly 1/alpha
    static constexpr float alpha_e = 0.25f; // Proportional
    static constexpr float alpha_d = 0.35f; // Derivative

    // Speeds
    static constexpr int base_speed = 255;     // 0 to 255
    static constexpr float turn_mult = 0.013f; // Scales turn
    static constexpr float max_turn = 1.5f;    // Motor speed from base_speed * (1 - max_turn) to base_speed * (1 + max_turn)

    // Car tuning
    static constexpr float solid_timeout = 1.0f;   // Seconds after solid to go blind to new solids
    static constexpr float donut_duration = 0.42f; // Seconds of donut
    static constexpr float donut_speed = 140;      // Speed of donut

    // Sensor tuning
    static constexpr float lost_threshold = 0.02f; // If sensor readings sum to less than this, no line is detected
    static constexpr float trust_threshold = 0.25f;
    static constexpr float solid_threshold = 7.0f;
};

struct Pins
{
    static constexpr int left_nslp_pin = 31; // nslp ==> awake & ready for PWM
    static constexpr int left_dir_pin = 29;
    static constexpr int left_pwm_pin = 40;

    static constexpr int right_nslp_pin = 11; // nslp ==> awake & ready for PWM
    static constexpr int right_dir_pin = 30;
    static constexpr int right_pwm_pin = 39;

    static constexpr int led_rf_pin = 41;
};
