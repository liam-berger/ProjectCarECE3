#pragma once

struct Pins
{
    static constexpr int left_nslp_pin = 31; // nslp ==> awake & ready for PWM
    static constexpr int left_dir_pin = 29;
    static constexpr int left_pwm_pin = 40;

    static constexpr int right_nslp_pin = 11; // nslp ==> awake & ready for PWM
    static constexpr int right_dir_pin = 30;
    static constexpr int right_pwm_pin = 39;

    static constexpr int led_rf_pin = 41;
    static constexpr int s1 = 73;
    static constexpr int s2 = 74;
};
