#pragma once

struct Pins
{
    static constexpr int left_nslp = 31; // nslp ==> awake & ready for PWM
    static constexpr int left_dir = 29;
    static constexpr int left_pwm = 40;

    static constexpr int right_nslp = 11; // nslp ==> awake & ready for PWM
    static constexpr int right_dir = 30;
    static constexpr int right_pwm = 39;

    static constexpr int led_rf = 41;
    static constexpr int led_red = 75;
    static constexpr int led_green = 76;
    static constexpr int led_blue = 77;
    static constexpr int s1 = 73;
    static constexpr int s2 = 74;
};
