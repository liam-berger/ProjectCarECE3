#pragma once

// 8 timers
class Timers
{
public:
    static Timers &get();

    bool ready(short channel);

    void reset(short channel);

    void set_duration(short channel, float seconds);

    // Set all durations
    void set_durations(float seconds);

    // Enable/disable all
    void set_enabled(bool value);

    // Enable/disable channels
    void set_enabled(short channel, bool value);

private:
    unsigned long last_ms[8] = {0};
    float durations[8] = {0.0f};
    bool enabled[8] = {true};
};
