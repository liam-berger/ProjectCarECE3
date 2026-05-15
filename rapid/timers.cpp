#include "timers.h"
#include <Arduino.h>

Timers &Timers::get()
{
    static Timers instance;
    return instance;
}

bool Timers::ready(short channel)
{
    if (enabled[channel])
        return false;
    return millis() - last_ms[channel] > durations[channel] * 1000;
}

void Timers::reset(short channel)
{
    last_ms[channel] = millis();
}

void Timers::set_duration(short channel, float seconds)
{
    durations[channel] = seconds;
}

// Set all durations
void Timers::set_durations(float seconds)
{
    for (int i = 0; i < 8; ++i)
        set_duration(i, seconds);
}

// Enable/disable all
void Timers::set_enabled(bool value)
{
    for (int i = 0; i < 8; ++i)
    {
        enabled[i] = value;
    }
}

// Enable/disable channels
void Timers::set_enabled(short channel, bool value)
{
    enabled[channel] = value;
}
