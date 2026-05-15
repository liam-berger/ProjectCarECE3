#pragma once

template <typename T>
inline constexpr T clamp(T x, T lo, T hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

template <typename T>
inline constexpr int abs(T x)
{
    return x < 0 ? -x : x;
}

template <typename T>
inline constexpr int direction(T x)
{
    return x > 0 ? LOW : HIGH;
}

inline constexpr int flip(int direction)
{
    if (direction == HIGH)
        return LOW;
    return HIGH;
}
