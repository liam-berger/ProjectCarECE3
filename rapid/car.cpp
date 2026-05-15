#include <Arduino.h>
#include "car.h"
#include "constants.h"
#include "helper.h"

void Car::drive(MotorCommand command)
{
    if (direction == 0)
    {
        analogWrite(Pins::left_pwm_pin, 0);
        analogWrite(Pins::right_pwm_pin, 0);
        return;
    }

    if (direction == -1)
    {
        command.left_dir = flip(command.left_dir);
        command.right_dir = flip(command.right_dir);
    }

    digitalWrite(Pins::left_dir_pin, command.left_dir);
    digitalWrite(Pins::right_dir_pin, command.right_dir);

    // Speeds
    analogWrite(Pins::left_pwm_pin, command.left_mag);
    analogWrite(Pins::right_pwm_pin, command.right_mag);
}

void Car::donut()
{
    const int temp_dir = direction;
    direction = 1;

    MotorCommand donut_command{Tuning::donut_speed, -Tuning::donut_speed, Tuning::donut_speed, Tuning::donut_speed, 1, 0};
    drive(donut_command);
    delay((uint32_t)(Tuning::donut_duration * 1000));

    direction = temp_dir;
}

bool Car::respond_solid(bool solid)
{
    if (!solid || millis() - last_solid < Tuning::solid_timeout * 1000)
        return false;

    // Update state
    last_solid = millis();
    ++solid_count;

    // Reverse via donut (blocking)
    if (solid_count == 1)
    {
        donut();
        return true;
    }

    // Otherwise, stop forever
    drive(stop_command);
    while (true)
    {
    } // Wait indefinitely
    return true;
}
