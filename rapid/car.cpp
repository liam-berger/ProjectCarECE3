#include <Arduino.h>
#include "car.h"
#include "pins.h"
#include "tuning.h"
#include "helper.h"

void Car::drive(MotorCommand command)
{
    if (direction == 0)
    {
        analogWrite(Pins::left_pwm, 0);
        analogWrite(Pins::right_pwm, 0);
        return;
    }

    if (direction == -1)
    {
        command.left_dir = flip(command.left_dir);
        command.right_dir = flip(command.right_dir);
    }

    digitalWrite(Pins::left_dir, command.left_dir);
    digitalWrite(Pins::right_dir, command.right_dir);

    // Speeds
    analogWrite(Pins::left_pwm, command.left_mag);
    analogWrite(Pins::right_pwm, command.right_mag);
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

void Car::roundabout()
{
    const int temp_dir = direction;
    direction = 1;

    MotorCommand rb_one{Tuning::rb_speed, Tuning::rb_speed, Tuning::rb_speed, Tuning::rb_speed,
                        solid_count == 1 ? 1 : 0,
                        solid_count == 1 ? 0 : 1};
    MotorCommand rb_one{Tuning::rb_speed, Tuning::rb_speed, Tuning::rb_speed, Tuning::rb_speed,
                        solid_count == 1 ? 0 : 1,
                        solid_count == 1 ? 1 : 0};

    drive(rb_one);
    delay((uint32_t)(Tuning::rb_one_duration * 1000));
    drive(rb_two);
    delay((uint32_t)(Tuning::rb_two_duration * 1000));
    drive(rb_one);
    delay((uint32_t)(Tuning::rb_one_duration * 1000));

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

bool Car::t2_respond_solid(bool solid)
{
    if (!solid || millis() - last_solid < Tuning::solid_timeout * 1000)
        return false;

    // Update state
    last_solid = millis();
    ++solid_count;

    // Reverse via donut (blocking)
    if (solid_count == 1 || solid_count == 3)
    {
        roundabout();
        return true;
    } 
    else if (solid_count == 2)
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

