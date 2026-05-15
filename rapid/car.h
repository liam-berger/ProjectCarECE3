#pragma once
#include "command.h"

class Car
{
public:
    void drive(MotorCommand command);

    void donut();

    bool respond_solid(bool solid);

private:
    // State
    short direction = 1;          // 1 forward, 0 stop, -1 backward
    int solid_count = 0;          // 0 is start, 1 is reverse, 2 is stop
    unsigned long last_solid = 0; // Time in ms of last solid
};
