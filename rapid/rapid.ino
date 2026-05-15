// V1.4 Architecture

#include <ECE3.h>
#include "pins.h"
#include "helper.h"
#include "command.h"
#include "timers.h"
#include "car.h"
#include "controller.h"
#include "sensors.h"

// Create global objects
Sensors sensors;
Controller controller;
Car car;
int track_type = 0; // 0 unset, 1 big, 2 small

void setup()
{
  pinMode(Pins::left_nslp_pin, OUTPUT);
  pinMode(Pins::left_dir_pin, OUTPUT);
  pinMode(Pins::left_pwm_pin, OUTPUT);

  pinMode(Pins::right_nslp_pin, OUTPUT);
  pinMode(Pins::right_dir_pin, OUTPUT);
  pinMode(Pins::right_pwm_pin, OUTPUT);

  digitalWrite(Pins::left_dir_pin, LOW);
  digitalWrite(Pins::left_nslp_pin, HIGH);

  digitalWrite(Pins::right_dir_pin, LOW);
  digitalWrite(Pins::right_nslp_pin, HIGH);

  pinMode(Pins::led_rf_pin, OUTPUT);

  ECE3_Init();
  Serial.begin(9600);

  // Timers::get().set_durations(1.0);
  // Timers::get().set_duration(2, 0.2);
  // Timers::get().set_duration(3, 0.1);
  // Timers::get().set_enabled(false); // Disable timers

  while (track_type == 0)
  {
    if (digitalRead(Pins::s1))
      track_type = 1;
    else if (digitalRead(Pins::s2))
      track_type = 2;
  }

  int wait_ms = 1000;
  int blinks = 10;
  for (int i = 0; i < blinks; ++i)
  {
    delay(wait_ms / blinks / 2);
    digitalWrite(Pins::led_rf_pin, HIGH);
    delay(wait_ms / blinks / 2);
    digitalWrite(Pins::led_rf_pin, LOW);
  }
}

void loop()
{
  ECE3_read_IR(sensors.values());

  float error = sensors.error();
  bool solid = sensors.solid();
  bool reset = car.respond_solid(solid);

  if (reset)
    controller.reset();
  MotorCommand command = controller.update(error);

  car.drive(command);

  // if (Timers::get().ready(1))
  // {
  //   Serial.print("Left: ");
  //   Serial.println(command.left);
  //   Serial.print("Right: ");
  //   Serial.println(command.right);
  //   Timers::get().reset(1);
  // }
}
