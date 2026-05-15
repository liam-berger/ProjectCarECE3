#pragma once

struct MotorCommand
{
  int left;
  int right;
  int left_mag;
  int right_mag;
  short left_dir;
  short right_dir;
};

const MotorCommand stop_command{0, 0, 0, 0, 0, 0};
