#include <ECE3.h>

// Helpers

struct Pins
{
  static constexpr int left_nslp_pin = 31; // nslp ==> awake & ready for PWM
  static constexpr int left_dir_pin = 29;
  static constexpr int left_pwm_pin = 40;

  static constexpr int right_nslp_pin = 11; // nslp ==> awake & ready for PWM
  static constexpr int right_dir_pin = 30;
  static constexpr int right_pwm_pin = 39;

  static constexpr int led_rf_pin = 41;
};

template <typename T>
constexpr T clamp(T x, T lo, T hi)
{
  if (x < lo)
    return lo;
  if (x > hi)
    return hi;
  return x;
}

template <typename T>
constexpr int abs(T x)
{
  return x < 0 ? -x : x;
}

template <typename T>
constexpr int direction(T x)
{
  return x > 0 ? LOW : HIGH;
}

struct MotorCommand
{
  int left;
  int right;
  int left_mag;
  int right_mag;
  int left_dir;
  int right_dir;
};

// 8 timers
class Timers
{
public:
  static Timers &get()
  {
    static Timers instance;
    return instance;
  }

  bool ready(short channel)
  {
    return millis() - last_ms[channel] > durations[channel] * 1000;
  }

  void reset(short channel)
  {
    last_ms[channel] = millis();
  }

  void set_duration(short channel, float seconds)
  {
    durations[channel] = seconds;
  }

  void set_durations(float seconds)
  {
    for (int i = 0; i < 8; ++i)
      set_duration(i, seconds);
  }

private:
  unsigned long last_ms[8] = {0};
  float durations[8] = {0.0};
};

// Implement PD feedback control
class Controller
{
public:
  MotorCommand update(float raw_error)
  {
    float dt = static_cast<float>(millis() - last_ms) / 1000.0;
    dt = clamp<float>(dt, 0.001, 0.1);
    last_ms = millis();

    // Filtered error as linear combination of current and past error
    e_filt = alpha_e * raw_error + (1.0f - alpha_e) * e_filt;

    float raw_d = (e_filt - prev_e_filt) / dt;
    prev_e_filt = e_filt;

    d_filt = alpha_d * raw_d + (1.0f - alpha_d) * d_filt;

    float turn = (kp * e_filt + kd * d_filt) * base_speed * turn_mult * 2;
    turn = clamp<float>(turn, -base_speed * max_turn, base_speed * max_turn);

    if (Timers::get().ready(0))
    {
      Serial.print("Position: ");
      Serial.println(kp * e_filt * base_speed * turn_mult);
      Serial.print("Derivative: ");
      Serial.println(kd * d_filt * base_speed * turn_mult);
      Timers::get().reset(0);
    }

    return {
        clamp<int>(int(base_speed - turn), -255, 255),
        clamp<int>(int(base_speed + turn), -255, 255),
        clamp<int>(abs(int(base_speed - turn)), 0, 255),
        clamp<int>(abs(int(base_speed + turn)), 0, 255),
        direction(base_speed - turn),
        direction(base_speed + turn),
    };
  }

private:
  // Tuning
  // How much do we change steering in response to error?
  float kp = 1.5f;  // Proportional
  float kd = 0.03f; // Derivative

  // How much do we carry over past error vs new error?
  // Higher alpha is more responsive, lower alpha is more smooth
  // Effective memory length is roughly 1/alpha
  float alpha_e = 0.25f; // Proportional
  float alpha_d = 0.35f; // Derivative

  // Speeds
  int base_speed = 30;    // 0 to 255
  float turn_mult = 0.02; // Scales turn
  float max_turn = 2.0;   // Motor speed from base_speed * (1 - max_turn) to base_speed * (1 + max_turn)

  // State
  float e_filt = 0;      // Filterer (weighted sum of current and past error)
  float prev_e_filt = 0; // Last step's filtered error to to calculate
  float d_filt = 0;
  unsigned long last_ms = 0;
};

// Multi-sensor data fusion
class Sensors
{
public:
  static constexpr int N = 8; // number of sensors

  uint16_t *values()
  {
    return raw;
  }

  float error()
  {
    float sum = 0;
    float weighted = 0;

    for (int i = 0; i < N; ++i)
    {
      float v = normalize(i, raw[i]);

      v = v * v; // Sharpen peaks
      // v = v * v * v; // Even sharper

      sum += v;
      weighted += pos[i] * v;
    }

    if (sum < lost_threshold)
    {
      // Use last line if no line is detected
      return last_error;
    }

    if (sum > trust_threshold)
    {
      last_error = weighted / sum;
    }
    else
    {
      if (Timers::get().ready(2))
      {
        Serial.print("Untrusted sum: ");
        Serial.println(sum);
        Timers::get().reset(2);
      }
    }

    return last_error;
  }

private:
  uint16_t raw[N];

  // Computed from spreadsheet
  const uint16_t min_vals[N] = {669, 734, 618, 757, 614, 711, 734, 805};
  const uint16_t max_vals[N] = {1652, 1635, 1039, 1565, 853, 1515, 1754, 1695};

  float last_error = 0;

  float normalize(int i, uint16_t x)
  {
    float n = (float(x) - float(min_vals[i])) / (float(max_vals[i]) - float(min_vals[i]));

    // Clamp away values outside of previously measure min and max
    n = clamp(n, 0.0f, 1.0f);

    // Deadband with small epsilon like the sheet formula
    const float eps = 0.05f;
    n = n * (1.0f + eps) - eps;

    // Clamp away values in the bottom epsilon to 0
    n = clamp(n, 0.0f, 1.0f);

    return n;
  }

  // Positions of sensors
  // Calculated via weighted average from spreadsheet
  // right -> left, same order as sensors 0 -> 7
  const float pos[8] = {
      -33.893f,
      -24.29f,
      -14.351f,
      -5.069f,
      5.307f,
      13.907f,
      23.785f,
      33.369f};

  // If sensor readings sum to less than this, no line is detected
  const float lost_threshold = 0.02f;
  const float trust_threshold = 0.25f;
};

// Create global objects
Sensors sensors;
Controller controller;

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

  int wait_ms = 2000;
  int blinks = 10;
  for (int i = 0; i < blinks; ++i)
  {
    delay(wait_ms / blinks / 2);
    digitalWrite(Pins::led_rf_pin, HIGH);
    delay(wait_ms / blinks / 2);
    digitalWrite(Pins::led_rf_pin, LOW);
  }

  Timers::get().set_durations(1.0);
  Timers::get().set_duration(2, 0.2);
}

void loop()
{
  ECE3_read_IR(sensors.values());

  float error = sensors.error();
  MotorCommand command = controller.update(error);

  digitalWrite(Pins::left_dir_pin, command.left_dir);
  digitalWrite(Pins::right_dir_pin, command.right_dir);

  analogWrite(Pins::left_pwm_pin, command.left_mag);
  analogWrite(Pins::right_pwm_pin, command.right_mag);

  if (Timers::get().ready(1))
  {
    Serial.print("Left: ");
    Serial.println(command.left);
    Serial.print("Right: ");
    Serial.println(command.right);
    Timers::get().reset(1);
  }
}
