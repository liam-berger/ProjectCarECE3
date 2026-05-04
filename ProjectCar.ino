#include <ECE3_LCD7.h>

const int left_nslp_pin = 31; // nslp ==> awake & ready for PWM
const int left_dir_pin = 29;
const int left_pwm_pin = 40;

const int right_nslp_pin = 11; // nslp ==> awake & ready for PWM
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

const int led_rf_pin = 41;

// Helpers

template <typename T>
T clamp(T x, T lo, T hi)
{
  if (x < lo)
    return lo;
  if (x > hi)
    return hi;
  return x;
}

struct MotorCommand
{
  int left;
  int right;
};

// Implement PD feedback control
class Controller
{
public:
  MotorCommand update(float raw_error)
  {
    // Filtered error as linear combination of current and past error
    e_filt = alpha_e * raw_error + (1.0f - alpha_e) * e_filt;

    float raw_d = e_filt - prev_e_filt;
    prev_e_filt = e_filt;

    d_filt = alpha_d * raw_d + (1.0f - alpha_d) * d_filt;

    float turn = kp * e_filt + kd * d_filt;
    turn = clamp<float>(turn, -55.0f, 55.0f);

    float turn_amount = fabs(e_filt) + 4.0f * fabs(d_filt);
    float base = max_base - 0.35f * turn_amount;
    base = clamp<float>(base, min_base, max_base);

    return {
        clamp<int>(int(base - turn), 0, 255),
        clamp<int>(int(base + turn), 0, 255)};

  private:
    // Tuning
    // How much do we change steering in response to error?
    float kp = 1.5f;  // Proportional
    float kd = 12.0f; // Derivative

    // How much do we carry over past error vs new error?
    // Higher alpha is more responsive, lower alpha is more smooth
    // Effective memory length is roughly 1/alpha
    float alpha_e = 0.25f; // Proportional
    float alpha_d = 0.35f; // Derivative

    // Clamp final speed, 0-100.
    int max_base = 110; // Max speed (e.g., on straights)
    int min_base = 65;  // Min speed (prevent stalling)

    // State
    float e_filt = 0;      // Filterer (weighted sum of current and past error)
    float prev_e_filt = 0; // Last step's filtered error to to calculate
    float d_filt = 0;
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

      last_error = weighted / sum;
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
      float n = float(x - min_vals[i]) / float(max_vals[i] - min_vals[i]);

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
    pinMode(left_nslp_pin, OUTPUT);
    pinMode(left_dir_pin, OUTPUT);
    pinMode(left_pwm_pin, OUTPUT);

    pinMode(right_nslp_pin, OUTPUT);
    pinMode(right_dir_pin, OUTPUT);
    pinMode(right_pwm_pin, OUTPUT);

    digitalWrite(left_dir_pin, LOW);
    digitalWrite(left_nslp_pin, HIGH);

    digitalWrite(right_dir_pin, LOW);
    digitalWrite(right_nslp_pin, HIGH);

    pinMode(led_rf_pin, OUTPUT);

    ECE3_Init();
    Serial.begin(9600);

    int wait_ms = 2000;
    int blinks = 10;
    for (int i = 0; i < blinks; ++i)
    {
      delay(wait_ms / blinks / 2);
      digitalWrite(led_rf_pin, HIGH);
      delay(wait_ms / blinks / 2);
      digitalWrite(led_rf_pin, LOW);
    }
  }

  void loop()
  {
    ECE3_read_IR(sensors.values());

    float error = sensors.error();
    MotorCommand command = controller.update(error);

    analogWrite(left_pwm_pin, command.left);
    analogWrite(right_pwm_pin, command.right);
  }
