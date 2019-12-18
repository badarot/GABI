#ifndef PID_h
#define PID_h

#include <Arduino.h>
#include <PID.h>

class PID {
  private:

  float kp, ki, kd;
  float minOut = 0.;
  float maxOut = 255.;

  unsigned long last_time;
  float error_sum, last_error;
  bool enableI, enableD;

  public:

  PID(float Kp, float Ki, float Kd, float min, float max) {
    PID::set_parameters(Kp, Ki, Kd);
    PID::output_limits(min, max);
  }

  float compute(float input, float target) {
    float output;

    // Calcula dt
    unsigned long now = micros();
    float dt = (float) (now - last_time) * 1e-6; // dt em segundos
    last_time = now;

    // Calcula resposta proporcional
    float error = target - input;
    output = kp * error;

    // Calcula resposta integral
    if (enableI) {
      error_sum += error;

      // Limita resposta integral
      // if (error_sum > maxOut) error_sum = maxOut;
      // else if(error_sum < minOut) error_sum = minOut;
      error_sum = constrain(error_sum, minOut, maxOut);

      output += ki * error_sum * dt;
    }

    // Calcula resposta diferencial
    if (enableD) {
      float dError = error - last_error;
      output += kd * dError / dt;
      last_error = error;
    }

    // Limita saida
    // if (output > maxOut) output = maxOut;
    // else if(output < minOut) output = minOut;
    output = constrain(output, minOut, maxOut);

    return output;
  }

  void set_parameters(float Kp, float Ki, float Kd) {
    kp = Kp;
    ki = Ki;
    kd = Kd;

    enableI = Ki != 0.0;
    enableD = Kd != 0.0;

    return;
  }

  void output_limits(float min, float max) {
    minOut = min;
    maxOut = max;
  }

  void reset() {
    error_sum = 0.0;
    last_error = 0.0;
  }
};

#endif
