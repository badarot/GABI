#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <EEPROM.h>

struct PID_params {
  float kp;
  float ki;
  float kd;
};

void serialClear() {
  while(Serial.available() > 0) {
    Serial.read();
  }
}

void loadParams(float *kp, float *ki, float *kd, uint16_t adress) {
  PID_params params;
  EEPROM.get(adress, params);
  *kp = params.kp;
  *ki = params.ki;
  *kd = params.kd;
}

void saveParams(float kp, float ki, float kd, uint16_t adress) {
  PID_params params;
  params.kp = kp;
  params.ki = ki;
  params.kd = kd;
  EEPROM.put(adress, params);
}

void serialSendValue(double value) {
  byte* val = (byte*) &value;
  // Serial.write(' ');
  Serial.write(val, 4);
}

void serialSendValue(int value) {
  byte* val = (byte*) &value;
  // Serial.write(' ');
  Serial.write(val, 2);
}
#endif
