#include <Arduino.h>
#include <TimerOne.h>

#define byteClear(x,y) (x &= ~y)
#define byteSet(x,y) (x |= y)

#define MOTOR_MICROSTEP 8
#define MOTOR_STEP_ANG (1.8 / MOTOR_MICROSTEP)
#define MOTOR_MAX_FREQ 5.0
#define MOTOR_MAX_SPEED (MOTOR_MAX_FREQ * 360.0)

#define ZERO_SPEED 100000L // Periodo maximo em um: 10 passos por segundo

#define MOTOR_ENABLE 12
// Pinos de controle do motor
// Usar pinos do 2 ao 7, mapeados no registrador PORTD
#define MOTOR_DIR1 4
#define MOTOR_STEP1 5
#define MOTOR_DIR2 6
#define MOTOR_STEP2 7

#define STEPPINS ((1 << MOTOR_STEP1) | (1 << MOTOR_STEP2))


int motor_dir;
float motor_speed;
unsigned long motor_period;
volatile long motor_steps;

void isrInterrupt() {
// TIMER 1 : STEPPER MOTOR1 SPEED CONTROL
  if (motor_dir == 0) // If we are not moving we dont generate a pulse
    return;
  // Gera pulso manipulando as portas diretamente
  byteSet(PORTD, STEPPINS); // STEP MOTOR 1
  // A duracao mínima do pulso é 1.9 um
  // delayMicroseconds(1);
  if (motor_dir > 0)
    motor_steps++;
  else
    motor_steps--;

  byteClear(PORTD, STEPPINS);
}

// Ativa driver
void motorEnable() {
  digitalWrite(MOTOR_ENABLE, LOW);
}

// Desativa driver e zera velocidade
void motorDisable() {
  digitalWrite(MOTOR_ENABLE, HIGH);
  Timer1.setPeriod(ZERO_SPEED);
  motor_speed = 0.0;
  motor_dir = motor_steps = 0;
}

void motorInit() {
  pinMode(MOTOR_DIR1, OUTPUT);
  pinMode(MOTOR_STEP1, OUTPUT);
  pinMode(MOTOR_DIR2, OUTPUT);
  pinMode(MOTOR_STEP1, OUTPUT);
  pinMode(MOTOR_ENABLE, OUTPUT);

  motorDisable();

  Timer1.initialize();
  Timer1.attachInterrupt(isrInterrupt);
}

// Entrada: velocidade em graus/s
// Saida: velocidade usada, limitada por MOTOR_MAX_SPEED
float motorSpeed(float new_speed) {
  new_speed = constrain(new_speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);

  // Define direção do motor
  if (new_speed > 0.0) {
    motor_dir = 1;
    bitClear(PORTD, MOTOR_DIR1);
    bitSet(PORTD, MOTOR_DIR2);
  }
  else {
    motor_dir = -1;
    bitSet(PORTD, MOTOR_DIR1);
    bitClear(PORTD, MOTOR_DIR2);
  }

  // Calcula periodo entre pulsos
  float abs_speed = abs(new_speed);
  if (abs_speed != motor_speed) {
    motor_speed = abs_speed;
    // Periodo entre passos em um
    motor_period = 1e6 / (motor_speed / MOTOR_STEP_ANG);

    // Limita velocidade minima em passos/s
    if (motor_period > ZERO_SPEED) {
      motor_speed = 0.0;
      motor_period = ZERO_SPEED;
      motor_dir = 0;
    }

    Timer1.setPeriod(motor_period);
  }
  return new_speed;
}
