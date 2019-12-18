
#include <Arduino.h>
#include <utils.h>
#include <mpu.h>
#include <motor.h>
#include <PID.h>

#define EEPROM_ADR_ANGLE 127
#define EEPROM_ADR_SPEED 42

#define SERIAL_READABLE

float angle, rotation;
float wheel_speed, robot_speed, estimated_speed, target_angle;

unsigned long last_time, now;

bool robot_enable, imu_data, set_speed;

unsigned long count;

// Serial buffer
char buffer[15];

float anglekp = 2.0; //2.5;
float angleki = 0;
float anglekd = 0.1; //0.25;

float speedkp = 0.003; //0.005;
float speedki = 0.01; //0.001;
float speedkd = 0.0005; //0.0005;

PID pidAngle = PID(anglekp, angleki, anglekd, -50, 50);
PID pidSpeed = PID(speedkp, speedki, speedkd, -15, 15);

void setup() {
	Serial.begin(115200);
	// Serial.setTimeout(100);

	// Atualiza os parametros do PID para os ultimos savos
	loadParams(&anglekp, &angleki, &anglekd, EEPROM_ADR_ANGLE);
	loadParams(&speedkp, &speedki, &speedkd, EEPROM_ADR_SPEED);

	pidAngle.set_parameters(anglekp, angleki, anglekd);
	pidSpeed.set_parameters(speedkp, speedki, speedkd);

	mpuInit();

	Serial.println("Parametros Usados:");
	Serial.print("Angulo >>");
	Serial.print(" kp: ");	Serial.print(anglekp, 4);
	Serial.print(" ki: ");	Serial.print(angleki, 4);
	Serial.print(" kd: ");	Serial.print(anglekd, 4);
	Serial.print("\nVelocidade >>");
	Serial.print(" kp: ");	Serial.print(speedkp, 4);
	Serial.print(" ki: ");	Serial.print(speedki, 4);
	Serial.print(" kd: ");	Serial.print(speedkd, 4);
	Serial.println();

	motorInit();
}

void loop() {
	// Checa de ha dados para leitura
	imu_data = mpuRead(&angle, &rotation);

	// ROBO DE PE e ativo
	if (robot_enable && imu_data) {
		imu_data = false;

		//entrada: velocidade do motor
		//saida: ângulo de inclinação
		target_angle = pidSpeed.compute(wheel_speed, 0.0);

		// entrada: ângulo de inclinação
		// saida: aceleracao do motor
		float accel = -1 * pidAngle.compute(angle, target_angle);

		// Atualizando velocidade do motor
		wheel_speed += accel;
		wheel_speed = motorSpeed(wheel_speed);

		#ifdef SERIAL_READABLE
		// Envia telemetria em formato legível
		Serial.print(angle);
		Serial.print(' ');
		Serial.print(target_angle);
		Serial.print(' ');
		Serial.print(wheel_speed/36.0);
		// Serial.print(' ');
		// Serial.print(rotation);
		Serial.print(' ');
		Serial.print(accel);
		Serial.println();

		#else
		// Envia telemetria em formato binário
			Serial.write('$');
			serialSendValue(angle);
			serialSendValue(target_angle);
			serialSendValue(wheel_speed/36.0);
			serialSendValue(accel);
			Serial.write('\r');
			Serial.write('\n');
		#endif

		// ROBO CAIU
		if (abs(angle) > 45.0) {
			// Desativa motor e reseta todas as variáveis de controle
			robot_enable = false;
			motorDisable();
			wheel_speed = 0.0;
			pidAngle.reset();
			pidSpeed.reset();

			// Limpa buffer do serial
			serialClear();
		}
	}

	// ROBO DEITADO esperando comandos sets the maximum milliseconds to wait for serial data. It defaults to 1000 milliseconds.

	// Checa se o robo pode ser reiniciado
	else if (!robot_enable && imu_data) {
		imu_data = false;

		if (abs(angle) < 15.0) {
			now = millis();
			if (now - last_time > 1000) {
				robot_enable = true;
				motorEnable();
			}
		}
		else
			last_time = millis();
	}

// Le serial para mudar parametros
	else if (!robot_enable && Serial.available() > 3) {
		byte len = Serial.readBytesUntil('\n', buffer, 15);
		// Adiciona fim do string
		buffer[len] = 0;

		// checa se deve salvar os novos parametros
		if (strcmp(buffer, "save") == 0) {
			saveParams(speedkp, speedki, speedkd, EEPROM_ADR_SPEED);
			saveParams(anglekp, angleki, anglekd, EEPROM_ADR_ANGLE);
			Serial.println("Parametros salvos");
		}
		// checa se deve recarregar prametros da memoria
	  else if (strcmp(buffer, "load") == 0) {
			loadParams(&anglekp, &angleki, &anglekd, EEPROM_ADR_ANGLE);
			loadParams(&speedkp, &speedki, &speedkd, EEPROM_ADR_SPEED);
			pidAngle.set_parameters(anglekp, angleki, anglekd);
			pidSpeed.set_parameters(speedkp, speedki, speedkd);

			Serial.println("Parametros Usados:");
			Serial.print("Angulo >>");
			Serial.print(" kp: ");	Serial.print(anglekp, 4);
			Serial.print(" ki: ");	Serial.print(angleki, 4);
			Serial.print(" kd: ");	Serial.print(anglekd, 4);
			Serial.print("\nVelocidade >>");
			Serial.print(" kp: ");	Serial.print(speedkp, 4);
			Serial.print(" ki: ");	Serial.print(speedki, 4);
			Serial.print(" kd: ");	Serial.print(speedkd, 4);
			Serial.println();
		}
		// Checa se ha novo parametros pro pid
		// msg: [a,s][p,i,d] <valor>
		else if (buffer[0] == 's') {
			float k = atof(buffer + 3);

			if (buffer[1] == 'p') speedkp = k;
			else if (buffer[1] == 'i') speedki = k;
			else if (buffer[1] == 'd') speedkd = k;

			pidSpeed.set_parameters(speedkp, speedki, speedkd);
			Serial.print("PID da velocidade::");
			Serial.print(" kp: "); Serial.print(speedkp, 4);
			Serial.print(" ki: "); Serial.print(speedki, 4);
			Serial.print(" kd: "); Serial.println(speedkd, 4);
		}
		else if (buffer[0] == 'a') {
			float k = atof(buffer + 3);

			if (buffer[1] == 'p') anglekp = k;
			else if (buffer[1] == 'i') angleki = k;
			else if (buffer[1] == 'd') anglekd = k;

			pidAngle.set_parameters(anglekp, angleki, anglekd);
			Serial.print("PID do angulo::");
			Serial.print(" kp: "); Serial.print(anglekp, 4);
			Serial.print(" ki: "); Serial.print(angleki, 4);
			Serial.print(" kd: "); Serial.println(anglekd, 4);
		}
		else {
			Serial.println("erro");
		}
	}
}
