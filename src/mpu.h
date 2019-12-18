// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project

#include "Arduino.h"
// #define I2CDEV_IMPLEMENTATION I2CDEV_BUILTIN_FASTWIRE
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		#include "Wire.h"
#endif

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// Define escala do giroscopio
#define GYRO_RANGE MPU6050_GYRO_FS_500
#if GYRO_RANGE == 0x00
  #define GYRO_SCALE 131.0
#elif GYRO_RANGE == 0x01
  #define GYRO_SCALE 65.5
#elif GYRO_RANGE == 0x02
  #define GYRO_SCALE 32.8
#else
  #define GYRO_SCALE 16.4
#endif

#define MPU_EQ_OFFSET 0.0 // Offset do ponto de equilibrio equilibrio

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
	 NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
	 depends on the MPU-6050's INT pin being connected to the Arduino's
	 external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
	 digital I/O pin 2.
 * ========================================================================= */



// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
int16_t rot;



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
		mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void mpuInit() {
	// join I2C bus (I2Cdev library doesn't do this automatically)
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
			Wire.begin();
			Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
			Fastwire::setup(400, true);
	#endif

	// initialize device
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();
	pinMode(INTERRUPT_PIN, INPUT);

	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	//OFFSETS
	//X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
	//-2224,     921,    1322,      72,     -37,     -17 (mesa da sala)
	//-2192,     947,    1318,      67,     -40,      -6 (mesa de IC)
  //-2176,     903,    1312,      65,     -72,      21 (mesa do HS)
  //-2180,     953,    4914,     103,     -47,    -140 (robo: na mao)
  //-2388,    1037,    1414,      77,     -38,     -30 (robo: de cabeca para baixo)
	mpu.setXAccelOffset(-2176);
	mpu.setYAccelOffset(1037);
	mpu.setZAccelOffset(1414);
	mpu.setXGyroOffset(77);
	mpu.setYGyroOffset(-38);
	mpu.setZGyroOffset(30);
	
	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    #ifdef CALIBRATE
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      Serial.println();
      mpu.PrintActiveOffsets();
    #endif
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
	} else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
	}
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

bool mpuRead(float* angle, float* rotation) {
		// Se nao ha nada pra ser lido retorna falso
		if (!mpuInterrupt) return false;

		// reset interrupt flag and get INT_STATUS byte
		mpuInterrupt = false;
		mpuIntStatus = mpu.getIntStatus();

		// get current FIFO count
		fifoCount = mpu.getFIFOCount();

		// check for overflow (this should never happen unless our code is too inefficient)
		if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      fifoCount = mpu.getFIFOCount();
      Serial.println(F("FIFO overflow!"));

		  return false;
    } // otherwise, check for DMP data ready interrupt (this should happen frequently) 
    else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
      // wait for correct available data length, should be a VERY short wait
      
      // while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      if (fifoCount < packetSize) return false;

      if (fifoCount > packetSize) Serial.println("Pacotes na fila");
      while(fifoCount >= packetSize) { // Lets catch up to NOW, someone is using the dreaded delay()!
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        
        fifoCount -= packetSize;
      }

      // read a packet from FIFO
      // mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      // fifoCount -= packetSize;
      // Sinaliza que ha mais pacotes disponiveis para leitura 
      if (fifoCount >= packetSize) {
      }

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      // mpu.dmpGetGyro(&gy, fifoBuffer);
      // *angle = ypr[1] * 180.0 / M_PI;
      // *rotation = (float) gy.y / GYRO_SCALE;

      // calculo do angulo pitch retirado da biblioteca
      *angle = atan2(gravity.x , sqrt(gravity.y * gravity.y + gravity.z * gravity.z));
      if (gravity.z > 0) {
          if(*angle > 0) {
              *angle = PI - *angle; 
          } else { 
              *angle = -PI - *angle;
          }
      }
      *angle = *angle * 180.0 / M_PI - MPU_EQ_OFFSET;

      // Calculo do GyroY retirado da biblioteca
      rot = (fifoBuffer[24] << 8) | fifoBuffer[25];
      *rotation = (float) rot / GYRO_SCALE;      

			return true;
		}
		return false;
}
