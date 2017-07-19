// MPU-6050 Short Example Sketch


#include<Wire.h>
#include<SimpleKalman.h>
#include <SoftwareSerial.h>

SoftwareSerial wavTrigger(11, 10); // RX(unsed), TX

#define HORN_SOUND 1
#define BRAKE_SOUND 2
#define GAS_SOUND 3
#define LEFT_SOUND 4
#define RIGHT_SOUND 5
#define JUMP_SOUND 6
#define IDLE_SOUND 7
#define HOT_SOUND 8

const int MPU_addr = 0x68; // I2C address of the MPU-6050

//Create filters for the accelerometer and gyro sensors
SimpleKalman AcXFilter(0.125, 32, 512, 0);
SimpleKalman AcYFilter(0.125, 32, 512, 0);
SimpleKalman AcZFilter(0.125, 32, 512, 0);
SimpleKalman GyXFilter(0.125, 32, 512, 0);
SimpleKalman GyYFilter(0.125, 32, 512, 0);
SimpleKalman GyZFilter(0.125, 32, 512, 0);

#define X_SENS 2
#define Y_SENS 4
#define BASELINE_COUNT 10 // number of samples to start with

double AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
double AcXLast, AcYLast, AcZLast, GyXLast, GyYLast, GyZLast;
double AcXFiltered, AcYFiltered, AcZFiltered, GyXFiltered, GyYFiltered, GyZFiltered;
int16_t leftCntr, rightCntr, gasCntr, brakeCntr = 0;

void setup() {
  //Serial debug
  Serial.begin(9600);
  Serial.print("MoxieBox Starting"); Serial.print(millis());

  //MPU-6050
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //Establish a baseline for the accelerator, gyro and temprature data.  Assumes kart will be stationary.
  Serial.print("Establising baseline of BASELINE_COUNT readings");
  for (int i; i < BASELINE_COUNT, i++;) {
    readMPU6050();
    filterMPU6050();
    delay(200);
  }
}


void loop() {

  //save all the accelerometer and gyro values so we can diff them
  AcXLast = AcX;
  AcYLast = AcY;
  AcZLast = AcZ;
  GyXLast = GyX;
  GyYLast = GyY;
  GyZLast = GyZ;

  //Get the current accelerometer and gyro values and filter them
  readMPU6050();
  filterMPU6050();

  //X is vertical, Y if front to back and Z left to right.

  if ((AcYFiltered - AcYLast) / AcYLast >= 0.05) {
    gasCntr = 0;
    brakeCntr++;
    if (brakeCntr >= 10) {
      Serial.print("BRAKE AcY= "); Serial.println(AcYFiltered);
      playSound(wavTrigger, BRAKE_SOUND);
      brakeCntr = 0;
    }
  }
  if ((AcYFiltered - AcYLast) / AcYLast > 0.05) {
    brakeCntr = 0;
    gasCntr++;
    if (gasCntr >= 10) {
      Serial.print("GAS AcY= "); Serial.println(AcYFiltered);
      playSound(wavTrigger, GAS_SOUND);
      gasCntr = 0;
    }
  }

  delay(50);
}


void readMPU6050() {  //Reads MPU-6050

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request 6 registers

  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  Serial.print("MPU6050"); Serial.print(millis());
  Serial.print(" AcX = "); Serial.print(AcX);
  Serial.print(" GyX = "); Serial.print(GyX);
  Serial.print(" AcY = "); Serial.print(AcY);
  Serial.print(" GyY = "); Serial.print(GyY);
  Serial.print(" AcZ = "); Serial.print(AcZ);
  Serial.print(" GyZ = "); Serial.print(GyZ);
  Serial.print(" Tmp = "); Serial.print(Tmp);
  Serial.println();
}

void filterMPU6050() {

  AcXFiltered = AcXFilter.getFilteredVal(AcX);
  AcYFiltered = AcYFilter.getFilteredVal(AcY);
  AcZFiltered = AcZFilter.getFilteredVal(AcZ);
  GyXFiltered = GyXFilter.getFilteredVal(GyX);
  GyYFiltered = GyYFilter.getFilteredVal(GyY);
  GyZFiltered = GyZFilter.getFilteredVal(GyZ);

  Serial.print("Filtered"); Serial.print(millis());
  Serial.print(" AcX* = "); Serial.print(AcXFiltered);
  Serial.print(" GyX* = "); Serial.print(GyXFiltered);
  Serial.print(" AcY* = "); Serial.print(AcYFiltered);
  Serial.print(" GyY* = "); Serial.print(GyYFiltered);
  Serial.print(" AcZ* = "); Serial.print(AcZFiltered);
  Serial.print(" GyZ* = "); Serial.print(GyZFiltered);
  Serial.println();
}


void playSound(SoftwareSerial serialInterface, int track) {
  byte txbuf[8];
  txbuf[0] = 0xf0;
  txbuf[1] = 0xaa;
  txbuf[2] = 0x08;
  txbuf[3] = 0x03;
  txbuf[4] = 0x00;
  txbuf[5] = (byte)track;
  txbuf[6] = (byte)(track >> 8);
  txbuf[7] = 0x55;
  serialInterface.write(txbuf, 8);
}
