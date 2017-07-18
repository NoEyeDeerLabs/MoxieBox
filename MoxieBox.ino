// MPU-6050 Short Example Sketch


#include<Wire.h>
const int MPU_addr = 0x68; // I2C address of the MPU-6050
#define X_SENS 2
#define Y_SENS 4

const int Baseline_samples = 10; //Number of samples to take
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int16_t varGyX, varAcY, leftCntr, rightCntr, gasCntr, brakeCntr = 0;
int32_t avgGyX, avgAcY, avgAcZ;
void setup() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
  baseline();
}


void loop() {


  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 10, true); // request 4 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
 
 /* GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
*/

  avgGyX = ((avgGyX * 4) + GyX) / 5;
  varGyX = avgGyX / X_SENS;
  if (avgGyX - GyX >= varGyX) {
    rightCntr = 0;
    leftCntr++;
    if (leftCntr >= 10) {
      Serial.print("left = "); Serial.println(GyX);
      leftCntr = 0;
    }
  }
  if (avgGyX - GyX  <= -varGyX) {
    leftCntr = 0;
    rightCntr++;
    if (rightCntr >= 10) {
      Serial.print("right = "); Serial.println(GyX);
      rightCntr = 0;
    }
  }

  avgAcY = ((avgAcY * 4) + AcY) / 5;
  varAcY = avgAcY / Y_SENS;
  if (avgAcY - AcY >= varAcY) {
    gasCntr = 0;
    brakeCntr++;
    if (brakeCntr >= 10) {
      Serial.print("brake = "); Serial.println(AcY);
      brakeCntr = 0;
    }
  }
  if (avgAcY - AcY  <= -varAcY) {
    brakeCntr = 0;
    gasCntr++;
    if (gasCntr >= 10) {
      Serial.print("gas = "); Serial.println(AcY);
      gasCntr = 0;
    }
  }


  delay(50);
}


void baseline() {  //Assumes MoxieBox is flat and level for 5 seconds after power on
  avgGyX, avgAcY, avgAcZ = 0;
  Serial.println("------");

  for (int i = 0; i <= Baseline_samples; i++) {

    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 10, true); // request 6 registers
    AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)

    AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    avgAcY = ((avgAcY * i) + AcY) / (i + 1);

    AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)

    GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    avgGyX = ((avgGyX * i) + GyX) / (i + 1);

 /*   GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
*/
    delay (50);

    Serial.println();
    Serial.print("GyX = "); Serial.print(GyX);
    Serial.print(" avgGyX = "); Serial.print(avgGyX);
    Serial.print(" | AcY = "); Serial.print(AcY);
    Serial.print(" AvgAcY = "); Serial.print(avgAcY);
  }

}
