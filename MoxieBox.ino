
// http://robertsonics.com/wav-trigger/

#include<Wire.h>
#include<SimpleKalman.h>
#include <SoftwareSerial.h>

SoftwareSerial wavTrigger(11, 10); // RX(unsed), TX

#define STARTUP_SOUND 1
#define ENABLED_SOUND 2
#define HOT_SOUND 3
#define HORN_SOUND 4
#define BRAKE_SOUND 5
#define GAS_SOUND 6
#define LEFT_SOUND 7
#define RIGHT_SOUND 8
#define JUMP_SOUND 9
#define IDLE_SOUND 10
#define NUM_SOUNDS 10

static long lockoutTimer[11];
#define LOCKOUT_TIME 5000

const int MPU_addr = 0x68; // I2C address of the MPU-6050

//Create filters for the accelerometer and gyro sensors
SimpleKalman AcXFilter(0.125, 32, 512, 0);
SimpleKalman AcYFilter(0.125, 32, 512, 0);
SimpleKalman AcZFilter(0.125, 32, 512, 0);
SimpleKalman GyXFilter(0.125, 32, 512, 0);
SimpleKalman GyYFilter(0.125, 32, 512, 0);
SimpleKalman GyZFilter(0.125, 32, 512, 0);

#define X_SENS 0.10
#define Y_SENS 0.10
#define Z_SENS 0.10

#define X_REPS 5
#define Y_REPS 5
#define Z_REPS 5


#define BASELINE_COUNT 10

// As the IMU is not in its normal orientation we remap X, Y and Z using two arrays
double accel[3];
double gyro[3];
double accelFiltered[3];
double gyroFiltered[3];
double accelLast[3];
double gyroLast[3];
double temprature; //temprature


#define X_AXIS 2
#define Y_AXIS 1
#define Z_AXIS 0

int16_t leftCntr, rightCntr, gasCntr, brakeCntr = 0;

// Switches and buttons
#define HORN_PIN 4
#define ENABLE_PIN 9
byte switches[] = {HORN_PIN, ENABLE_PIN};
#define NUM_SWITCHES sizeof(switches)
byte switchState[NUM_SWITCHES]; //LOW means switch is on or button is pressed
bool leadingEdge[NUM_SWITCHES], trailingEdge[NUM_SWITCHES];

#define DEBOUNCE_MILLIS 10  // switch debouncer interval

void setup() {
  int i;

  //Serial debug
  Serial.begin(9600);
  Serial.print("MoxieBox Starting"); Serial.println(millis());

  //Wav Trigger
  wavTrigger.begin(57000);

  //MPU-6050
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // Enable switches - note using MCU pull-ups
  for (i = 0; i < NUM_SWITCHES; i++) {
    pinMode(switches[i], INPUT);
    digitalWrite(switches[i], HIGH);
  }

  //Establish a baseline for the accelerator, gyro and temprature data.  Assumes kart will be stationary.
  Serial.println("Establising baseline");
  for (i = 0; i <= 10; i++) {
    Serial.println(i);
    readMPU6050();
    filterMPU6050();
    delay(200);
  }

  //reset all of the lockout timers
  for (i = 0; i < NUM_SOUNDS; i++) {
    lockoutTimer[i] = millis();
  }

  //Let's tell the world that we are up and running.
  playSound(wavTrigger, ENABLED_SOUND);
  Serial.print("It's time for the Boxie that makes some moxie! "); Serial.println(millis());
}

void loop() {

  checkSwitches();

  //save all the accelerometer and gyro values so we can diff them
  accelLast[X_AXIS] = accelFiltered[X_AXIS];
  accelLast[Y_AXIS] = accelFiltered[Y_AXIS];
  accelLast[Z_AXIS] = accelFiltered[Z_AXIS];
  gyroLast[X_AXIS] = gyroFiltered[X_AXIS];
  gyroLast[Y_AXIS] = gyroFiltered[Y_AXIS];
  gyroLast[Z_AXIS] = gyroFiltered[Z_AXIS];


  //Get the current accelerometer and gyro values and filter them
  readMPU6050();
  filterMPU6050();
 
// --- X AXIS ---
  //LEFT detection
  if ((accelFiltered[X_AXIS] - accelLast[X_AXIS]) / accelLast[X_AXIS] >= X_SENS
      && lockoutTimer[LEFT_SOUND] < millis()) {
    rightCntr = 0;
    leftCntr++;
    if (leftCntr == X_REPS ) {
      Serial.print("LEFT AcX= "); Serial.println(accelFiltered[X_AXIS]);
      //stop the right turn sound from being played
      lockoutTimer[RIGHT_SOUND] = millis() + LOCKOUT_TIME;
      playSound(wavTrigger, LEFT_SOUND);
      leftCntr = 0;
    }
  }

  //RIGHT detection
  if ((accelFiltered[X_AXIS] - accelLast[X_AXIS]) / accelLast[X_AXIS] <= -X_SENS
      && lockoutTimer[RIGHT_SOUND] < millis()) {
    leftCntr = 0;
    rightCntr++;
    if (rightCntr == X_REPS ) {
      Serial.print("RIGHT AcX= "); Serial.println(accelFiltered[X_AXIS]);
      //stop the left turn sound from being played
      lockoutTimer[LEFT_SOUND] = millis() + LOCKOUT_TIME;
      playSound(wavTrigger, RIGHT_SOUND);
      rightCntr = 0;
    }
  }


  // --- Y AXIS ---
  //BRAKE detection

  
  if ((accelFiltered[Y_AXIS] - accelLast[Y_AXIS]) / accelLast[Y_AXIS] >= Y_SENS
      && lockoutTimer[BRAKE_SOUND] < millis()) {
    gasCntr = 0;
    brakeCntr++;
    if (brakeCntr == Y_REPS ) {
      Serial.print("BRAKE AcY= "); Serial.println(accelFiltered[Y_AXIS]);
      //stop the gas sound from being played
      lockoutTimer[GAS_SOUND] = millis() + LOCKOUT_TIME;
            brakeCntr = 0;
      playSound(wavTrigger, BRAKE_SOUND);

    }
  }

  //Gas detection
  if ((accelFiltered[Y_AXIS] - accelLast[Y_AXIS]) / accelLast[Y_AXIS] <= -Y_SENS
      && lockoutTimer[GAS_SOUND] < millis()) {
    brakeCntr = 0;
    gasCntr++;
    if (gasCntr == Y_REPS ) {
      Serial.print("GAS AcY= "); Serial.println(accelFiltered[Y_AXIS]);
      //stop the brake sound from being played
      lockoutTimer[BRAKE_SOUND] = millis() + LOCKOUT_TIME;
            gasCntr = 0;
      playSound(wavTrigger, GAS_SOUND);
    }
  }
   

  



  delay(50);
}


void readMPU6050() {  //Reads MPU-6050

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request 7 registers

  accel[X_AXIS] = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  accel[Y_AXIS] = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accel[Z_AXIS] = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  temprature = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyro[X_AXIS] = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyro[Y_AXIS] = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyro[Z_AXIS] = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

 /* Serial.print("MPU6050"); Serial.print(millis());
  Serial.print(" AcX = "); Serial.print(accel[X_AXIS]);
  Serial.print(" GyX = "); Serial.print(gyro[X_AXIS]);
  Serial.print(" AcY = "); Serial.print(accel[Y_AXIS]);
  Serial.print(" GyY = "); Serial.print(gyro[Y_AXIS]);
  Serial.print(" AcZ = "); Serial.print(accel[Z_AXIS]);
  Serial.print(" GyZ = "); Serial.print(gyro[Z_AXIS]);
  Serial.print(" Tmp = "); Serial.print(temprature);
  Serial.println(); 
  */
}

void filterMPU6050() {

  accelFiltered[X_AXIS] = AcXFilter.getFilteredVal(accel[X_AXIS]);
  accelFiltered[Y_AXIS] = AcYFilter.getFilteredVal(accel[Y_AXIS]);
  accelFiltered[Z_AXIS] = AcZFilter.getFilteredVal(accel[Z_AXIS]);
  gyroFiltered[X_AXIS] = GyXFilter.getFilteredVal(accel[X_AXIS]);
  gyroFiltered[Y_AXIS] = GyYFilter.getFilteredVal(accel[Y_AXIS]);
  gyroFiltered[Z_AXIS] = GyZFilter.getFilteredVal(accel[Z_AXIS]);

  Serial.print("Filtered"); Serial.print(millis());
  Serial.print(" AcX* = "); Serial.print(accelFiltered[X_AXIS]);
  Serial.print(" GyX* = "); Serial.print(gyroFiltered[X_AXIS]);
  Serial.print(" AcY* = "); Serial.print(accelFiltered[Y_AXIS]);
  Serial.print(" GyY* = "); Serial.print(gyroFiltered[Y_AXIS]);
  Serial.print(" AcZ* = "); Serial.print(accelFiltered[Z_AXIS]);
  Serial.print(" GyZ* = "); Serial.print(gyroFiltered[Z_AXIS]);
  Serial.println();
}


void checkSwitches()
{
  static byte previousState[NUM_SWITCHES];
  static byte switchState[NUM_SWITCHES];
  static long lasttime;
  byte index;
  if (millis() < lasttime) {
    lasttime = millis(); // we wrapped around, lets just try again
  }

  if ((lasttime + DEBOUNCE_MILLIS) > millis()) {
    return; // go round again
  }
  // time to check the switches

  //reset the timer
  lasttime = millis();

  //loop round each switch in turn
  for (index = 0; index < NUM_SWITCHES; index++) {
    leadingEdge[index] = false;
    trailingEdge[index] = false;

    switchState[index] = digitalRead(switches[index]);   // read the switch status

    if (switchState[index] == previousState[index]) {
      if ((switchState[index] == LOW) && (switchState[index] == LOW)) {
        // just switchState
        leadingEdge[index] = true;
      }
      else if ((switchState[index] == HIGH) && (switchState[index] == HIGH)) {
        // just released
        trailingEdge[index] = true;
      }
      switchState[index] = !switchState[index];  // remember, digital HIGH means NOT switchState
    }
    //Serial.println(switchState[index], DEC);
    previousState[index] = switchState[index];   // keep a running tally of the switches
  }
}

void playSound(SoftwareSerial serialInterface, int track) {
  byte txbuf[8];
  txbuf[0] = 0xf0; //SOM1
  txbuf[1] = 0xaa; //SOM2
  txbuf[2] = 0x08; //Length
  txbuf[3] = 0x03; //Resume 0x01 is play poly
  txbuf[4] = 0x00; //
  txbuf[5] = (byte)track;
  txbuf[6] = (byte)(track >> 8);
  txbuf[7] = 0x55; //EOM
  serialInterface.write(txbuf, 8);
}
