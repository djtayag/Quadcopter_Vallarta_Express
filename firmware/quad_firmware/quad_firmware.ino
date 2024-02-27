#include <radio.h>      // Header file with pin definitions and setup
#include <SerLCD.h>
#include "Control.h"
#include "quad_remote.h"
#include <Adafruit_Sensor.h>
#include <QuadClass_LSM6DSOX.h>
#include <Adafruit_Simple_AHRS.h>

#define RAD_TO_DEG 57.295779513082320876798154814105

#define YAW0 123
#define ROLL0 121
#define PITCH0 128

float motorVals[4]; // motor throttle values

quad_data_t orientation;

// buffer for receiving data
uint8_t receive[PACKET_SIZE];

// reset flag for calibrating IMU
int resetFlag;

// Create LSM9DS0 board instance.
QuadClass_LSM6DSOX lsm = QuadClass_LSM6DSOX();
Adafruit_Simple_AHRS *ahrs = NULL;
Adafruit_Sensor *_accel = NULL;
Adafruit_Sensor *_gyro = NULL;
Adafruit_Sensor *_mag = NULL;  // No Yaw orientation | Always NULL

unsigned int startMillis;
unsigned int currentMillis;
unsigned int loopEnd;

const int led1 = 16;
const int led2 = 17;
const int led3 = 18;
const int led4 = 36;
const int pretty_leds = 34;

// for complementary filter
float cf_angle_pitch = 0.00;
float cf_angle_roll = 0.00;
float pitch_angle_offset = 0.00;
float roll_angle_offset = 0.00;

float yaw_rate = 0.00;
float gyro_angle_pitch = 0.00;
float gyro_angle_roll = 0.00;
float gain = 0.98;

float pitch_rate_offset = 0.00;
float pitch_offset = 0.00;
float roll_rate_offset = 0.00;
float roll_offset = 0.00;

// PID vars - 0 -> 2 = yaw, roll, pitch
float remoteSetAngle[3];
float PIDout[3];

float curr_error_yaw;
float error_yaw_sum;
float last_error_yaw;

float curr_error_roll;
float error_roll_sum;
float last_error_roll;

float curr_error_pitch;
float error_pitch_sum;
float last_error_pitch;

float p_co_yaw;
float i_co_yaw;
float d_co_yaw;

float p_co_roll;
float i_co_roll;
float d_co_roll;

float p_co_pitch;
float i_co_pitch;
float d_co_pitch;

float calculateCf(float cf_angle_prev, float gyro_raw, float acc_angle, unsigned int dt) {
  return gain * (cf_angle_prev + gyro_raw * RAD_TO_DEG * dt) + (1 - gain) * (acc_angle);
}

float calculateGyroAngle(float gyro_angle_prev, float gyro_raw, unsigned int dt) {
  return gyro_angle_prev + gyro_raw * RAD_TO_DEG * dt;
}

void setupSensor() {

   if (!lsm.begin_I2C()) {
    Serial.println("Failed to find LSM6DSOX chip");
    while (1) {
      delay(10);
    }
  }
  _accel = lsm.getAccelerometerSensor();
  _gyro = lsm.getGyroSensor();
  ahrs = new Adafruit_Simple_AHRS(_accel, _mag, _gyro);
    
}

void calculateSetAngles(uint8_t yaw_remote, uint8_t roll_remote, uint8_t pitch_remote) {
  float YAW_BOUND, ROLL_BOUND, PITCH_BOUND;
  float YAW_DENOM, ROLL_DENOM, PITCH_DENOM;

  // upper bounds
  if (YAW0 == 128) {
    YAW_BOUND = 255.00;
    YAW_DENOM = 256.00;
  } else {
    YAW_BOUND = float(YAW0*2);
    YAW_DENOM = YAW_BOUND;
  }

  if (ROLL0 == 128) {
    ROLL_BOUND = 255.00;
    ROLL_DENOM = 256.00;
  } else {
    ROLL_BOUND = float(ROLL0*2);
    ROLL_DENOM = ROLL_BOUND;
  }

  if (PITCH0 == 128) {
    PITCH_BOUND = 255.00;
    PITCH_DENOM = 256.00;
  } else {
    PITCH_BOUND = float(PITCH0*2);
    PITCH_DENOM = PITCH_BOUND;
  }  

  // writing the set angles based on gimbal values
  if (yaw_remote >= YAW_BOUND) {
    receive[3] = YAW_BOUND;
  }
  remoteSetAngle[0] = float(receive[3]) * (120)/(YAW_DENOM) - 60.00;

  if (roll_remote >= ROLL_BOUND) {
    receive[4] = ROLL_BOUND;
  }
  remoteSetAngle[1] = float(receive[4]) * (20.00)/(ROLL_DENOM) - 10.00 - 2.8;

  if (pitch_remote >= PITCH_BOUND) {
    receive[5] = PITCH_BOUND;
  }
  remoteSetAngle[2] = -float(receive[5]) * (20.00)/(PITCH_DENOM) + 10.00 + 1.2;
}

void viewGimbalPositions() {
  Serial.print("YAW: ");
  Serial.print(receive[3]);
  Serial.print(" ");
  Serial.print("ROLL: ");
  Serial.print(receive[4]);
  Serial.print(" ");
  Serial.print("PITCH: ");
  Serial.print(receive[5]);
  Serial.println(" ");
}

void printSetAngles() {
  Serial.print(remoteSetAngle[0]);
  Serial.print(" ");
  Serial.print(remoteSetAngle[1]);
  Serial.print(" ");
  Serial.print(remoteSetAngle[2]);
  Serial.println(" ");
}

void adjustOffsets() {
    int t1, t2;
    // calculating offsets for complimentary filter
    for (int i = 0; i < 30; i++) {
      ahrs->getQuadOrientation(&orientation);

      pitch_rate_offset += orientation.pitch_rate;
      pitch_offset += orientation.pitch;
      roll_rate_offset += orientation.roll_rate;
      roll_offset += orientation.roll;
    }

    pitch_rate_offset = pitch_rate_offset/30;
    pitch_offset = pitch_offset/30;
    roll_rate_offset = roll_rate_offset/30;
    roll_offset = roll_offset/30;
}

void printOffsets() {
  Serial.println("OFFSETS: ");
  Serial.println("=================");

  Serial.print("pitch rate: ");
  Serial.println(pitch_rate_offset);

  Serial.print("pitch: ");
  Serial.println(pitch_offset);

  Serial.print("roll rate: ");
  Serial.println(roll_rate_offset);

  Serial.print("roll: ");
  Serial.println(roll_offset);
  Serial.println("");
}

void disableQuad() {
  // analogWrite(pretty_leds, 0);

  digitalWrite(18, 0);

  analogWrite(3, 0); // front right
  analogWrite(4, 0); // front left
  analogWrite(8, 0); // bottom right
  analogWrite(5, 0); // botttom left
}

void fly() {

  // for the test copter
  analogWrite(pretty_leds, 127);

  analogWrite(3, motorVals[0]); // front right
  analogWrite(4, motorVals[1]); // front left
  analogWrite(8, motorVals[2]); // back right
  analogWrite(5, motorVals[3]); // back left


  // for our quad
  // digitalWrite(18, 1);

  // analogWrite(5, motorVals[0]); // front right
  // analogWrite(3, motorVals[1]); // front left
  // analogWrite(4, motorVals[2]); // back right
  // analogWrite(8, motorVals[3]); // back left

}

void loadPIDcoef() {
  /* PID coefficients YAW */
  // P 
  p_co_yaw = receive[14] + (receive[15] * 0.01);

  // I
  i_co_yaw = receive[16] + (receive[17] * 0.01);

  // D
  d_co_yaw = receive[18] + (receive[19] * 0.01);

  /* PID coefficients ROLL */
  // P 
  p_co_roll = receive[20] + (receive[21] * 0.01);

  // I
  i_co_roll = receive[22] + (receive[23] * 0.01);

  // D
  d_co_roll = receive[24] + (receive[25] * 0.01);

  /* PID coefficients PITCH */
  // P 
  p_co_pitch = receive[26] + (receive[27] * 0.01);

  // I
  i_co_pitch = receive[28] + (receive[29] * 0.01);

  // D
  d_co_pitch = receive[30] + (receive[31] * 0.01);

  // Serial.println("yaw coefficients: ");
  // Serial.print("P - ");
  // Serial.print(p_co_yaw);
  // Serial.print(" I - ");
  // Serial.print(i_co_yaw);
  // Serial.print(" D - ");
  // Serial.println(d_co_yaw);
  // Serial.println("");

  // Serial.println("roll coefficients: ");
  // Serial.print("P - ");
  // Serial.print(p_co_roll);
  // Serial.print(" I - ");
  // Serial.print(i_co_roll);
  // Serial.print(" D - ");
  // Serial.println(d_co_roll);
  // Serial.println("");

  // Serial.println("pitch coefficients: ");
  // Serial.print("P - ");
  // Serial.print(p_co_pitch);
  // Serial.print(" I - ");
  // Serial.print(i_co_pitch);
  // Serial.print(" D - ");
  // Serial.println(d_co_pitch);
  // Serial.println("");
}

void PID(float curr_yaw, float curr_roll, float curr_pitch, float dt) {
  // Serial.println(dt);
  loadPIDcoef();

  // Serial.println(receive[2]);

  // Serial.print(curr_roll);
  // Serial.print(" ");
  // Serial.println(curr_pitch);

  // YAW
  curr_error_yaw = curr_yaw - remoteSetAngle[0];
  error_yaw_sum = error_yaw_sum + curr_error_yaw;
  if (receive[2] < 2 || i_co_yaw == 0) {
    error_yaw_sum = 0;
  }

  // yaw pid out
  PIDout[0] = (p_co_yaw * curr_error_yaw) + (i_co_yaw * (error_yaw_sum * dt)) + (d_co_yaw * ((curr_error_yaw - last_error_yaw) / dt));
  last_error_yaw = curr_error_yaw;

  // ROLL
  curr_error_roll = curr_roll - remoteSetAngle[1];
  error_roll_sum = error_roll_sum + curr_error_roll;
  if (receive[2] < 2 || i_co_roll == 0) {
    error_roll_sum = 0;
  }

  // roll pid out
  PIDout[1] = (p_co_roll * curr_error_roll) + (i_co_roll * (error_roll_sum * dt)) + (d_co_roll * ((curr_error_roll - last_error_roll) / dt));
  last_error_roll = curr_error_roll;

  // PITCH
  curr_error_pitch = curr_pitch - remoteSetAngle[2];
  // Serial.print(curr_error_pitch);
  // Serial.print(" ");
  // Serial.println((curr_error_pitch - last_error_pitch) / dt);
  error_pitch_sum = error_pitch_sum + curr_error_pitch;
  if (receive[2] < 2 || i_co_pitch == 0) {
    error_pitch_sum = 0;
  }

  // pitch pid out
  // Serial.println(curr_error_pitch - last_error_pitch);
  PIDout[2] = (p_co_pitch * curr_error_pitch) + (i_co_pitch * (error_pitch_sum * dt)) + (d_co_pitch * ((curr_error_pitch - last_error_pitch) / dt));
  last_error_pitch = curr_error_pitch;

  // Serial.println("PID outputs:");
  // Serial.print("Yaw - ");
  // Serial.println(PIDout[0]);
  // Serial.print(" Roll - ");
  // Serial.print(PIDout[1]);
  // Serial.print(" Pitch - ");
  // Serial.println(PIDout[2]);
}

void setMotorValues() {
  // Serial.println(receive[2]);
  if (receive[2] < 2) {
    motorVals[0] = 0;
    motorVals[1] = 0;
    motorVals[2] = 0;
    motorVals[3] = 0;
  } else {
    motorVals[0] = receive[2] + PIDout[0] + PIDout[2] + PIDout[1];
    motorVals[1] = receive[2] - PIDout[0] + PIDout[2] - PIDout[1];
    motorVals[2] = receive[2] - PIDout[0] - PIDout[2] + PIDout[1];
    motorVals[3] = receive[2] + PIDout[0] - PIDout[2] - PIDout[1];

    // motorVals[0] = receive[2] + PIDout[2];
    // motorVals[1] = receive[2] + PIDout[2];
    // motorVals[2] = receive[2] - PIDout[2];
    // motorVals[3] = receive[2] - PIDout[2];

  }

  // Handling saturation
  for (int i = 0; i < 4; i++) {
    if (motorVals[i] < 0) {
      motorVals[i] = 0;
    }
    if (motorVals[i] > 255) {
      motorVals[i] = 255;
    }
  }
}

void setup() {
  Serial.begin(115200);
  rfBegin(19);
  rfWrite(MAGIC_NUMBER);

  setupSensor();

  // for the LEDs
  pinMode(18,OUTPUT); // for our quad
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);

  startMillis = millis();
  loopEnd = millis();

  // gyro / accel setup
  lsm.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  lsm.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);

  lsm.setGyroDataRate(LSM6DS_RATE_833_HZ);
  lsm.setAccelDataRate(LSM6DS_RATE_833_HZ);
  
  lsm.setAccelCompositeFilter(LSM6DS_CompositeFilter_LPF2, LSM6DS_CompositeFilter_ODR_DIV_20);
  // lsm.setGyroHPF(true, LSM6DS_gyroHPF_0);

  resetFlag = 1;
}

void loop() {
  currentMillis = millis();
  float time = currentMillis - loopEnd - 0.00;
  // Serial.println(time);

  // reading the remote
  
  if (rfAvailable() == PACKET_SIZE) {
    // populate the buffer with data read from radio
    rfRead(receive, PACKET_SIZE);
    startMillis = currentMillis;
  }

  // Calculate the complementary filter
  ahrs->getQuadOrientation(&orientation);
  
  // Pitch complimentary filter
  cf_angle_pitch = calculateCf(cf_angle_pitch, orientation.pitch_rate - pitch_rate_offset, orientation.pitch - pitch_offset, time/1000.0);
  // Roll complimentary filter
  cf_angle_roll = calculateCf(cf_angle_roll, orientation.roll_rate - roll_rate_offset, orientation.roll - roll_offset, time/1000.0);  

  // Serial.print(orientation.pitch_rate);
  // Serial.print(" ");
  // Serial.println(orientation.roll_rate);
  /* view the pitch and roll angles */
  // Serial.print(cf_angle_pitch);
  // Serial.print(" ");
  // Serial.println(cf_angle_roll);

  //unfilitered yaw rate
  yaw_rate = orientation.yaw_rate;

  // Serial.println(orientation.yaw_rate);

  /* For changing/viewing the YAW0, ROLL0, PITCH0 values */
  /* TODO: UNCOMMENT TO SET DEFINES FOR YAW, ROLL, AND PITCH */
  // viewGimbalPositions();

  // PID section -- receive[3], receive[4], receive[5] = yaw, roll, pitch respectively
  calculateSetAngles(receive[3], receive[4], receive[5]);

  /* For printing set angles */
  printSetAngles();

  // PID
  PID(yaw_rate, cf_angle_roll, cf_angle_pitch, time/1000.0);
  setMotorValues();

  if (currentMillis - startMillis >= 1000) {
    disableQuad();
  } else {
    // if packet matches
    if (receive[0] == MAGIC_NUMBER) {
      // if we JUST armed the quadcopter, calibrate the IMU offsets
      if (receive[1] && resetFlag) {
        adjustOffsets();

        /* For viewing gyro and acc offset */
        // printOffsets();

        resetFlag = 0;
      } else if (receive[1] && !resetFlag) { // quad is armed, ready to fly
        fly();
      } else { // quad is not armed, DO NOT fly
        disableQuad();
      }
    }
  }

  loopEnd = currentMillis;
}