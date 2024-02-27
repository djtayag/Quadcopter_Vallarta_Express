#include "Arduino.h"
#include "Control.h"
#include "quad_remote.h"
#include <SerLCD.h>
#include <EEPROM.h>

int initiate_calibrate = 0;
int write_pid_eeprom = 0;
int THROTTLE_LOWER_BOUND = 0;
int THROTTLE_UPPER_BOUND = 0;
int YAW_LOWER_BOUND = 0;
int YAW_UPPER_BOUND = 0;
int ROLL_LOWER_BOUND = 0;
int ROLL_UPPER_BOUND = 0;
int PITCH_LOWER_BOUND = 0;
int PITCH_UPPER_BOUND = 0;
// EEPROM organized throttle, yaw, roll, pitch (lower first, then upper)

void loadGimbalValues() {
  THROTTLE_LOWER_BOUND = readIntFromEEPROM(0);
  THROTTLE_UPPER_BOUND = readIntFromEEPROM(2);
  YAW_LOWER_BOUND = readIntFromEEPROM(4);
  YAW_UPPER_BOUND = readIntFromEEPROM(6);
  ROLL_LOWER_BOUND = readIntFromEEPROM(8);
  ROLL_UPPER_BOUND = readIntFromEEPROM(10);
  PITCH_LOWER_BOUND = readIntFromEEPROM(12);
  PITCH_UPPER_BOUND = readIntFromEEPROM(14);
}

void updateValues(Control *temp) {
  unsigned int temp_throttle, temp_yaw, temp_roll, temp_pitch;

  // Gimbals
  temp_throttle = analogRead(PIN_THROTTLE);
  temp_yaw = analogRead(PIN_YAW);
  temp_roll = analogRead(PIN_ROLL);
  temp_pitch = analogRead(PIN_PITCH);
  
  temp_throttle = constrain(temp_throttle, THROTTLE_LOWER_BOUND, THROTTLE_UPPER_BOUND);
  temp_yaw = constrain(temp_yaw, YAW_LOWER_BOUND, YAW_UPPER_BOUND);
  temp_roll = constrain(temp_roll, ROLL_LOWER_BOUND, ROLL_UPPER_BOUND);
  temp_pitch = constrain(temp_pitch, PITCH_LOWER_BOUND, PITCH_UPPER_BOUND);

  temp->throttle = map(temp_throttle, THROTTLE_LOWER_BOUND, THROTTLE_UPPER_BOUND, 0, 255);
  temp->yaw = map(temp_yaw, YAW_LOWER_BOUND, YAW_UPPER_BOUND, 0, 255);
  temp->roll = map(temp_roll, ROLL_LOWER_BOUND, ROLL_UPPER_BOUND, 0, 255);
  temp->pitch = map(temp_pitch, PITCH_LOWER_BOUND, PITCH_UPPER_BOUND, 0, 255);

  temp->up = is_pressed(BUTTON_UP_PIN);
  temp->down = is_pressed(BUTTON_DOWN_PIN);
  temp->left = is_pressed(BUTTON_LEFT_PIN);
  temp->right = is_pressed(BUTTON_RIGHT_PIN);
  temp->center = is_pressed(BUTTON_CENTER_PIN);

  temp->btn1 = is_pressed(BUTTON1_PIN);
  temp->btn2 = is_pressed(BUTTON2_PIN);

  // set the arm flag to 1 if gimbals are in the arm position
  if (temp->throttle < 5 && temp->yaw > 250 && temp->roll > 250 && temp->pitch > 250 && !temp->armed) {
    temp->armed = 1;
    lcd.setBacklight(199,0,57);
  }

  // disarm quad when left and right is pressed
  if (temp->left && temp->right && temp->armed) {
    temp->armed = 0;
    lcd.setBacklight(255,255,255);
  }

  // handle write to eeprom (PID)
  if (temp->center) {
    write_pid_eeprom++;
  }

  if (!temp->center) {
    write_pid_eeprom = 0;
  }

  if (write_pid_eeprom == 800) {
    savePIDtoEEPROM();
  }

  // Handle calibrating
  if (temp->down && temp->btn2 && temp->armed == 0) {
    initiate_calibrate++;
  }

  if (!(temp->down && temp->btn2)) {
    initiate_calibrate = 0;
  }

  if (initiate_calibrate == 300) {
     temp->calibrate_flag = 1;
  }
}

void printValues(Control *temp) {
    Serial.print("Magic number: ");
    Serial.println(temp->magic);

    Serial.print("Throttle: ");
    Serial.println(temp->throttle);
    Serial.print("Yaw: ");
    Serial.println(temp->yaw);
    Serial.print("Roll: ");
    Serial.println(temp->roll);
    Serial.print("Pitch: ");
    Serial.println(temp->pitch);

    Serial.print("Up: ");
    Serial.println(temp->up);
    Serial.print("Down: ");
    Serial.println(temp->down);
    Serial.print("Left: ");
    Serial.println(temp->left);
    Serial.print("Right: ");
    Serial.println(temp->right);
    Serial.print("Center: ");
    Serial.println(temp->center);

    Serial.print("Button1: ");
    Serial.println(temp->btn1);
    Serial.print("Button2: ");
    Serial.println(temp->btn2);

    Serial.print("Calibrate flag: ");
    Serial.println(temp->calibrate_flag);

    Serial.print("Armed: ");
    Serial.println(temp->armed);

    // Serial.println("PID Coefficients: ");
    // Serial.println("YAW - ");
    // Serial.print(temp->yaw_p_ones);
    // Serial.print(" ");
    // Serial.print(temp->yaw_p_decimal);
    // Serial.print(" | ");
    // Serial.print(temp->yaw_i_ones);
    // Serial.print(" ");
    // Serial.print(temp->yaw_i_decimal);
    // Serial.print(" | ");
    // Serial.print(temp->yaw_d_ones);
    // Serial.print(" ");
    // Serial.println(temp->yaw_d_decimal);

    // Serial.println("ROLL - ");
    // Serial.print(temp->roll_p_ones);
    // Serial.print(" ");
    // Serial.print(temp->roll_p_decimal);
    // Serial.print(" | ");
    // Serial.print(temp->roll_i_ones);
    // Serial.print(" ");
    // Serial.print(temp->roll_i_decimal);
    // Serial.print(" | ");
    // Serial.print(temp->roll_d_ones);
    // Serial.print(" ");
    // Serial.println(temp->roll_d_decimal);

    // Serial.println("PITCH - ");
    // Serial.print(temp->pitch_p_ones);
    // Serial.print(" ");
    // Serial.print(temp->pitch_p_decimal);
    // Serial.print(" | ");
    // Serial.print(temp->pitch_i_ones);
    // Serial.print(" ");
    // Serial.print(temp->pitch_i_decimal);
    // Serial.print(" | ");
    // Serial.print(temp->pitch_d_ones);
    // Serial.print(" ");
    // Serial.println(temp->pitch_d_decimal);

    Serial.println("==============="); 
}

void calibrate(Control *temp) {
  temp->calibrate_flag = 0;
  lcd.setBacklight(80,200,120);
  delay(1000);
  lcd.clear();

  lcd.write("Calibrating...");
  delay(1000);
  lcd.clear();

  // Throttle LOWER_BOUND
  lcd.write("Hold throttle down 5...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold throttle down 4...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold throttle down 3...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold throttle down 2...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold throttle down 1...");
  delay(1000);
  lcd.clear();

  writeIntIntoEEPROM(0, analogRead(PIN_THROTTLE));
  delay(500);

  // Throttle UPPER_BOUND
  lcd.write("Hold throttle up 5...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold throttle up 4...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold throttle up 3...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold throttle up 2...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold throttle up 1...");
  delay(1000);
  lcd.clear();

  writeIntIntoEEPROM(2, analogRead(PIN_THROTTLE));
  delay(500);

  // Yaw UPPER_BOUND
  lcd.write("Hold yaw left 5...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold yaw left 4...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold yaw left 3...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold yaw left 2...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold yaw left 1...");
  delay(1000);
  lcd.clear();

  writeIntIntoEEPROM(6, analogRead(PIN_YAW));
  delay(500);

  // Yaw LOWER_BOUND
  lcd.write("Hold yaw right 5...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold yaw right 4...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold yaw right 3...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold yaw right 2...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold yaw right 1...");
  delay(1000);
  lcd.clear();

  writeIntIntoEEPROM(4, analogRead(PIN_YAW));
  delay(500);

  // Roll LOWER_BOUND
  lcd.write("Hold roll left 5...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold roll left 4...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold roll left 3...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold roll left 2...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold roll left 1...");
  delay(1000);
  lcd.clear();

  writeIntIntoEEPROM(8, analogRead(PIN_ROLL));
  delay(500);

  // Roll UPPER_BOUND
  lcd.write("Hold roll right 5...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold roll right 4...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold roll right 3...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold roll right 2...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold roll right 1...");
  delay(1000);
  lcd.clear();

  writeIntIntoEEPROM(10, analogRead(PIN_ROLL));
  delay(500);

  // Pitch LOWER_BOUND
  lcd.write("Hold pitch up 5...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold pitch up 4...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold pitch up 3...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold pitch up 2...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold pitch up 1...");
  delay(1000);
  lcd.clear();

  writeIntIntoEEPROM(12, analogRead(PIN_PITCH));
  delay(500);

  // Pitch UPPER_BOUND
  lcd.write("Hold pitch down 5...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold pitch down 4...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold pitch down 3...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold pitch down 2...");
  delay(1000);
  lcd.clear();

  lcd.write("Hold pitch down 1...");
  delay(1000);
  lcd.clear();

  writeIntIntoEEPROM(14, analogRead(PIN_PITCH));
  delay(500);

  lcd.setBacklight(255,255,255);
  delay(2000);
  lcd.write("COMPLETE!");
  delay(1000);
  lcd.clear();
}