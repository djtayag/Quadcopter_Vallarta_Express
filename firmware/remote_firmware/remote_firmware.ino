#include <radio.h>
#include "quad_remote.h"      // Header file with pin definitions and setup
#include <SerLCD.h>
#include "Control.h"
#include <EEPROM.h>

#define YAW_P_STRING " Yaw P"
#define YAW_I_STRING " Yaw I"
#define YAW_D_STRING " Yaw D"
#define ROLL_P_STRING " Roll P"
#define ROLL_I_STRING " Roll I"
#define ROLL_D_STRING " Roll D"
#define PITCH_P_STRING " Pitch P"
#define PITCH_I_STRING " Pitch I"
#define PITCH_D_STRING " Pitch D"

Control temp = {};
uint8_t b[PACKET_SIZE];

String msg;

/* 
 *  example PID_coefficients[0][0] = yaw_p_ones
 *          PID_coefficients[0][1] = yaw_p_decimal
 */
uint8_t PID_co[3][6]; // 3 for each of the yaw, roll, pitch; and each control has PID ones + decimal

// row specifies which control: yaw, roll, pitch
// col specifies ones/decimals for PID coefficietnts
int row = 0;
int col = 0;

int PIDflag = 1;

unsigned int startMillis;
unsigned int currentMillis;
const unsigned long PERIOD = 100;

// populate the PID coefficients array with coefficients stored in EEPROM (addr 16-33)
void loadPIDcoefficients() {
  int addr = 16;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 6; j++) {
      PID_co[i][j] = EEPROM.read(addr);
      addr++;
    }
  }
}

void update_display() {
	lcd.clear();
	// lcd.setCursor(column, row);
	lcd.print(knob1.getCurrentPos());
}

void knobs_update() {
  if (PIDflag) {
    if (col % 2 == 0) {
      PID_co[row][col] = (PID_co[row][col] + 1) % 10;
    } else {
      PID_co[row][col] = (PID_co[row][col] + 1) % 100;
    }
    Serial.println(PID_co[row][col]);
    updatePIDscreen();
  }
}


void btn1_pressed(bool down) {
	if (down) {
    if (PIDflag) {
      if (col % 2 == 0) {
        PID_co[row][col] = (PID_co[row][col] + 1) % 10;
      } else {
        PID_co[row][col] = (PID_co[row][col] + 1) % 100;
      }
      updatePIDscreen();
    }
  }
}

void btn2_pressed(bool down) {
	if(down) {
    if (PIDflag) {
      if (col % 2 == 0) {
        PID_co[row][col] = ((PID_co[row][col] - 1) % 10 + 10) % 10;
      } else {
        PID_co[row][col] = ((PID_co[row][col] - 1) % 100 + 100) % 100;
      }
      updatePIDscreen();
    }
  }
}

void knob_pressed(bool down) {
	if(down) {
    if (PIDflag) {
      PID_co[row][col] = knob1.setCurrentPos(0); // reset to 0
      updatePIDscreen();
    }
  }
}


void btn_up_pressed(bool down) {
	if(down) {
    if (PIDflag) {
		  row = (row + 1) % 3; // 0 - yaw, 1 - pitch, ...  
      // Serial.println(row);
      updatePIDscreen();
    }
  }
}

void btn_down_pressed(bool down) {
	if(down) {
    if (PIDflag) {
      row = ((row-1) % 3 + 3) % 3;
      // Serial.println(row);
      updatePIDscreen();
    }
	} 
}

void btn_left_pressed(bool down) {
	if(down) {
		if (PIDflag) {
      col = ((col-1) % 6 + 6) % 6; // 0 - p_ones, 1 - p_decimal, 2 - i_ones...
      // Serial.println(col);
      updatePIDscreen();
    }
  }
}

void btn_right_pressed(bool down) {
	if(down) {
    if (PIDflag) {
      col = (col + 1) % 6;
      // Serial.println(col);
      updatePIDscreen();
    }
	} 
}

    
void btn_center_pressed(bool down) {
}

void sendToFCB(Control *temp) {
  // initialize values of the array 
  b[0] = temp->magic;
  b[1] = temp->armed;
  b[2] = temp->throttle;
  b[3] = temp->yaw;
  b[4] = temp->roll;
  b[5] = temp->pitch;
  b[6] = temp->up;
  b[7] = temp->down;
  b[8] = temp->left;
  b[9] = temp->right;
  b[10] = temp->center;
  b[11] = temp->btn1;
  b[12] = temp->btn2;
  b[13] = temp->calibrate_flag;

  b[14] = PID_co[0][0];
  b[15] = PID_co[0][1];
  b[16] = PID_co[0][2];
  b[17] = PID_co[0][3];
  b[18] = PID_co[0][4];
  b[19] = PID_co[0][5];

  b[20] = PID_co[1][0];
  b[21] = PID_co[1][1];
  b[22] = PID_co[1][2];
  b[23] = PID_co[1][3];
  b[24] = PID_co[1][4];
  b[25] = PID_co[1][5];

  b[26] = PID_co[2][0];
  b[27] = PID_co[2][1];
  b[28] = PID_co[2][2];
  b[29] = PID_co[2][3];
  b[30] = PID_co[2][4];
  b[31] = PID_co[2][5];

  rfWrite(b, PACKET_SIZE);
}

void writeIntIntoEEPROM(int address, int number)
{ 
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
}

int readIntFromEEPROM(int address)
{
  byte byte1 = EEPROM.read(address);
  byte byte2 = EEPROM.read(address + 1);

  return (byte1 << 8) + byte2;
}

// reset function
void(* resetFunc) (void) = 0;

void updatePIDscreen() {
  lcd.clear();
  int ones, decimal;
  int onesCursor = 10;
  int decimalCursor = 12;
  int decimalPoint = 11;

  // initialize ones and decimal
  if (col % 2 == 0) { //  tuning the ones place
    ones = PID_co[row][col];
    decimal = PID_co[row][col+1];
  } else {            // tuning the decimal place
    ones = PID_co[row][col-1];
    decimal = PID_co[row][col];
  }

  lcd.setCursor(0, 0);
  // which control are we changing?
  if (row == 0) {
    if (col == 0 || col == 1) {
      msg = YAW_P_STRING;
    } else if (col == 2 || col == 3) {
      msg = YAW_I_STRING;
    } else {
      msg = YAW_D_STRING;
    }
  } else if (row == 1) {
    if (col == 0 || col == 1) {
      msg = ROLL_P_STRING;
    } else if (col == 2 || col == 3) {
      msg = ROLL_I_STRING;
    } else {
      msg = ROLL_D_STRING;
    }
  } else {
    if (col == 0 || col == 1) {
      msg = PITCH_P_STRING;
    } else if (col == 2 || col == 3) {
      msg = PITCH_I_STRING;
    } else {
      msg = PITCH_D_STRING;
    }
  }

  lcd.print(msg);

  // print PID coefficient on LCD
  lcd.setCursor(onesCursor, 0);
  lcd.print(ones);
  lcd.setCursor(decimalPoint, 0);
  lcd.print(".");
  if (decimal < 10) { // need to add one leading 0
    lcd.setCursor(decimalCursor, 0);
    lcd.print("0");
    lcd.print(decimal);
  } else { // no need to add leading 0
    lcd.setCursor(decimalCursor, 0);
    lcd.print(decimal);
  }
  
  // set cursor on screen
  lcd.cursor();
  if (col % 2 == 0) {
    lcd.setCursor(onesCursor, 0);
  } else {
    lcd.setCursor(decimalCursor, 0);
  }

}

void savePIDtoEEPROM() {
  PIDflag = 0;
  lcd.clear();
  lcd.noCursor();
  lcd.setCursor(0, 0);
  lcd.print("SAVING PID!");
  delay(1000);

  int addr = 16;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 6; j++) {
      EEPROM.write(addr, PID_co[i][j]);
      addr++;
    }
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SAVED!");
  delay(1000);

  resetFunc();
}

void setup() {
  Serial.begin(115200);

  // Configuring battery voltage reading
  analogReference(INTERNAL);
  pinMode(BATTERY_SENSE_PIN, INPUT);

  delay(100);
  quad_remote_setup();
  loadGimbalValues();
  loadPIDcoefficients();

  // setting up radio
  rfBegin(19);

  // The buttons and the knob trigger these call backs.       
	knobs_update_cb = knobs_update; 
	knob1_btn_cb = knob_pressed;
	btn1_cb = btn1_pressed;
	btn2_cb = btn2_pressed;
	btn_up_cb =  btn_up_pressed;
	btn_down_cb = btn_down_pressed;
	btn_left_cb =  btn_left_pressed;
	btn_right_cb = btn_right_pressed;
	btn_center_cb =  btn_center_pressed;

  updatePIDscreen();
  startMillis = millis();
}

void loop() {
  if (rfAvailable()) {
    if (rfRead() == MAGIC_NUMBER) {
      (&temp)->armed = 0;
    }
  }
  updateValues(&temp);
  // printValues(&temp);

  // calibrate gimbals
  if (temp.calibrate_flag) {
    lcd.noCursor();
    PIDflag = 0;
    calibrate(&temp);
    resetFunc();
  }

  currentMillis = millis();
  if (currentMillis - startMillis >= PERIOD) {
    sendToFCB(&temp);
    startMillis = currentMillis;
  }
}