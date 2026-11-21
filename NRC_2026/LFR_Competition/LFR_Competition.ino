/*
 ==============================================================================
   COMPETITION GRADE 6-SENSOR PID LINE FOLLOWER ROBOT (LFR)
 ==============================================================================
   Pin Configuration:
   ----------------------------------------------------------------------------
   IR SENSORS (Analog pins):
     S1 (Far Left / Branch)  : A7 (Analog-only on Nano)
     S2 (Mid Left)           : A6 (Analog-only on Nano)
     S3 (Center Left)        : A3
     S4 (Center Right)       : A2
     S5 (Mid Right)          : A1
     S6 (Far Right / Branch) : A0

   MOTOR DRIVER (TB6612FNG):
     PWMA (Left Motor Speed) : D3
     AIN2 (Left Motor Dir)   : D4
     AIN1 (Left Motor Dir)   : D5
     STBY (Standby - HIGH)   : D6
     BIN1 (Right Motor Dir)  : D7
     BIN2 (Right Motor Dir)  : D8
     PWMB (Right Motor Speed): D9

   BUTTONS (Active LOW with internal pull-up):
     UP / NEXT               : D2
     DOWN / PREV             : D10
     OK / ENTER              : D11

   DISPLAY (0.96" I2C OLED SSD1306 - 128x64):
     SDA                     : A4
     SCL                     : A5
 ==============================================================================
*/

#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ------------------- OLED Configuration -------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ------------------- Pin Definitions -------------------
// Sensors
const uint8_t sensorPins[6] = {A7, A6, A3, A2, A1, A0};

// TB6612FNG Motor Driver
#define PIN_PWMA 3
#define PIN_AIN2 4
#define PIN_AIN1 5
#define PIN_STBY 6
#define PIN_BIN1 7
#define PIN_BIN2 8
#define PIN_PWMB 9

// Buttons
#define BTN_UP   2
#define BTN_DOWN 10
#define BTN_OK   11

// ------------------- EEPROM Storage Structure -------------------
struct ConfigData {
  float Kp;
  float Ki;
  float Kd;
  int baseSpeed;
  int maxSpeed;
  int turnSpeed;
  int minVal[6];
  int maxVal[6];
  int threshold[6];
  bool isBlackLine; // true = Black line on white surface, false = White on black
  uint8_t magic;    // Validation byte to verify EEPROM has been initialized
} config;

#define EEPROM_MAGIC 0x47

// ------------------- Runtime Variables -------------------
enum RobotState {
  STATE_MENU,
  STATE_RUNNING,
  STATE_CALIBRATING,
  STATE_SENSOR_TEST,
  STATE_TUNE_PARAM
};

RobotState currentState = STATE_MENU;

int rawSensor[6];
int normSensor[6];
bool digSensor[6];

// PID Variables
float P = 0, I = 0, D = 0, PID_Value = 0;
float error = 0, previous_error = 0;
int lastPosition = 0;

// Menu System Variables
int menuIndex = 0;
const int TOTAL_MENU_ITEMS = 9;
const char* const menuItems[] = {
  "1. Start Run",
  "2. Calibrate Sensors",
  "3. Sensor Monitor",
  "4. Tune Kp",
  "5. Tune Ki",
  "6. Tune Kd",
  "7. Base Speed",
  "8. Max Speed",
  "9. Line Mode"
};

int tuningParamIndex = 0;

// ==============================================================================
//  MOTOR DRIVER FUNCTIONS
// ==============================================================================
void initMotors() {
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT);

  digitalWrite(PIN_STBY, HIGH); // Enable driver
  stopMotors();
}

void setMotors(int leftSpeed, int rightSpeed) {
  // Constrain speeds to limits
  leftSpeed = constrain(leftSpeed, -config.maxSpeed, config.maxSpeed);
  rightSpeed = constrain(rightSpeed, -config.maxSpeed, config.maxSpeed);

  // Left Motor Control
  if (leftSpeed > 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    analogWrite(PIN_PWMA, leftSpeed);
  } else if (leftSpeed < 0) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
    analogWrite(PIN_PWMA, -leftSpeed);
  } else {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, HIGH); // Short-brake
    analogWrite(PIN_PWMA, 0);
  }

  // Right Motor Control
  if (rightSpeed > 0) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
    analogWrite(PIN_PWMB, rightSpeed);
  } else if (rightSpeed < 0) {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
    analogWrite(PIN_PWMB, -rightSpeed);
  } else {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, HIGH); // Short-brake
    analogWrite(PIN_PWMB, 0);
  }
}

void stopMotors() {
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, LOW);
  analogWrite(PIN_PWMA, 0);
  analogWrite(PIN_PWMB, 0);
}

// ==============================================================================
//  BUTTON HANDLING (DEBOUNCED)
// ==============================================================================
void initButtons() {
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_OK, INPUT_PULLUP);
}

bool isButtonPressed(uint8_t pin) {
  if (digitalRead(pin) == LOW) {
    delay(20); // Simple debounce
    if (digitalRead(pin) == LOW) {
      while (digitalRead(pin) == LOW); // Wait for release
      delay(20);
      return true;
    }
  }
  return false;
}

// ==============================================================================
//  EEPROM LOAD & SAVE
// ==============================================================================
void loadConfig() {
  EEPROM.get(0, config);
  if (config.magic != EEPROM_MAGIC) {
    // Factory Defaults
    config.Kp = 0.045;
    config.Ki = 0.000;
    config.Kd = 0.350;
    config.baseSpeed = 160;
    config.maxSpeed = 230;
    config.turnSpeed = 150;
    config.isBlackLine = true;
    for (int i = 0; i < 6; i++) {
      config.minVal[i] = 100;
      config.maxVal[i] = 900;
      config.threshold[i] = 500;
    }
    config.magic = EEPROM_MAGIC;
    saveConfig();
  }
}

void saveConfig() {
  EEPROM.put(0, config);
}

// ==============================================================================
//  SENSOR READ & POSITION CALCULATION
// ==============================================================================
void readSensors() {
  for (int i = 0; i < 6; i++) {
    // S1=A7 and S2=A6 can ONLY be read using analogRead!
    rawSensor[i] = analogRead(sensorPins[i]);

    // Normalize sensor 0 - 1000
    int clamped = constrain(rawSensor[i], config.minVal[i], config.maxVal[i]);
    if (config.isBlackLine) {
      // Black surface gives higher ADC value
      normSensor[i] = map(clamped, config.minVal[i], config.maxVal[i], 0, 1000);
      digSensor[i] = (rawSensor[i] > config.threshold[i]);
    } else {
      // White line on black surface
      normSensor[i] = map(clamped, config.minVal[i], config.maxVal[i], 1000, 0);
      digSensor[i] = (rawSensor[i] < config.threshold[i]);
    }
  }
}

// Calculates line position from -2500 (Far Left) to +2500 (Far Right)
// 0 is perfectly centered
int getLinePosition() {
  readSensors();

  long weightedSum = 0;
  long sum = 0;
  // Sensor weights: S1=-2500, S2=-1500, S3=-500, S4=+500, S5=+1500, S6=+2500
  const int weights[6] = {-2500, -1500, -500, 500, 1500, 2500};

  int activeCount = 0;
  for (int i = 0; i < 6; i++) {
    if (normSensor[i] > 200) { // Noise cutoff
      weightedSum += (long)normSensor[i] * weights[i];
      sum += normSensor[i];
      activeCount++;
    }
  }

  // If line is completely lost (all sensors on white)
  if (sum < 300) {
    // Return last known direction to aggressively turn back to the line
    if (lastPosition < -500) return -3000;
    if (lastPosition > 500)  return 3000;
    return lastPosition;
  }

  lastPosition = weightedSum / sum;
  return lastPosition;
}

// ==============================================================================
//  AUTO SENSOR CALIBRATION ROUTINE
// ==============================================================================
void runCalibration() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 10);
  display.println(F("CALIBRATING..."));
  display.setCursor(5, 30);
  display.println(F("Move robot across"));
  display.setCursor(5, 42);
  display.println(F("all line paths!"));
  display.display();

  // Reset mins and maxs
  for (int i = 0; i < 6; i++) {
    config.minVal[i] = 1023;
    config.maxVal[i] = 0;
  }

  unsigned long startTime = millis();
  // Calibrate for 6 seconds
  while (millis() - startTime < 6000) {
    for (int i = 0; i < 6; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < config.minVal[i]) config.minVal[i] = val;
      if (val > config.maxVal[i]) config.maxVal[i] = val;
    }

    // Progress bar
    int progress = map(millis() - startTime, 0, 6000, 0, 120);
    display.fillRect(4, 55, progress, 6, SSD1306_WHITE);
    display.display();
    delay(10);
  }

  // Compute thresholds
  for (int i = 0; i < 6; i++) {
    config.threshold[i] = (config.minVal[i] + config.maxVal[i]) / 2;
  }
  saveConfig();

  display.clearDisplay();
  display.setCursor(20, 25);
  display.println(F("CALIBRATION"));
  display.setCursor(40, 40);
  display.println(F("SAVED!"));
  display.display();
  delay(1200);
  currentState = STATE_MENU;
}

// ==============================================================================
//  PID LINE FOLLOWING LOGIC
// ==============================================================================
void runPID() {
  int position = getLinePosition();

  // Check Special Conditions:
  // 1. Extreme Line Loss Handling
  if (position <= -2800) {
    // Sharp Left Turn / Spin
    setMotors(-config.turnSpeed, config.turnSpeed);
    return;
  } else if (position >= 2800) {
    // Sharp Right Turn / Spin
    setMotors(config.turnSpeed, -config.turnSpeed);
    return;
  }

  // 2. 90-Degree Left Branch Detection (S1 & S2 triggered)
  if (digSensor[0] && digSensor[1] && !digSensor[5]) {
    setMotors(-config.turnSpeed, config.turnSpeed);
    delay(30);
    return;
  }

  // 3. 90-Degree Right Branch Detection (S5 & S6 triggered)
  if (digSensor[4] && digSensor[5] && !digSensor[0]) {
    setMotors(config.turnSpeed, -config.turnSpeed);
    delay(30);
    return;
  }

  // Standard PID Calculation
  error = position; // Center error = 0
  P = error;
  I = I + error;
  I = constrain(I, -20000, 20000); // Anti-windup
  D = error - previous_error;
  previous_error = error;

  PID_Value = (config.Kp * P) + (config.Ki * I) + (config.Kd * D);

  int leftMotorSpeed = config.baseSpeed + (int)PID_Value;
  int rightMotorSpeed = config.baseSpeed - (int)PID_Value;

  setMotors(leftMotorSpeed, rightMotorSpeed);
}

// ==============================================================================
//  OLED DISPLAY & MENU PAGES
// ==============================================================================
void drawMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(30, 2);
  display.println(F("= LFR MENU ="));
  display.drawLine(0, 12, 127, 12, SSD1306_WHITE);

  int startIdx = (menuIndex / 4) * 4;
  for (int i = 0; i < 4; i++) {
    int item = startIdx + i;
    if (item >= TOTAL_MENU_ITEMS) break;

    int y = 16 + (i * 12);
    if (item == menuIndex) {
      display.fillRect(0, y - 1, 128, 11, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    } else {
      display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }
    display.setCursor(4, y);
    display.print(menuItems[item]);
  }
  display.display();
}

void drawSensorTest() {
  readSensors();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(20, 2);
  display.println(F("SENSOR MONITOR"));
  display.drawLine(0, 11, 127, 11, SSD1306_WHITE);

  // Draw 6 sensor bar graphs
  for (int i = 0; i < 6; i++) {
    int x = 6 + (i * 20);
    int barHeight = map(normSensor[i], 0, 1000, 0, 35);
    display.drawRect(x, 16, 16, 37, SSD1306_WHITE);
    if (barHeight > 0) {
      display.fillRect(x + 2, 16 + (37 - barHeight), 12, barHeight, SSD1306_WHITE);
    }

    display.setCursor(x + 5, 55);
    display.print(digSensor[i] ? "1" : "0");
  }
  display.display();
}

void drawTuning() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(15, 4);
  display.println(F("PARAMETER TUNE"));
  display.drawLine(0, 14, 127, 14, SSD1306_WHITE);

  display.setCursor(10, 24);
  display.setTextSize(1);
  display.print(menuItems[menuIndex]);

  display.setCursor(20, 42);
  display.setTextSize(2);
  switch (menuIndex) {
    case 3: display.print(config.Kp, 3); break;
    case 4: display.print(config.Ki, 4); break;
    case 5: display.print(config.Kd, 3); break;
    case 6: display.print(config.baseSpeed); break;
    case 7: display.print(config.maxSpeed); break;
    case 8: display.print(config.isBlackLine ? "BLACK" : "WHITE"); break;
  }
  display.display();
}

// ==============================================================================
//  SETUP & MAIN LOOP
// ==============================================================================
void setup() {
  Serial.begin(115200);

  initMotors();
  initButtons();
  loadConfig();

  // Initialize OLED (Address 0x3C is standard)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    // If 0x3C fails, try 0x3D
    display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(15, 18);
  display.println(F("HIGH SPEED LFR"));
  display.setCursor(22, 34);
  display.println(F("PID SYSTEM"));
  display.setCursor(28, 50);
  display.println(F("Ready!"));
  display.display();
  delay(1200);

  drawMenu();
}

void loop() {
  switch (currentState) {
    case STATE_MENU:
      if (isButtonPressed(BTN_UP)) {
        menuIndex = (menuIndex - 1 + TOTAL_MENU_ITEMS) % TOTAL_MENU_ITEMS;
        drawMenu();
      }
      if (isButtonPressed(BTN_DOWN)) {
        menuIndex = (menuIndex + 1) % TOTAL_MENU_ITEMS;
        drawMenu();
      }
      if (isButtonPressed(BTN_OK)) {
        switch (menuIndex) {
          case 0: // Start Run
            currentState = STATE_RUNNING;
            display.clearDisplay();
            display.setCursor(30, 25);
            display.setTextSize(2);
            display.setTextColor(SSD1306_WHITE);
            display.println(F("RUNNING"));
            display.setTextSize(1);
            display.setCursor(10, 48);
            display.println(F("Press OK to stop"));
            display.display();
            I = 0;
            previous_error = 0;
            break;
          case 1: // Calibrate
            currentState = STATE_CALIBRATING;
            runCalibration();
            break;
          case 2: // Sensor Test
            currentState = STATE_SENSOR_TEST;
            break;
          case 3: // Tune Kp
          case 4: // Tune Ki
          case 5: // Tune Kd
          case 6: // Base Speed
          case 7: // Max Speed
          case 8: // Line Mode
            currentState = STATE_TUNE_PARAM;
            drawTuning();
            break;
        }
      }
      break;

    case STATE_RUNNING:
      runPID();
      // Press OK anytime to stop
      if (digitalRead(BTN_OK) == LOW) {
        stopMotors();
        while (digitalRead(BTN_OK) == LOW);
        delay(100);
        currentState = STATE_MENU;
        drawMenu();
      }
      break;

    case STATE_SENSOR_TEST:
      drawSensorTest();
      if (isButtonPressed(BTN_OK)) {
        currentState = STATE_MENU;
        drawMenu();
      }
      break;

    case STATE_TUNE_PARAM:
      if (isButtonPressed(BTN_UP)) {
        switch (menuIndex) {
          case 3: config.Kp += 0.005; break;
          case 4: config.Ki += 0.001; break;
          case 5: config.Kd += 0.020; break;
          case 6: config.baseSpeed = constrain(config.baseSpeed + 10, 50, 255); break;
          case 7: config.maxSpeed  = constrain(config.maxSpeed + 10, 50, 255); break;
          case 8: config.isBlackLine = !config.isBlackLine; break;
        }
        drawTuning();
      }
      if (isButtonPressed(BTN_DOWN)) {
        switch (menuIndex) {
          case 3: config.Kp = max(0.000f, config.Kp - 0.005f); break;
          case 4: config.Ki = max(0.000f, config.Ki - 0.001f); break;
          case 5: config.Kd = max(0.000f, config.Kd - 0.020f); break;
          case 6: config.baseSpeed = constrain(config.baseSpeed - 10, 50, 255); break;
          case 7: config.maxSpeed  = constrain(config.maxSpeed - 10, 50, 255); break;
          case 8: config.isBlackLine = !config.isBlackLine; break;
        }
        drawTuning();
      }
      if (isButtonPressed(BTN_OK)) {
        saveConfig(); // Persist changes to EEPROM
        currentState = STATE_MENU;
        drawMenu();
      }
      break;

    default:
      currentState = STATE_MENU;
      break;
  }
}
