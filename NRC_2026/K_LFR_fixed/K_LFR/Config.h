#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================================================
// DIU SQUAD - LFR Robot Pinout (Arduino Nano)
// 6-Channel IR Sensor Array + TB6612FNG Driver
// ============================================================

// 1. TB6612FNG Motor Driver
#define PIN_PWMA        3    // Left Motor PWM (D3)
#define PIN_AIN2        4    // Left Motor IN2 (D4)
#define PIN_AIN1        5    // Left Motor IN1 (D5)
#define PIN_STBY        6    // Standby (D6)
#define PIN_BIN1        7    // Right Motor IN1 (D7)
#define PIN_BIN2        8    // Right Motor IN2 (D8)
#define PIN_PWMB        9    // Right Motor PWM (D9)

// 2. 6-Channel IR Sensor Array (Direct Analog Pins)
#define PIN_S1          A7   // S1 (A7)
#define PIN_S2          A6   // S2 (A6)
#define PIN_S3          A3   // S3 (A3)
#define PIN_S4          A2   // S4 (A2)
#define PIN_S5          A1   // S5 (A1)
#define PIN_S6          A0   // S6 (A0)

#define NUM_SENSORS     6

// 3. OLED Display (I2C: SDA=A4, SCL=A5)
#define OLED_WIDTH      128
#define OLED_HEIGHT     64
#define OLED_RESET      -1
#define OLED_ADDR       0x3C

// 4. Three Push Buttons (Active LOW with INPUT_PULLUP)
#define PIN_BTN_UP      2    // UP (D2)
#define PIN_BTN_DOWN    10   // DOWN (D10)
#define PIN_BTN_ENTER   11   // OK / ENTER (D11)

// Aliases for compatibility
#define PIN_BTN_NEXT    PIN_BTN_DOWN
#define PIN_BTN_PREV    PIN_BTN_UP

// ============================================================
// Constants & Limits
// ============================================================
#define LINE_CENTER     7500L        // Position is always scaled to 0..15000, center 7500
#define SENSOR_STEP     (LINE_MAX_POS / (NUM_SENSORS - 1))  // 3000 per sensor for 6 ch
#define LINE_MAX_POS    15000L
#define SENSOR_THRESH   350          // Normalized threshold (0..1000) for digital bitmask
#define EEPROM_MAGIC    0x4B4C       // 'KL' signature
#define EEPROM_VERSION  9            // bumped: struct size changed (6 sensors)
#define MAX_PATH_LEN    48

// Button events
enum ButtonEvent {
  BTN_NONE = 0,
  BTN_UP_PRESS,
  BTN_DOWN_PRESS,
  BTN_ENTER_PRESS,
  BTN_ENTER_LONG,
  BTN_NEXT_PRESS = BTN_DOWN_PRESS,
  BTN_PREV_PRESS = BTN_UP_PRESS
};

// Robot high-level mode
enum RobotState {
  STATE_MENU = 0,
  STATE_RUNNING
};

// Turn memory direction
enum TurnDir {
  DIR_NONE = 0,
  DIR_LEFT = -1,
  DIR_RIGHT = 1
};

// Persistent configuration stored in EEPROM
struct Settings {
  uint16_t magic;
  uint8_t  version;

  // PID parameters (scaled integer: kp/1000, ki/10000, kd/1000)
  uint16_t kp;
  uint16_t ki;
  uint16_t kd;

  // Speeds (0..255)
  int16_t baseSpeed;
  int16_t maxSpeed;
  int16_t turnSpeed;
  int16_t boostSpeed;

  // Sensor & Track options
  uint8_t highIsBlack;       // 1 if black line gives higher ADC value, 0 if lower
  uint8_t manualInverse;     // 1 = start in white-line-on-black mode
  uint8_t autoInverseEn;     // 1 = enable AUTO_INVERSE
  uint8_t advPidEn;          // 1 = enable PID_ADVANCED (curve slow-down + adaptive D)
  uint8_t speedBoostEn;      // 1 = enable SPEED_BOOST on straights
  uint8_t junctionEn;        // 1 = enable ADVANCED_JUNCTION
  uint8_t circleEn;          // 1 = enable CIRCLE_HANDLER
  uint8_t blackAreaEn;       // 1 = enable BLACK_AREA finish/cross handler
  uint8_t safetyUTurnEn;     // 1 = enable SAFETY_UTURN recovery

  // Motor direction flags (flip to 1 if a motor spins backward)
  uint8_t invertLeftMotor;
  uint8_t invertRightMotor;

  // Competition tuning extras
  uint16_t blackStopMs;      // Finish box duration threshold (ms)
  uint8_t  circleDir;        // 0 = Auto (lastDir/Right), 1 = Always Right, 2 = Always Left
  uint8_t  reverseSensors;   // 1 = Flip C0..C11 order if C0 is on the right side

  // Path learning & Junction Priority
  uint8_t  juncPriority;     // 0=PATH, 1=STATE (S->L->R), 2=LEFT (L->S->R), 3=RIGHT (R->S->L)
  uint8_t  pathRunEn;        // 1 = use learned path at junctions
  uint8_t  pathLen;          // Length of recorded path
  char     path[MAX_PATH_LEN]; // e.g. "SLRRLSRE"

  // Calibration arrays
  uint16_t sensorMin[NUM_SENSORS];
  uint16_t sensorMax[NUM_SENSORS];
};

// ============================================================
// Shared Globals (defined in K_LFR.ino)
// ============================================================
extern Adafruit_SSD1306 display;
extern Settings cfg;
extern RobotState robotState;

extern uint16_t sensorRaw[NUM_SENSORS];
extern uint16_t sensorNorm[NUM_SENSORS];
extern uint16_t sensorBits;
extern uint8_t  activeCount;
extern int32_t  linePosition;
extern int16_t  lineError;
extern int16_t  lastError;
extern int8_t   lastDir;
extern bool     isInvertedLine;
extern bool     oledReady;
extern bool     runningPathMode;
extern uint8_t  currentPathStep;
extern uint32_t raceStartMs;

// ============================================================
// Function Prototypes across .ino modules
// ============================================================

// Motor.ino
void motorInit();
void setMotors(int16_t leftSpeed, int16_t rightSpeed);
void motorBrake();
void motorStop();

// Sensor.ino
void sensorInit();
void readRawSensors();
void readLineSensors();
void runCalibration(bool spinRobot);

// EEPROM.ino
void loadDefaults();
void loadEEPROM();
void saveEEPROM();

// Button_Oled.ino
void uiInit();
ButtonEvent readButtons();
void waitReleaseAll();
void showSplash();
void showStatusScreen(const __FlashStringHelper* title, const __FlashStringHelper* line2);

// Menu.ino
void menuReset();
void handleMenu();

// PID.ino
void resetPID();
int16_t computePID(int16_t error);

// PID_ADVANCED.ino
int16_t  computeAdvancedBaseSpeed(int16_t absError, int16_t currentBase);
uint16_t computeAdaptiveKp(int16_t absError);

// Inverse.ino
void applyInverse();

// AUTO_INVERSE.ino
void resetAutoInverse();
void updateAutoInverse();

// ADVANCED_JUNCTION.ino
void resetJunction();
bool handleAdvancedJunction();
bool pivotUntilLine(int8_t dir);

// BLACK_AREA.ino
void resetBlackArea();
bool handleBlackArea();

// CIRCLE_HANDLER.ino
void resetCircleHandler();
bool handleCircle();

// SAFETY_UTURN.ino
void resetSafetyUTurn();
bool handleSafetyUTurn();

// SPEED_BOOST.ino
void resetSpeedBoost();
int16_t updateSpeedBoost(int16_t error, int16_t baseSpd);

// Line_F.ino
void startLineFollower();
void runLineFollower();
void applySteeringMix(int16_t baseSpd, int16_t pidOut);

// Combine.ino (Path Memory & Hand Calibration)
void storePath(char dir);
void optimizePath();
void drawPathFormatted(int16_t startY, int8_t highlightIdx = -1);
void handleHandCalibrationAndPath();
void viewEditPathMenu();

#endif

