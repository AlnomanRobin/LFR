#include "Config.h"

// ============================================================
// Global Objects & State
// ============================================================
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET);
Settings cfg;
RobotState robotState = STATE_MENU;

uint16_t sensorRaw[NUM_SENSORS];
uint16_t sensorNorm[NUM_SENSORS];
uint16_t sensorBits   = 0;
uint8_t  activeCount  = 0;
int32_t  linePosition = LINE_CENTER;
int16_t  lineError    = 0;
int16_t  lastError    = 0;
int8_t   lastDir      = DIR_NONE;
bool     isInvertedLine  = false;
bool     oledReady       = false;
bool     runningPathMode = false;
uint8_t  currentPathStep = 0;
uint32_t raceStartMs     = 0;

void setup() {
  motorInit();
  motorStop();

  sensorInit();
  loadEEPROM();
  uiInit();

  isInvertedLine = (cfg.manualInverse != 0);
  menuReset();
}

void loop() {
  if (robotState == STATE_MENU) {
    handleMenu();
  } else {
    runLineFollower();
  }
}

