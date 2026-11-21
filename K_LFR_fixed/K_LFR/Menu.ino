#include "Config.h"

enum MenuScreen {
  SCR_MAIN = 0,
  SCR_SENSORS,
  SCR_PID,
  SCR_MODES,
  SCR_MOTOR_TEST
};

static MenuScreen curScreen = SCR_MAIN;
static uint8_t    cursor    = 0;
static bool       editing   = false;
static bool       dirtyDraw = true;
static uint32_t   lastSensorDrawMs = 0;
static uint8_t    sensorViewMode   = 0; // 0=NRM bars, 1=RAW bars, 2=NUM values
static bool       motorRunningTest = false;

#define MAIN_COUNT       12
#define PID_COUNT        9
#define MODE_COUNT       15
#define MOTOR_TEST_COUNT 8

void menuReset() {
  curScreen = SCR_MAIN;
  cursor    = 0;
  editing   = false;
  dirtyDraw = true;
}

static const __FlashStringHelper* mainLabel(uint8_t idx) {
  switch (idx) {
    case 0:  return F("1. RUN LFR (PATH)");
    case 1:  return F("2. STATE RUN (SLR)");
    case 2:  return F("3. LEFT RUN (LSR)");
    case 3:  return F("4. RIGHT RUN (RSL)");
    case 4:  return F("5. CALIBRATE(AUTO)");
    case 5:  return F("6. CALIBRATE(HAND)");
    case 6:  return F("7. VIEW/EDIT PATH");
    case 7:  return F("8. IR SENSOR TEST");
    case 8:  return F("9. MOTOR TEST");
    case 9:  return F("10. PID & SPEEDS");
    case 10: return F("11. MODULES/MODES");
    case 11: return F("12. RESET DEFAULT");
    default: return F("");
  }
}

static const __FlashStringHelper* motorTestLabel(uint8_t idx) {
  switch (idx) {
    case 0: return F("1. AUTO TEST ALL");
    case 1: return F("2. LEFT FORWARD");
    case 2: return F("3. LEFT REVERSE");
    case 3: return F("4. RIGHT FORWARD");
    case 4: return F("5. RIGHT REVERSE");
    case 5: return F("6. BOTH FORWARD");
    case 6: return F("7. BOTH REVERSE");
    case 7: return F("8. < BACK");
    default: return F("");
  }
}

static const __FlashStringHelper* pidLabel(uint8_t idx) {
  switch (idx) {
    case 0: return F("1. Kp");
    case 1: return F("2. Ki");
    case 2: return F("3. Kd");
    case 3: return F("4. BaseSpd");
    case 4: return F("5. MaxSpd");
    case 5: return F("6. TurnSpd");
    case 6: return F("7. BoostSpd");
    case 7: return F("8. BlkStopMs");
    case 8: return F("9. < SAVE/BACK");
    default: return F("");
  }
}

static const __FlashStringHelper* modeLabel(uint8_t idx) {
  switch (idx) {
    case 0:  return F("1. Auto Inv");
    case 1:  return F("2. Man Inv");
    case 2:  return F("3. Adv PID");
    case 3:  return F("4. Spd Boost");
    case 4:  return F("5. Adv Junc");
    case 5:  return F("6. Circle");
    case 6:  return F("7. BlackArea");
    case 7:  return F("8. U-Turn");
    case 8:  return F("9. High=Blk");
    case 9:  return F("10. Inv L Mtr");
    case 10: return F("11. Inv R Mtr");
    case 11: return F("12. Rev Sens");
    case 12: return F("13. Junc Rule");
    case 13: return F("14. Circle Dir");
    case 14: return F("15. < BACK");
    default: return F("");
  }
}

static uint8_t* getModePtr(uint8_t idx) {
  switch (idx) {
    case 0:  return &cfg.autoInverseEn;
    case 1:  return &cfg.manualInverse;
    case 2:  return &cfg.advPidEn;
    case 3:  return &cfg.speedBoostEn;
    case 4:  return &cfg.junctionEn;
    case 5:  return &cfg.circleEn;
    case 6:  return &cfg.blackAreaEn;
    case 7:  return &cfg.safetyUTurnEn;
    case 8:  return &cfg.highIsBlack;
    case 9:  return &cfg.invertLeftMotor;
    case 10: return &cfg.invertRightMotor;
    case 11: return &cfg.reverseSensors;
    case 12: return &cfg.pathRunEn;
    default: return nullptr;
  }
}

static void printFixed(uint16_t val, uint16_t div) {
  display.print(val / div);
  display.print('.');
  uint16_t rem = val % div;
  if (div == 10000 && rem < 1000) display.print('0');
  if (div >= 1000  && rem < 100)  display.print('0');
  if (rem < 10) display.print('0');
  display.print(rem);
}

static void adjustPidParam(uint8_t idx, int8_t dir) {
  switch (idx) {
    case 0: cfg.kp = (uint16_t)constrain((int16_t)cfg.kp + dir * 5, 0, 1000); break;
    case 1: cfg.ki = (uint16_t)constrain((int16_t)cfg.ki + dir * 5, 0, 1000); break;
    case 2: cfg.kd = (uint16_t)constrain((int16_t)cfg.kd + dir * 20, 0, 5000); break;
    case 3: cfg.baseSpeed   = constrain(cfg.baseSpeed   + dir * 5, 40, 255); break;
    case 4: cfg.maxSpeed    = constrain(cfg.maxSpeed    + dir * 5, 60, 255); break;
    case 5: cfg.turnSpeed   = constrain(cfg.turnSpeed   + dir * 5, 40, 255); break;
    case 6: cfg.boostSpeed  = constrain(cfg.boostSpeed  + dir * 5, 60, 255); break;
    case 7: cfg.blackStopMs = (uint16_t)constrain((int16_t)cfg.blackStopMs + dir * 20, 80, 400); break;
  }
}

static void startMenuFrame(const __FlashStringHelper* title) {
  if (!oledReady) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(title);
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
}

static void drawMainMenu() {
  startMenuFrame(F("DIU LFR  [6CH]"));

  uint8_t startIdx = (cursor >= 5) ? (cursor - 4) : 0;
  for (uint8_t row = 0; row < 5; row++) {
    uint8_t idx = startIdx + row;
    if (idx >= MAIN_COUNT) break;
    uint8_t y = 14 + row * 10;
    display.setCursor(0, y);
    display.print((idx == cursor) ? F("> ") : F("  "));
    display.print(mainLabel(idx));
  }
  display.display();
}

static void drawPidMenu() {
  startMenuFrame(editing ? F("EDIT VALUE (UP/DN)") : F("PID & SPEEDS"));

  uint8_t startIdx = (cursor >= 5) ? (cursor - 4) : 0;
  for (uint8_t row = 0; row < 5; row++) {
    uint8_t idx = startIdx + row;
    if (idx >= PID_COUNT) break;
    uint8_t y = 14 + row * 10;
    display.setCursor(0, y);
    display.print((idx == cursor) ? (editing ? F("* ") : F("> ")) : F("  "));
    display.print(pidLabel(idx));

    if (idx < 8) {
      display.setCursor(78, y);
      switch (idx) {
        case 0: printFixed(cfg.kp, 1000);  break;
        case 1: printFixed(cfg.ki, 10000); break;
        case 2: printFixed(cfg.kd, 1000);  break;
        case 3: display.print(cfg.baseSpeed);   break;
        case 4: display.print(cfg.maxSpeed);    break;
        case 5: display.print(cfg.turnSpeed);   break;
        case 6: display.print(cfg.boostSpeed);  break;
        case 7: display.print(cfg.blackStopMs); break;
      }
    }
  }
  display.display();
}

static void drawModesMenu() {
  startMenuFrame(F("MODULES & MODES"));

  uint8_t startIdx = (cursor >= 5) ? (cursor - 4) : 0;
  for (uint8_t row = 0; row < 5; row++) {
    uint8_t idx = startIdx + row;
    if (idx >= MODE_COUNT) break;
    uint8_t y = 14 + row * 10;
    display.setCursor(0, y);
    display.print((idx == cursor) ? F("> ") : F("  "));
    display.print(modeLabel(idx));

    if (idx == 12) {
      display.setCursor(96, y);
      if (cfg.juncPriority == 0) display.print(F("PATH"));
      else if (cfg.juncPriority == 1) display.print(F("STAT"));
      else if (cfg.juncPriority == 2) display.print(F("LEFT"));
      else display.print(F("RGHT"));
    } else if (idx == 13) {
      display.setCursor(100, y);
      if (cfg.circleDir == 0) display.print(F("AUTO"));
      else if (cfg.circleDir == 1) display.print(F("RGT"));
      else display.print(F("LFT"));
    } else {
      uint8_t* ptr = getModePtr(idx);
      if (ptr != nullptr) {
        display.setCursor(104, y);
        display.print((*ptr) ? F("ON") : F("OFF"));
      }
    }
  }
  display.display();
}

static void drawSensorView() {
  isInvertedLine = (cfg.manualInverse != 0);
  readLineSensors();

  if (!oledReady) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  if (sensorViewMode == 0) display.print(F("NRM"));
  else if (sensorViewMode == 1) display.print(F("RAW"));
  else display.print(F("NUM"));

  display.print(F(" E:"));
  display.print(lineError);
  display.setCursor(86, 0);
  display.print(cfg.highIsBlack ? F("B:HI") : F("B:LO"));
  display.print(F(" C"));
  display.print(activeCount);

  if (sensorViewMode == 2) {
    // Numeric raw ADC readout for all 6 channels (S1..S6)
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      uint8_t col = i % 3;
      uint8_t row = i / 3;
      uint8_t x = col * 42;
      uint8_t y = 14 + row * 16;
      display.setCursor(x, y);
      display.print('S');
      display.print(i + 1);
      display.print(':');
      display.print(sensorRaw[i] / 10);
    }
  } else {
    // 6 vertical bar graphs across screen width
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      uint16_t val  = (sensorViewMode == 1) ? sensorRaw[i] : sensorNorm[i];
      uint16_t maxV = (sensorViewMode == 1) ? 1023 : 1000;
      uint8_t h = (uint8_t)((uint32_t)val * 36UL / maxV);
      uint8_t x = i * 20 + 7;
      display.drawRect(x, 14, 14, 36, SSD1306_WHITE);
      if (h > 0) {
        display.fillRect(x + 1, 14 + (36 - h), 12, h, SSD1306_WHITE);
      }
      if (sensorBits & (1U << i)) {
        display.fillRect(x + 5, 52, 4, 3, SSD1306_WHITE);
      }
    }
  }

  display.setCursor(0, 56);
  display.print(F("UP/DN:View  OK:Back"));
  display.display();
}

static void applySingleMotorTest(uint8_t idx) {
  switch (idx) {
    case 1: setMotors(135, 0);    break; // LEFT FWD
    case 2: setMotors(-135, 0);   break; // LEFT REV
    case 3: setMotors(0, 135);    break; // RIGHT FWD
    case 4: setMotors(0, -135);   break; // RIGHT REV
    case 5: setMotors(135, 135);  break; // BOTH FWD
    case 6: setMotors(-135, -135);break; // BOTH REV
    default: motorStop();         break;
  }
}

static bool testDelayAbortable(uint16_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (digitalRead(PIN_BTN_ENTER) == LOW) {
      motorStop();
      waitReleaseAll();
      return true;
    }
    delay(10);
  }
  return false;
}

static void runAutoMotorTest() {
  static const int16_t steps[6][2] = {
    { 135,    0},
    {-135,    0},
    {   0,  135},
    {   0, -135},
    { 135,  135},
    {-135, -135}
  };

  for (uint8_t s = 0; s < 6; s++) {
    showStatusScreen(F("AUTO MOTOR TEST"), motorTestLabel(s + 1));
    setMotors(steps[s][0], steps[s][1]);
    if (testDelayAbortable(750)) return;
    motorStop();
    if (testDelayAbortable(200)) return;
  }

  motorStop();
  showStatusScreen(F("MOTOR TEST DONE"), F("Check direction!"));
  delay(650);
}

static void drawMotorTestMenu() {
  startMenuFrame(motorRunningTest ? F("RUNNING (ENTER=STOP)") : F("MOTOR TEST"));

  uint8_t startIdx = (cursor >= 5) ? (cursor - 4) : 0;
  for (uint8_t row = 0; row < 5; row++) {
    uint8_t idx = startIdx + row;
    if (idx >= MOTOR_TEST_COUNT) break;
    uint8_t y = 14 + row * 10;
    display.setCursor(0, y);
    display.print((idx == cursor) ? (motorRunningTest ? F("* ") : F("> ")) : F("  "));
    display.print(motorTestLabel(idx));
  }
  display.display();
}

void handleMenu() {
  ButtonEvent ev = readButtons();

  if (curScreen == SCR_SENSORS) {
    if (ev == BTN_DOWN_PRESS) {
      sensorViewMode = (sensorViewMode + 1) % 3;
    } else if (ev == BTN_UP_PRESS) {
      sensorViewMode = (sensorViewMode == 0) ? 2 : (sensorViewMode - 1);
    } else if (ev == BTN_ENTER_PRESS || ev == BTN_ENTER_LONG) {
      curScreen = SCR_MAIN;
      cursor    = 7;
      dirtyDraw = true;
      return;
    }
    if (millis() - lastSensorDrawMs >= 60) {
      lastSensorDrawMs = millis();
      drawSensorView();
    }
    return;
  }

  if (curScreen == SCR_MOTOR_TEST) {
    if (ev == BTN_DOWN_PRESS) {
      if (motorRunningTest) {
        motorStop();
        motorRunningTest = false;
      }
      cursor = (cursor + 1) % MOTOR_TEST_COUNT;
      dirtyDraw = true;
    } else if (ev == BTN_UP_PRESS) {
      if (motorRunningTest) {
        motorStop();
        motorRunningTest = false;
      }
      cursor = (cursor == 0) ? (MOTOR_TEST_COUNT - 1) : (cursor - 1);
      dirtyDraw = true;
    } else if (ev == BTN_ENTER_PRESS || ev == BTN_ENTER_LONG) {
      dirtyDraw = true;
      if (ev == BTN_ENTER_LONG || cursor == MOTOR_TEST_COUNT - 1) {
        motorStop();
        motorRunningTest = false;
        curScreen = SCR_MAIN;
        cursor    = 8;
      } else if (cursor == 0) {
        motorRunningTest = false;
        waitReleaseAll();
        runAutoMotorTest();
      } else {
        motorRunningTest = !motorRunningTest;
        if (motorRunningTest) {
          applySingleMotorTest(cursor);
        } else {
          motorStop();
        }
      }
    }

    if (dirtyDraw) {
      dirtyDraw = false;
      drawMotorTestMenu();
    }
    return;
  }

  if (ev == BTN_UP_PRESS) {
    dirtyDraw = true;
    if (curScreen == SCR_MAIN) {
      cursor = (cursor == 0) ? (MAIN_COUNT - 1) : (cursor - 1);
    } else if (curScreen == SCR_PID) {
      if (editing) adjustPidParam(cursor, +1);
      else cursor = (cursor == 0) ? (PID_COUNT - 1) : (cursor - 1);
    } else if (curScreen == SCR_MODES) {
      cursor = (cursor == 0) ? (MODE_COUNT - 1) : (cursor - 1);
    }
  } else if (ev == BTN_DOWN_PRESS) {
    dirtyDraw = true;
    if (curScreen == SCR_MAIN) {
      cursor = (cursor + 1) % MAIN_COUNT;
    } else if (curScreen == SCR_PID) {
      if (editing) adjustPidParam(cursor, -1);
      else cursor = (cursor + 1) % PID_COUNT;
    } else if (curScreen == SCR_MODES) {
      cursor = (cursor + 1) % MODE_COUNT;
    }
  } else if (ev == BTN_ENTER_PRESS || ev == BTN_ENTER_LONG) {
    dirtyDraw = true;
    if (curScreen == SCR_MAIN) {
      switch (cursor) {
        case 0:
          cfg.juncPriority = 0;
          startLineFollower();
          return;
        case 1:
          cfg.juncPriority = 1;
          startLineFollower();
          return;
        case 2:
          cfg.juncPriority = 2;
          startLineFollower();
          return;
        case 3:
          cfg.juncPriority = 3;
          startLineFollower();
          return;
        case 4:  runCalibration(true);  break;
        case 5:  handleHandCalibrationAndPath(); break;
        case 6:  viewEditPathMenu(); break;
        case 7:  curScreen = SCR_SENSORS; sensorViewMode = 0; break;
        case 8:  curScreen = SCR_MOTOR_TEST; cursor = 0; motorRunningTest = false; break;
        case 9:  curScreen = SCR_PID;   cursor = 0; editing = false; break;
        case 10: curScreen = SCR_MODES; cursor = 0; break;
        case 11:
          loadDefaults();
          saveEEPROM();
          showStatusScreen(F("FACTORY RESET"), F("Defaults Restored"));
          delay(800);
          break;
      }
    } else if (curScreen == SCR_PID) {
      if (ev == BTN_ENTER_LONG || cursor == PID_COUNT - 1) {
        saveEEPROM();
        editing   = false;
        curScreen = SCR_MAIN;
        cursor    = 9;
      } else {
        editing = !editing;
      }
    } else if (curScreen == SCR_MODES) {
      if (ev == BTN_ENTER_LONG || cursor == MODE_COUNT - 1) {
        saveEEPROM();
        isInvertedLine = (cfg.manualInverse != 0);
        curScreen = SCR_MAIN;
        cursor    = 10;
      } else if (cursor == 12) {
        cfg.juncPriority = (cfg.juncPriority + 1) % 4;
      } else if (cursor == 13) {
        cfg.circleDir = (cfg.circleDir + 1) % 3;
      } else {
        uint8_t* ptr = getModePtr(cursor);
        if (ptr != nullptr) {
          *ptr = (*ptr) ? 0 : 1;
        }
      }
    }
  }

  if (dirtyDraw) {
    dirtyDraw = false;
    if (curScreen == SCR_MAIN) {
      drawMainMenu();
    } else if (curScreen == SCR_PID) {
      drawPidMenu();
    } else if (curScreen == SCR_MODES) {
      drawModesMenu();
    } else if (curScreen == SCR_MOTOR_TEST) {
      drawMotorTestMenu();
    }
  }
}

