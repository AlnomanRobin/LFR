#include "Config.h"

// ============================================================
// Combine.ino - Hand Calibration Path Recording & Interactive Editor
// ============================================================

void storePath(char dir) {
  if (cfg.pathLen < MAX_PATH_LEN - 1) {
    cfg.path[cfg.pathLen++] = dir;
    cfg.path[cfg.pathLen]   = '\0';
  }
}

void optimizePath() {
  if (cfg.pathLen < 3) return;

  bool changed = false;
  uint8_t i = 0;

  while (i + 2 < cfg.pathLen) {
    if (cfg.path[i + 1] == 'B') {
      char before = cfg.path[i];
      char after  = cfg.path[i + 2];
      char res    = ' ';

      // Standard LSRB Maze Reduction Rules:
      if      (before == 'L' && after == 'L') res = 'S';
      else if (before == 'L' && after == 'S') res = 'R';
      else if (before == 'R' && after == 'L') res = 'S';
      else if (before == 'S' && after == 'L') res = 'R';
      else if (before == 'S' && after == 'S') res = 'B';
      else if (before == 'L' && after == 'R') res = 'B';
      else if (before == 'R' && after == 'R') res = 'S';
      else if (before == 'S' && after == 'R') res = 'L';
      else if (before == 'R' && after == 'S') res = 'L';

      if (res != ' ') {
        cfg.path[i] = res;
        for (uint8_t j = i + 1; j + 2 < cfg.pathLen; j++) {
          cfg.path[j] = cfg.path[j + 2];
        }
        cfg.pathLen -= 2;
        cfg.path[cfg.pathLen] = '\0';
        changed = true;
        if (i > 0) i--;
        continue;
      }
    }
    i++;
  }

  if (changed) {
    saveEEPROM();
  }
}

// Draw path on OLED with optional highlighted step [X]
void drawPathFormatted(int16_t startY, int8_t highlightIdx) {
  if (!oledReady) return;

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, startY);

  if (cfg.pathLen == 0) {
    display.print(F("(Move robot on line)"));
    return;
  }

  for (uint8_t i = 0; i < cfg.pathLen; i++) {
    if (i > 0) {
      if (display.getCursorX() + 26 > 124) {
        display.setCursor(0, display.getCursorY() + 9);
      }
      display.print(F("->"));
    }

    if (i == highlightIdx) {
      display.print('[');
      display.print(cfg.path[i]);
      display.print(']');
    } else {
      display.print(cfg.path[i]);
    }
  }
}

// Overload for general calls
void drawPathFormatted(int16_t startY) {
  drawPathFormatted(startY, -1);
}

// ============================================================
// Hand Calibration & Live Path Learning
// Calibrates sensors AND memorizes S -> L -> R -> R -> L -> S -> R -> E
// ============================================================
void handleHandCalibrationAndPath() {
  motorStop();
  waitReleaseAll();

  // Reset sensor calibration ranges
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    cfg.sensorMin[i] = 1023;
    cfg.sensorMax[i] = 0;
  }

  // Start with empty path; records real junctions encountered on the track
  cfg.path[0] = '\0';
  cfg.pathLen = 0;

  enum HandState {
    HC_TRACKING = 0,
    HC_IN_JUNCTION,
    HC_IN_FINISH
  } hcState = HC_TRACKING;

  uint32_t juncStartMs     = 0;
  uint32_t finishStartMs   = 0;
  uint32_t cooldownUntilMs = 0;
  bool     sawLeftBranch   = false;
  bool     sawRightBranch  = false;
  uint16_t leftDevCount    = 0;
  uint16_t rightDevCount   = 0;

  uint32_t lastDrawMs = 0;
  bool     needRedraw = true;

  while (true) {
    uint32_t now = millis();

    // 1. Read Raw Sensors & dynamically update Min/Max
    readRawSensors();
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      if (sensorRaw[i] < cfg.sensorMin[i]) cfg.sensorMin[i] = sensorRaw[i];
      if (sensorRaw[i] > cfg.sensorMax[i]) cfg.sensorMax[i] = sensorRaw[i];
    }

    // 2. Compute normalized values & error
    readLineSensors();

    // 3. Branch and Finish detection flags
    bool leftBranch  = (sensorNorm[0] >= 500 && (sensorNorm[1] >= 400 || activeCount >= 3));
    bool rightBranch = (sensorNorm[5] >= 500 && (sensorNorm[4] >= 400 || activeCount >= 3));
    bool centerLine  = (sensorNorm[2] >= 450 || sensorNorm[3] >= 450);
    bool allBlack    = (activeCount >= 5);

    // 4. Junction & Path State Machine
    if (now >= cooldownUntilMs) {
      switch (hcState) {
        case HC_TRACKING:
          if (allBlack) {
            finishStartMs = now;
            hcState       = HC_IN_FINISH;
          } else if (leftBranch || rightBranch) {
            juncStartMs    = now;
            sawLeftBranch  = leftBranch;
            sawRightBranch = rightBranch;
            leftDevCount   = 0;
            rightDevCount  = 0;
            hcState        = HC_IN_JUNCTION;
          }
          break;

        case HC_IN_JUNCTION:
          if (leftBranch)  sawLeftBranch  = true;
          if (rightBranch) sawRightBranch = true;

          if (lineError < -1200) leftDevCount++;
          if (lineError > 1200)  rightDevCount++;

          if (!leftBranch && !rightBranch && centerLine && (now - juncStartMs >= 110)) {
            char decision = 'S';
            if (leftDevCount >= 3 || (sawLeftBranch && !sawRightBranch && leftDevCount > rightDevCount)) {
              decision = 'L';
            } else if (rightDevCount >= 3 || (sawRightBranch && !sawLeftBranch && rightDevCount > leftDevCount)) {
              decision = 'R';
            } else {
              decision = 'S';
            }

            if (cfg.pathLen < MAX_PATH_LEN - 1) {
              cfg.path[cfg.pathLen++] = decision;
              cfg.path[cfg.pathLen]   = '\0';
              needRedraw = true;
            }

            hcState         = HC_TRACKING;
            cooldownUntilMs = now + 420;
          }
          break;

        case HC_IN_FINISH:
          if (allBlack) {
            if (now - finishStartMs >= 280) {
              if ((cfg.pathLen == 0 || cfg.path[cfg.pathLen - 1] != 'E') && cfg.pathLen < MAX_PATH_LEN - 1) {
                cfg.path[cfg.pathLen++] = 'E';
                cfg.path[cfg.pathLen]   = '\0';
                needRedraw = true;
              }
              hcState         = HC_TRACKING;
              cooldownUntilMs = now + 900;
            }
          } else {
            hcState = HC_TRACKING;
          }
          break;
      }
    }

    // 5. Push Button Controls:
    // DOWN (D10): Undo last path step
    if (digitalRead(PIN_BTN_DOWN) == LOW) {
      delay(40);
      if (digitalRead(PIN_BTN_DOWN) == LOW) {
        if (cfg.pathLen > 0) {
          cfg.pathLen--;
          cfg.path[cfg.pathLen] = '\0';
          needRedraw = true;
        }
        waitReleaseAll();
      }
    }

    // UP (D2): Manually add a step (cycles S -> L -> R)
    if (digitalRead(PIN_BTN_UP) == LOW) {
      delay(40);
      if (digitalRead(PIN_BTN_UP) == LOW) {
        if (cfg.pathLen < MAX_PATH_LEN - 1) {
          char last = (cfg.pathLen > 0) ? cfg.path[cfg.pathLen - 1] : 'R';
          char nextCh = (last == 'S') ? 'L' : ((last == 'L') ? 'R' : 'S');
          cfg.path[cfg.pathLen++] = nextCh;
          cfg.path[cfg.pathLen]   = '\0';
          needRedraw = true;
        }
        waitReleaseAll();
      }
    }

    // ENTER (A3): Save Calibration & Path and Exit
    if (digitalRead(PIN_BTN_ENTER) == LOW) {
      delay(50);
      if (digitalRead(PIN_BTN_ENTER) == LOW) {
        waitReleaseAll();

        // Ensure path ends with 'E' if not empty
        if (cfg.pathLen > 0 && cfg.path[cfg.pathLen - 1] != 'E' && cfg.pathLen < MAX_PATH_LEN - 1) {
          cfg.path[cfg.pathLen++] = 'E';
          cfg.path[cfg.pathLen]   = '\0';
        } else if (cfg.pathLen == 0) {
          strncpy(cfg.path, "LRRLSRE", MAX_PATH_LEN - 1);
          cfg.pathLen = strlen(cfg.path);
        }

        // Validate sensor min/max
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
          if (cfg.sensorMax[i] <= cfg.sensorMin[i] + 30) {
            cfg.sensorMin[i] = 150;
            cfg.sensorMax[i] = 850;
          }
        }

        // Set priority to PATH so RUN LFR executes this learned path
        cfg.juncPriority = 0;
        cfg.pathRunEn    = 1;
        saveEEPROM();

        if (oledReady) {
          display.clearDisplay();
          display.setTextSize(1);
          display.setTextColor(SSD1306_WHITE);
          display.setCursor(0, 0);
          display.print(F("PATH & CALIB SAVED!"));
          display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

          drawPathFormatted(14, -1);

          display.setCursor(0, 54);
          display.print(F("Press OK -> RUN LFR"));
          display.display();
        }

        while (digitalRead(PIN_BTN_ENTER) == HIGH) {
          delay(15);
        }
        waitReleaseAll();
        return;
      }
    }

    // 6. Live OLED Screen Refresh
    if (oledReady && (needRedraw || (now - lastDrawMs >= 80))) {
      needRedraw = false;
      lastDrawMs = now;

      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);

      display.setCursor(0, 0);
      display.print(F("CALIB & MAP ["));
      display.print(cfg.pathLen);
      display.print(F("]"));

      display.setCursor(84, 0);
      display.print(F("C:"));
      display.print(activeCount);

      display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

      drawPathFormatted(13, -1);

      display.setCursor(0, 54);
      display.print(F("OK:Save UP:+ DN:Undo"));
      display.display();
    }

    delay(2);
  }
}

// ============================================================
// Interactive Path Editor (VIEW / EDIT PATH)
// Press ENTER to select/edit a step, UP/DOWN to change direction,
// then OK to save. RUN LFR will run according to this path.
// ============================================================
void viewEditPathMenu() {
  waitReleaseAll();

  if (cfg.pathLen == 0) {
    cfg.path[0] = 'S';
    cfg.path[1] = 'E';
    cfg.path[2] = '\0';
    cfg.pathLen = 2;
  }

  uint8_t selStep  = 0;
  bool    editMode = false;
  bool    redraw   = true;

  while (true) {
    uint32_t now = millis();

    if (redraw && oledReady) {
      redraw = false;
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);

      if (!editMode) {
        // Navigation Screen
        display.setCursor(0, 0);
        display.print(F("EDIT PATH [len:"));
        display.print(cfg.pathLen);
        display.print(F("]"));
        display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

        display.setCursor(0, 12);
        display.print(F("Step "));
        display.print(selStep + 1);
        display.print(F("/"));
        display.print(cfg.pathLen);
        display.print(F(": > "));
        display.print(cfg.path[selStep]);
        display.print(F(" <"));

        drawPathFormatted(23, selStep);

        display.setCursor(0, 54);
        display.print(F("OK:Edit UP/DN:Step"));
      } else {
        // Step Editing Screen
        display.setCursor(0, 0);
        display.print(F("CHANGE STEP "));
        display.print(selStep + 1);
        display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

        display.setCursor(20, 20);
        display.print(F("DIRECTION: [ "));
        display.print(cfg.path[selStep]);
        display.print(F(" ]"));

        display.setCursor(14, 36);
        display.print(F("(S, L, R, E)"));

        display.setCursor(0, 54);
        display.print(F("OK:Set UP/DN:Change"));
      }
      display.display();
    }

    // Button Handling:
    // UP (D2 - Move Up / Previous step / Change):
    if (digitalRead(PIN_BTN_UP) == LOW) {
      delay(40);
      if (digitalRead(PIN_BTN_UP) == LOW) {
        if (!editMode) {
          selStep = (selStep == 0) ? (cfg.pathLen - 1) : (selStep - 1);
        } else {
          char c = cfg.path[selStep];
          if      (c == 'S') cfg.path[selStep] = 'L';
          else if (c == 'L') cfg.path[selStep] = 'R';
          else if (c == 'R') cfg.path[selStep] = 'E';
          else               cfg.path[selStep] = 'S';
        }
        redraw = true;
        waitReleaseAll();
      }
    }

    // DOWN (D10 - Move Down / Next step / Change):
    if (digitalRead(PIN_BTN_DOWN) == LOW) {
      delay(40);
      if (digitalRead(PIN_BTN_DOWN) == LOW) {
        if (!editMode) {
          selStep = (selStep + 1) % cfg.pathLen;
        } else {
          char c = cfg.path[selStep];
          if      (c == 'S') cfg.path[selStep] = 'E';
          else if (c == 'E') cfg.path[selStep] = 'R';
          else if (c == 'R') cfg.path[selStep] = 'L';
          else               cfg.path[selStep] = 'S';
        }
        redraw = true;
        waitReleaseAll();
      }
    }

    // A1 (ENTER / OK):
    if (digitalRead(PIN_BTN_ENTER) == LOW) {
      uint32_t pressStart = millis();
      while (digitalRead(PIN_BTN_ENTER) == LOW && (millis() - pressStart < 600)) {
        delay(10);
      }

      bool isLongPress = (millis() - pressStart >= 600);

      if (editMode) {
        // Confirm modification of current step
        editMode         = false;
        cfg.juncPriority = 0; // Set to PATH mode
        cfg.pathRunEn    = 1;
        saveEEPROM();
        redraw = true;
        waitReleaseAll();
      } else {
        if (isLongPress) {
          // Long press ENTER: Save path and exit back to main menu
          cfg.juncPriority = 0; // PATH mode active for RUN LFR
          cfg.pathRunEn    = 1;
          saveEEPROM();
          showStatusScreen(F("PATH SAVED!"), F("Select RUN LFR to run"));
          delay(800);
          waitReleaseAll();
          return;
        } else {
          // Short press ENTER: Enter editing mode for selected step
          editMode = true;
          redraw   = true;
          waitReleaseAll();
        }
      }
    }

    delay(10);
  }
}
