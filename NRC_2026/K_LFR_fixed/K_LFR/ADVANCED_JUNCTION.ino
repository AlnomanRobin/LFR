#include "Config.h"

static uint32_t lastJunctionMs = 0;

void resetJunction() {
  lastJunctionMs = 0;
}

bool pivotUntilLine(int8_t dir) {
  uint32_t startMs = millis();
  int16_t spd = cfg.turnSpeed;

  // Step 1: Brief roll forward so wheels align over intersection axis
  setMotors(cfg.baseSpeed / 2, cfg.baseSpeed / 2);
  delay(35);

  // Step 2: Sharp pivot in requested direction
  if (dir == DIR_LEFT) {
    setMotors(-spd, spd);
  } else {
    setMotors(spd, -spd);
  }

  // Step 3: Must spin until center sensors leave the old line
  // (Prevents false immediate reacquisition of the starting line)
  while (millis() - startMs < 250) {
    if (digitalRead(PIN_BTN_ENTER) == LOW && (millis() - raceStartMs > 800)) {
      delay(30);
      if (digitalRead(PIN_BTN_ENTER) == LOW) {
        motorStop();
        return true;
      }
    }
    readLineSensors();
    if ((sensorBits & 0x0CU) == 0) {
      break; // Center sensors completely cleared the old line
    }
    delay(3);
  }

  // Step 4: Continue spinning until center sensors lock onto the new branch
  bool reacquired = false;
  uint32_t searchStart = millis();
  while (millis() - searchStart < 700) {
    if (digitalRead(PIN_BTN_ENTER) == LOW && (millis() - raceStartMs > 800)) {
      delay(30);
      if (digitalRead(PIN_BTN_ENTER) == LOW) {
        motorBrake();
        delay(60);
        motorStop();
        robotState = STATE_MENU;
        waitReleaseAll();
        menuReset();
        return true;
      }
    }

    readLineSensors();
    // Center sensors (S3 or S4) lock onto the new branch
    if ((sensorBits & 0x0CU) != 0 && activeCount <= 3) {
      reacquired = true;
      break;
    }
    delay(2);
  }

  // Step 5: Counter-pulse brake cancels rotational inertia cleanly on the line
  if (reacquired) {
    if (dir == DIR_LEFT) {
      setMotors(spd / 2, -spd / 2);
    } else {
      setMotors(-spd / 2, spd / 2);
    }
    delay(20);
  }
  motorStop();

  lastJunctionMs = millis();
  resetPID();
  return true;
}

bool handleAdvancedJunction() {
  if (!cfg.junctionEn && cfg.juncPriority != 0) {
    return false;
  }

  // Prevent repeated detection of the same junction
  if (millis() - lastJunctionMs < 240) {
    return false;
  }

  // 6 sensors partition:
  // Outer left: S1 (0), inner left: S2 (1)
  // Center: S3 (2), S4 (3)
  // Inner right: S5 (4), outer right: S6 (5)
  // A true 90-deg branch or turn MUST reach the outer sensor (0 on left, 5 on right)
  bool leftBranch  = (sensorNorm[0] >= 500 && (sensorNorm[1] >= 400 || activeCount >= 3));
  bool rightBranch = (sensorNorm[5] >= 500 && (sensorNorm[4] >= 400 || activeCount >= 3));

  // If no branch is seen, let normal line PID run smoothly
  if (!leftBranch && !rightBranch) {
    return false;
  }

  // Advance slightly to verify if a straight path exists through the junction
  setMotors(cfg.baseSpeed / 2, cfg.baseSpeed / 2);
  delay(30);
  readLineSensors();

  // Straight line continuation exists if center sensors (S3 or S4) detect line
  bool straightAvail = ((sensorBits & 0x0CU) != 0) || (abs(lineError) < 1200);

  // Determine action according to configured priority:
  // 0 = PATH (Custom/Learned Path sequence)
  // 1 = STATE RUN (Straight -> Left -> Right)
  // 2 = LEFT RUN  (Left -> Straight -> Right)
  // 3 = RIGHT RUN (Right -> Straight -> Left)
  char act = 'S';

  if (cfg.juncPriority == 0) {
    // Mode A: Path Run (Custom / Hand Calibrated sequence)
    if (currentPathStep < cfg.pathLen) {
      act = cfg.path[currentPathStep++];

      // Intelligent Fallback for Missing Paths:
      if (act == 'L' && !leftBranch) {
        // Commanded 'L' but Left is missing: take Straight if available, else Right
        act = straightAvail ? 'S' : (rightBranch ? 'R' : 'S');
      } else if (act == 'R' && !rightBranch) {
        // Commanded 'R' but Right is missing: take Straight if available, else Left
        act = straightAvail ? 'S' : (leftBranch ? 'L' : 'S');
      } else if (act == 'S' && !straightAvail) {
        // Commanded 'S' but Straight is missing (e.g. T-junction): take Left else Right
        act = leftBranch ? 'L' : (rightBranch ? 'R' : 'L');
      }
    } else {
      act = straightAvail ? 'S' : (leftBranch ? 'L' : 'R');
    }
  } else if (cfg.juncPriority == 1) {
    // Priority A: State Run (Straight -> Left -> Right)
    // The robot must choose the straight path whenever available.
    if (straightAvail)      act = 'S';
    else if (leftBranch)    act = 'L';
    else if (rightBranch)   act = 'R';
    else                    act = 'S';
  } else if (cfg.juncPriority == 2) {
    // Priority B: Left Run (Left -> Straight -> Right)
    if (leftBranch)         act = 'L';
    else if (straightAvail) act = 'S';
    else if (rightBranch)   act = 'R';
    else                    act = 'L';
  } else if (cfg.juncPriority == 3) {
    // Priority C: Right Run (Right -> Straight -> Left)
    if (rightBranch)        act = 'R';
    else if (straightAvail) act = 'S';
    else if (leftBranch)    act = 'L';
    else                    act = 'R';
  }

  // Execute the selected action
  lastJunctionMs = millis();

  if (act == 'S') {
    // Continue straight across the junction line
    setMotors(cfg.baseSpeed, cfg.baseSpeed);
    delay(40);
    return false; // Resume PID tracking forward
  } else if (act == 'L') {
    lastDir = DIR_LEFT;
    return pivotUntilLine(DIR_LEFT);
  } else if (act == 'R') {
    lastDir = DIR_RIGHT;
    return pivotUntilLine(DIR_RIGHT);
  } else if (act == 'E') {
    if (millis() - raceStartMs < 2000) {
      // Ignore finish during initial 2s race launch
      setMotors(cfg.baseSpeed, cfg.baseSpeed);
      delay(40);
      return false;
    }
    motorBrake();
    delay(100);
    motorStop();
    showStatusScreen(F("GOAL REACHED!"), F("ENTER -> Menu"));
    waitReleaseAll();
    while (readButtons() != BTN_ENTER_PRESS) {
      delay(10);
    }
    waitReleaseAll();
    robotState = STATE_MENU;
    menuReset();
    return true;
  }

  return false;
}
