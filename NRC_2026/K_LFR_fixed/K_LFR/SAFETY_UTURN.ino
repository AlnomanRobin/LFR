#include "Config.h"

static uint32_t lostStartMs = 0;

void resetSafetyUTurn() {
  lostStartMs = 0;
}

static bool recoverLostLine(int8_t dir) {
  int16_t spd = cfg.turnSpeed;
  uint32_t startMs = millis();

  // Pivot in the direction of the lost line
  if (dir == DIR_LEFT) {
    setMotors(-spd, spd);
  } else {
    setMotors(spd, -spd);
  }

  // Must first spin until any residual line reading clears (if any)
  while (millis() - startMs < 100) {
    if (digitalRead(PIN_BTN_ENTER) == LOW && (millis() - raceStartMs > 800)) {
      motorStop();
      return true;
    }
    readLineSensors();
    if (activeCount == 0) break;
    delay(2);
  }

  // Now spin until center sensors (S3 or S4) cleanly lock onto the line
  bool found = false;
  uint32_t searchStart = millis();
  while (millis() - searchStart < 1200) {
    if (digitalRead(PIN_BTN_ENTER) == LOW && (millis() - raceStartMs > 800)) {
      motorStop();
      return true;
    }

    readLineSensors();

    // Lock onto line when center sensors (S3 or S4) see the line
    if ((sensorBits & 0x0CU) != 0 || (sensorNorm[2] >= 350 || sensorNorm[3] >= 350)) {
      found = true;
      break;
    }
    delay(2);
  }

  // If found, counter-pulse brake to stop dead-center on the line
  if (found) {
    if (dir == DIR_LEFT) {
      setMotors(spd / 2, -spd / 2);
    } else {
      setMotors(-spd / 2, spd / 2);
    }
    delay(25);
    motorStop();
    resetPID();
    lastDir   = DIR_NONE;
    lastError = 0;
    lostStartMs = 0;
    return true;
  }

  // If still not found in chosen direction, sweep opposite direction
  dir = (dir == DIR_LEFT) ? DIR_RIGHT : DIR_LEFT;
  if (dir == DIR_LEFT) {
    setMotors(-spd, spd);
  } else {
    setMotors(spd, -spd);
  }

  searchStart = millis();
  while (millis() - searchStart < 1800) {
    if (digitalRead(PIN_BTN_ENTER) == LOW && (millis() - raceStartMs > 800)) {
      motorStop();
      return true;
    }

    readLineSensors();
    if ((sensorBits & 0x0CU) != 0 || (sensorNorm[2] >= 350 || sensorNorm[3] >= 350)) {
      found = true;
      break;
    }
    delay(2);
  }

  if (found) {
    if (dir == DIR_LEFT) {
      setMotors(spd / 2, -spd / 2);
    } else {
      setMotors(-spd / 2, spd / 2);
    }
    delay(25);
    motorStop();
    resetPID();
    lastDir   = DIR_NONE;
    lastError = 0;
    lostStartMs = 0;
    return true;
  }

  // Safety stop if line could not be found anywhere
  motorBrake();
  delay(80);
  motorStop();
  showStatusScreen(F("LINE LOST!"), F("ENTER -> Menu"));
  delay(700);
  waitReleaseAll();
  robotState = STATE_MENU;
  menuReset();
  return true;
}

bool handleSafetyUTurn() {
  // If line is present (activeCount > 0 or any sensor >= 220), not lost
  if (activeCount > 0) {
    if (lostStartMs != 0) {
      lostStartMs = 0;
      resetPID();
    }
    return false;
  }

  // Quick check across all sensors for any signal above noise
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (sensorNorm[i] >= 220) {
      if (lostStartMs != 0) {
        lostStartMs = 0;
        resetPID();
      }
      return false;
    }
  }

  uint32_t now = millis();
  if (lostStartMs == 0) {
    lostStartMs = now;
  }

  uint32_t lostDur = now - lostStartMs;

  // If user disabled safety recovery in menu:
  if (!cfg.safetyUTurnEn) {
    if (lostDur >= 350) {
      motorBrake();
      delay(80);
      motorStop();
      lostStartMs = 0;
      showStatusScreen(F("LINE LOST!"), F("ENTER -> Menu"));
      delay(700);
      waitReleaseAll();
      robotState = STATE_MENU;
      menuReset();
      return true;
    }
    return false;
  }

  // Phase 1: If line was lost recently (< 80 ms), roll forward gently
  // to bridge small dashed gaps without immediately spinning
  if (lostDur < 80 && abs(lastError) < 1500) {
    int16_t gapSpd = (cfg.baseSpeed * 2) / 3;
    setMotors(gapSpd, gapSpd);
    return true;
  }

  // Phase 2: Line is genuinely lost — execute controlled recovery toward last known side!
  int8_t searchDir = (lastDir != DIR_NONE) ? lastDir : ((lastError < 0) ? DIR_LEFT : DIR_RIGHT);
  return recoverLostLine(searchDir);
}
