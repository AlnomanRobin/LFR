#include "Config.h"

static uint32_t lastLoopUs = 0;
#define PID_PERIOD_US  1000UL   // Deterministic 1 kHz control loop

void applySteeringMix(int16_t baseSpd, int16_t pidOut) {
  int16_t leftSpeed  = baseSpd + pidOut;
  int16_t rightSpeed = baseSpd - pidOut;
  int16_t maxSpd     = cfg.maxSpeed;

  // Steering-priority saturation: shift outer-wheel overflow to inner wheel
  // so 100% of PID turn authority is preserved on fast sharp curves
  if (leftSpeed > maxSpd) {
    rightSpeed -= (leftSpeed - maxSpd);
    leftSpeed   = maxSpd;
  } else if (rightSpeed > maxSpd) {
    leftSpeed  -= (rightSpeed - maxSpd);
    rightSpeed  = maxSpd;
  }

  leftSpeed  = constrain(leftSpeed, -maxSpd, maxSpd);
  rightSpeed = constrain(rightSpeed, -maxSpd, maxSpd);

  setMotors(leftSpeed, rightSpeed);
}

void startLineFollower() {
  currentPathStep = 0;
  resetPID();
  resetAutoInverse();
  resetJunction();
  resetBlackArea();
  resetCircleHandler();
  resetSafetyUTurn();
  resetSpeedBoost();

  lastDir = DIR_NONE;

  const __FlashStringHelper* title = F("RUN LFR (PATH)");
  if (cfg.juncPriority == 1)      title = F("STATE RUN");
  else if (cfg.juncPriority == 2) title = F("LEFT RUN");
  else if (cfg.juncPriority == 3) title = F("RIGHT RUN");

  showStatusScreen(title, F("ENTER = Cancel"));
  waitReleaseAll();

  uint32_t t0 = millis();
  while (millis() - t0 < 600) {
    if (digitalRead(PIN_BTN_ENTER) == LOW) {
      waitReleaseAll();
      robotState = STATE_MENU;
      menuReset();
      return;
    }
    delay(10);
  }

  showStatusScreen(title, F("ENTER = Stop"));
  raceStartMs = millis();
  lastLoopUs  = micros();
  robotState  = STATE_RUNNING;
}

void runLineFollower() {
  // Debounced emergency stop via ENTER button (must hold for 60ms, ignoring first 600ms grace period)
  static uint32_t btnStopHoldMs = 0;
  if (digitalRead(PIN_BTN_ENTER) == LOW && (millis() - raceStartMs > 600)) {
    if (btnStopHoldMs == 0) {
      btnStopHoldMs = millis();
    } else if (millis() - btnStopHoldMs >= 60) {
      motorBrake();
      delay(80);
      motorStop();
      runningPathMode = false;
      showStatusScreen(F("STOPPED BY USER"), F("ENTER -> Menu"));
      delay(700);
      waitReleaseAll();
      robotState = STATE_MENU;
      menuReset();
      btnStopHoldMs = 0;
      return;
    }
  } else {
    btnStopHoldMs = 0;
  }

  // Enforce deterministic 1 kHz loop period for stable Kd
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastLoopUs) < PID_PERIOD_US) {
    return;
  }
  lastLoopUs = nowUs;

  // 1. Read all 6 analog sensors (includes manual/active Inverse)
  readLineSensors();

  // 2. Auto-detect Black<->White track polarity transitions
  updateAutoInverse();

  // 3. Line lost / dashed gap / U-turn / off-track safety stop
  if (handleSafetyUTurn()) {
    return;
  }

  // 4. Sharp 90-degree corners, T-junctions, cross-junctions & Path Follower
  if (handleAdvancedJunction()) {
    return;
  }

  // 5. Sustained Black Finish Box
  if (handleBlackArea()) {
    return;
  }

  // 6. Y-fork / Circle / Roundabout split handling
  if (handleCircle()) {
    return;
  }

  // 7. Standard + Advanced PID with straightaway Speed Boost & steering priority
  int16_t pidOut  = computePID(lineError);
  int16_t dynBase = computeAdvancedBaseSpeed(abs(lineError), cfg.baseSpeed);
  dynBase         = updateSpeedBoost(lineError, dynBase);

  applySteeringMix(dynBase, pidOut);
}
