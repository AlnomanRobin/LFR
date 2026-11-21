#include "Config.h"

static uint32_t blackStartMs    = 0;
static uint32_t blackLastSeenMs = 0;

void resetBlackArea() {
  blackStartMs    = 0;
  blackLastSeenMs = 0;
}

bool handleBlackArea() {
  if (!cfg.blackAreaEn) {
    return false;
  }

  uint32_t now = millis();

  // Grace period: NEVER trigger finish box in the first 2.0 seconds of running
  // (Prevents false finish stop on start line, start box, or initial launch)
  if (now - raceStartMs < 2000) {
    blackStartMs = 0;
    return false;
  }

  // All or nearly all 6 sensors active (cross-line or finish box)
  if (activeCount >= 5) {
    if (blackStartMs == 0) {
      blackStartMs = now;
    }
    blackLastSeenMs = now;

    // Drive straight across the black patch while timing its duration
    setMotors(cfg.baseSpeed, cfg.baseSpeed);

    // Sustained black box (>= blackStopMs) = Finish Box -> stop cleanly
    if (now - blackStartMs >= cfg.blackStopMs) {
      motorBrake();
      delay(120);
      motorStop();
      showStatusScreen(F("FINISH DETECTED"), F("ENTER -> Menu"));
      waitReleaseAll();
      while (readButtons() != BTN_ENTER_PRESS) {
        delay(10);
      }
      waitReleaseAll();
      robotState = STATE_MENU;
      menuReset();
      resetBlackArea();
    }
    return true;
  }

  // Allow tiny 15 ms flicker tolerance inside a noisy black box
  if (blackStartMs != 0 && (now - blackLastSeenMs > 15)) {
    blackStartMs = 0;
  }
  return false;
}
