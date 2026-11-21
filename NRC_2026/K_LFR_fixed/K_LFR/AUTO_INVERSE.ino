#include "Config.h"

static uint32_t invDetectStartMs = 0;
static uint32_t lastFlipMs       = 0;

void resetAutoInverse() {
  invDetectStartMs = 0;
  lastFlipMs       = 0;
  isInvertedLine   = (cfg.manualInverse != 0);
}

void updateAutoInverse() {
  if (!cfg.autoInverseEn) {
    return;
  }

  uint32_t now = millis();
  if (now - lastFlipMs < 350) {
    return;
  }

  // Inverted pattern: left edge (0) AND right edge (5) see "line" (active)
  // while at least one center sensor (2,3) sees background (inactive).
  bool leftOuterActive  = (sensorNorm[0] > 650);
  bool rightOuterActive = (sensorNorm[5] > 650);

  uint8_t midInactive = 0;
  for (uint8_t i = 2; i <= 3; i++) {
    if (sensorNorm[i] < 350) {
      midInactive++;
    }
  }

  if (leftOuterActive && rightOuterActive && midInactive >= 1 && activeCount >= 3) {
    if (invDetectStartMs == 0) {
      invDetectStartMs = now;
    } else if (now - invDetectStartMs >= 25) {
      isInvertedLine   = !isInvertedLine;
      invDetectStartMs = 0;
      lastFlipMs       = now;
      // Re-read immediately with the new polarity so PID sees the true line right away
      readLineSensors();
    }
  } else {
    invDetectStartMs = 0;
  }
}

