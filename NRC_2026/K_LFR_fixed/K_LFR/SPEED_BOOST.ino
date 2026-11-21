#include "Config.h"

static uint32_t straightStartMs = 0;
static uint32_t lastBoostTickMs = 0;
static int16_t  boostOffset     = 0;

void resetSpeedBoost() {
  straightStartMs = 0;
  lastBoostTickMs = 0;
  boostOffset     = 0;
}

int16_t updateSpeedBoost(int16_t error, int16_t baseSpd) {
  if (!cfg.speedBoostEn) {
    boostOffset = 0;
    return baseSpd;
  }

  uint32_t now = millis();
  int16_t absErr = abs(error);

  if (absErr <= 1200 && activeCount >= 1 && activeCount <= 3) {
    if (straightStartMs == 0) {
      straightStartMs = now;
      lastBoostTickMs = now;
    }

    if (now - straightStartMs >= 110) {
      int16_t maxExtra = cfg.boostSpeed - baseSpd;
      if (maxExtra > 0 && (now - lastBoostTickMs >= 3)) {
        uint16_t steps = (uint16_t)((now - lastBoostTickMs) / 3);
        lastBoostTickMs = now;
        boostOffset += (int16_t)steps;
        if (boostOffset > maxExtra) boostOffset = maxExtra;
      }
      return baseSpd + boostOffset;
    }
  } else {
    // Smooth ramp-down when approaching a curve or junction
    straightStartMs = 0;
    lastBoostTickMs = 0;
    if (boostOffset > 0) {
      boostOffset = (boostOffset > 8) ? (boostOffset - 8) : 0;
      return baseSpd + boostOffset;
    }
    boostOffset = 0;
  }

  return baseSpd;
}
