#include "Config.h"

int16_t computeAdvancedBaseSpeed(int16_t absError, int16_t currentBase) {
  if (!cfg.advPidEn) {
    return currentBase;
  }

  // Smooth curve braking: reduce forward speed as error grows past 2200
  if (absError <= 2200) {
    return currentBase;
  }

  int32_t drop = ((int32_t)(absError - 2200) * 55L) / 5300L;
  int16_t minCornerSpeed = max((int16_t)65, (int16_t)(currentBase - 55));
  int16_t adjusted = currentBase - (int16_t)drop;

  return constrain(adjusted, minCornerSpeed, currentBase);
}

uint16_t computeAdaptiveKp(int16_t absError) {
  // Slightly gentler Kp on straightaways to suppress oscillation,
  // stronger Kp on steep off-center errors for crisp cornering.
  if (absError < 1500) {
    return (uint16_t)(((uint32_t)cfg.kp * 85UL) / 100UL);
  } else if (absError > 4500) {
    return (uint16_t)(((uint32_t)cfg.kp * 135UL) / 100UL);
  }
  return cfg.kp;
}
