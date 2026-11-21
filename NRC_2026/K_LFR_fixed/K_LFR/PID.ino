#include "Config.h"

static int32_t pidIntegral  = 0;
static int16_t pidPrevError = 0;
static int16_t pidDerivFilt = 0;

void resetPID() {
  pidIntegral  = 0;
  pidPrevError = 0;
  pidDerivFilt = 0;
  lastError    = 0;
}

int16_t computePID(int16_t error) {
  uint16_t activeKp = cfg.kp;
  if (cfg.advPidEn) {
    activeKp = computeAdaptiveKp(abs(error));
  }

  // Anti-windup integral (only accumulate on moderate steady-state errors)
  if (cfg.ki > 0 && abs(error) < 4000) {
    pidIntegral += error;
    pidIntegral = constrain(pidIntegral, -15000L, 15000L);
  } else if (abs(error) < 120) {
    pidIntegral = (pidIntegral * 4) / 5;
  }

  // Filtered derivative: 65% previous + 35% current (smoother D response)
  int16_t rawDeriv = error - pidPrevError;
  pidDerivFilt = (int16_t)(((int32_t)pidDerivFilt * 65L + (int32_t)rawDeriv * 35L) / 100L);
  pidPrevError = error;
  lastError    = error;

  // Scaled integer arithmetic (Kp: /1000, Ki: /10000, Kd: /1000)
  int32_t output = (((int32_t)activeKp * (int32_t)error) / 1000L)
                 + (((int32_t)cfg.ki * pidIntegral) / 10000L)
                 + (((int32_t)cfg.kd * (int32_t)pidDerivFilt) / 1000L);

  int16_t maxTurn = (cfg.maxSpeed * 2);
  output = constrain(output, (int32_t)(-maxTurn), (int32_t)maxTurn);

  return (int16_t)output;
}
