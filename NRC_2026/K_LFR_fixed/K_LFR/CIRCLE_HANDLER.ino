#include "Config.h"

static uint32_t circleActiveMs = 0;
static int8_t   circleBranch   = DIR_RIGHT;

void resetCircleHandler() {
  circleActiveMs = 0;
  circleBranch   = DIR_RIGHT;
}

bool handleCircle() {
  if (!cfg.circleEn) {
    return false;
  }

  // Detect a split line (Y-fork / circle entry):
  // Two separated clusters with an inactive gap between them
  int8_t leftMost  = -1;
  int8_t rightMost = -1;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (sensorNorm[i] >= SENSOR_THRESH) {
      if (leftMost < 0) leftMost = i;
      rightMost = i;
    }
  }

  if (leftMost < 0 || leftMost > 1 || rightMost < 4 || (rightMost - leftMost < 3) || activeCount < 2) {
    return false;
  }

  // Count inactive gap sensors between leftMost and rightMost
  uint8_t gapCount = 0;
  for (uint8_t i = (uint8_t)leftMost + 1; i < (uint8_t)rightMost; i++) {
    if (sensorNorm[i] < 250) {
      gapCount++;
    }
  }

  if (gapCount < 1) {
    return false;
  }

  uint32_t now = millis();
  if (now - circleActiveMs > 400) {
    if (cfg.circleDir == 1) {
      circleBranch = DIR_RIGHT;
    } else if (cfg.circleDir == 2) {
      circleBranch = DIR_LEFT;
    } else {
      circleBranch = (lastDir == DIR_LEFT) ? DIR_LEFT : DIR_RIGHT;
    }
  }
  circleActiveMs = now;

  // Compute weighted position using ONLY the chosen branch half
  uint32_t wSum     = 0;
  uint32_t tWeight  = 0;
  uint8_t  startIdx = (circleBranch == DIR_LEFT) ? 0 : (NUM_SENSORS / 2);
  uint8_t  endIdx   = (circleBranch == DIR_LEFT) ? (NUM_SENSORS / 2 - 1) : (NUM_SENSORS - 1);

  for (uint8_t i = startIdx; i <= endIdx; i++) {
    if (sensorNorm[i] > 100) {
      wSum    += (uint32_t)sensorNorm[i] * ((uint32_t)i * SENSOR_STEP);
      tWeight += (uint32_t)sensorNorm[i];
    }
  }

  if (tWeight > 0) {
    linePosition = (int32_t)(wSum / tWeight);
    lineError    = (int16_t)(linePosition - LINE_CENTER);

    int16_t pidOut = computePID(lineError);
    int16_t spd    = computeAdvancedBaseSpeed(abs(lineError), cfg.baseSpeed);
    applySteeringMix(spd, pidOut);
    return true;
  }

  return false;
}
