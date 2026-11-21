#include "Config.h"

void applyInverse() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensorNorm[i] = 1000 - sensorNorm[i];
  }
}

