#include "Config.h"

void loadDefaults() {
  cfg.magic   = EEPROM_MAGIC;
  cfg.version = EEPROM_VERSION;

  // Scaled integer PID (position scaled to 0..15000 for 6 ch): kp=38 (0.038), ki=0 (0.0000), kd=320 (0.320)
  cfg.kp = 38;
  cfg.ki = 0;
  cfg.kd = 320;

  cfg.baseSpeed  = 150;
  cfg.maxSpeed   = 230;
  cfg.turnSpeed  = 135;
  cfg.boostSpeed = 240;

  cfg.highIsBlack    = 1;
  cfg.manualInverse  = 0;
  cfg.autoInverseEn  = 1;
  cfg.advPidEn       = 1;
  cfg.speedBoostEn   = 1;
  cfg.junctionEn     = 1;
  cfg.circleEn       = 1;
  cfg.blackAreaEn    = 1;
  cfg.safetyUTurnEn  = 1;

  cfg.invertLeftMotor  = 0;
  cfg.invertRightMotor = 0;

  cfg.blackStopMs    = 300;
  cfg.circleDir      = 0;
  cfg.reverseSensors = 0;

  cfg.juncPriority   = 0; // 0=PATH, 1=STATE (S->L->R), 2=LEFT (L->S->R), 3=RIGHT (R->S->L)
  cfg.pathRunEn      = 1;
  cfg.pathLen        = 7;
  strncpy(cfg.path, "LRRLSRE", MAX_PATH_LEN - 1);
  cfg.path[MAX_PATH_LEN - 1] = '\0';

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    cfg.sensorMin[i] = 150;
    cfg.sensorMax[i] = 850;
  }
}

void loadEEPROM() {
  EEPROM.get(0, cfg);
  if (cfg.magic != EEPROM_MAGIC || cfg.version != EEPROM_VERSION) {
    loadDefaults();
    saveEEPROM();
  }
}

void saveEEPROM() {
  cfg.magic   = EEPROM_MAGIC;
  cfg.version = EEPROM_VERSION;
  EEPROM.put(0, cfg);
}

