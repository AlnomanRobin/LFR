#include "Config.h"

static uint32_t centeredSinceMs = 0;

const uint8_t SENSOR_PINS[NUM_SENSORS] = {PIN_S1, PIN_S2, PIN_S3, PIN_S4, PIN_S5, PIN_S6};

void sensorInit() {
  pinMode(PIN_S3, INPUT);
  pinMode(PIN_S4, INPUT);
  pinMode(PIN_S5, INPUT);
  pinMode(PIN_S6, INPUT);

#ifdef ADCSRA
  // Set ADC prescaler to 32 (500 kHz ADC clock -> ~28 us/read, >1 kHz full scan)
  ADCSRA = (ADCSRA & 0xF8) | 0x05;
#endif
}

void readRawSensors() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    uint8_t pinIdx = cfg.reverseSensors ? (NUM_SENSORS - 1 - i) : i;
    sensorRaw[i] = (uint16_t)analogRead(SENSOR_PINS[pinIdx]);
  }
}

void readLineSensors() {
  readRawSensors();

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    uint16_t sMin = cfg.sensorMin[i];
    uint16_t sMax = cfg.sensorMax[i];
    uint16_t val  = sensorRaw[i];

    if (sMax <= sMin + 10) {
      sensorNorm[i] = 0;
      continue;
    }

    int32_t norm = ((int32_t)val - (int32_t)sMin) * 1000L / (int32_t)(sMax - sMin);
    norm = constrain(norm, 0L, 1000L);

    if (!cfg.highIsBlack) {
      norm = 1000L - norm;
    }

    sensorNorm[i] = (uint16_t)norm;
  }

  // Apply software line inversion if active (white line on black surface)
  if (isInvertedLine) {
    applyInverse();
  }

  // Build bitmask, active sensor count, and weighted average line position
  uint32_t weightedSum = 0;
  uint32_t totalWeight = 0;
  uint16_t bits = 0;
  uint8_t  cnt  = 0;

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    uint16_t v = sensorNorm[i];

    if (v >= SENSOR_THRESH) {
      bits |= (1U << i);
      cnt++;
    }

    // Ignore low-level background noise below 80/1000 in weighted average
    if (v > 80) {
      weightedSum += (uint32_t)v * ((uint32_t)i * SENSOR_STEP);
      totalWeight += (uint32_t)v;
    }
  }

  sensorBits  = bits;
  activeCount = cnt;

  if (cnt > 0 || totalWeight >= 120) {
    linePosition = (int32_t)(weightedSum / totalWeight);
    lineError    = (int16_t)(linePosition - LINE_CENTER);

    // Track last known side of the line; clear when cleanly centered on a straight
    if (lineError < -2200) {
      lastDir = DIR_LEFT;
      centeredSinceMs = 0;
    } else if (lineError > 2200) {
      lastDir = DIR_RIGHT;
      centeredSinceMs = 0;
    } else if (abs(lineError) < 1200 && cnt <= 3) {
      uint32_t now = millis();
      if (centeredSinceMs == 0) {
        centeredSinceMs = now;
      } else if (now - centeredSinceMs >= 50) {
        lastDir = DIR_NONE;
      }
    } else {
      centeredSinceMs = 0;
    }
  } else {
    centeredSinceMs = 0;
    // No line detected: hold last known direction
    if (lastDir == DIR_LEFT) {
      linePosition = 0;
      lineError    = -LINE_CENTER;
    } else if (lastDir == DIR_RIGHT) {
      linePosition = LINE_MAX_POS;
      lineError    = LINE_CENTER;
    } else {
      lineError    = lastError;
    }
  }
}

void runCalibration(bool spinRobot) {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    cfg.sensorMin[i] = 1023;
    cfg.sensorMax[i] = 0;
  }

  waitReleaseAll();
  const uint16_t totalSteps = 160;

  for (uint16_t step = 0; step < totalSteps; step++) {
    if (digitalRead(PIN_BTN_ENTER) == LOW) {
      motorStop();
      loadEEPROM();
      showStatusScreen(F("CALIB CANCELLED"), F("Previous kept"));
      waitReleaseAll();
      delay(700);
      return;
    }

    if (spinRobot) {
      if (step < 40) {
        setMotors(115, -115);
      } else if (step < 120) {
        setMotors(-115, 115);
      } else {
        setMotors(115, -115);
      }
    }

    readRawSensors();
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      if (sensorRaw[i] < cfg.sensorMin[i]) cfg.sensorMin[i] = sensorRaw[i];
      if (sensorRaw[i] > cfg.sensorMax[i]) cfg.sensorMax[i] = sensorRaw[i];
    }

    if (oledReady && (step % 20 == 0)) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.print(F("CALIBRATING IR..."));
      display.setCursor(0, 16);
      display.print(spinRobot ? F("Auto-sweep active") : F("Move robot across line"));

      uint8_t barW = (uint8_t)((uint32_t)step * 120UL / totalSteps);
      display.drawRect(4, 34, 120, 12, SSD1306_WHITE);
      display.fillRect(4, 34, barW, 12, SSD1306_WHITE);
      display.setCursor(0, 54);
      display.print(F("ENTER: Cancel"));
      display.display();
    }

    delay(20);
  }

  motorStop();

  // Safety guard for any unreached channel
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (cfg.sensorMax[i] <= cfg.sensorMin[i] + 30) {
      cfg.sensorMin[i] = 150;
      cfg.sensorMax[i] = 850;
    }
  }

  saveEEPROM();
  showStatusScreen(F("CALIBRATION DONE"), F("Saved to EEPROM"));
  delay(800);
}
