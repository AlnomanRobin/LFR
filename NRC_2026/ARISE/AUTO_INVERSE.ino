/****************************************
   DUAL-POLARITY AUTO INVERSE & FINISH
****************************************/

bool checkAutoInverseAndFinish(bool &finished)
{
  byte activeCount = 0;
  for(byte i = 0; i < 6; i++)
  {
    if(sensorDigital[i]) activeCount++;
  }

  // 5 or 6 sensors flooded with track color
  if(activeCount >= 5)
  {
    if(floodStartTime == 0) floodStartTime = millis();
    unsigned long held = millis() - floodStartTime;

    // Check opposite polarity (is there an opposite-colored line in the middle?)
    byte oppCount = 0;
    for(byte i = 0; i < 6; i++)
    {
      bool oppState = false;
      if(!inverseMode)
      {
        // Normal mode (black line): looking for white line (< mid) inside black flood
        if(sensorValue[i] < sensorMid[i]) oppState = true;
      }
      else
      {
        // Inverse mode (white line): looking for black line (> mid) inside white flood
        if(sensorValue[i] > sensorMid[i]) oppState = true;
      }

      if(oppState) oppCount++;
    }

    // 1. Auto Inversion Transition:
    // If opposite line exists in the center (1-3 sensors) and sustained for >90ms
    if(autoInverseEnabled && (oppCount >= 1 && oppCount <= 3) && held > 90 && (millis() - lastFlipTime > 600))
    {
      inverseMode = !inverseMode;
      lastFlipTime = millis();
      floodStartTime = 0;
      gapStartTime = 0;
      lastError = 0;

      // Pulse LED indicator
      digitalWrite(LED_PIN, HIGH);
      delay(20);
      digitalWrite(LED_PIN, LOW);

      // Smooth step across polarity boundary
      motor(gapSpeed, gapSpeed);
      delay(30);
      return true;
    }

    // 2. Solid Finish / Stop Box:
    // All sensors active, NO opposite line in center, sustained > 200ms, and run time > 2.5s
    if(oppCount == 0 && held > 200 && (millis() - runStartTime > 2500))
    {
      brakeMotor();
      finished = true;
      return true;
    }
  }
  else
  {
    floodStartTime = 0;
  }

  return false;
}

void autoInverseDetect()
{
  bool dummy = false;
  checkAutoInverseAndFinish(dummy);
}
