void readSensor()
{
  for(byte i=0; i<6; i++)
  {
    sensorValue[i] = analogRead(sensorPin[i]);
  }
}

void digitalSensor()
{
  readSensor();

  for(byte i=0; i<6; i++)
  {
    sensorDigital[i] = (sensorValue[i] > sensorMid[i]) ? 1 : 0;
  }

  if(inverseMode)
  {
    for(byte i=0; i<6; i++)
    {
      sensorDigital[i] = !sensorDigital[i];
    }
  }
}

void sensorTest()
{
  oled.clear();

  while(!button(BTN_OK))
  {
    digitalSensor();

    // Inverted Top Header
    oled.set1X();
    oled.setCursor(0, 0);
    oled.setInvertMode(true);
    oled.print("  LIVE SENSOR TEST  ");
    oled.clearToEOL();
    oled.setInvertMode(false);

    // Graphical 6-sensor visual bar
    oled.setCursor(8, 2);
    for(byte i = 0; i < 6; i++)
    {
      oled.print("[");
      if(sensorDigital[i])
      {
        oled.setInvertMode(true);
        oled.print("#");
        oled.setInvertMode(false);
      }
      else
      {
        oled.print(" ");
      }
      oled.print("] ");
    }
    oled.clearToEOL();

    // Sensor Pin Names
    oled.setCursor(8, 3);
    oled.print("S1  S2  S3  S4  S5  S6");
    oled.clearToEOL();

    // Formatted Analog Values
    oled.setCursor(0, 5);
    oled.print("L: ");
    for(byte i = 0; i < 3; i++)
    {
      if(sensorValue[i] < 1000) oled.print(" ");
      if(sensorValue[i] < 100)  oled.print(" ");
      if(sensorValue[i] < 10)   oled.print(" ");
      oled.print(sensorValue[i]);
      oled.print(" ");
    }
    oled.clearToEOL();

    oled.setCursor(0, 6);
    oled.print("R: ");
    for(byte i = 3; i < 6; i++)
    {
      if(sensorValue[i] < 1000) oled.print(" ");
      if(sensorValue[i] < 100)  oled.print(" ");
      if(sensorValue[i] < 10)   oled.print(" ");
      oled.print(sensorValue[i]);
      oled.print(" ");
    }
    oled.clearToEOL();

    oled.setCursor(0, 7);
    oled.print("  [Press OK to Exit] ");
    oled.clearToEOL();

    delay(60);
  }

  delay(200);
}

void calibration()
{
  oled.clear();
  oled.set1X();
  oled.setCursor(0, 0);
  oled.setInvertMode(true);
  oled.print(" SENSOR CALIBRATION ");
  oled.clearToEOL();
  oled.setInvertMode(false);

  oled.setCursor(0, 2);
  oled.print("Swipe robot on line");
  oled.setCursor(0, 4);
  oled.print("Sampling sensors...");
  oled.setCursor(0, 6);
  oled.print("UP: Auto   OK: Save");

  for(byte i=0; i<6; i++)
  {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  delay(300); // Debounce button release

  unsigned long lastDisplay = 0;

  while(!button(BTN_OK))
  {
    // If user presses BTN_UP, execute smooth motor sweep on floor
    if(button(BTN_UP))
    {
      oled.setCursor(0, 4);
      oled.print("Motor Sweeping...   ");

      unsigned long sweepStart = millis();
      while(millis() - sweepStart < 3000)
      {
        if((millis() - sweepStart) % 500 < 250)
        {
          motor(-120, 120);
        }
        else
        {
          motor(120, -120);
        }

        for(byte i=0; i<6; i++)
        {
          int val = analogRead(sensorPin[i]);
          if(val < sensorMin[i]) sensorMin[i] = val;
          if(val > sensorMax[i]) sensorMax[i] = val;
        }

        if(button(BTN_OK)) break;
      }

      stopMotor();
      oled.setCursor(0, 4);
      oled.print("Sweep Done! Ready.  ");
    }

    // Continuous background sampling (for manual swiping by hand)
    for(byte i=0; i<6; i++)
    {
      int val = analogRead(sensorPin[i]);
      if(val < sensorMin[i]) sensorMin[i] = val;
      if(val > sensorMax[i]) sensorMax[i] = val;
    }

    // Live display of center sensor min/max
    if(millis() - lastDisplay > 150)
    {
      lastDisplay = millis();
      oled.setCursor(0, 4);
      oled.print("S3 L:");
      oled.print(sensorMin[2]);
      oled.print(" H:");
      oled.print(sensorMax[2]);
      oled.clearToEOL();
    }
  }

  stopMotor();

  // Calculate midpoints with safety validation
  for(byte i=0; i<6; i++)
  {
    if(sensorMax[i] - sensorMin[i] > 100)
    {
      sensorMid[i] = (sensorMin[i] + sensorMax[i]) / 2;
    }
    else
    {
      // Safe default if sensor saw no contrast
      sensorMid[i] = 512;
    }
  }

  saveCalibration();

  // Display results
  oled.clear();
  oled.setCursor(0, 0);
  oled.setInvertMode(true);
  oled.print(" CALIBRATION SAVED! ");
  oled.clearToEOL();
  oled.setInvertMode(false);

  oled.setCursor(0, 2);
  oled.print("S1-S3: ");
  for(byte i=0; i<3; i++)
  {
    oled.print(sensorMid[i]);
    oled.print(" ");
  }

  oled.setCursor(0, 4);
  oled.print("S4-S6: ");
  for(byte i=3; i<6; i++)
  {
    oled.print(sensorMid[i]);
    oled.print(" ");
  }

  oled.setCursor(0, 7);
  oled.print("Saved to EEPROM!");
  delay(2200);
}
