const char* menuItem(byte i)
{
  switch(i)
  {
    case 1:  return "Motor Test";
    case 2:  return "Sensor Test";
    case 3:  return "Straight Run";
    case 4:  return "Left Priority";
    case 5:  return "Right Priority";
    case 6:  return "State Priority";
    case 7:  return "Inverse Mode";
    case 8:  return "Combine Mode";
    case 9:  return "Calibration";
    case 10: return "Speed/PID Tune";
  }

  return "";
}

byte topIndex = 1;

void displayMenu()
{
  oled.set1X();

  // Top Title Bar (Inverted White Header Banner)
  oled.setCursor(0, 0);
  oled.setInvertMode(true);
  oled.print("  DIU ROCKET SQUAD  ");
  oled.clearToEOL();
  oled.setInvertMode(false);

  // Keep menuIndex inside visible window [topIndex, topIndex + 5]
  if(menuIndex < topIndex)
  {
    topIndex = menuIndex;
  }
  else if(menuIndex > topIndex + 5)
  {
    topIndex = menuIndex - 5;
  }

  // Display 6 visible menu items
  for(byte r = 0; r < 6; r++)
  {
    byte idx = topIndex + r;
    oled.setCursor(0, r + 1);

    if(idx <= 10)
    {
      bool isSel = (idx == menuIndex);
      oled.setInvertMode(isSel);

      if(isSel)
      {
        oled.print(">");
      }
      else
      {
        oled.print(" ");
      }

      if(idx < 10) oled.print(" ");
      oled.print(idx);
      oled.print(". ");
      oled.print(menuItem(idx));

      oled.clearToEOL();
      oled.setInvertMode(false);
    }
    else
    {
      oled.setInvertMode(false);
      oled.clearToEOL();
    }
  }

  // Bottom Status Bar
  oled.setCursor(0, 7);
  oled.setInvertMode(false);
  oled.print("[");
  if(menuIndex < 10) oled.print("0");
  oled.print(menuIndex);
  oled.print("/10]  UP/DN  OK:Run");
  oled.clearToEOL();
}

void tuneSpeedPID()
{
  byte param = 0; // 0: baseSpeed, 1: Kp, 2: Kd
  delay(300);     // Debounce button press

  oled.clear();

  while(param < 3)
  {
    oled.setCursor(0, 0);
    oled.setInvertMode(true);
    oled.print("  TUNE SPEED / PID  ");
    oled.clearToEOL();
    oled.setInvertMode(false);

    // Speed display
    oled.setCursor(0, 2);
    oled.setInvertMode(param == 0);
    oled.print(param == 0 ? " > Base Speed: " : "   Base Speed: ");
    oled.print(baseSpeed);
    oled.clearToEOL();

    // Kp display
    oled.setCursor(0, 4);
    oled.setInvertMode(param == 1);
    oled.print(param == 1 ? " > Kp Gain   : " : "   Kp Gain   : ");
    oled.print(Kp, 1);
    oled.clearToEOL();

    // Kd display
    oled.setCursor(0, 6);
    oled.setInvertMode(param == 2);
    oled.print(param == 2 ? " > Kd Gain   : " : "   Kd Gain   : ");
    oled.print(Kd, 1);
    oled.clearToEOL();

    oled.setInvertMode(false);
    oled.setCursor(0, 7);
    oled.print("[UP/DN]+/-  [OK]Next");
    oled.clearToEOL();

    // Wait for button press
    while(1)
    {
      if(button(BTN_UP))
      {
        if(param == 0) baseSpeed = min(255, baseSpeed + 5);
        else if(param == 1) Kp = min(60.0, Kp + 1.0);
        else if(param == 2) Kd = min(60.0, Kd + 1.0);
        break;
      }

      if(button(BTN_DOWN))
      {
        if(param == 0) baseSpeed = max(100, baseSpeed - 5);
        else if(param == 1) Kp = max(5.0, Kp - 1.0);
        else if(param == 2) Kd = max(5.0, Kd - 1.0);
        break;
      }

      if(button(BTN_OK))
      {
        param++;
        break;
      }
    }
  }

  // Save tuned parameters to EEPROM
  savePIDSettings();

  oled.clear();
  oled.setInvertMode(true);
  oled.setCursor(0, 0);
  oled.print("  SETTINGS SAVED!   ");
  oled.clearToEOL();
  oled.setInvertMode(false);

  oled.setCursor(0, 2);
  oled.print("Speed : ");
  oled.print(baseSpeed);

  oled.setCursor(0, 4);
  oled.print("Kp    : ");
  oled.print(Kp, 1);

  oled.setCursor(0, 5);
  oled.print("Kd    : ");
  oled.print(Kd, 1);

  oled.setCursor(0, 7);
  oled.print("Saved to EEPROM!");
  delay(1800);
}

void menuSystem()
{
  oled.clear();
  displayMenu();

  while(1)
  {
    if(button(BTN_UP))
    {
      menuIndex--;
      if(menuIndex < 1) menuIndex = 10;
      displayMenu();
    }

    if(button(BTN_DOWN))
    {
      menuIndex++;
      if(menuIndex > 10) menuIndex = 1;
      displayMenu();
    }

    if(button(BTN_OK))
    {
      oled.clear();

      switch(menuIndex)
      {
        case 1:  motorTest(); break;
        case 2:  sensorTest(); break;
        case 3:  lineFollow(); break;
        case 4:  leftPriorityMode(); break;
        case 5:  rightPriorityMode(); break;
        case 6:  statePriorityMode(); break;
        case 7:  inverseModeRun(); break;
        case 8:  combineMode(); break;
        case 9:  calibration(); break;
        case 10: tuneSpeedPID(); break;
      }

      oled.clear();
      displayMenu();
    }
  }
}
