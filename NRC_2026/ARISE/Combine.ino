/****************************************
         COMBINE MODE & MAZE FUNCTIONS
****************************************/

char path[128];
int pathIndex = 0;
char lastTurn = 'S';

bool crossJunction  = false;
bool leftJunction   = false;
bool rightJunction  = false;
bool straightPath   = false;
bool deadEnd        = false;
bool finishDetected = false;

void storePath(char dir)
{
  if(pathIndex < 128)
  {
    path[pathIndex++] = dir;
  }
}

void optimizePath()
{
  if(pathIndex < 3 || path[pathIndex - 2] != 'B')
  {
    return;
  }

  char turnBefore = path[pathIndex - 3];
  char turnAfter  = path[pathIndex - 1];
  char resolved   = ' ';

  // Standard Maze Reduction Rules (LSRB):
  if(turnBefore == 'L' && turnAfter == 'L') resolved = 'S';
  else if(turnBefore == 'L' && turnAfter == 'S') resolved = 'R';
  else if(turnBefore == 'R' && turnAfter == 'L') resolved = 'S';
  else if(turnBefore == 'S' && turnAfter == 'L') resolved = 'R';
  else if(turnBefore == 'S' && turnAfter == 'S') resolved = 'B';
  else if(turnBefore == 'L' && turnAfter == 'R') resolved = 'B';
  else if(turnBefore == 'R' && turnAfter == 'R') resolved = 'S';
  else if(turnBefore == 'S' && turnAfter == 'R') resolved = 'L';
  else if(turnBefore == 'R' && turnAfter == 'S') resolved = 'L';

  if(resolved != ' ')
  {
    path[pathIndex - 3] = resolved;
    pathIndex -= 2;
  }
}

void detectJunction()
{
  digitalSensor();

  crossJunction  = false;
  leftJunction   = false;
  rightJunction  = false;
  straightPath   = false;
  deadEnd        = false;
  finishDetected = false;

  byte sum = 0;
  for(byte i = 0; i < 6; i++)
  {
    sum += sensorDigital[i];
  }

  // 1. Line Lost: Gap Crossing, Sharp Curve Recovery, or Dead End
  if(sum == 0)
  {
    if(abs(lastError) < 2.2)
    {
      if(gapStartTime == 0) gapStartTime = millis();
      if(millis() - gapStartTime < GAP_TIMEOUT)
      {
        // Keep moving forward across potential dashed gap; calculatePID handles it
        return;
      }
      deadEnd = true;
    }
    // If abs(lastError) >= 2.5, line slipped on a sharp curve -> let calculatePID recover it!
    return;
  }

  gapStartTime = 0;

  // 2. Potential Junction, Inverted Zone, or Finish Box
  if(sensorDigital[0] || sensorDigital[5] || sum >= 4)
  {
    // Step A: Glide forward 24ms to capture both Left & Right branches on tilted entry
    byte leftHits  = sensorDigital[0] ? 1 : 0;
    byte rightHits = sensorDigital[5] ? 1 : 0;

    motor(165, 165);
    for(byte k = 0; k < 4; k++)
    {
      delay(6);
      digitalSensor();
      if(sensorDigital[0]) leftHits++;
      if(sensorDigital[5]) rightHits++;
    }

    bool sawLeft  = (leftHits >= 2);
    bool sawRight = (rightHits >= 2);

    if(!sawLeft && !sawRight && sum < 5)
    {
      // Momentary sensor flicker, not a real junction
      return;
    }

    // Step B: Advance forward so wheel axle aligns over the intersection & sensors clear the 3cm bar
    motor(175, 175);
    delay(70);
    digitalSensor();

    byte postSum = 0;
    for(byte i = 0; i < 6; i++) postSum += sensorDigital[i];

    // Check if flooded (>= 5 sensors active after ~95ms forward travel)
    if(postSum >= 5)
    {
      // Check opposite polarity in center (white line inside black zone, or black line inside white zone)
      byte oppCount = 0;
      for(byte i = 0; i < 6; i++)
      {
        bool oppState = (!inverseMode) ? (sensorValue[i] < sensorMid[i]) : (sensorValue[i] > sensorMid[i]);
        if(oppState) oppCount++;
      }

      if(oppCount >= 1 && oppCount <= 3 && autoInverseEnabled && (millis() - lastFlipTime > 600))
      {
        inverseMode = !inverseMode;
        lastFlipTime = millis();
        lastError = 0;
        digitalWrite(LED_PIN, HIGH);
        delay(20);
        digitalWrite(LED_PIN, LOW);
        return;
      }

      // Advance slightly further to confirm large Finish Box (vs thick intersection tape)
      motor(150, 150);
      delay(45);
      digitalSensor();
      postSum = 0;
      for(byte i = 0; i < 6; i++) postSum += sensorDigital[i];

      if(postSum >= 5 && (millis() - runStartTime > 2000))
      {
        finishDetected = true;
        return;
      }
    }

    // Straight path continues ahead if center sensors still detect a line after clearing horizontal bar
    straightPath = (sensorDigital[1] || sensorDigital[2] || sensorDigital[3] || sensorDigital[4]);

    if(sawLeft && sawRight)
    {
      crossJunction = true;
    }
    else if(sawLeft)
    {
      leftJunction = true;
    }
    else if(sawRight)
    {
      rightJunction = true;
    }
    return;
  }

  // Normal line follow
  if(sensorDigital[2] || sensorDigital[3])
  {
    straightPath = true;
  }
}

//-------------------------------------

void turnLeft()
{
  brakeMotor();
  delay(20);

  // Stage 1: Spin LEFT until center sensors leave any straight path (minimum 50ms kick)
  motor(-turnSpeed, turnSpeed);
  unsigned long start = millis();
  while((millis() - start < 50) || ((sensorDigital[2] || sensorDigital[3]) && (millis() - start < 450)))
  {
    digitalSensor();
  }

  // Stage 2: Keep spinning LEFT until center sensors acquire the left branch
  while(!sensorDigital[2] && !sensorDigital[3] && (millis() - start < 950))
  {
    digitalSensor();
  }

  // Active counter-pulse brake to cancel rotational inertia cleanly on the line
  motor(turnSpeed, -turnSpeed);
  delay(25);
  stopMotor();

  lastError = 0;
  gapStartTime = 0;

  storePath('L');
}

//-------------------------------------

void turnRight()
{
  brakeMotor();
  delay(20);

  // Stage 1: Spin RIGHT until center sensors leave any straight path (minimum 50ms kick)
  motor(turnSpeed, -turnSpeed);
  unsigned long start = millis();
  while((millis() - start < 50) || ((sensorDigital[2] || sensorDigital[3]) && (millis() - start < 450)))
  {
    digitalSensor();
  }

  // Stage 2: Keep spinning RIGHT until center sensors acquire the right branch
  while(!sensorDigital[2] && !sensorDigital[3] && (millis() - start < 950))
  {
    digitalSensor();
  }

  // Active counter-pulse brake to cancel rotational inertia cleanly on the line
  motor(-turnSpeed, turnSpeed);
  delay(25);
  stopMotor();

  lastError = 0;
  gapStartTime = 0;

  storePath('R');
}

//-------------------------------------

void goStraight()
{
  // Already cleared junction in detectJunction(); brief straight pulse then resume PID
  motor(baseSpeed, baseSpeed);
  delay(20);
  lastError = 0;
  storePath('S');
}

//-------------------------------------

void uTurn()
{
  safeUTurn();
}

//-------------------------------------

void runShortestPath()
{
  oled.clear();
  oled.setCursor(0, 0);
  oled.print("SPRINT PHASE");
  oled.setCursor(0, 2);
  oled.print("Path: ");
  for(int i = 0; i < pathIndex; i++)
  {
    oled.print(path[i]);
  }
  oled.setCursor(0, 6);
  oled.print("[Press OK to Run]");

  while(!button(BTN_OK));
  delay(300);

  runStartTime = millis();
  int step = 0;
  while(!button(BTN_OK) && step < pathIndex)
  {
    detectJunction();

    if(finishDetected)
    {
      brakeMotor();
      oled.clear();
      oled.setCursor(20, 3);
      oled.print("WINNER!");
      delay(2000);
      return;
    }

    if(crossJunction || leftJunction || rightJunction)
    {
      char act = path[step++];
      if(act == 'L') turnLeft();
      else if(act == 'R') turnRight();
      else if(act == 'S') goStraight();
    }
    else
    {
      calculatePID();
    }
  }

  stopMotor();
}

//-------------------------------------

void leftPriorityMode()
{
  runStartTime   = millis();
  gapStartTime   = 0;
  floodStartTime = 0;
  lastError      = 0;

  oled.clear();
  oled.set1X();
  oled.setCursor(0, 0);
  oled.setInvertMode(true);
  oled.print(" RUN: LEFT PRIORITY ");
  oled.clearToEOL();
  oled.setInvertMode(false);
  oled.setCursor(0, 3);
  oled.print("Mode: Left Turn Pref");
  oled.setCursor(0, 7);
  oled.print("[Press OK to Stop]");

  while(!button(BTN_OK))
  {
    detectJunction();

    if(finishDetected)
    {
      brakeMotor();
      oled.clear();
      oled.set1X();
      oled.setCursor(0, 0);
      oled.setInvertMode(true);
      oled.print("  FINISH REACHED!   ");
      oled.clearToEOL();
      oled.setInvertMode(false);
      oled.setCursor(14, 3);
      oled.print("DIU_Rocket_Squad");
      delay(2000);
      break;
    }
    else if(crossJunction || leftJunction)
    {
      turnLeft();
    }
    else if(rightJunction)
    {
      if(straightPath)
      {
        goStraight();
      }
      else
      {
        turnRight();
      }
    }
    else if(deadEnd)
    {
      uTurn();
    }
    else
    {
      calculatePID();
    }
  }

  stopMotor();
}

//-------------------------------------

void rightPriorityMode()
{
  runStartTime   = millis();
  gapStartTime   = 0;
  floodStartTime = 0;
  lastError      = 0;

  oled.clear();
  oled.set1X();
  oled.setCursor(0, 0);
  oled.setInvertMode(true);
  oled.print(" RUN: RIGHT PRIORITY");
  oled.clearToEOL();
  oled.setInvertMode(false);
  oled.setCursor(0, 3);
  oled.print("Mode: Right Turn Pref");
  oled.setCursor(0, 7);
  oled.print("[Press OK to Stop]");

  while(!button(BTN_OK))
  {
    detectJunction();

    if(finishDetected)
    {
      brakeMotor();
      oled.clear();
      oled.set1X();
      oled.setCursor(0, 0);
      oled.setInvertMode(true);
      oled.print("  FINISH REACHED!   ");
      oled.clearToEOL();
      oled.setInvertMode(false);
      oled.setCursor(14, 3);
      oled.print("DIU_Rocket_Squad");
      delay(2000);
      break;
    }
    else if(crossJunction || rightJunction)
    {
      turnRight();
    }
    else if(leftJunction)
    {
      if(straightPath)
      {
        goStraight();
      }
      else
      {
        turnLeft();
      }
    }
    else if(deadEnd)
    {
      uTurn();
    }
    else
    {
      calculatePID();
    }
  }

  stopMotor();
}

//-------------------------------------

void straightMode()
{
  runStartTime   = millis();
  gapStartTime   = 0;
  floodStartTime = 0;
  lastError      = 0;

  oled.clear();
  oled.set1X();
  oled.setCursor(0, 0);
  oled.setInvertMode(true);
  oled.print(" RUN: STRAIGHT MODE ");
  oled.clearToEOL();
  oled.setInvertMode(false);
  oled.setCursor(0, 3);
  oled.print("Mode: Straight Pref");
  oled.setCursor(0, 7);
  oled.print("[Press OK to Stop]");

  while(!button(BTN_OK))
  {
    detectJunction();

    if(finishDetected)
    {
      brakeMotor();
      oled.clear();
      oled.set1X();
      oled.setCursor(0, 0);
      oled.setInvertMode(true);
      oled.print("  FINISH REACHED!   ");
      oled.clearToEOL();
      oled.setInvertMode(false);
      oled.setCursor(14, 3);
      oled.print("DIU_Rocket_Squad");
      delay(2000);
      break;
    }
    else if(crossJunction)
    {
      if(straightPath) goStraight();
      else turnLeft();
    }
    else if(leftJunction)
    {
      if(straightPath) goStraight();
      else turnLeft();
    }
    else if(rightJunction)
    {
      if(straightPath) goStraight();
      else turnRight();
    }
    else if(deadEnd)
    {
      uTurn();
    }
    else
    {
      calculatePID();
    }
  }

  stopMotor();
}

//-------------------------------------

void statePriorityMode()
{
  runStartTime   = millis();
  gapStartTime   = 0;
  floodStartTime = 0;
  lastError      = 0;

  oled.clear();
  oled.set1X();
  oled.setCursor(0, 0);
  oled.setInvertMode(true);
  oled.print(" RUN: STATE PRIORITY");
  oled.clearToEOL();
  oled.setInvertMode(false);
  oled.setCursor(0, 3);
  oled.print("Mode: Memory Dynamic");
  oled.setCursor(0, 7);
  oled.print("[Press OK to Stop]");

  while(!button(BTN_OK))
  {
    detectJunction();

    if(finishDetected)
    {
      brakeMotor();
      oled.clear();
      oled.set1X();
      oled.setCursor(0, 0);
      oled.setInvertMode(true);
      oled.print("  FINISH REACHED!   ");
      oled.clearToEOL();
      oled.setInvertMode(false);
      oled.setCursor(14, 3);
      oled.print("DIU_Rocket_Squad");
      delay(2000);
      break;
    }

    if(lastTurn == 'L')
    {
      if(crossJunction || leftJunction)
      {
        turnLeft();
        lastTurn = 'L';
      }
      else if(rightJunction)
      {
        turnRight();
        lastTurn = 'R';
      }
      else if(deadEnd)
      {
        uTurn();
      }
      else
      {
        calculatePID();
      }
    }
    else if(lastTurn == 'R')
    {
      if(crossJunction || rightJunction)
      {
        turnRight();
        lastTurn = 'R';
      }
      else if(leftJunction)
      {
        turnLeft();
        lastTurn = 'L';
      }
      else if(deadEnd)
      {
        uTurn();
      }
      else
      {
        calculatePID();
      }
    }
    else
    {
      if(crossJunction && straightPath)
      {
        goStraight();
        lastTurn = 'S';
      }
      else if(crossJunction || leftJunction)
      {
        turnLeft();
        lastTurn = 'L';
      }
      else if(rightJunction)
      {
        turnRight();
        lastTurn = 'R';
      }
      else if(deadEnd)
      {
        uTurn();
      }
      else
      {
        calculatePID();
      }
    }
  }

  stopMotor();
}

//-------------------------------------

void combineMode()
{
  runStartTime   = millis();
  gapStartTime   = 0;
  floodStartTime = 0;
  lastError      = 0;

  oled.clear();
  oled.set1X();
  oled.setCursor(0, 0);
  oled.setInvertMode(true);
  oled.print(" COMBINE MAZE MODE  ");
  oled.clearToEOL();
  oled.setInvertMode(false);

  oled.setCursor(0, 2);
  oled.print("1. Map Exploration");
  oled.setCursor(0, 4);
  oled.print("2. Path Optimize");
  oled.setCursor(0, 7);
  oled.print("[Press OK to Start]");

  while(!button(BTN_OK));
  delay(300);

  runStartTime = millis();
  pathIndex = 0;
  bool reachedFinish = false;

  while(!button(BTN_OK))
  {
    detectJunction();

    if(finishDetected)
    {
      brakeMotor();
      reachedFinish = true;
      break;
    }
    else if(deadEnd)
    {
      uTurn();
      optimizePath();
    }
    else if(crossJunction || leftJunction)
    {
      turnLeft();
      optimizePath();
    }
    else if(rightJunction)
    {
      if(straightPath)
      {
        goStraight();
      }
      else
      {
        turnRight();
      }
      optimizePath();
    }
    else
    {
      calculatePID();
    }
  }

  stopMotor();

  if(reachedFinish)
  {
    oled.clear();
    oled.set1X();
    oled.setCursor(0, 0);
    oled.setInvertMode(true);
    oled.print("  MAZE SOLVED 100%  ");
    oled.clearToEOL();
    oled.setInvertMode(false);

    oled.setCursor(0, 2);
    oled.print("DIU_Rocket_Squad");
    oled.setCursor(0, 4);
    oled.print("Opt Path: ");
    for(int i = 0; i < pathIndex; i++)
    {
      oled.print(path[i]);
    }
    oled.setCursor(0, 7);
    oled.print("Ready for Short Run!");
    delay(2000);

    runShortestPath();
  }
}
