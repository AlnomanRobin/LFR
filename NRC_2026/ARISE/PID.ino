/****************************************
   CHAMPIONSHIP PID & TRACK RECOVERY
****************************************/

void calculatePID()
{
  digitalSensor();

  int weight[6] = { -5, -3, -1, 1, 3, 5 };

  long sum = 0;
  byte activeCount = 0;

  for(byte i = 0; i < 6; i++)
  {
    if(sensorDigital[i])
    {
      sum += weight[i];
      activeCount++;
    }
  }

  // -------------------------------------------------------------
  // 1. GAP CROSSING & SHARP TURN RECOVERY (LINE LOST: activeCount == 0)
  // -------------------------------------------------------------
  if(activeCount == 0)
  {
    // A. Dashed Line / Gap on Straight Track:
    if(abs(lastError) < 2.2)
    {
      if(gapStartTime == 0) gapStartTime = millis();

      if(millis() - gapStartTime < GAP_TIMEOUT)
      {
        motor(gapSpeed, gapSpeed);
        return;
      }
    }

    // B. Sharp Turn Recovery (Pivot fast towards last known line side)
    if(lastError > 0)
    {
      motor(turnSpeed, -turnSpeed); // Rotate Right
    }
    else if(lastError < 0)
    {
      motor(-turnSpeed, turnSpeed); // Rotate Left
    }
    return;
  }

  // Line is acquired -> Reset gap timer
  gapStartTime = 0;

  // -------------------------------------------------------------
  // 2. POSITION & HIGH-SPEED ADAPTIVE BASE SPEED
  // -------------------------------------------------------------
  error = (float)sum / activeCount;

  int currentBase;
  if(abs(error) <= 1.2 && !sensorDigital[0] && !sensorDigital[5])
  {
    // Straight or near-straight (works even if only 1 center sensor sees the line)
    currentBase = min(255, baseSpeed + 25);
  }
  else if(abs(error) <= 1.8)
  {
    // Mild curve -> maintain full tuned baseSpeed
    currentBase = baseSpeed;
  }
  else
  {
    // Sharp curve -> reduce baseSpeed progressively so inside wheel can reverse cleanly
    currentBase = baseSpeed - (int)((abs(error) - 1.5) * 24);
    currentBase = constrain(currentBase, 90, baseSpeed);
  }

  // -------------------------------------------------------------
  // 3. PID FORMULATION
  // -------------------------------------------------------------
  integral = constrain(integral + error, -60, 60);

  float derivative = error - lastError;

  float PID = (Kp * error) + (Ki * integral) + (Kd * derivative);

  int left  = currentBase + (int)PID;
  int right = currentBase - (int)PID;

  left  = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  motor(left, right);

  lastError = error;
}
