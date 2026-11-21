void safeUTurn()
{
  brakeMotor();
  delay(20);

  // Stage 1: Kick off dead-end area
  motor(-turnSpeed, turnSpeed);
  unsigned long start = millis();
  while((millis() - start < 160) || ((sensorDigital[2] || sensorDigital[3]) && (millis() - start < 500)))
  {
    digitalSensor();
  }

  // Stage 2: Keep rotating until center sensors lock onto the return line
  while(!sensorDigital[2] && !sensorDigital[3] && (millis() - start < 1100))
  {
    digitalSensor();
  }

  // Crisp counter-pulse brake to stop square on the line
  motor(turnSpeed, -turnSpeed);
  delay(25);
  stopMotor();

  lastError = 0;
  gapStartTime = 0;

  storePath('B');
}
