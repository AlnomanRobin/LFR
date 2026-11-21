void motor(int left,int right)
{
  digitalWrite(LFWD, left > 0);
  digitalWrite(LBWD, left < 0);

  digitalWrite(RFWD, right > 0);
  digitalWrite(RBWD, right < 0);

  analogWrite(LPWM, constrain(abs(left), 0, 400));
  analogWrite(RPWM, constrain(abs(right), 0, 400));
}

void stopMotor()
{
  motor(0, 0);
}

void brakeMotor()
{
  // Electronic H-bridge short brake (safe regardless of current spin direction)
  digitalWrite(LFWD, HIGH);
  digitalWrite(LBWD, HIGH);
  digitalWrite(RFWD, HIGH);
  digitalWrite(RBWD, HIGH);
  analogWrite(LPWM, 400);
  analogWrite(RPWM, 400);
  delay(20);

  stopMotor();
}

void motorTest()
{
  oled.clear();

  const char* stages[] = {
    "LEFT FORWARD",
    "LEFT BACKWARD",
    "RIGHT FORWARD",
    "RIGHT BACKWARD",
    "BOTH FORWARD",
    "BOTH BACKWARD"
  };

  int testSpeeds[6][2] = {
    {180, 0},
    {-180, 0},
    {0, 180},
    {0, -180},
    {180, 180},
    {-180, -180}
  };

  for(byte s = 0; s < 6; s++)
  {
    // Inverted Header
    oled.set1X();
    oled.setCursor(0, 0);
    oled.setInvertMode(true);
    oled.print("  MOTOR DIAGNOSTIC  ");
    oled.clearToEOL();
    oled.setInvertMode(false);

    oled.setCursor(0, 2);
    oled.print("Stage [");
    oled.print(s + 1);
    oled.print("/6]:");
    oled.clearToEOL();

    oled.setCursor(0, 4);
    oled.setInvertMode(true);
    oled.print(" > ");
    oled.print(stages[s]);
    oled.clearToEOL();
    oled.setInvertMode(false);

    oled.setCursor(0, 7);
    oled.print("[Press OK to Abort]");
    oled.clearToEOL();

    motor(testSpeeds[s][0], testSpeeds[s][1]);

    unsigned long start = millis();
    while(millis() - start < 1200)
    {
      if(button(BTN_OK))
      {
        stopMotor();
        return;
      }
    }
    brakeMotor();
    delay(300);
  }

  stopMotor();
}
