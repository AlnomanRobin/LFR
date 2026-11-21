/****************************************
   CHAMPIONSHIP STRAIGHT RUN / RACE MODE
****************************************/

void lineFollow()
{
  runStartTime   = millis();
  gapStartTime   = 0;
  floodStartTime = 0;
  lastError      = 0;

  oled.clear();
  oled.set1X();
  oled.setCursor(0, 0);
  oled.setInvertMode(true);
  oled.print("  RACE MODE ACTIVE  ");
  oled.clearToEOL();
  oled.setInvertMode(false);

  oled.setCursor(0, 2);
  oled.print("Team: DIU Rocket Sq");
  oled.setCursor(0, 4);
  oled.print("Mode: High Speed PID");
  oled.setCursor(0, 7);
  oled.print("[Press OK to Stop]");

  bool finished = false;

  while(!button(BTN_OK))
  {
    // Auto-detect inverted sections and finish box
    if(checkAutoInverseAndFinish(finished))
    {
      if(finished)
      {
        brakeMotor();
        digitalWrite(LED_PIN, HIGH);

        oled.clear();
        oled.set1X();
        oled.setCursor(0, 0);
        oled.setInvertMode(true);
        oled.print("   CHAMPION RUN!    ");
        oled.clearToEOL();
        oled.setInvertMode(false);

        oled.set2X();
        oled.setCursor(20, 2);
        oled.print("WINNER!");

        oled.set1X();
        oled.setCursor(14, 5);
        oled.print("FINISH REACHED!");
        oled.setCursor(14, 7);
        oled.print("DIU_Rocket_Squad");

        delay(2500);
        digitalWrite(LED_PIN, LOW);
        break;
      }
      continue;
    }

    calculatePID();
  }

  stopMotor();
}
