void inverseModeRun()
{
  oled.clear();
  oled.set1X();
  oled.setCursor(0, 0);
  oled.setInvertMode(true);
  oled.print(" INVERSE LINE MODE  ");
  oled.clearToEOL();
  oled.setInvertMode(false);

  oled.setCursor(0, 3);
  oled.print("Tracking: White Line");
  oled.setCursor(0, 7);
  oled.print("[Press OK to Stop]");

  inverseMode = true;

  while(!button(BTN_OK))
  {
    calculatePID();
  }

  stopMotor();
  inverseMode = false;
}