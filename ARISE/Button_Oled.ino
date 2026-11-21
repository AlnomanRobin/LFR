void splash()
{
  oled.clear();

  // Inverted Top Banner
  oled.set1X();
  oled.setInvertMode(true);
  oled.setCursor(0, 0);
  oled.print("  DIU ROCKET SQUAD  ");
  oled.clearToEOL();
  oled.setInvertMode(false);

  // Big Centered Logo/Brand
  oled.set2X();
  oled.setCursor(46, 2);
  oled.print("DIU");

  // Subtitle
  oled.set1X();
  oled.setCursor(16, 5);
  oled.print("CHAMPIONSHIP LFR");

  // Animated Startup Loading Bar
  oled.setCursor(8, 7);
  oled.print("[");
  for(byte b = 0; b < 14; b++)
  {
    oled.print("=");
    delay(40);
  }
  oled.print("] OK");
  delay(350);
}

byte button(byte pin)
{
 if(!digitalRead(pin))
 {
   delay(20);

   while(!digitalRead(pin));

   return 1;
 }

 return 0;
}

void text(String t,int x,int y)
{
 oled.setCursor(x,y);
 oled.print(t);
}