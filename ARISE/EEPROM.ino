#define EEPROM_MAGIC     0x5A
#define EEPROM_PID_MAGIC 0x5C

void saveCalibration()
{
  EEPROM.update(100, EEPROM_MAGIC);
  for(byte i=0; i<6; i++)
  {
    EEPROM.put(i*2, sensorMid[i]);
  }
}

void loadCalibration()
{
  if(EEPROM.read(100) == EEPROM_MAGIC)
  {
    for(byte i=0; i<6; i++)
    {
      EEPROM.get(i*2, sensorMid[i]);

      // Safety clamp against invalid EEPROM values
      if(sensorMid[i] < 50 || sensorMid[i] > 950)
      {
        sensorMid[i] = 512;
      }
    }
  }
  else
  {
    // Default fallback midpoint if uncalibrated
    for(byte i=0; i<6; i++)
    {
      sensorMid[i] = 512;
    }
  }
}

void savePIDSettings()
{
  EEPROM.update(101, EEPROM_PID_MAGIC);
  EEPROM.put(20, baseSpeed);
  EEPROM.put(24, Kp);
  EEPROM.put(28, Kd);
}

void loadPIDSettings()
{
  if(EEPROM.read(101) == EEPROM_PID_MAGIC)
  {
    EEPROM.get(20, baseSpeed);
    EEPROM.get(24, Kp);
    EEPROM.get(28, Kd);

    // Sanity checks against invalid EEPROM data
    if(baseSpeed < 100 || baseSpeed > 255) baseSpeed = 210;
    if(Kp < 5.0 || Kp > 60.0) Kp = 25.0;
    if(Kd < 5.0 || Kd > 60.0) Kd = 22.0;
  }
  else
  {
    // Default values if never tuned
    baseSpeed = 210;
    Kp = 25.0;
    Kd = 22.0;
  }
}
