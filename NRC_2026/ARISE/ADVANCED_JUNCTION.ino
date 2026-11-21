bool confirmNode()
{
  byte count=0;

  for(byte i=0;i<5;i++)
  {
    digitalSensor();

    byte sum=0;

    for(byte j=0;j<6;j++)
    {
      sum+=sensorDigital[j];
    }

    if(sum>=4)
    {
      count++;
    }

    delay(2);
  }

  return (count>=3);
}