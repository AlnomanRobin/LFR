bool blackArea()
{
  byte sum=0;

  for(byte i=0;i<6;i++)
  {
    sum+=sensorDigital[i];
  }

  if(sum==6)
  {
    return true;
  }

  return false;
}

void handleBlackArea()
{
   motor(200,200);

   delay(120);

   while(blackArea())
   {
      digitalSensor();
   }
}