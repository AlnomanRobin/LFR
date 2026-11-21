bool circleDetected()
{
   byte sum=0;

   for(byte i=0;i<6;i++)
   {
      sum+=sensorDigital[i];
   }

   return (sum>=5);
}

void handleCircle()
{
   motor(200,200);

   delay(180);

   while(circleDetected())
   {
      calculatePID();
   }
}
