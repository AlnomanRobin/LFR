void autoSpeed()
{
  if(
      sensorDigital[2] &&
      sensorDigital[3] &&
      !sensorDigital[1] &&
      !sensorDigital[4]
    )
  {
      baseSpeed = 250;
  }
  else
  {
      baseSpeed = 200;
  }
}