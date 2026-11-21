int lastPosition = 0;

void calculatePIDAdvanced()
{
  digitalSensor();

  long numerator = 0;
  long denominator = 0;

  int weight[6] =
  {
    -500,
    -300,
    -100,
     100,
     300,
     500
  };

  for(byte i=0; i<6; i++)
  {
    if(sensorDigital[i])
    {
      numerator += weight[i];
      denominator++;
    }
  }

  int position;

  if(denominator == 0)
  {
    position = lastPosition;
  }
  else
  {
    position = numerator / denominator;
    lastPosition = position;
  }

  error = position;

  float derivative = error - lastError;

  integral = constrain(integral + error, -2000, 2000);

  // Position is scaled by 100, so divide by 100 to match PID constants
  float correction = ((Kp * error) + (Ki * integral) + (Kd * derivative)) / 100.0;

  int leftMotorSpeed = constrain(baseSpeed + (int)correction, -255, 255);
  int rightMotorSpeed = constrain(baseSpeed - (int)correction, -255, 255);

  motor(leftMotorSpeed, rightMotorSpeed);

  lastError = error;
}
