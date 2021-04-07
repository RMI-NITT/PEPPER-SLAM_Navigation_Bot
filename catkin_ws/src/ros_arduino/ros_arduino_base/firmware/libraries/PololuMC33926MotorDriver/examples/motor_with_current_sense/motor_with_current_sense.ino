#include "PololuMC33926.h"

//Pins need to configured for board being used.
MC33926 motor(2,3,5,4,6);

void setup()
{
  Serial.begin(9600);
  motor.init();
  motor.flip_motor_direction();
}

void loop()
{
  if(!motor.fault())
  {
    for (int x = 0; x < 256;x++)
    {
      motor.set_pwm(x);
      Serial.println(motor.motor_current());
      delay(25);
    }
  }
}

