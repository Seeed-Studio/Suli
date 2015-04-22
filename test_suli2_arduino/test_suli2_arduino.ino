#include "Arduino.h"
#include "Wire.h"
#include "Ultrasonic.h"
#include "SoftwareSerial.h"

#define ARDUINO_USE_I2C             1
#define ARDUINO_USE_SOFTWARE_SERIAL 1

Ultrasonic ultra(2);

void setup()
{
    Serial.begin(9600);
}

void loop()
{
  uint32_t dist = ultra.MeasureInCentimeters();
  ultra.test_i2c(0,0);
  int ana = ultra.test_analog(A0);

  Serial.println(dist);

  ultra.test_pwm(9);

  ultra.test_uart(9, 10);

}
