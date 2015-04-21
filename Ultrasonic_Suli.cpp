/*
 * Ultrasonic_Suli.cpp
 * A library for ultrasonic ranger
 *
 * Copyright (c) 2012 seeed technology inc.
 * Website    : www.seeed.cc
 * Author     : jacobyan
 * Create Time: AUG,2014
 * Change Log :
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "Ultrasonic_Suli.h"



IO_T  ioUltrasonic;

void Ultrasonic_init(int pin)
{
  suli_pin_init(&ioUltrasonic, pin, SULI_OUTPUT);
}


/*The measured distance from the range 0 to 400 Centimeters*/
uint32_t MeasureInCentimeters(void)
{

  suli_pin_dir(&ioUltrasonic,SULI_OUTPUT);
  suli_pin_write(&ioUltrasonic,SULI_LOW);
  suli_delay_us(2);
  suli_pin_write(&ioUltrasonic,SULI_HIGH);
  suli_delay_ms(5);
  suli_pin_write(&ioUltrasonic,SULI_LOW);
  suli_pin_dir(&ioUltrasonic,SULI_INPUT);
  long duration;
  duration=suli_pin_pulse_in(&ioUltrasonic,SULI_HIGH,10000);
  long RangeInCentimeters;
  RangeInCentimeters = duration/29/2;
  return RangeInCentimeters;
}


/*The measured distance from the range 0 to 157 Inches*/
uint32_t MeasureInInches(void)
{
  suli_pin_dir(&ioUltrasonic,SULI_OUTPUT);
  suli_pin_write(&ioUltrasonic,SULI_LOW);
  suli_delay_us(2);
  suli_pin_write(&ioUltrasonic,SULI_HIGH);
  suli_delay_ms(5);
  suli_pin_write(&ioUltrasonic,SULI_LOW);
  suli_pin_dir(&ioUltrasonic,SULI_INPUT);
  long duration;
  duration=suli_pin_pulse_in(&ioUltrasonic,SULI_HIGH,10000);
  long RangeInInches;
  RangeInInches = duration/74/2;
  return RangeInInches;
}

I2C_T i2c;

void test_i2c(int pin_sda, int pin_scl)
{
  suli_i2c_init(&i2c, pin_sda, pin_scl);
  char *str = "hello";
  suli_i2c_write(&i2c, 0x00, (uint8_t *)str, 5);
}

ANALOG_T analog;

int test_analog(int pin)
{
  suli_analog_init(&analog, pin);
  return suli_analog_read(&analog);
}

PWM_T pwm;

void test_pwm(int pin)
{
  suli_pwm_init(&pwm, pin);
  suli_pwm_frequency(&pwm, 1000);
  suli_pwm_output(&pwm, 0.5);
}

UART_T uart;
void test_uart(int pin_tx, int pin_rx)
{
  suli_uart_init(&uart, pin_tx, pin_rx, 115200);
  suli_uart_write_float(&uart, 100.1234);
}
