/***************************************************************************/
//  Function: Header file for Ultrasonic Ranger
//  Hardware: Grove - Ultrasonic Ranger
//  Arduino IDE: Arduino-1.0
//  Author:  LG
//  Date:    Jan 17,2013
//  Version: v1.0 modified by FrankieChu
//  by www.seeedstudio.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
//
/*****************************************************************************/

#ifndef __Ultrasonic_H__
#define __Ultrasonic_H__

#include "suli2.h"
#include "Ultrasonic_Suli.h"

class Ultrasonic
{
public:
    Ultrasonic(int pin)
    {
        Ultrasonic_init(pin);
    }

    inline uint32_t MeasureInCentimeters(void)
    {
        return MeasureInCentimeters();
    }

    inline uint32_t MeasureInInches(void)
    {
        return MeasureInInches();
    }

    inline void test_i2c(int pin_sda, int pin_scl)
    {
        test_i2c(pin_sda, pin_scl);
    }

    inline int test_analog(int pin)
    {
        test_analog(pin);
    }

    inline void test_pwm(int pin)
    {
        test_pwm(pin);
    }

    inline void test_uart(int pintx, int pinrx)
    {
        test_uart(pintx, pinrx);
    }

};



#endif
