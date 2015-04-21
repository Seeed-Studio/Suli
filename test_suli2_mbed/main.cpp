#include "mbed.h"
#include "Ultrasonic.h"


Ultrasonic ultra(P1_0);
Ultrasonic ultra2(P3_0);


int main() {
    uint32_t dist = ultra.MeasureInCentimeters();
    ultra.test_i2c(P1_1, P1_2);
    int ana = ultra.test_analog(P1_5);
    ultra2.test_pwm(P1_18);
    ultra.test_uart(P0_2, P0_3);
    return 0;
}

