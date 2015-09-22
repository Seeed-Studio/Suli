#include "Suli.h"
#include "Arduino.h"
#include "Wire.h"
#include "SoftwareSerial.h"

void setup(){

	suli_uart_write_int((void*)Serial, 11,11);
}

void loop(){

}