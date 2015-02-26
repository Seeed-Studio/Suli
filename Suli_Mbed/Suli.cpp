// seeedstudio grove hal.cpp for mbed
// 2014-3-5

#include "mbed.h"
#include "Suli.h"


/*
 * IO initialize
 * *pio - IO
 * pin - pin name
 */
void suli_pin_init(IO_T *pio, PIN_T pin)
{
    gpio_init(pio, pin);
}


/*
 * set IO direction
 * - pio: IO device pointer
 * - dir: INPUT or OUTPUT
 */
void suli_pin_dir(IO_T *pio, DIR_T dir)
{
    gpio_dir(pio, dir);
}


/*
 * write to IO
 * - pio: IO device pointer
 * - state: HIGH or LOW
 */
void suli_pin_write(IO_T *pio, int16 state)
{
    gpio_write(pio, state);
}


/*
 * read IO state
 * - pio: IO device pointer
 * return HIGH or LOW
 */
int16 suli_pin_read(IO_T *pio)
{
    return gpio_read(pio);
}


/**
 * Reads a pulse (either HIGH or LOW) on a pin. For example, if value is HIGH, 
 * suli_pulse_in() waits for the pin to go HIGH, starts timing, 
 * then waits for the pin to go LOW and stops timing. Returns the length of the pulse in microseconds. 
 * Gives up and returns 0 if no pulse starts within a specified time out.
 * para -
 * pin: pins which you want to read the pulse.
 * state: type of pulse to read: either HIGH or LOW. (int)
 * timeout (optional): the number of microseconds to wait for the pulse to start; default is one second (unsigned long)
 */
uint16 suli_pulse_in(uint8 pin, uint8 state, uint32 timeout)
{
    // add code here
    return 0;
}


/*
 * Analog Init
 * - aio: gpio device pointer
 * - pin: pin name
 */
void suli_analog_init(ANALOG_T * aio, PIN_T pin)
{
		analogin_init(aio, pin);
}
	
    
/*
 * Analog Read
 * As usually, 10bit ADC is enough, to increase the compatibility, will use only 10bit.
 * if if your ADC is 12bit, you need to >>2, or your ADC is 8Bit, you need to <<2
 */
int16 suli_analog_read(ANALOG_T * aio)
{
		return (analogin_read_u16(aio)>>6);
}


/*
 * delay us
 */
void suli_delay_us(uint32 us)
{
    wait_us(us);
}


/*
 * delay ms
 */
void suli_delay_ms(uint32 ms)
{
    wait_ms(ms);
}


/*
 * Returns the number of milliseconds since your board began running the current program. 
 * This number will overflow (go back to zero), after approximately 50 days.
 */
uint32 suli_millis()
{
    // add code here
    return us_ticker_read()/1000;
}


/*
 * Returns the number of microseconds since your board began running the current program. 
 * This number will overflow (go back to zero), after approximately 70 minutes.
 * Note: there are 1,000 microseconds in a millisecond and 1,000,000 microseconds in a second.
 */
uint32 suli_micros()
{
    // add code here
    return us_ticker_read();
}


/*
 * I2C interface initialize. 
 */
void suli_i2c_init(void * i2c_device)
{
    // i2c_device
}


/*
 * write a buff to I2C
 * - i2c_device: i2c device pointer
 * - dev_addr: device address
 * - data: data buff
 * - len: data lenght
 */
uint8 suli_i2c_write(void * i2c_device, uint8 dev_addr, uint8 *data, uint8 len)
{
    ((I2C *)i2c_device)->write(dev_addr, (const char*)data, len);
    return 0;
}


/*
 * read a buff to I2C
 * - i2c_device: i2c device pointer
 * - dev_addr: device address
 * - data: data buff
 * - len: data lenght
 * return
 */
uint8 suli_i2c_read(void * i2c_device, uint8 dev_addr, uint8 *buff, uint8 len)
{
    uint8 err = ((I2C *)i2c_device)->read(dev_addr, (char*)buff, len);
    return err ? 0 : len;
}


/*
 * UART Init
 * - uart_device: uart device pointer
 * - uart_num: for some MCU, there's more than one uart, this is the number of uart
 * - baud： baudrate
 */
void suli_uart_init(void * uart_device, int16 uart_num, uint32 baud)
{
    ((Serial*)uart_device) -> baud(baud);
}


/*
 * Send a Buff to uart
 * - uart_device: uart device pointer
 * - uart_num: uart number
 * - *data: buff to sent
 * - len: data length
 */
void suli_uart_send(void * uart_device, int16 uart_num, uint8 *data, uint16 len)
{
    for(int i=0; i<len; i++)
    {
        ((Serial*)uart_device) ->putc(data[i]);
    }
}


/*
 * seed a byte to uart
 */
void suli_uart_send_byte(void * uart_device, int16 uart_num, uint8 data)
{
    ((Serial*)uart_device) -> putc(data);
}


/*
 * read a byte from uart
 */
uint8 suli_uart_read_byte(void * uart_device, int16 uart_num)
{
    return ((Serial*)uart_device) -> getc();
}


/*
 * if uart get data, return 1-readable, 0-unreadable
 */
uint16 suli_uart_readable(void * uart_device, int16 uart_num)
{
    return ((Serial*)uart_device) -> readable();
}


/*
 * write a float
 * num - number to write
 * decimal - x decimal point
 */
void suli_uart_write_float(void * uart_device, int16 uart_num, float num, uint8 decimal)
{
    char str[5];
    sprintf(str,"%%.%df", decimal);
    ((Serial*)uart_device) -> printf(str, num);
}


/*
 * write an integer
 * num - number to write
 */
void suli_uart_write_int(void * uart_device, int16 uart_num, int32 num)
{
    ((Serial*)uart_device) -> printf("%ld", num);
}
