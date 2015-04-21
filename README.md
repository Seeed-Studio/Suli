What's new with Suli2?
-----------------

Suli2现阶段被做为Seeed内部开发模块驱动的策略基础, 以求达到代码重用的目的. 即当需要将某模块适配到新平台时, 只需要移植相应平台的Suli2, 理论上基于Suli2编写的库只需少量改动或不改动就可以适配新平台了.

- Suli2中大量采用了宏定义和inline函数的方式实现各API,以期达到更少的资源占用和最高的效率
- 同时统一第一个参数均为输出参数, 便于记忆和使用
- 重命名了部分API
- 增加了PWM相关的API
- 合并多个平台的suli到一个头文件和一个源文件, 便于维护和分发

Suli2在Arduino平台的库中可以每个库都包含.

Suli2在mbed平台的使用: 建议将所有库文件放在一级目录, 只保留一份suli2.h和suli2.cpp即可.

将Suli2的库向c平台移植时, 需要将.cpp后辍改成.c.



<br>
APIs 
---------------------------

**IO related APIs**

	void suli_pin_init(IO_T *, PIN_T, PIN_DIR)	
	void suli_pin_dir(IO_T *, PIN_DIR )
	void suli_pin_write(IO_T *, PIN_HIGH_LOW)
	int suli_pin_read(IO_T *)
	uint32_t suli_pin_pulse_in(IO_T *, what_state, timeout)
	
	
	
IO_T * : Output parameter, you need to define a global variable with the type IO_T and pass in the pointer of it.The source type of IO_T varies with different platforms. e.g. Under mbed, gpio_t is defined into IO_T. So users need to define the global variable uniformly with the IO_T type to make the high level driver adapt with different platforms without code modification.
	         
PIN_T : Input parameter, actually this parameter is usually an integer.

PIN_DIR : Input parameter, varies with different platforms.  
Users need to pass in one of the following macros: SULI_INPUT, SULI_OUTPUT.

PIN_HIGH_LOW :  Should be one of the following macros: SULI_HIGH, SULI_LOW.
 

**Analog related APIs**

	void suli_analog_init(ANALOG_T * aio, PIN_T pin);
	uint16_t suli_analog_read(ANALOG_T * aio);
	
ANALOG_T * : Output, need to use the unified ANALOG_T type to make the high level driver adapt with different platforms without code modification.

**PWM related APIs**

	void suli_pwm_init(PWM_T *, int pin)
	void suli_pwm_frequency(PWM_T *, uint32_t Hz)
	void suli_pwm_output(PWM_T *, float percent)
	
PWM_T * : Output, need to use the unified PWM_T type.

**Time related APIs**

	void suli_delay_us(uint32 us)
	void suli_delay_ms(uint32 ms)
	uint32 suli_millis(void)
	uint32 suli_micros(void)

**I2C Interface related APIs**

	void suli_i2c_init(I2C_T *i2c_device, int pin_sda, int pin_clk)
	uint8_t suli_i2c_write(I2C_T *i2c_device, uint8_t dev_addr, uint8_t *data, int len)
	uint8_t suli_i2c_read(I2C_T *i2c_device, uint8_t dev_addr, uint8_t *buff, int len)

**UART related APIs**

	void suli_uart_init(UART_T *, int pin_tx, int pin_rx, uint32_t baud)
	void suli_uart_write(UART_T *uart, uint8_t data)
	int suli_uart_write_bytes(UART_T *uart, uint8_t *data, int len)
	void suli_uart_write_float(UART_T *uart, float float_num, int decimal=2)
	void suli_uart_write_int(UART_T *uart, int32_t num)
	uint8_t suli_uart_read(UART_T *uart)
	int suli_uart_read_bytes(UART_T *uart, uint8_t *buff, int len)
	int suli_uart_read_bytes_timeout(UART_T *uart, uint8_t *buff, int len, int timeout_ms=1000)
	int suli_uart_readable(UART_T *uart)



Any questions please email author xuguang.shao@seeed.cc






[![Analytics](https://ga-beacon.appspot.com/UA-46589105-3/suli2)](https://github.com/igrigorik/ga-beacon)
