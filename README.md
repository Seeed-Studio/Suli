#What is Suly
-----------------
Sounds like a girl’s name. Actually it means Seeed Unified Library Interface. It’s our big thing to do in the near future. 

We have hundreds of sensors, actuators, displays and the other useful module. In the past time, we mainly focus on Arduino application, and we only supplied Arduino Library. So if you use an another platform such as Raspberry, LauchPad, or MCS51, you need to porting our Library, and sometimes it's not an easy work. 

Thus sometime we will get some trouble, some custom will ask us to supply some library for a non-Arduino platform, What we can do is telling them, hey man, we have Arduino library only. Every time I talk about this I feel shame. 

So, what I think is that if there's a way can make our libraries to fit all platform. It’s reaaly a hard job. But so luchy that we find it out funally, it's Suli - Seeed Unified Library Interface, which can make all of(at least 90%) our library fit many common platform, or at least make it easy to porting. 





<br>
#How does Suly works
---------------------------
Have a look at the following image,

![](http://www.seeedstudio.com/wiki/images/b/b1/Suly_layer.jpg)

We can find that there are three layer and the lowest layer is Suli. Then Libray layer, then Application layer. All of our Libary will use Suli only. Thus, for different platform, we can just porting Suli, then the Library can be used. That means different platform has their Suli, Suli for Arudino, Suli for Mbed, Suli for.... 

Thres are serval points about Suli:

- Different platform have different Suli, it’s for a particular platform.
- Suli-compatible Library is platform independent.
- Suli-compatible Library will call the functon in Suli.

Suli has two files, Suli.h and Suli.cpp(.c). And Suli should implement the following function,

GPIO type:

	typedef     int     IO_T;                           // IO type
	typedef     int     PIN_T;                          // pin name
	typedef     int     DIR_T;                          // pin direction
	typedef int ANALOG_T;                               // pin analog

GPIO state:

	#define HAL_PIN_INPUT   INPUT          // INPUT and OUTPUT was declared in Arduino IDE
	#define HAL_PIN_OUTPUT  OUTPUT
	#define HAL_PIN_HIGH    HIGH           // HIGH and LOW was declered in Arduino IDE
	#define HAL_PIN_LOW     LOW


Data type:

	typedef signed char     int8;
	typedef unsigned char   uint8;
	typedef signed short    int16;
	typedef unsigned short  uint16;
	typedef signed long     int32;
	typedef unsigned long   uint32;

Digital IO Operation:

	void seeed_hal_pin_init(IO_T *pio, PIN_T pin);      // pin initialize
	void seeed_hal_pin_dir(IO_T *pio, DIR_T dir);       // set pin direction
	void seeed_hal_pin_write(IO_T *pio, int16 state);   // write pin
	int16 seeed_hal_pin_read(IO_T *pio);                // read pin
	uint16 seeed_hal_pulse_in(IO_T *pio, uint8 state, uint32 timeout);

Analog IO Operation:

	void seeed_hal_analog_init(ANALOG_T * aio, PIN_T pin);
	int16 seeed_hal_analog_read(ANALOG_T * aio);

Delay

	void seeed_hal_delay_us(uint32 us);                 // delay us
	void seeed_hal_delay_ms(uint32 ms);                 // delay ms

Times

	uint32 seeed_hal_millis(void);
	uint32 seeed_hal_micros(void);

I2C Interface:

	void seeed_hal_i2c_init(void * i2c_device);
	uint8 seeed_hal_i2c_write(void * i2c_device, uint8 dev_addr, uint8 *data, uint8 len);
	uint8 seeed_hal_i2c_read(void * i2c_device, uint8 dev_addr, uint8 *buff, uint8 *len);

UART:

	void seeed_hal_uart_init(void * uart_device, int16 uart_num, uint32 baud);
	void seeed_hal_uart_send(void * uart_device, int16 uart_num, uint8 *data, uint16 len);
	void seeed_hal_uart_send_byte(void * uart_device, int16 uart_num, uint8 data);
	uint8 seeed_hal_uart_read_byte(void * uart_device, int16 uart_num);
	uint16 seeed_hal_uart_readable(void * uart_device, int16 uart_num);

You can find that Suli include digital IO operation, analog IO operation, I2C, and Uart function, maybe it’s not very comprehensive, but base on my experience, it’s enought for quite a lot of our libraries, of curse if we find that we need more funciton someday, we’ll add to it. 





<br>
#What is next
---------------------------
Our plan is make all library of Seeed to Suli compatible and porting some common platform Suli, this’s platform will include: Arduino, Mbed, Raspberry, LaunchPad in the first stage. This work may be finished before June. 






<br>

----

This code is written by luweicong@seeedstudio.com for seeed studio<br>
and is licensed under [The MIT License](http://opensource.org/licenses/mit-license.php). Check License.txt for more information.<br>

Seeed Studio is an open hardware facilitation company based in Shenzhen, China. <br>
Benefiting from local manufacture power and convenient global logistic system, <br>
we integrate resources to serve new era of innovation. Seeed also works with <br>
global distributors and partners to push open hardware movement.<br>




[![Analytics](https://ga-beacon.appspot.com/UA-46589105-3/Tick_Tock_Shield_V2)](https://github.com/igrigorik/ga-beacon)