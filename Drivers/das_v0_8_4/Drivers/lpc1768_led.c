/** 
  * @file lpc1768_led.c
  * @brief Driver for onboard LEDs on mbed module with LPC1768 Microcontroller
  */
  
#include "lpc1768_led.h"

/* ================ Public Functions ================ */

/**
  * Initializaton of pins as output
  * @brief Should be first function call, before using other LED functions
  * @param void
  * @return void
  */
void LED_init()
{
	GPIO_PinDirection(PORT_1,PIN_18, OUTPUT);
	GPIO_PinDirection(PORT_1,PIN_20, OUTPUT);
	GPIO_PinDirection(PORT_1,PIN_21, OUTPUT);
	GPIO_PinDirection(PORT_1,PIN_23, OUTPUT);
	
	GPIO_PinFunction(PORT_1,PIN_18, GPIO_PINSEL_FUNCTION_0);
	GPIO_PinFunction(PORT_1,PIN_20, GPIO_PINSEL_FUNCTION_0);
	GPIO_PinFunction(PORT_1,PIN_21, GPIO_PINSEL_FUNCTION_0);
	GPIO_PinFunction(PORT_1,PIN_23, GPIO_PINSEL_FUNCTION_0);	
}	

/**
  * Turns on LED with specified number
  * @param LED_no - unsigned integer, number of LED - 0,1,2 or 3
  * @return void
  */	
void LED_on(uint8_t LED_no)
{
	if (LED_no < LED_MAX)
	{
		if (LED_no == LED_0)
			GPIO_PinWrite(PORT_1,PIN_18, HIGH);
		else if (LED_no == LED_1)
			GPIO_PinWrite(PORT_1,PIN_20, HIGH);
		else if (LED_no == LED_2)
			GPIO_PinWrite(PORT_1,PIN_21, HIGH);
		else
			GPIO_PinWrite(PORT_1,PIN_23, HIGH);
	}
}

/**
  * Turns off LED with specified number
  * @param LED_no - unsigned integer, number of LED - 0,1,2 or 3
  * @return void
  */
void LED_off(uint8_t LED_no)
{
	if (LED_no < LED_MAX)
	{
		if (LED_no == LED_0)
			GPIO_PinWrite(PORT_1,PIN_18, LOW);
		else if (LED_no == LED_1)
			GPIO_PinWrite(PORT_1,PIN_20, LOW);
		else if (LED_no == LED_2)
			GPIO_PinWrite(PORT_1,PIN_21, LOW);
		else
			GPIO_PinWrite(PORT_1,PIN_23, LOW);
	}
}

/* End of file lpc1768_led.h */
