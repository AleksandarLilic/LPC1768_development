/** 
  * @file lpc1768_led.h
  * @brief Driver for onboard LEDs on mbed module with LPC1768 Microcontroller
  */

/* ================ HEADER GUARD ================ */

#ifndef _LPC1768_LED_H
#define _LPC1768_LED_H

/* ================ HEADER INCLUDES ================ */

#include "lpc1768_gpio.h"

/* ================ Data Structures ================ */

/**
  * enum LED_Number
  * @brief Defines numbers/names for onboard LEDs on mbed module
  */
typedef enum
{
	LED_0,
	LED_1,
	LED_2,
	LED_3,	
	LED_MAX
} LED_number;

/* ================ Public Functions ================ */

/**
  * Initializaton of pins as output
  * @brief Should be first function call, before using other LED functions
  * @param void
  * @return void
  */
void LED_init(void);

/**
  * Turns on LED with specified number
  * @param LED_no - unsigned integer, number of LED - 0,1,2 or 3
  * @return void
  */
void LED_on(uint8_t LED_no);

/**
  * Turns off LED with specified number
  * @param LED_no - unsigned integer, number of LED - 0,1,2 or 3
  * @return void
  */
void LED_off(uint8_t LED_no);

#endif /* lpc1768_led.h */
