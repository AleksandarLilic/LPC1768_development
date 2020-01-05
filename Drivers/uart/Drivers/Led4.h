#ifndef _led4_H
#define _led4_H

#include "gpio_driver.h"


void LED_init(void);

void LED_on(uint8_t LED_no);

void LED_off(uint8_t LED_no);


typedef enum
{
	LED_0,
	LED_1,
	LED_2,
	LED_3,
	
	LED_MAX
} LED_number;

#endif