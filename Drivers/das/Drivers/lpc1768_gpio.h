
#ifndef LPC1768_GPIO_H
#define LPC1768_GPIO_H


//====================== MC include file ============
#include "lpc17xx.h"
//===================================================



//================ GPIO pin initalization macros =====


//  Pin Function select values
#define GPIO_PINSEL_FUNCTION_0            	((uint32_t) 0x00)
#define GPIO_PINSEL_FUNCTION_1							((uint32_t) 0x01)
#define GPIO_PINSEL_FUNCTION_2							((uint32_t) 0x02)
#define GPIO_PINSEL_FUNCTION_3							((uint32_t) 0x03)


//  Pin mode (pull-up/pull-down)
#define GPIO_PINMODE_PULLUP											((uint32_t) 0x00)
#define GPIO_PINMODE_REPEATER										((uint32_t) 0x01)
#define GPIO_PINMODE_NOPULLUPNOPULLDOWN					((uint32_t) 0x02)
#define GPIO_PINMODE_PULLDOWN										((uint32_t) 0x03)

//  Pin open drain mode
#define GPIO_PINMODE_OPENDRAIN                  ((uint32_t) 0x01)

//  GPIO port addresses
#define GPIO_PORT0                              (LPC_GPIO0)
#define GPIO_PORT1                              (LPC_GPIO1)
#define GPIO_PORT2                              (LPC_GPIO2)
#define GPIO_PORT3                              (LPC_GPIO3)
#define GPIO_PORT4                              (LPC_GPIO4)

//  GPIO pin direction
#define OUTPUT						1
#define INPUT 						0

//  pin value
#define HIGH              1
#define LOW               0

//  bit manipulation macros
#define  isBitSet(x,bit)          (((x)&(1 << bit)))!= 0u)   // if bit = 1 return true


#define BIT_15 15


// =========== GPIO pin settings structure ====================
typedef struct
{
	uint32_t port;
	
	uint32_t pin;               // GPIO pin to be configured
	
	uint32_t function;
	
	uint32_t mode;
	
	uint32_t direction;
	
//	uint32_t opdrain;

} GPIO_pin_settings;

//========================= GPIO pins ===========================
typedef enum
{
    PORT_0, PORT_1, PORT_2, PORT_3, PORT_4, PORT_MAX,
  
	
}GPIO_Port; 

typedef enum
{
	PIN_0,PIN_1, PIN_2, PIN_3,PIN_4,PIN_5, PIN_6, PIN_7,PIN_8,PIN_9, PIN_10, PIN_11,PIN_12,PIN_13, PIN_14, PIN_15,
	
	PIN_16,PIN_17, PIN_18, PIN_19,PIN_20,PIN_21, PIN_22, PIN_23,PIN_24,PIN_25, PIN_26, PIN_27,PIN_28,PIN_29, PIN_30, PIN_31, PIN_MAX
	
} GPIO_pins;

	

//============== Driver APIs ===================================

void GPIO_PinInit(GPIO_pin_settings pin_config); 

void GPIO_PinFunction( uint8_t port_no, uint16_t pin_no, uint32_t func);

void GPIO_PinDirection(uint8_t port_no, uint16_t pin_no, uint8_t direction);

uint8_t GPIO_PinRead(uint8_t port_no, uint16_t pin_no);

void GPIO_PinWrite( uint8_t port_no, uint16_t pin_no, uint8_t value);




#endif //LPC1768_GPIO_H
