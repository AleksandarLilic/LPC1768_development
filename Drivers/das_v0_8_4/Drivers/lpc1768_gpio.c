#include "lpc1768_gpio.h"


//==================== SELECT REGISTER FUNCTION =================
/* 
		port_no ---> PORT_0 ... PORT_4
	  pin_no ----> PIN_0 .... PIN_31
	  func ----> GPIO_PINSEL_FUNCTION_0, GPIO_PINSEL_FUNCTION_1,GPIO_PINSEL_FUNCTION_2,GPIO_PINSEL_FUNCTION_3
	 
=================================================================*/

void GPIO_PinFunction( uint8_t port_no, uint16_t pin_no, uint32_t func)
{
	uint32_t *ptr_PINCON;
	uint8_t sel_reg;
	uint16_t pin = pin_no;
	
	if ( (port_no < PORT_MAX) && (pin_no < PIN_MAX) && (func <= GPIO_PINSEL_FUNCTION_3)) //Handle the request only if it is in range
	{
		
		switch (port_no)             // select PINSEL reg
		{
			case 0x00:
				sel_reg = 0x00u;
				break;
			
			case 0x01:
				sel_reg = 0x02u;
				break;
			
			case 0x02:
				sel_reg = 0x04u;
				break;
			
			case 0x03:
				sel_reg = 0x06u;
				break;
			
			case 0x04:
				sel_reg = 0x08u;
				break;
		}
		
		if (pin_no > BIT_15)    // PIN 16 - 31   select next register and start from bit 0
		{
			sel_reg++;
			pin = pin - 16;
		}		
		
		pin = pin << 1; //multiply by 2 for shifting (function is selected with 2 bits)
		
		ptr_PINCON =  ((uint32_t *)&LPC_PINCON ->PINSEL0 + sel_reg);  /// PINSEL address
		
		*(uint32_t *)(ptr_PINCON) &= ~(0x03UL << pin); // clear function bits
    *(uint32_t *)(ptr_PINCON) |= ((func) << pin); 	// select function 
		
	}
}

//==================== SELECT PIN DIRECTION =================
/* 
		port_no ---> PORT_0 ... PORT_4
	  pin_no ----> PIN_0 .... PIN_31
	  direction ----> OUTPUT, INPUT
	 
=================================================================*/


void GPIO_PinDirection(uint8_t port_no, uint16_t pin_no, uint8_t direction)
{
	
	if ( (port_no < PORT_MAX) && (pin_no < PIN_MAX)) //Handle the request only if it is in range
	{
		
		switch (port_no)             // select PINSEL reg
		{
			case 0x00:
				LPC_GPIO0 -> FIODIR &= ~( 1 << pin_no); //clear FIODIR bit for selected pin
				LPC_GPIO0 -> FIODIR |= (direction << pin_no); // 
				break;
			
			case 0x01:
				LPC_GPIO1 -> FIODIR &= ~( 1 << pin_no); //clear FIODIR bit for selected pin
				LPC_GPIO1 -> FIODIR |= (direction << pin_no); //
				break;
			
			case 0x02:
				LPC_GPIO2 -> FIODIR &= ~( 1 << pin_no); //clear FIODIR bit for selected pin
				LPC_GPIO2 -> FIODIR |= (direction << pin_no); //
				break;
			
			case 0x03:
				LPC_GPIO3 -> FIODIR &= ~( 1 << pin_no); //clear FIODIR bit for selected pin
				LPC_GPIO3 -> FIODIR |= (direction << pin_no); //
				break;
			
			case 0x04:
				LPC_GPIO4 -> FIODIR &= ~( 1 << pin_no); //clear FIODIR bit for selected pin
				LPC_GPIO4 -> FIODIR |= (direction << pin_no); //
				break;
		}
	}
}

//==================== READ PIN VALUE =================
/* 
		port_no ---> PORT_0 ... PORT_4
	  pin_no ----> PIN_0 .... PIN_31
	 
	 
=================================================================*/

uint8_t GPIO_PinRead(uint8_t port_no, uint16_t pin_no)
{
	uint8_t value=0;
	
	if ( (port_no < PORT_MAX) && (pin_no < PIN_MAX)) //Handle the request only if it is in range
	{
		
		switch (port_no)             // select PINSEL reg
		{
			case 0x00:
				value = ((isBitSet(LPC_GPIO0 -> FIOPIN, pin_no)? ((uint8_t) 0x01) : ((uint8_t) 0x00)); // read value on pin from FIOPIN reg
				break;
			
			case 0x01:
				value = ((isBitSet(LPC_GPIO1 -> FIOPIN, pin_no)? ((uint8_t) 0x01) : ((uint8_t) 0x00)); // read value on pin from FIOPIN reg
				break;
			
			case 0x02:
				value = ((isBitSet(LPC_GPIO2 -> FIOPIN, pin_no)? ((uint8_t) 0x01) : ((uint8_t) 0x00)); // read value on pin from FIOPIN reg
				break;
			
			case 0x03:
				value = ((isBitSet(LPC_GPIO3 -> FIOPIN, pin_no)? ((uint8_t) 0x01) : ((uint8_t) 0x00)); // read value on pin from FIOPIN reg
				break;
			
			case 0x04:
				value = ((isBitSet(LPC_GPIO4 -> FIOPIN, pin_no)? ((uint8_t) 0x01) : ((uint8_t) 0x00)); // read value on pin from FIOPIN reg
				break;
		}
		
	}
	return value;
}


void GPIO_PinWrite( uint8_t port_no, uint16_t pin_no, uint8_t value)
{
	LPC_GPIO_TypeDef *LPC_GPIO_PORT;

	if ( (port_no < PORT_MAX) && (pin_no < PIN_MAX)) //Handle the request only if it is in range
	{
		
		LPC_GPIO_PORT = (LPC_GPIO_TypeDef *) ( LPC_GPIO_BASE + (port_no << 5)); //select GPIOx base addresss
		
		if (value == 1)
			LPC_GPIO_PORT -> FIOSET |= (1 << pin_no);
		else
			LPC_GPIO_PORT -> FIOCLR |= (1 << pin_no);
	}
}

void GPIO_PinInit(GPIO_pin_settings pin_config)
{
		GPIO_PinFunction(pin_config.port, pin_config.pin, pin_config.function);
		GPIO_PinDirection(pin_config.port, pin_config.pin, pin_config.direction);
		
}




