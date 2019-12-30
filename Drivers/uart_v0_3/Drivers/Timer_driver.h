#ifndef _TIMER_H
#define _TIMER_H



/********************************************************/
/* header includes */
#include "GPIO_driver.h"



/********************************************************/
/*! @name	 				Macros		        */
/********************************************************/


/* ================= Interrupt register ==================
		The Interrupt Register consists of 4 bits for the match interrupts and 4 bits for the capture
interrupts.          */

#define MR0_Interrupt_Flag							((uint32_t) 1 << 0)
#define MR1_Interrupt_Flag							((uint32_t) 1 << 1)
#define MR2_Interrupt_Flag					 		((uint32_t) 1 << 2)
#define MR3_Interrupt_Flag							((uint32_t) 1 << 3)
#define CR0_Interrupt_Flag							((uint32_t) 1 << 4)
#define CR1_Interrupt_Flag							((uint32_t) 1 << 5)


/* ================= Timer Control register ==================
		The Timer Control Register (TCR) is used to control the operation of the Timer/Counter.        */

#define Counter_Enable						((uint32_t) 1 << 0)   
#define Counter_Reset							((uint32_t) 1 << 1)

/* ================= Count Control Register ==================
		The Count Control Register (CTCR) is used to select between Timer and Counter mode,
and in Counter mode to select the pin and edge(s) for counting.           */

#define Timer_Mode								((uint32_t) 0x0 << 0)
#define Counter_Mode_Rising				((uint32_t) 0x1 << 0)
#define Counter_Mode_Falling			((uint32_t) 0x2 << 0)
#define Counter_Mode_Both					((uint32_t) 0x3 << 0)

#define Counter_CAP0							((uint32_t) 0x0 << 2)
#define Counter_CAP1							((uint32_t) 0x1 << 2)


/* ================= Match Control Register ==================
		The Match Control Register is used to control what operations are performed when one of
the Match Registers matches the Timer Counter.         */


#define MR0I											((uint32_t) 0x1 << 0)
#define MR0R											((uint32_t) 0x1 << 1)
#define MR0S											((uint32_t) 0x1 << 2)
#define MR1I											((uint32_t) 0x1 << 3)
#define MR1R											((uint32_t) 0x1 << 4)
#define MR1S											((uint32_t) 0x1 << 5)
#define MR2I											((uint32_t) 0x1 << 6)
#define MR2R											((uint32_t) 0x1 << 7)
#define MR2S											((uint32_t) 0x1 << 8)
#define MR3I											((uint32_t) 0x1 << 9)
#define MR3R											((uint32_t) 0x1 << 10)
#define MR3S											((uint32_t) 0x1 << 11)

/* ================= Capture Control Register ==================
		The Capture Control Register is used to control whether one of the four Capture Registers
is loaded with the value in the Timer Counter when the capture event occurs, and whether
an interrupt is generated by the capture event. */

#define CAP0RE										((uint32_t) 0x1 <<0)
#define CAP0FE										((uint32_t) 0x1 <<1)
#define CAP0I											((uint32_t) 0x1 <<2)

#define CAP1RE										((uint32_t) 0x1 <<3)
#define CAP1FE										((uint32_t) 0x1 <<4)
#define CAP1I											((uint32_t) 0x1 <<5)

/* ================= External Match Register ==================
The External Match Register provides both control and status of the external match pins. */

#define EM0												((uint32_t) 0x1 <<0)
#define EM1												((uint32_t) 0x1 <<1)
#define EM2												((uint32_t) 0x1 <<2)
#define EM3												((uint32_t) 0x1 <<3)

#define EMC0_Clear								((uint32_t) 0x1 <<4)
#define EMC0_SET									((uint32_t) 0x2 <<4)
#define EMC0_Toggle								((uint32_t) 0x3 <<4)

#define EMC1_Clear								((uint32_t) 0x1 <<6)
#define EMC1_SET									((uint32_t) 0x2 <<6)
#define EMC1_Toggle								((uint32_t) 0x3 <<6)

#define EMC2_Clear								((uint32_t) 0x1 <<8)
#define EMC2_SET									((uint32_t) 0x2 <<8)
#define EMC2_Toggle								((uint32_t) 0x3 <<8)

#define EMC3_Clear								((uint32_t) 0x1 <<10)
#define EMC3_SET									((uint32_t) 0x2 <<10)
#define EMC3_Toggle								((uint32_t) 0x3 <<10)





#define TIMER_PCLK_BIT         1



/********************************************************/
/*! @name	 				Data Structures	        */
/********************************************************/

typedef enum
{
	TIMER_MODE,
	
	COUNTER_MODE
	
}Timer_Mode_enum;


typedef struct 
{
	LPC_TIM_TypeDef 		*TIMx;					// timer0/1/2/3
	
	uint32_t						Timer_interval_ms;						//   timer interval
	
	uint32_t						MCR_Value;
	
	
}Timer_init_struc;



/********************************************************/
/*! @name	 				API        */
/********************************************************/

void Timer_init( Timer_init_struc *init_struc);

void Timer_start(LPC_TIM_TypeDef *TIMERx);

void Timer_stop(LPC_TIM_TypeDef *TIMERx);

void Timer_setTime(LPC_TIM_TypeDef *TIMERx, uint32_t time);




#endif