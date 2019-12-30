/** 
 * @file lpc1768_timer.c
 * @brief Peripheral driver for UART peripheral on LPC1768 Microcontroller
 */

#include "lpc1768_timer.h"

/* ================ Private Functions ================ */

/**
  * Timer peripheral configuration
  * @brief First function to be executed within Timer_init() function
  * @param *TIMx - pointer to LPC_TIM_TypeDef instance (defined in lpc17xx.h - LPC_TIMx, x={0...3})
  * @param *init_struc - pointer to Timer_init_struc instance
  * @return void
  */
static void Timer_configure(LPC_TIM_TypeDef *TIMERx, Timer_init_struc *init_struc)
{	
	TIMERx -> MCR = init_struc -> MCR_Value;	
}

/**
  * Timer peripheral clock fetch & prescaler setting
  * @brief Second function to be executed within Timer_init() function
  * @param *TIMx - pointer to LPC_TIM_TypeDef instance (defined in lpc17xx.h - LPC_TIMx, x={0...3})
  * @return uint32_t prescaler value
  */
static uint32_t getPrescalarValue(LPC_TIM_TypeDef *TIMx)
{
	uint32_t temp,value;
	uint32_t Timer_Pclk;	
	if (TIMx == LPC_TIM0){
		temp = (LPC_SC -> PCLKSEL0 >> 2) & 0x03;
	}
	else if (TIMx == LPC_TIM1){
		temp = (LPC_SC -> PCLKSEL0 >> 4) & 0x03;
	}
	else if (TIMx == LPC_TIM2){
		temp = (LPC_SC -> PCLKSEL1 >> 12) & 0x03;
	}
	else{
		temp = (LPC_SC -> PCLKSEL1 >> 14) & 0x03;
	}
	
	switch( temp )
    {
		case 0x00:
			Timer_Pclk = SystemCoreClock/4;
			break;
		case 0x01:
			Timer_Pclk = SystemCoreClock;
			break; 
		case 0x02:
			Timer_Pclk = SystemCoreClock/2;
			break; 
		case 0x03:
			Timer_Pclk = SystemCoreClock/8;
			break;
    } 
		value = Timer_Pclk / 1000 - 1;    //prescalar value for 1ms		
		return value;		
}

/* ================ Public Functions ================ */

/**
  * Timer peripheral initialization
  * @brief Should be first function call after setting up initialization parameters
  * @param *init_struc - pointer to Timer_init_struc instance
  * @return void
  */
void Timer_init(Timer_init_struc *init_struc)
{	
	Timer_configure(init_struc->TIMx, init_struc);
	init_struc -> TIMx -> PR = getPrescalarValue(init_struc->TIMx);
	init_struc -> TIMx -> MR0 = init_struc->Timer_interval_ms;	
}

/**
  * Timer peripheral start
  * @brief Starts Timer peripheral with set parameters
  * @param *TIMERx - pointer to LPC_TIM_TypeDef instance (defined in lpc17xx.h - LPC_TIMx, x={0...3})
  * @return void
  */
void Timer_start(LPC_TIM_TypeDef *TIMERx)
{
	TIMERx -> TC = 0x0;
	TIMERx -> TCR |= 0x1;
}

/**
  * Timer peripheral stop
  * @brief Stops Timer peripheral
  * @param *TIMERx - pointer to LPC_TIM_TypeDef instance (defined in lpc17xx.h - LPC_TIMx, x={0...3})
  * @return void
  */
void Timer_stop(LPC_TIM_TypeDef *TIMERx)
{	
	TIMERx -> TCR = 0x0;
}

/**
  * Timer peripheral interval setting
  * @brief Sets Timer peripheral match register to required value to achive specified interval sent as parameter 
  * @param *TIMERx - pointer to LPC_TIM_TypeDef instance (defined in lpc17xx.h - LPC_TIMx, x={0...3})
  * @param uint32_t time - interval at which timer is required to fire
  * @return void
  */
void Timer_setTime(LPC_TIM_TypeDef *TIMERx, uint32_t time)
{
	TIMERx -> MR0 = time;	
}

/* End of file lpc1768_timer.c */
