#include "timer_driver.h"
	
static uint32_t getPrescalarValue(LPC_TIM_TypeDef *TIMx)
{
	uint32_t temp,value;
	uint32_t Timer_Pclk;
	
	if (TIMx == LPC_TIM0)
	{
		temp = (LPC_SC ->PCLKSEL0 >> 2) & 0x03;
	}
	else if (TIMx == LPC_TIM1)
		temp = (LPC_SC ->PCLKSEL0 >> 4) & 0x03;
	else if (TIMx == LPC_TIM2)
		temp = (LPC_SC ->PCLKSEL1 >> 12) & 0x03;
	else
		temp = (LPC_SC ->PCLKSEL1 >> 14) & 0x03;
	
	
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

static void Timer_configure(LPC_TIM_TypeDef *TIMERx, Timer_init_struc *init_struc)
{
	
	TIMERx->MCR = init_struc ->MCR_Value;
	
}


void Timer_init( Timer_init_struc *init_struc)
{
	
	Timer_configure(init_struc->TIMx, init_struc);
	init_struc->TIMx->PR = getPrescalarValue(init_struc->TIMx);
	init_struc->TIMx->MR0 = init_struc->Timer_interval_ms;
	
}

void Timer_start(LPC_TIM_TypeDef *TIMERx)
{
	TIMERx->TC	=	0x0;
	TIMERx->TCR |= 0x1;
}

void Timer_stop(LPC_TIM_TypeDef *TIMERx)
{
	
	TIMERx->TCR = 0x0;
}

void Timer_setTime(LPC_TIM_TypeDef *TIMERx, uint32_t time)
{
	TIMERx -> MR0 = time;	
}

