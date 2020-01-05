#include "lpc1768_adc.h"
#include "Led4.h"

/***************************************************************************/
/**** 							Private functions						****/
/***************************************************************************/


/* ================ ADC Power Enable - PCONP ================ */

static void ADC_Power_Enable (ADC_Init_s *init_handle){
	
	LPC_SC -> PCONP |= ADC0_PCONP;
	LPC_ADC -> ADCR |= ADC0_PDN_BIT;
	
}

static void ADC_Power_Disable (ADC_Init_s *init_handle){
	
	LPC_ADC -> ADCR &= ~ADC0_PDN_BIT;
	LPC_SC -> PCONP &= ~ADC0_PCONP;
	
}

static void ADC_GetClock(ADC_Init_s *init_handle){
	
	uint32_t var_pclk, adc_pclk;
	
	var_pclk = (LPC_SC ->PCLKSEL0 >> 24) & 0x03;
		switch(var_pclk){
			case 0x00:
			adc_pclk = SystemCoreClock/4;
			break;
			case 0x01:
			adc_pclk = SystemCoreClock;
			break; 
			case 0x02:
			adc_pclk = SystemCoreClock/2;
			break; 
			case 0x03:
			adc_pclk = SystemCoreClock/8;
			break;
		}
		init_handle -> ADCPclk = adc_pclk;
}

static void ADC_SetSampleRate(ADC_Init_s *init_handle){	
	
	LPC_ADC -> ADCR |= 0x1000; // /*(ADC0_CLKDIV_MASK &*/ ((uint32_t)16<<8); //should be <<8
	//implement full function
}


static void ADC_PinInit (ADC_Init_s *init_handle){
	
	LPC_ADC -> ADCR |= 0x01; //select ad0.0
	GPIO_PinFunction (PORT_0, PIN_23, GPIO_PINSEL_FUNCTION_1);
	GPIO_PinDirection(PORT_0,PIN_23, INPUT);	
	//implement full function
}

/***************************************************************************/
/**** 							Public functions						****/
/***************************************************************************/


void ADC_E_I (ADC_Init_s *init_handle){
	
	LPC_ADC -> ADINTEN = 0x1;
	//implement full function	
}

void ADC_D_I (ADC_Init_s *init_handle){
	
	LPC_ADC -> ADINTEN &= ~0x1;
	//implement full function	
}


volatile uint8_t ADC0_INT0;

void ADC_Init(ADC_Init_s *init_handle){
	
	ADC_Power_Enable (init_handle);
	ADC_GetClock (init_handle);
	ADC_SetSampleRate (init_handle);
	ADC_PinInit (init_handle);
}

void ADC_StartConversion(ADC_Data_s *data_handle){
	
	LPC_ADC -> ADCR |= ADC0_START_BIT;	
}

void ADC_ReadData(ADC_Data_s *data_handle){
	
	if(data_handle->cnt>ADC_BUFF_LEN-1)
		data_handle->cnt=0;	
	uint32_t var = LPC_ADC -> ADDR0;
	if(var & ADC0_DONE_BIT){
		data_handle -> Data[data_handle->cnt] = (var & ADC0_DATA_MASK) >> 4;
	}
	data_handle->cnt++;	
}


