/** 
  * @file lpc1768_adc.h
  * @brief Peripheral driver for ADC peripheral on LPC1768 Microcontroller
  */

/* ================ HEADER GUARD ================ */

#ifndef _LPC1768_ADC_H
#define _LPC1768_ADC_H

/* ================ HEADER INCLUDES ================ */

#include "lpc17xx.h"

/* ================ Bit Macro Defs ================ */

#define ADC0_PCONP		((uint32_t)1<<12)	// ADC power/clock control bit
#define ADC0_DONE_BIT	((uint32_t)1<<31)	// Bit is set when conversion completes
#define ADC0_PDN_BIT	((uint32_t)1<<21)	// A/D converter is operational
#define ADC0_START_BIT	((uint32_t)1<<24)	// Start conversion now


/* ================ Masks ================ */

#define ADC0_CLKDIV_MASK	0xFF00				// Mask for ADC clock divider
#define ADC0_DATA_MASK		((uint32_t)0xFFF0)	// Mask for ADC Data value
#define ADC0_START_MASK		((uint32_t)1<<26 | (uint32_t)1<<25 | (uint32_t)1<<24) // Mask

/* ================ Global variables ================ */

extern volatile uint8_t ADC0_INT0;

/* ================ Const ================ */

#define	V_REF 			(double)3300000		// mV *1000
#define R_MAX			(double)4095000		// bits *1000
#define V_OFFSET		(double)500 		// mV
#define ADC_BUFF_LEN	(uint32_t) 24		// Buffer Length

/* ================ Data Structures ================ */

/**
  * struct ADC_Init_s
  * @brief Holds parameters for ADC peripheral initialization
  */
typedef struct
{
	uint32_t ADCNumber;
	uint32_t SampleScaler;	// scaler of pclk
	uint32_t FetchDataRate;	// in seconds
	uint32_t ADCPclk;	
}ADC_Init_s;

/**
  * struct ADC_Data_s
  * @brief Holds data for ADC peripheral
  */
typedef struct
{
	uint32_t Data[ADC_BUFF_LEN];
	uint32_t cnt;	
}ADC_Data_s;

/* ================ Private Functions ================ */

/**
  * ADC peripheral power enable
  * @brief First function to be executed within ADC_init() function
  * @param *init_handle - pointer to ADC_Init_s instance
  * @return void
  */
static void ADC_Power_Enable (ADC_Init_s *init_handle);

/**
  * ADC peripheral power disnable
  * @param *init_handle - pointer to ADC_Init_s instance
  * @return void
  */
static void ADC_Power_Disable (ADC_Init_s *init_handle);

/**
  * ADC peripheral clock fetch
  * @brief Second function to be executed within ADC_init() function
  * @param *init_handle - pointer to ADC_Init_s instance
  * @return void
  */
static void ADC_GetClock (ADC_Init_s *init_handle);

/**
  * ADC peripheral sample rate setting
  * @brief Third function to be executed within ADC_init() function
  * @param *init_handle - pointer to ADC_Init_s instance
  * @return void
  */
static void ADC_SetSampleRate (ADC_Init_s *init_handle);

/**
  * ADC peripheral pinout
  * @brief Fourth function to be executed within ADC_init() function
  * @param *init_handle - pointer to ADC_Init_s instance
  * @return void
  */
static void ADC_PinInit (ADC_Init_s *init_handle);

/* ================ Public Functions ================ */

/**
  * ADC peripheral initialization
  * @brief Should be first function call after setting up initialization parameters
  * @param *init_handle - pointer to ADC_Init_s instance
  * @return void
  */
void ADC_Init(ADC_Init_s *init_handle);

/**
  * ADC peripheral conversion start
  * @brief Starts sampling specified pin
  * @param *data_handle - pointer to ADC_Data_s instance
  * @return void
  */
void ADC_StartConversion(ADC_Data_s *data_handle);

/**
  * ADC peripheral buffer data read
  * @brief Reads data from ADC buffer and sends it to data_handle->Data[] array 
  * @param *data_handle - pointer to ADC_Data_s instance
  * @return void
  */
void ADC_ReadData(ADC_Data_s *data_handle);

/**
  * ADC Enable Interrupt Function
  */
void ADC_E_I (ADC_Init_s *init_handle);

/**
  * ADC Disable Interrupt Function
  */
void ADC_D_I (ADC_Init_s *init_handle);

#endif /* LPC1768_ADC */
