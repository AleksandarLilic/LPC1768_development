#include "lpc17xx.h"
#include "lpc1768_led.h"
#include "lpc1768_adc.h"
#include "lpc1768_gpio.h"
#include "lpc1768_uart.h"
#include "lpc1768_timer.h"
#include "SPI_transfer_BME280.h"
#include "bme280.h"

static volatile uint8_t T0_flag;
static volatile uint8_t T_up = 0;

// ========= Function prototypes ========

void delay_function(uint32_t time);	
int8_t set_sensor_normal_mode(struct bme280_dev *dev);	

int main(){
	
// ========= INIT FUNCTIONS ========
	
// System clock Init
	SystemInit();
	
// 4 LEDs Init
	LED_init();
	
// SPI Init
	SPI_Init_Cfg_Struc spi_init_cfg;
	SPI_Init_Cfg_Struc *spi_init_cfg_ptr = &spi_init_cfg;
	
	spi_init_cfg_ptr -> CPOL = 0;
	spi_init_cfg_ptr -> CPHA = 0;
	spi_init_cfg_ptr -> ClockRate = 1000000;
	spi_init_cfg_ptr -> DataOrder = 0;			// MSB first
	spi_init_cfg_ptr -> Mode = 1;				// SPI_master mode
	
	SPI_Init(spi_init_cfg_ptr);	
	
// TIMER0 init
	Timer_init_struc TIM0_init_struc;
	Timer_init_struc *TIM0ptr = &TIM0_init_struc;
	
	TIM0_init_struc.TIMx = LPC_TIM0;
	TIM0_init_struc.MCR_Value = 0x7;
	TIM0_init_struc.Timer_interval_ms = 1;
	
	Timer_init(TIM0ptr);
	
// TIMER1 init
	Timer_init_struc TIM1_init_struc;
	Timer_init_struc *TIM1ptr = &TIM1_init_struc;
	
	TIM1_init_struc.TIMx = LPC_TIM1;
	TIM1_init_struc.MCR_Value = 0x3;
	TIM1_init_struc.Timer_interval_ms = 200;
	
	Timer_init(TIM1ptr);
		
// UART2 init
	UART_Init_s uart0_init_struc;
	UART_Init_s *uart0_init_handle = &uart0_init_struc;
	
	uart0_init_handle -> BaudRate = 115200;
	uart0_init_handle -> WordLength = 8;
	uart0_init_handle -> StopBits = 1;
	uart0_init_handle -> Parity = 0;
	uart0_init_handle -> UARTx = LPC_UART2;
	
	UART_Data_s uart0_data_struc;
	UART_Data_s *uart0_data_handle = &uart0_data_struc;
	
	UART_Init(uart0_init_handle, uart0_data_handle);
	
// ADC Init
	ADC_Init_s adc0_init_struc;
	ADC_Init_s *adc0_init_handle = &adc0_init_struc;
	
	adc0_init_handle -> ADCNumber = 0;
	adc0_init_handle -> SampleScaler = 1;
	
	ADC_Init(adc0_init_handle);
	
	
// ========= INTERRUPT ENABLE ========

	NVIC_EnableIRQ(SPI_IRQn);
	NVIC_EnableIRQ(TIMER0_IRQn);
	NVIC_EnableIRQ(TIMER1_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	NVIC_EnableIRQ(ADC_IRQn);	
	
// ========= DATA STRUCTS INIT ========	
	
//BME280 Data Init
	int8_t status_bme280_init;
// BME280 structure init
	struct bme280_dev bme280_init_struct;
	struct bme280_dev *bme280_init_ptr = &bme280_init_struct;
	
	bme280_init_ptr->dev_id = 0;
	bme280_init_ptr->intf = BME280_SPI_INTF;
	bme280_init_ptr->read = spi_read;
	bme280_init_ptr->write = spi_write;
	bme280_init_ptr->delay_ms = delay_function;	
	
	status_bme280_init = bme280_init(bme280_init_ptr);
	if(status_bme280_init == BME280_OK)
	
	set_sensor_normal_mode(bme280_init_ptr);
	
// Enable Receive Buffer Register Interrupt
	UART_E_RBR_I(uart0_data_handle);

// ADC Data Init	
	ADC_Data_s adc0_data_struc;
	ADC_Data_s *adc0_data_handle = &adc0_data_struc;
	ADC_E_I(adc0_init_handle);
	ADC_StartConversion(adc0_data_handle);
	
//==========	VARIABLE DEFINITIONS	=========
	int8_t bme280_val_status = 1;
	struct bme280_data comp_data; // final sensor values: temp, hum, pressure

	uint32_t buffer[4] = {0x0};
	uint32_t *buffer_ptr = &buffer[0];
	
	// default period for sending data to app;
	uint32_t app_timer1_cnt = 25;  // timer 1 period = 200ms => 5*200ms = 1s
	uint32_t send_data_flag = 0;
	uint32_t app_cnt_sent = 25;
	uint32_t app_new_timer_val_flag = 0;
	
	
	uint32_t app = 0;
	
	double adc_val = 0;	
	
	Timer_start(LPC_TIM1);
	
	//UART_tx(uart0_data_handle,buffer_ptr,1,ISR);
	
//		uint32_t buffer56[16] = {0x0};
//	uint32_t *buffer_ptr56 = &buffer56[0];
//	
//		buffer56[0]=0xFF0000AA;
//	for(uint16_t i=1; i<16; i++)
//		buffer56[i]=buffer56[0] + i*0x00000100;
	
	//UART_tx(uart0_data_handle,buffer_ptr56,9,Blocking);
	
	
	
	
	// while(1) function below only checks for Flags that are set by ISRs
	// Only if there are Flags present, appropriate action is taken
	// This is considered to be Interrupt driven system since this polling
	
	while(1){
		
		if(T_up == 1){			
			T_up = 0;
			app_timer1_cnt--;
			if(app_timer1_cnt == 0 | app_new_timer_val_flag == 1){
				app_new_timer_val_flag = 0;
				app_timer1_cnt = app_cnt_sent;
				send_data_flag = 1;
			}				
			bme280_val_status = bme280_get_sensor_data(BME280_ALL, &comp_data, bme280_init_ptr);
			if(bme280_val_status == 0)
				LED_off(0); // if status OK
			else
				LED_on(0); // if status NOT OK
		}		
		
		if(ADC0_INT0 == 1){
			ADC_ReadData(adc0_data_handle);
			ADC0_INT0 = 0;
			double sum = 0;
			for(int i=0; i<ADC_BUFF_LEN; i++){
				sum += adc0_data_handle->Data[i];					
			}
			sum = sum / ADC_BUFF_LEN;
			adc_val = (V_REF / (R_MAX / sum)) - V_OFFSET;
			ADC_E_I(adc0_init_handle);
			ADC_StartConversion(adc0_data_handle);
		}			
			
		if(uart0_data_handle->Flags->THRE == 1){
			UART_tx(uart0_data_handle,0,0,ISR);
		}
		
		if(uart0_data_handle->Flags->RBR == 1){
			app_new_timer_val_flag = 1;
			UART_rx(uart0_data_handle);
			uint8_t cmd = 0x30;
			cmd = uart0_data_handle->rx_buffer_a_8[uart0_data_handle->rx_offset-1];
			switch(cmd){
				case 0x30:
					app_cnt_sent = 5; // period = 1 sec
				break;
				case 0x31:
					app_cnt_sent = 10; // period = 2 sec
				break;
				case 0x32:
					app_cnt_sent = 25; // period = 5 sec
				break;
				case 0x33:
					app_cnt_sent = 50; // period = 10 sec
				break;
				default:
					buffer[0] = 0x00;
					LED_on(1); // error code
					//UART_tx(uart0_data_handle, buffer_ptr, 1, Blocking);
				break;
			}
		}
		if(send_data_flag == 1){
			send_data_flag = 0;
//			buffer[0] = comp_data.temperature;
//			buffer[1] = comp_data.humidity;
//			buffer[2] = comp_data.pressure;
//			buffer[3] = adc_val;
			buffer[0] = app++;
			buffer[1] = app++;
			buffer[2] = app++;
			buffer[3] = app++;
			
			UART_tx(uart0_data_handle, buffer_ptr, 4, Blocking);
		}					
	}
}

// ========= FUNCTION DEFINITIONS ========

int8_t set_sensor_normal_mode(struct bme280_dev *dev)
{
	int8_t bme280_val;
	uint8_t settings_sel;
	//struct bme280_data comp_data;

	/* Recommended mode of operation: Indoor navigation */
	dev->settings.osr_h = BME280_OVERSAMPLING_1X;
	dev->settings.osr_p = BME280_OVERSAMPLING_16X;
	dev->settings.osr_t = BME280_OVERSAMPLING_2X;
	dev->settings.filter = BME280_FILTER_COEFF_16;
	dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	bme280_val = bme280_set_sensor_settings(settings_sel, dev);
	bme280_val = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);
	
	return bme280_val;
}

void delay_function(uint32_t time)
{
	T0_flag = 0;
	Timer_setTime(LPC_TIM0, time);
	Timer_start(LPC_TIM0);
	while (T0_flag == 0);
}

// ========= IRQ Handlers ========

void TIMER0_IRQHandler(void)
{
	LPC_TIM0->IR = 0x1;
	T0_flag = 1;
}

void TIMER1_IRQHandler(void)
{
	LPC_TIM1->IR = 0x1;
	T_up = 1;
}

void SPI_IRQHandler()
{
	uint32_t tr_status;
	//uint8_t dummy;
	
	LPC_SPI->SPINT = SPI_SPINT_INTFLAG; // clear Interrupt
	tr_status = LPC_SPI->SPSR;		//clear status reg

	tr_status = (tr_status & SPI_STATUS_REG_MSK) >> 3;
	
	if (tr_status == SPI_NOFAULT)     // no fault during transfer, continue to process data
	{
		Spi_SetFlag();
	}
	else
	{
		Spi_SetStatus(TRANSFER_FAILED);
	}
}

void UART2_IRQHandler(void){
	UART2_IRQHandler_F();
}

void ADC_IRQHandler(void){
	
	LPC_ADC->ADINTEN &= ~0x1; // disable interrupt	
	ADC0_INT0 = 1;	
}

/* End of file main.c */
