#include "lpc17xx.h"
#include "Led4.h"
#include "lpc1768_uart.h"	

int main(){
	
// System clock Init
	SystemInit();
	
// 4 LEDs Init
	LED_init();
	
// ========= INIT FUNCTIONS ========	
	
// UARTx init
	UART_Init_s uart0_init_struc;
	UART_Init_s *uart0_init_handle = &uart0_init_struc;
	
	UART_Data_s uart0_data_struc;
	UART_Data_s *uart0_data_handle = &uart0_data_struc;
	
	uart0_init_handle -> BaudRate = 115200;
	uart0_init_handle -> WordLength = 8;
	uart0_init_handle -> StopBits = 1;
	uart0_init_handle -> Parity = 0;
	uart0_init_handle -> UARTx = LPC_UART2;
	
	UART_Init(uart0_init_handle, uart0_data_handle);

	
// ========= INTERRUPT ENABLE ========
	NVIC_EnableIRQ(UART0_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART3_IRQn);	
	
// Enable Receive Buffer Register Interrupt
	UART_E_RBR_I(uart0_data_handle);
	
//==========	VARIABLE DEFINITIONS =========

	uint32_t test[16] = {0x0};
	uint32_t *test_ptr = &test[0];
	
	uint32_t temp;
	uint32_t *ptr = &temp;
	
	uint32_t DR=0;
	
	test[0]=0xFF0000AA;
	for(uint16_t i=1; i<16; i++)
		test[i]=test[0] + i*0x00000100;
	
	UART_tx(uart0_data_handle,test_ptr,9,Blocking);	
	//UART_tx(uart0_data_handle,test_ptr,9,ISR);	

	while(1){			
		
		if(uart0_data_handle->Flags->THRE==1){
			UART_tx(uart0_data_handle,0,0,ISR);
		}
		if(uart0_data_handle->Flags->RBR==1){
			UART_rx(uart0_data_handle);
			DR=1;
			//temp = 0x000000FF & uart0_data_handle->rx_buffer_a_8[(uart0_data_handle->rx_offset)-1];
			//UART_tx(uart0_data_handle,ptr,1, ISR);
		}
		if(DR==1){
			LED_on(0);
			if(uart0_data_handle->rx_buffer_a_8[(uart0_data_handle->rx_offset-1)]==0xD){
				for(uint32_t i=0; i<uart0_data_handle->rx_offset-1; i++){
					temp = 0x000000FF & uart0_data_handle->rx_buffer_a_8[i];
					UART_tx(uart0_data_handle,ptr,1, Blocking);				
				}
				DR=0;
				uart0_data_handle->rx_offset=0;
			}			
		}
	}		
}

// ========= IRQ Handlers ========

void UART0_IRQHandler(void){
	UART0_IRQHandler_F();
}

void UART2_IRQHandler(void){
	UART2_IRQHandler_F();
}

void UART3_IRQHandler(void){
	UART3_IRQHandler_F();
}


// End of file
