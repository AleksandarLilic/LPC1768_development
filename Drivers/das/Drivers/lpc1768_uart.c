/** 
 * @file lpc1768_uart.c
 * @brief Peripheral driver for UART peripheral on LPC1768 Microcontroller
 */

#include "lpc1768_uart.h"
#include "lpc1768_gpio.h"

/* ================ Private Functions ================ */

/**
  * Error Flags struct instance for each UART peripheral
  */
UART_Flags F0;
UART_Flags *F0_ptr = &F0;
UART_Flags F2;
UART_Flags *F2_ptr = &F2;
UART_Flags F3;
UART_Flags *F3_ptr = &F3;

/**
  * UART peripheral power enable
  * @brief Second function to be executed within UART_init() function
  * @param *init_handle - pointer to UART_Init_s instance
  * @return void
  */
static void UART_Power_Enable(UART_Init_s *init_handle)
{
	uint32_t uart_n = init_handle->UARTNumber;	
	switch(uart_n){
		
		case 0x0:
		LPC_SC->PCONP |= UART0_PCONP_ON;
		break;
		case 0x1:
		LPC_SC->PCONP |= UART1_PCONP_ON;
		break;
		case 0x2:
		LPC_SC->PCONP |= UART2_PCONP_ON;
		break;
		case 0x3:
		LPC_SC->PCONP |= UART3_PCONP_ON;
		break;		
	}
}

/**
  * UART peripheral power disable
  * @param *init_handle - pointer to UART_Init_s instance
  * @return void
  */
static void UART_Power_Disable(UART_Init_s *init_handle)
{
	uint32_t uart_n = init_handle->UARTNumber;	
	switch(uart_n){
		
		case 0x0:
		LPC_SC -> PCONP &= ~UART0_PCONP_ON;
		break;
		case 0x1:
		LPC_SC -> PCONP &= ~UART1_PCONP_ON;
		break;
		case 0x2:
		LPC_SC -> PCONP &= ~UART2_PCONP_ON;
		break;
		case 0x3:
		LPC_SC -> PCONP &= ~UART3_PCONP_ON;
		break;		
	}
}

/**
  * UART peripheral number fetching and flags reset
  * @brief First function to be executed within UART_init() function
  * @param *init_handle - pointer to UART_Init_s instance
  * @param *data_handle - pointer to UART_Data_s instance
  * @return void
  */
static void UART_Get_Number(UART_Init_s *init_handle, UART_Data_s *data_handle)
{
	if(init_handle->UARTx == LPC_UART0){
		init_handle->UARTNumber = 0;
		data_handle->Flags = F0_ptr;
	}
	else if(init_handle->UARTx == LPC_UART2){
		init_handle->UARTNumber = 2;
		data_handle->Flags = F2_ptr;
	}
	else if(init_handle->UARTx == LPC_UART3){
		init_handle->UARTNumber = 3;
		data_handle->Flags = F3_ptr;
	}
	data_handle->UARTx = init_handle->UARTx;
	data_handle->UARTNumber = init_handle->UARTNumber;
	
	data_handle->offset = 0;
	data_handle->Flags->ERR = 0;
	data_handle->Flags->RBR = 0;
	data_handle->Flags->TO = 0;
	data_handle->Flags->THRE = 0;
}

/**
  * UART peripheral clock fetch
  * @brief Third function to be executed within UART_init() function
  * @param *init_handle - pointer to UART_Init_s instance
  * @return uint32_t clock value
  */
static uint32_t UART_GetClock(UART_Init_s *init_handle)
{		
	uint32_t uart_n = init_handle->UARTNumber;
	uint32_t var_pclk, uart_pclk;
	
	switch(uart_n){
				
		case 0x0:
		var_pclk = (LPC_SC->PCLKSEL0 >> 6) & 0x03;
		switch(var_pclk){
			case 0x00:
			uart_pclk = SystemCoreClock/4;
			break;
			case 0x01:
			uart_pclk = SystemCoreClock;
			break; 
			case 0x02:
			uart_pclk = SystemCoreClock/2;
			break; 
			case 0x03:
			uart_pclk = SystemCoreClock/8;
			break;
		}
		break;
		
		case 0x1:
		var_pclk = (LPC_SC->PCLKSEL0 >> 8) & 0x03;
		switch(var_pclk){
			case 0x00:
			uart_pclk = SystemCoreClock/4;
			break;
			case 0x01:
			uart_pclk = SystemCoreClock;
			break; 
			case 0x02:
			uart_pclk = SystemCoreClock/2;
			break; 
			case 0x03:
			uart_pclk = SystemCoreClock/8;
			break;
		}
		break;
		
		case 0x2:
		var_pclk = (LPC_SC->PCLKSEL1 >> 16) & 0x03;
		switch(var_pclk){
			case 0x00:
			uart_pclk = SystemCoreClock/4;
			break;
			case 0x01:
			uart_pclk = SystemCoreClock;
			break; 
			case 0x02:
			uart_pclk = SystemCoreClock/2;
			break; 
			case 0x03:
			uart_pclk = SystemCoreClock/8;
			break;
		}
		break;
		
		case 0x3:
		var_pclk = (LPC_SC->PCLKSEL1 >> 18) & 0x03;
		switch(var_pclk){
			case 0x00:
			uart_pclk = SystemCoreClock/4;
			break;
			case 0x01:
			uart_pclk = SystemCoreClock;
			break; 
			case 0x02:
			uart_pclk = SystemCoreClock/2;
			break; 
			case 0x03:
			uart_pclk = SystemCoreClock/8;
			break;
		}
		break;				
	}	
	return uart_pclk;	
}

/**
  * UART peripheral baud rate setting
  * @brief Fourth function to be executed within UART_init() function
  * @param *init_handle - pointer to UART_Init_s instance
  * @param uint32_t uart_pclk - clock value
  * @return void
  */
static void UART_SetBaudRate(UART_Init_s *init_handle, uint32_t uart_pclk)
{	
	uint32_t BaudRate = init_handle->BaudRate;
	uint32_t var_RegValue = (uart_pclk / (16 * BaudRate));	
	
	init_handle->UARTx->LCR |= 0x80;
	init_handle->UARTx->DLL =  var_RegValue & 0xFF;
	init_handle->UARTx->DLM = (var_RegValue >> 0x08);
	init_handle->UARTx->LCR &= ~(SBIT_DLAB);			// Clear DLAB after setting DLL,DLM
}

/**
  * UART peripheral FIFO buffers enable
  * @brief Fifth function to be executed within UART_init() function
  * @param *init_handle - pointer to UART_Init_s instance
  * @return void
  */
static void UART_EnableBuffers(UART_Init_s *init_handle)
{		
	init_handle->UARTx->FCR = SBIT_FIFO | SBIT_RXFIFO | SBIT_TXFIFO; // UARTx FIFO tx & rx enable and clear
}

/**
  * UART peripheral frame format setting
  * @brief Sixth function to be executed within UART_init() function
  * @param *init_handle - pointer to UART_Init_s instance
  * @return void
  */
static void UART_Format(UART_Init_s *init_handle)
{
	uint32_t word_l = init_handle->WordLength;
	uint32_t stop_b = init_handle->StopBits;
	uint32_t parity = init_handle->Parity;
	
	uint32_t word_l_u32, stop_b_u32, parity_u32;	

		switch(word_l){
			case 0x05:
			word_l_u32 = 0x00;
			break;
			case 0x06:
			word_l_u32 = 0x01;
			break;
			case 0x07:
			word_l_u32 = 0x02;
			break;
			case 0x08:
			word_l_u32 = 0x03;
			break;
		}
		
		if(stop_b == 0x01)
			stop_b_u32 = 0x00;
		else
			stop_b_u32 = 0x01;			
		
		if(parity == 0x00)
			parity_u32 = 0x00;
		else
			parity_u32 = 0x01;
		
		init_handle->UARTx->LCR = (word_l_u32 << SBIT_WORD_LENGTH) 
		| (stop_b_u32 << SBIT_STOP_BIT)
		| (parity_u32 << SBIT_PARITY_EN) 
		| (1<<SBIT_DLAB); // Word Length, Stop Bit and Parity Settings, leave divisor latch
}

/**
  * UART peripheral pin initialization based on UARTx parameter
  * @brief Seventh function to be executed within UART_init() function
  * @param *init_handle - pointer to UART_Init_s instance
  * @return void
  */
static void UART_PinInit(UART_Init_s *init_handle){
	
	uint32_t uart_n = init_handle->UARTNumber;	
	switch(uart_n){
		
		case 0x0:
		GPIO_PinFunction (PORT_0, PIN_2, GPIO_PINSEL_FUNCTION_1);
		GPIO_PinFunction (PORT_0, PIN_3, GPIO_PINSEL_FUNCTION_1);
		break;
		case 0x1:
		GPIO_PinFunction (PORT_0, PIN_15, GPIO_PINSEL_FUNCTION_1);
		GPIO_PinFunction (PORT_0, PIN_16, GPIO_PINSEL_FUNCTION_1);
		break;
		case 0x2:
		GPIO_PinFunction (PORT_0, PIN_10, GPIO_PINSEL_FUNCTION_1);
		GPIO_PinFunction (PORT_0, PIN_11, GPIO_PINSEL_FUNCTION_1);
		break;
		case 0x3:
		GPIO_PinFunction (PORT_0, PIN_0, GPIO_PINSEL_FUNCTION_2);
		GPIO_PinFunction (PORT_0, PIN_1, GPIO_PINSEL_FUNCTION_2);
		break;		
	}	
}

/**
  * Generic function for converting uint32_t to four uint8_t
  * @brief Used for transmission
  * @brief Microcontroller is 32-bit architecture, UART protocol is 8-bit data
  * @param *data_handle - pointer to UART_Data_s instance
  * @return void
  */
static void UART_LtS(UART_Data_s *data_handle)
{	
	uint32_t len, var, cnt, i = 0x0;
	len = (data_handle->tx_length)-1;
	cnt = (data_handle->tx_counter)-1;
	if(cnt >= 3){
		i=4;
	}
	else{
		i=cnt+1;
	}	
	
	while(i){
		var = data_handle->tx_buffer_ptr_32[len-cnt];
		data_handle->tx_counter--;
		for(int z=0; z<4; z++){
			data_handle->tx_buffer_a_8[data_handle->offset] = (var>>(z*8)) & 0xFF;
			data_handle->offset++;
		}
		cnt--;	
		i--;		
	}
}

static void UART_load_fifo(UART_Data_s *data_handle)
{	
	uint32_t cnt_l, cnt_h;
	cnt_l = 0;
	cnt_h = data_handle->offset-1;
	while(cnt_l<=cnt_h){
		data_handle->UARTx->THR = data_handle->tx_buffer_a_8[cnt_l];
		cnt_l++;
	}
	data_handle->offset=0;
}

/**
  * Blocking approach for sending data
  * @brief Used for data transmission, blocks all other actions until all data is sent
  * @param *tx_buffer - pointer to first 32-bit data for transmission
  * @param len - length of buffer that needs to be sent
  * @return void
  */
static void UART_Blocking_tx(UART_Data_s *data_handle, uint32_t *tx_buffer, uint32_t len)
{
	UART_D_THRE_I(data_handle);
	data_handle->data_fetch = 1;
	data_handle->tx_buffer_ptr_32 = tx_buffer;
	data_handle->tx_length = len;
	data_handle->tx_counter = len;
	
	while(data_handle->tx_counter>0){
		if(data_handle->UARTx->LSR & UART_LSR_THRE){ // if tx buffer is empty
			UART_LtS(data_handle);
			UART_load_fifo(data_handle);
		}
	}
	UART_E_THRE_I(data_handle);
}

/**
  * Interrupt driven approach for sending data
  * @brief Used for data transmission, requires Interrupt handling in main() function
  * @param *tx_buffer - pointer to first 32-bit data for transmission
  * @param len - length of buffer that needs to be sent
  * @return void
  */
static void UART_IRQ_tx(UART_Data_s *data_handle, uint32_t *tx_buffer, uint32_t len)
{	
	if(data_handle->Flags->THRE==0){
		if((data_handle->UARTx->LSR & UART_LSR_THRE)){
		// First function call and THR Empty
			data_handle->data_fetch = 1;
			data_handle->tx_buffer_ptr_32 = tx_buffer;
			data_handle->tx_length = len;
			data_handle->tx_counter = len;
	
			UART_LtS(data_handle);
			UART_load_fifo(data_handle);
			data_handle->UARTx->IER |= SBIT_THRE_IE; // Enable UARTx THRE Interrupt
		}
		else {			
			// First function call, THR not Empty, Interrupt main when empty
			data_handle->UARTx->IER |= SBIT_THRE_IE; // Enable UARTx THRE Interrupt
		}
	}
	else { // THRE_F=1
		// Interrupt call
		data_handle->Flags->THRE = 0;
		
		if((data_handle->tx_counter) == 0){
				UART_D_THRE_I(data_handle);
				data_handle->data_fetch = 0;
		}
		else{
			if(data_handle->data_fetch==0){
				data_handle->data_fetch = 1;
				data_handle->tx_buffer_ptr_32 = tx_buffer;
				data_handle->tx_length = len;
				data_handle->tx_counter = len;
	
				UART_LtS(data_handle);
				UART_load_fifo(data_handle);
			}
			else{
				data_handle->Flags->THRE = 0;
				UART_LtS(data_handle);
				UART_load_fifo(data_handle);
				if((data_handle->tx_counter) == 0){
					UART_D_THRE_I(data_handle);
					data_handle->data_fetch = 0;
				}		
			}			
		}
	}
}

/* ================ Public Functions ================ */

/**
  * UART peripheral initialization
  * @brief Should be first function call after setting up initialization parameters
  * @param *init_handle - pointer to UART_Init_s instance
  * @param *data_handle - pointer to UART_Data_s instance
  * @return void
  */
void UART_Init(UART_Init_s *init_handle, UART_Data_s *data_handle)
{	
	uint32_t uart_pclk;	
	UART_Get_Number(init_handle, data_handle);
	UART_Power_Enable(init_handle);
	uart_pclk = UART_GetClock(init_handle);
	UART_SetBaudRate(init_handle, uart_pclk);
	UART_EnableBuffers(init_handle);
	UART_PinInit(init_handle);
	UART_Format(init_handle);
}

/**
  * UART transmission
  * @brief Universal transmission function, sends data from tx_buffer, number of consecutive
  * is defined by len parameter
  * @param *data_handle - pointer to UART_Data_s instance
  * @param *tx_buffer - pointer to first 32-bit data for transmission
  * @param len - length of buffer that needs to be sent
  * @param type - ISR(Iterrupt Service Routine) or Blocking
  * @return void
  */
void UART_tx(UART_Data_s *data_handle, uint32_t *tx_buffer, uint32_t len, txType type)
{
	if(type==Blocking)
		UART_Blocking_tx(data_handle, tx_buffer, len);
	else if(type==ISR)
		UART_IRQ_tx(data_handle, tx_buffer, len);		
}

/**
  * UART reception
  * @brief Universal reception function, stores received data in *data_handle->rx_buffer_a_8
  * @param *data_handle - pointer to UART_Data_s instance
  * @return void
  */
void UART_rx(UART_Data_s *data_handle)
{	
	data_handle->Flags->RBR = 0;
	
	if(data_handle->rx_offset > RX_BUFFER_SIZE){ //increase RX_BUUFER_SIZE for bigger buffer
		data_handle->rx_offset=0;
	}
	data_handle -> rx_buffer_a_8[data_handle->rx_offset] = data_handle->UARTx->RBR;
	data_handle->rx_offset++;
	
	UART_E_RBR_I(data_handle); // enables RBR interrupt
}

/* ================ Interrupts ================ */

/**
  * UART Enable RBR (Receive Buffer Register) Interrupt Function
  */
void UART_E_RBR_I(UART_Data_s *data_handle){
	data_handle->UARTx->IER |= SBIT_RBR_IE;
}

/**
  * UART Disable RBR (Receive Buffer Register) Interrupt Function
  */
void UART_D_RBR_I(UART_Data_s *data_handle){
	data_handle->UARTx->IER &= ~SBIT_RBR_IE;
}

/**
  * UART Enable THRE (Transmit Hold Register Empty) Interrupt Function
  */
void UART_E_THRE_I(UART_Data_s *data_handle){
	data_handle->UARTx->IER |= ~SBIT_THRE_IE;	
}

/**
  * UART Disable THRE (Transmit Hold Register Empty) Interrupt Function
  */
void UART_D_THRE_I(UART_Data_s *data_handle){
	data_handle->UARTx->IER &= ~SBIT_THRE_IE;	
}

/**
  * UART0 Interrupt Handler Function
  */
void UART0_IRQHandler_F(void){	
	uint8_t IntStatus = (LPC_UART0->IIR);
	uint8_t status = IntStatus & UART_INT_STATUS;
	uint8_t ID = IntStatus & UART_INT_ID;
	ID = ID >> 1;
	if(status==0){
		switch(ID){
			case 0x03:
				F0.ERR = 1;
			break;
			case 0x02:				
				F0.RBR = 1;		// receive data available
				LPC_UART0->IER &= ~SBIT_RBR_IE;
			break;
			case 0x06:
				F0.TO = 1;		// character time-out indicator
			break;
			case 0x01:
				F0.THRE = 1;	//transmit hold register empty
			break;
		}			
	}
	//else
		// error with interrupt, volatile var again		
}

/**
  * UART2 Interrupt Handler Function
  */
void UART2_IRQHandler_F(void){	
	uint8_t IntStatus = (LPC_UART2->IIR);
	uint8_t status = IntStatus & UART_INT_STATUS;
	uint8_t ID = IntStatus & UART_INT_ID;
	ID = ID >> 1;
	
	if(status==0){
		switch(ID){
			case 0x03:
				F2.ERR = 1;
			break;
			case 0x02:				
				F2.RBR = 1;		// receive data available
				LPC_UART2->IER &= ~SBIT_RBR_IE;
			break;
			case 0x06:
				F2.TO = 1;		// character time-out indicator
			break;
			case 0x01:
				F2.THRE = 1;	//transmit hold register empty
			break;
		}			
	}
	//else
		// error with interrupt, volatile var again		
}

/**
  * UART3 Interrupt Handler Function
  */
void UART3_IRQHandler_F(void){	
	uint8_t IntStatus = (LPC_UART3->IIR);
	uint8_t status = IntStatus & UART_INT_STATUS;
	uint8_t ID = IntStatus & UART_INT_ID;
	ID = ID >> 1;
	
	if(status==0){
		switch(ID){
			case 0x03:
				F3.ERR = 1;
			break;
			case 0x02:				
				F3.RBR = 1;	// receive data available
				LPC_UART3->IER &= ~SBIT_RBR_IE;
			break;
			case 0x06:
				F3.TO = 1;	// character time-out indicator
			break;
			case 0x01:
				F3.THRE = 1; //transmit hold register empty
			break;
		}			
	}
	//else
		// error with interrupt, volatile var again		
}

/* End of file lpc1768_uart.c */
