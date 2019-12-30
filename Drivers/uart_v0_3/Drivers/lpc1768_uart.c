#include "lpc1768_uart.h"
#include "Led4.h"

/***************************************************************************/
/**** 							Private functions						****/
/***************************************************************************/

UART_Flags F0;
UART_Flags *F0_ptr = &F0;
UART_Flags F2;
UART_Flags *F2_ptr = &F2;
UART_Flags F3;
UART_Flags *F3_ptr = &F3;


/* ================ UARTn Power Enable - PCONP ================ */

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

/* ================ UARTn Peripheral Power Disable - PCONP ================ */

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

/* ================ UARTn Configure & Forward UART Number ================ */

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


/* ================ UARTn Get Peripheral Clock Select - PCLKSEL0 & PCLKSEL1 ================ */

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

static void UART_SetBaudRate(UART_Init_s *init_handle, uint32_t uart_pclk)
{	
	uint32_t BaudRate = init_handle->BaudRate;
	uint32_t var_RegValue = (uart_pclk / (16 * BaudRate));	
	
	init_handle->UARTx->LCR |= 0x80;
	init_handle->UARTx->DLL =  var_RegValue & 0xFF;
	init_handle->UARTx->DLM = (var_RegValue >> 0x08); //& 0xFF;
	init_handle->UARTx->LCR &= ~(SBIT_DLAB);	// Clear DLAB after setting DLL,DLM
}

static void UART_EnableBuffers(UART_Init_s *init_handle)
{		
	init_handle->UARTx->FCR = SBIT_FIFO | SBIT_RXFIFO | SBIT_TXFIFO; // UARTx FIFO tx & rx enable and clear
}

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

static void UART_PinInit(UART_Init_s *init_handle){
	
	uint32_t uart_n = init_handle->UARTNumber;	
	switch(uart_n){
		
		case 0x0:
		GPIO_PinFunction (PORT_0, PIN_2, GPIO_PINSEL_FUNCTION_1);
		//GPIO_PinDirection( PORT_0, PIN_2, OUTPUT); // Not Needed
		GPIO_PinFunction (PORT_0, PIN_3, GPIO_PINSEL_FUNCTION_1);
		//GPIO_PinDirection( PORT_0, PIN_2, INPUT); // Not Needed
		break;
		case 0x1:
		GPIO_PinFunction (PORT_0, PIN_15, GPIO_PINSEL_FUNCTION_1);
		//GPIO_PinDirection( PORT_0, PIN_15, OUTPUT);
		GPIO_PinFunction (PORT_0, PIN_16, GPIO_PINSEL_FUNCTION_1);
		//GPIO_PinDirection( PORT_0, PIN_16, INPUT);
		break;
		case 0x2:
		GPIO_PinFunction (PORT_0, PIN_10, GPIO_PINSEL_FUNCTION_1);
		//GPIO_PinDirection( PORT_0, PIN_10, OUTPUT);
		GPIO_PinFunction (PORT_0, PIN_11, GPIO_PINSEL_FUNCTION_1);
		//GPIO_PinDirection( PORT_0, PIN_11, INPUT);
		break;
		case 0x3:
		GPIO_PinFunction (PORT_0, PIN_0, GPIO_PINSEL_FUNCTION_2);
		//GPIO_PinDirection( PORT_0, PIN_1, OUTPUT);
		GPIO_PinFunction (PORT_0, PIN_1, GPIO_PINSEL_FUNCTION_2);
		//GPIO_PinDirection( PORT_0, PIN_1, INPUT);
		break;		
	}	
}

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


/***************************************************************************/
/**** 							Public functions						****/
/***************************************************************************/

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

void UART_tx(UART_Data_s *data_handle, uint32_t *tx_buffer, uint32_t len, txType type)
{
	if(type==Blocking)
		UART_Blocking_tx(data_handle, tx_buffer, len);
	else if(type==ISR)
		UART_IRQ_tx(data_handle, tx_buffer, len);
		
}

static void UART_Blocking_tx(UART_Data_s *data_handle, uint32_t *tx_buffer, uint32_t len)
{
	data_handle->data_fetch = 1;
	data_handle->tx_buffer_ptr_32 = tx_buffer;
	data_handle->tx_length = len;
	data_handle->tx_counter = len;
	
	while(data_handle->tx_counter>0){
		if(data_handle->UARTx->LSR & UART_LSR_THRE){
			UART_LtS(data_handle);
			UART_load_fifo(data_handle);
		}
	}
}

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
			data_handle->UARTx->IER |= SBIT_THRE_IE; // Enable UARTX THRE Interrupt
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

void UART_rx(UART_Data_s *data_handle)
{	
	data_handle->Flags->RBR = 0;
	
	if(data_handle->rx_offset > RX_BUFFER_SIZE){ //increase RX_BUUFER_SIZE for bigger buffer
		data_handle->rx_offset=0;
	}
	data_handle -> rx_buffer_a_8[data_handle->rx_offset] = data_handle->UARTx->RBR;
	//data_handle->rx_buffer_a_8[0] = data_handle->UARTx->RBR;
	data_handle->rx_offset++;
	
	UART_E_RBR_I(data_handle); // enables RBR interrupt
	//data_handle->UARTx->IER |= SBIT_RBR_IE; // enables RBR interrupt
}

//void uart_TxChar(char ch)
//{
//	uint8_t f = 0; 
//	while(f==0){
//			if(~LPC_UART0->LSR) // Wait for Previous transmission
//			{	
//				f++;
//				LPC_UART0->THR=ch;
//				LED_on(1);				
//			}			
//		}   
//}

void UART_D_THRE_I(UART_Data_s *data_handle){
	data_handle->UARTx->IER &= ~SBIT_THRE_IE;	
}

void UART_E_RBR_I(UART_Data_s *data_handle){
	data_handle->UARTx->IER |= SBIT_RBR_IE; // enables RBR interrupt
}

void UART_D_RBR_I(UART_Data_s *data_handle){
	data_handle->UARTx->IER &= ~SBIT_RBR_IE; // disables RBR interrupt
}

void UART0_IRQHandler_F(void){	
	uint8_t IntStatus = (LPC_UART0->IIR);
	uint8_t status = IntStatus & UART_INT_STATUS;
	uint8_t ID = IntStatus & UART_INT_ID;
	ID = ID >> 1;
	if(status==0){
		switch(ID){
			case 0x03:
				F0.ERR = 1;	// error occured VOLATILE VARS, TO MAIN
			break;
			case 0x02:				
				F0.RBR = 1;	// receive data available
				LPC_UART0->IER &= ~SBIT_RBR_IE;
			break;
			case 0x06:
				F0.TO = 1;	// character time-out indicator
			break;
			case 0x01:
				F0.THRE = 1;	// THRE
			break;
		}			
	}
	//else
		// error with interrupt, volatile var again		
}

void UART2_IRQHandler_F(void){	
	uint8_t IntStatus = (LPC_UART2->IIR);
	uint8_t status = IntStatus & UART_INT_STATUS;
	uint8_t ID = IntStatus & UART_INT_ID;
	ID = ID >> 1;
	
	if(status==0){
		switch(ID){
			case 0x03:
				F2.ERR = 1;	// error occured VOLATILE VARS, TO MAIN
			break;
			case 0x02:				
				F2.RBR = 1;	// receive data available
				LPC_UART2->IER &= ~SBIT_RBR_IE;
			break;
			case 0x06:
				F2.TO = 1;	// character time-out indicator
			break;
			case 0x01:
				F2.THRE = 1;	// THRE
			break;
		}			
	}
	//else
		// error with interrupt, volatile var again		
}

void UART3_IRQHandler_F(void){	
	uint8_t IntStatus = (LPC_UART3->IIR);
	uint8_t status = IntStatus & UART_INT_STATUS;
	uint8_t ID = IntStatus & UART_INT_ID;
	ID = ID >> 1;
	
	if(status==0){
		switch(ID){
			case 0x03:
				F3.ERR = 1;	// error occured VOLATILE VARS, TO MAIN
			break;
			case 0x02:				
				F3.RBR = 1;	// receive data available
				LPC_UART3->IER &= ~SBIT_RBR_IE;
			break;
			case 0x06:
				F3.TO = 1;	// character time-out indicator
			break;
			case 0x01:
				F3.THRE = 1;	// THRE
			break;
		}			
	}
	//else
		// error with interrupt, volatile var again		
}
