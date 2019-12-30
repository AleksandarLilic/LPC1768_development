
/* ================ HEADER GUARD ================ */

#ifndef _LPC1768_UART_H
#define _LPC1768_UART_H

#include "lpc17xx.h"
#include "gpio_driver.h"

/* ================ Bit Macro Defs ================ */

#define UART0_PCONP_ON		((uint32_t)1<<3)	// UART0 power/clock control bit
#define UART1_PCONP_ON		((uint32_t)1<<4)	// UART1 power/clock control bit
#define UART2_PCONP_ON		((uint32_t)1<<24)	// UART2 power/clock control bit
#define UART3_PCONP_ON		((uint32_t)1<<25)	// UART3 power/clock control bit
#define SBIT_DLAB			((uint32_t)1<<7)	// Divisor Latch Access Bit
#define SBIT_FIFO			((uint32_t)1)		// FIFO enable bit
#define SBIT_RXFIFO			((uint32_t)1<<1)	// FIFO RX clear bit
#define SBIT_TXFIFO			((uint32_t)1<<2)	// FIFO TX clear bit
#define SBIT_WORD_LENGTH	((uint32_t)0)		// Word Length bits
#define SBIT_STOP_BIT		((uint32_t)1<<2)	// Stop Bit Select
#define SBIT_PARITY_EN		((uint32_t)1<<3)	// Parity Enable Bit
#define SBIT_RBR_IE			((uint32_t)1)		// Receive Data Availabe Interrupt Enable
#define SBIT_THRE_IE		((uint32_t)1<<1)	// Transmit Hold Register Empty Interrupt Enable

/* ================ UARTn Interrupt Identification Register - UnIIR ================ */

#define UART_INT_STATUS		((uint32_t)1)		// Interrupt Status
#define UART_INT_ID			((uint32_t)0x7<<1)	// Interrupt ID Mask

/* ================ UARTn Line Status Register - UnLSR ================ */

#define UART_LSR_RDR		((uint32_t)1)		// Read Data Ready
#define UART_LSR_OE			((uint32_t)1<<1)	// Overwrite Error - FIFO full and new data in RSR
#define UART_LSR_PE			((uint32_t)1<<2)	// Parity Error occurred in transmission
#define UART_LSR_FE			((uint32_t)1<<3)	// Framing Error
#define UART_LSR_BI			((uint32_t)1<<4)	// Break Interrupt
#define UART_LSR_THRE		((uint32_t)1<<5)	// Transmitter Holding Buffer Empty
#define UART_LSR_TEMT		((uint32_t)1<<6)	// Transmitter Empty - both THR and TSR
#define UART_LSR_RXFE		((uint32_t)1<<7)	// Char with error loaded into RBR

/* ================ UARTn Line Control Register - UnLCR ================ */

#define UART_LCR_WLS5		~((uint32_t)0x3)		// 5-bit character length
#define UART_LCR_WLS6		((uint32_t)1)			// 6-bit character length
#define UART_LCR_WLS7		((uint32_t)1<<1)		// 7-bit character length
#define UART_LCR_WLS8		((uint32_t)0)			// 8-bit character length
#define UART_LCR_SBS1		~((uint32_t)1<<2)		// 1 stop bit
#define UART_LCR_SBS2		((uint32_t)1<<2)		// 2 stop bits
#define UART_LCR_PARITY_DISABLE		~((uint32_t)1<<3)	// Disable parity generation and checking
#define UART_LCR_PARITY_ENABLE		((uint32_t)1<<3)	// Enable parity generation and checking
#define UART_LCR_PARITY_SELECT_ODD	~((uint32_t)0x30)	// Odd parity
#define UART_LCR_PARITY_SELECT_EVEN	((uint32_t)0x10) 	// Even parity
#define UART_LCR_PARITY_SELECT_F1S	((uint32_t)0x20)	// Forced "1" stick parity
#define UART_LCR_PARITY_SELECT_F0S	((uint32_t)0x30)	// Forced "0" stick parity
#define UART_LCR_BC_ENABLE			~((uint32_t)1<<6)	// Disable break transmission
#define UART_LCR_BC_DISABLE			((uint32_t)1<<6)	// Enable break transmission
#define UART_LCR_DLAB_DISABLE		~((uint32_t)1<<7)	// Disable access to Divisor Latches
#define UART_LCR_DLAB_ENSABLE		((uint32_t)1<<7)	// Enable access to Divisor Latches

/* ================ Buffers ================ */

#define RX_BUFFER_SIZE 32

/* ================ Data Structures ================ */

/**
 * Holds error flags for UART peripheral
 * 
 * @param none 
 * @return none
 */
typedef struct
{
	uint32_t ERR;	//
	uint32_t RBR;
	uint32_t TO;
	uint32_t THRE;	
}UART_Flags;

typedef struct
{
	uint32_t 	BaudRate;		// Set in UART_SetBaudRate
	uint32_t 	WordLength;		// Set in UART_Format
	uint32_t 	StopBits;		// Set in UART_Format
	uint32_t 	Parity;			// Set in UART_Format
	uint32_t 	UARTNumber;
	LPC_UART_TypeDef *UARTx;
}UART_Init_s;

typedef struct
{
	uint32_t 	tx_length;
	uint32_t	rx_length;
	uint8_t		tx_buffer_a_8[16];
	uint8_t		rx_buffer_a_8[RX_BUFFER_SIZE];
	uint32_t	*tx_buffer_ptr_32;
	uint32_t	tx_counter;
	uint8_t		data_fetch;
	uint8_t 	offset;
	uint8_t		rx_offset;
	uint8_t		UARTNumber;
	LPC_UART_TypeDef *UARTx;
	UART_Flags *Flags;
}UART_Data_s;

typedef enum
{
	Blocking,
	ISR
}txType;


/* ================ Private Functions ================ */

static void UART_Power_Enable(UART_Init_s *init_handle);
static void UART_Power_Disable(UART_Init_s *init_handle);
static void UART_Get_Number(UART_Init_s *init_handle, UART_Data_s *data_handle);
static uint32_t UART_GetClock(UART_Init_s *init_handle);
static void UART_SetBaudRate(UART_Init_s *init_handle, uint32_t uart_pclk);
static void UART_EnableBuffers(UART_Init_s *init_handle);
static void UART_Format(UART_Init_s *init_handle);
static void UART_PinInit(UART_Init_s *init_handle);

static void UART_LtS(UART_Data_s *data_handle);
static void UART_StL(UART_Data_s *data_handle);

static void UART_load_fifo(UART_Data_s *data_handle);

static void UART_IRQ_tx(UART_Data_s *data_handle, uint32_t *tx_buffer, uint32_t len);
static void UART_Blocking_tx(UART_Data_s *data_handle, uint32_t *tx_buffer, uint32_t len);

/* ================ Public Functions ================ */

/**
  * UART peripheral initialization
  * *init_handle: pointer to UART_Init_s structure
  */
void UART_Init(UART_Init_s *init_handle, UART_Data_s *data_handle);

/**
  * UART data transmission
  * *data_handle: pointer to UART_Data structure
  * *tx_buffer: pointer to the TX buffer
  * len: length of the data to be transmitted
  */
void UART_tx(UART_Data_s *data_handle, uint32_t *tx_buffer, uint32_t len, txType type);

/**
  * UART data reception
  * *data_handle: pointer to UART_Data structure
  * *rx_buffer: pointer to the RX buffer
  * len: length of the data to be received
  */
void UART_rx(UART_Data_s *data_handle);//, uint8_t *rx_ptr);

/**
  * UART Interrupt Handler Function
  */	
void UART0_IRQHandler_F(void);

/**
  * UART Interrupt Handler Function
  */	
void UART2_IRQHandler_F(void);

/**
  * UART Interrupt Handler Function
  */	
void UART3_IRQHandler_F(void);

/**
  * UART Interrupt Handler Function
  */	
void UART0_IRQHandler_F(void);

/**
  * UART Enable RBR Interrupt Function
  */	
void UART_E_RBR_I(UART_Data_s *data_handle);

/**
  * UART Disable RBR Interrupt Function
  */	
void UART_D_RBR_I(UART_Data_s *data_handle);

/**
  * UART Disable THRE Interrupt Function
  */	
void UART_D_THRE_I(UART_Data_s *data_handle);


//void uart_TxChar(char ch);

#endif /* LPC1768_UART */

