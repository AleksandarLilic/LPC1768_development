
/*============ HEADER GUARD =========  */

#ifndef _SPI_DRIVER_H
#define _SPI_DRIVER_H


#include "lpc17xx.h"
#include "gpio_driver.h"


#define SPI_CPHA_BIT  3
#define SPI_CPOL_BIT	4
#define SPI_LSBF_BIT	6
#define SPI_MSTR_BIT	5


/* ======== Macros for SPI control register ================ */

/** Bit enable, the SPI controller sends and receives the number
 * of bits selected by bits 11:8 */
#define SPI_SPCR_BIT_EN						((uint32_t)(1<<2))
/** Clock phase control bit */
#define SPI_SPCR_CPHA_SECOND		((uint32_t)(1<<3))
/** Clock polarity control bit */
#define SPI_SPCR_CPOL_LOW				 ((uint32_t)(1<<4))
/** SPI master mode enable */
#define SPI_SPCR_MSTR						 ((uint32_t)(1<<5))
/** LSB enable bit */
#define SPI_SPCR_LSBF						((uint32_t)(1<<6))
/** SPI interrupt enable bit */
#define SPI_SPCR_SPIE						((uint32_t)(1<<7))
/**	When bit 2 of this register is 1, this field controls the
number of bits per transfer */
#define SPI_SPCR_BITS(n)				((n==0) ? ((uint32_t)0) : ((uint32_t)((n&0x0F)<<8)))
/** SPI Control bit mask */
#define SPI_SPCR_BITMASK				((uint32_t)(0xFFC))


/*********************************************************************//**
 * 						Macro defines for  SPI Status Register
 **********************************************************************/
/** Slave abort */
#define SPI_SPSR_ABRT        ((uint32_t)(1<<3))
/** Mode fault */
#define SPI_SPSR_MODF        ((uint32_t)(1<<4))
/** Read overrun */
#define SPI_SPSR_ROVR        ((uint32_t)(1<<5))
/** Write collision */
#define SPI_SPSR_WCOL        ((uint32_t)(1<<6))
/** SPI transfer complete flag */
#define SPI_SPSR_SPIF        ((uint32_t)(1<<7))
/** SPI Status bit mask */
#define SPI_SPSR_BITMASK     ((uint32_t)(0xF8))


/*********************************************************************//**
 * Macro defines for SPI Data Register
 **********************************************************************/
/** SPI Data low bit-mask */
#define SPI_SPDR_LO_MASK    ((uint32_t)(0xFF))
/** SPI Data high bit-mask */
#define SPI_SPDR_HI_MASK    ((uint32_t)(0xFF00))
/** SPI Data bit-mask */
#define SPI_SPDR_BITMASK    ((uint32_t)(0xFFFF))

/*********************************************************************//**
 * Macro defines for SPI Clock Counter Register
 **********************************************************************/
/** SPI clock counter setting */
#define SPI_SPCCR_COUNTER(n)     ((uint32_t)(n&0xFF))
/** SPI clock counter bit-mask */
#define SPI_SPCCR_BITMASK        ((uint32_t)(0xFF))


/*********************************************************************//**
 * Macro defines for SPI Interrupt Register
 **********************************************************************/
/** SPI interrupt flag */
#define SPI_SPINT_INTFLAG     ((uint32_t)(1<<0))
/** SPI interrupt register bit mask */
#define SPI_SPINT_BITMASK     ((uint32_t)(0x01))



#define SPI_POWER_BIT            ((uint32_t) 1 << 8)


/*********************************************************************//**
 * SPI configuration parameter defines
 **********************************************************************/
/** Clock phase control bit */
#define SPI_CPHA_FIRST            ((uint32_t)(0))
#define SPI_CPHA_SECOND            SPI_SPCR_CPHA_SECOND
#define PARAM_SPI_CPHA(n)     ((n==SPI_CPHA_FIRST) || (n==SPI_CPHA_SECOND))
 
/** Clock polarity control bit */
#define SPI_CPOL_HI                ((uint32_t)(0))
#define SPI_CPOL_LO                SPI_SPCR_CPOL_LOW
#define PARAM_SPI_CPOL(n)    ((n==SPI_CPOL_HI) || (n==SPI_CPOL_LO))
 
/** SPI master mode enable */
#define SPI_SLAVE_MODE            ((uint32_t)(0))
#define SPI_MASTER_MODE            SPI_SPCR_MSTR
#define PARAM_SPI_MODE(n)    ((n==SPI_SLAVE_MODE) || (n==SPI_MASTER_MODE))
 
/** LSB enable bit */
#define SPI_DATA_MSB_FIRST        ((uint32_t)(0))
#define SPI_DATA_LSB_FIRST        SPI_SPCR_LSBF
#define PARAM_SPI_DATA_ORDER(n) ((n==SPI_DATA_MSB_FIRST) || (n==SPI_DATA_LSB_FIRST))
 
/** SPI data bit number defines */
#define SPI_DATABIT_16        SPI_SPCR_BITS(0)        /*!< Databit number = 16 */
#define SPI_DATABIT_8         SPI_SPCR_BITS(0x08)     /*!< Databit number = 8 */
#define SPI_DATABIT_9         SPI_SPCR_BITS(0x09)     /*!< Databit number = 9 */
#define SPI_DATABIT_10        SPI_SPCR_BITS(0x0A)     /*!< Databit number = 10 */
#define SPI_DATABIT_11        SPI_SPCR_BITS(0x0B)     /*!< Databit number = 11 */
#define SPI_DATABIT_12        SPI_SPCR_BITS(0x0C)     /*!< Databit number = 12 */
#define SPI_DATABIT_13        SPI_SPCR_BITS(0x0D)     /*!< Databit number = 13 */
#define SPI_DATABIT_14        SPI_SPCR_BITS(0x0E)     /*!< Databit number = 14 */
#define SPI_DATABIT_15        SPI_SPCR_BITS(0x0F)     /*!< Databit number = 15 */
#define PARAM_SPI_DATABIT(n)    ((n==SPI_DATABIT_16) || (n==SPI_DATABIT_8)


#define SPI_ENABLE_BIT 						((uint32_t) 01U << 8)  


/*********************************************************************//**
 *					 DATA STRUCTURES
 **********************************************************************/


// configuration info

typedef struct 
{
	uint32_t			CPHA;
	
	uint32_t			CPOL;
	
	uint32_t			Mode;								// Master or Slave
	
	uint32_t			DataOrder;          //MSB or LSB

	uint32_t			ClockRate;
	
}SPI_Init_Cfg_Struc;


typedef struct
{
	uint32_t		tx_length;
	
	uint32_t		rx_length;
	
	uint8_t			*tx_buffer;
	
	uint8_t			*rx_buffer;
	
	uint32_t		rx_counter;
	
	uint32_t		tx_counter;
	
	uint32_t		status;
	
}SPI_Transfer_Cfg_Struc;


/*********************************************************************//**
 * 						public functions
 **********************************************************************/
	
void SPI_Init(SPI_Init_Cfg_Struc *init_cfg);

void SPI_DeInit(void);

//void SPI_SetClock(uint32_t clock);

void SPI_MasterSendData(uint8_t *buffer, uint32_t length);

void SPI_MasterReceiveData(uint8_t *buffer, uint32_t length);

void SPI_SlaveSendData(uint8_t *buffer, uint32_t length);

void SPI_SlaveReceiveData(uint8_t *buffer, uint32_t length);

void SPI_TransferData(const void *data_out,void *data_in,  uint32_t length);

void SPI_IRQHandler_func(void);


 void SPI_EnableInterrupt();

 void SPI_DisableInterrupt();
#endif







