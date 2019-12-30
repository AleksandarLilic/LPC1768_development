#include <stdint.h>
#include "lpc17xx.h"
#include "gpio_driver.h"
#include "Spi_Driver_1768.h"
#include "LED4.h"

#define TRANSFER_IN_PROGRESS_READ	(0x01)
#define TRANSFER_IN_PROGRESS_SEND	(0x05)
#define TRANSFER_COMPLETE_S				(0x00)
#define TRANSFER_FAILED						(0x02)
#define SPI_NOFAULT								(0x00)

#define SPI_STATUS_REG_MSK				(0x78)
#define SPI_SPSR_IF_BIT						(0x80)



typedef struct
{
	uint16_t		tx_length;
	
	uint16_t		data_length;
	
	uint8_t			*tx_buffer;
	
	uint8_t			*data_buffer;
	
	uint32_t		tx_counter;
	
	uint32_t		data_counter;
	
	volatile int8_t		status;
	
}SPI_Transfer_info;


int8_t spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

int8_t spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

void save_read_data(void);

void send_data(void);

void Spi_SetStatus(uint8_t st);

int8_t Spi_GetStatus(void);

void Spi_ClearFlag();

void Spi_SetFlag();


