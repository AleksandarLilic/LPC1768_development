#include "SPI_transfer_BME280.h"



	 static SPI_Transfer_info transfer_struc;
	 static SPI_Transfer_info *transfer = &transfer_struc;
	 static volatile uint8_t transferFlag = 0;
	 

  /* 			SPI_READ 
	
     * The parameter dev_id can be used as a variable to select which Chip Select pin has
     * to be set low to activate the relevant device on the SPI bus
     
	
     * Data on the bus should be like
     * |----------------+---------------------+-------------|
     * | MOSI           | MISO                | Chip Select |
     * |----------------+---------------------|-------------|
     * | (don't care)   | (don't care)        | HIGH        |
     * | (reg_addr)     | (don't care)        | LOW         |
     * | (don't care)   | (reg_data[0])       | LOW         |
     * | (....)         | (....)              | LOW         |
     * | (don't care)   | (reg_data[len - 1]) | LOW         |
     * | (don't care)   | (don't care)        | HIGH        |
     * |----------------+---------------------|-------------|
     */
		 
		 
int8_t spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
	
	// not needed for reading
	transfer->tx_buffer =0;
	transfer->tx_counter =0;
	transfer->tx_length =0;
	
	// read info
	transfer->data_buffer = reg_data;
	transfer->data_length = len;
	transfer->data_counter = 0;
	
	// Send reg_addr to sensor
	GPIO_PinWrite(PORT_0, PIN_16, LOW);				//slave select 
	LPC_SPI -> SPDR = reg_addr;
	transfer->status = TRANSFER_IN_PROGRESS_READ;
	
	SPI_EnableInterrupt();
	
	//wait to receive all data
	while(transfer->status == TRANSFER_IN_PROGRESS_READ)
	{
		if (transferFlag == 1)
			save_read_data();
	}
	rslt = transfer->status;
  GPIO_PinWrite(PORT_0, PIN_16, HIGH);
	
  return rslt;
}

 /*
     * The parameter dev_id can be used as a variable to select which Chip Select pin has
     * to be set low to activate the relevant device on the SPI bus
     */

    /*
     * Data on the bus should be like
     * |---------------------+--------------+-------------|
     * | MOSI                | MISO         | Chip Select |
     * |---------------------+--------------|-------------|
     * | (don't care)        | (don't care) | HIGH        |
     * | (reg_addr)          | (don't care) | LOW         |
     * | (reg_data[0])       | (don't care) | LOW         |
     * | (....)              | (....)       | LOW         |
     * | (reg_data[len - 1]) | (don't care) | LOW         |
     * | (don't care)        | (don't care) | HIGH        |
     * |---------------------+--------------|-------------|
     */

int8_t spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
  int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
	
	uint8_t dummy;
	dummy = LPC_SPI->SPDR;

	// send info
	transfer->tx_buffer =	reg_data;
	transfer->tx_counter =	0;
	transfer->tx_length =	len;
	
	// not needed for sending 
	transfer->data_buffer = 0;
	transfer->data_length = 0;
	transfer->data_counter = 0;
	
	// Send control byte (reg_addr) to sensor
	GPIO_PinWrite(PORT_0, PIN_16, LOW);				//slave select 
	LPC_SPI -> SPDR = reg_addr;
	transfer->status = TRANSFER_IN_PROGRESS_SEND;
	
	SPI_EnableInterrupt();
	
	while(transfer->status == TRANSFER_IN_PROGRESS_SEND)
	{
		if (transferFlag == 1)
		{
			send_data();
		}
	}
	
  GPIO_PinWrite(PORT_0, PIN_16, HIGH);	//slave select HIGH
	rslt = transfer->status;

  return rslt;
}


	/*	

			========= Function to put received data to memory and clear the read flag ==============

	*/

void save_read_data(void)
{
	uint8_t data;
	
	Spi_ClearFlag();
	data = LPC_SPI->SPDR;
	

	if( (transfer->data_counter <= transfer->data_length) & (transfer->data_counter !=0) )
	{
		//send data to memory
		*(transfer->data_buffer) = data;
		
		//increment buffer pointer
		transfer->data_buffer++;
		
	}
	if ( transfer->data_counter == transfer->data_length)
	{
		SPI_DisableInterrupt();
		transfer->status = TRANSFER_COMPLETE_S;
	}
	else
	{
		LPC_SPI->SPDR = 0xAA; // dummy byte to start transfer
		
		transfer->data_counter++;
	}

	
}


	/*	

			========= Function to write data to SPDR ==============

	*/


void send_data(void)
{
	uint8_t dummy;
	
	Spi_ClearFlag();
	dummy = LPC_SPI->SPDR;   //clear SPDR
	
	
	// transfer completed
	if(transfer->tx_counter == transfer->tx_length)
	{
		SPI_DisableInterrupt();
		transfer->status = TRANSFER_COMPLETE_S;
	}
	else
	{
	// write data to SPDR to start transfer
	LPC_SPI->SPDR = *(transfer->tx_buffer);
	
	transfer->tx_buffer++;
	transfer->tx_counter++;
	}
}

		/*	

			========= Function to set the status ==============

	*/

void Spi_SetStatus(uint8_t st)
{
	transfer->status = st;
}

	/*	

			========= Function that returns the current status ==============

	*/
int8_t Spi_GetStatus(void)
{
	return transfer->status;
}

	/*	

			========= Function to set the Read flag ==============

	*/

void Spi_SetFlag()
{
	transferFlag = 1;
}

	/*	

			========= Function to clear the Read flag ==============

	*/
void Spi_ClearFlag()
{
	transferFlag = 0;
}

