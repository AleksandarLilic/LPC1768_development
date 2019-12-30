#include "lpc1768_spi.h"
#include "lpc1768_gpio.h"

SPI_Transfer_Cfg_Struc	tran_cfg;
SPI_Transfer_Cfg_Struc *transfer_cfg = &tran_cfg;




/*********************************************************************//**
 * 						Private functions
 **********************************************************************/
	

//===============ENABLE SPI ==============================
	static void SPI_Enable()
{
	LPC_SC -> PCONP |= SPI_ENABLE_BIT;   // enable power to SPI
}
//===============DISABLE SPI ==============================

static void SPI_Disable()
{
	LPC_SC -> PCONP &= ~SPI_ENABLE_BIT;
}
//===============SET SPI CLOCK ==============================

static void SPI_SetClock(SPI_Init_Cfg_Struc *init_cfg)
{
		uint32_t clock;
    uint32_t spi_peripheralclk;
    uint32_t prescale, temp;
		uint32_t temp1=0;
	
	clock = init_cfg->ClockRate;
	
	temp1 = (LPC_SC ->PCLKSEL0 >> 16) & 0x03;
	
	switch( temp1  )
    {
          case 0x00:
            spi_peripheralclk = SystemCoreClock/4;
            break;
          case 0x01:
            spi_peripheralclk = SystemCoreClock;
            break; 
          case 0x02:
            spi_peripheralclk = SystemCoreClock/2;
            break; 
          case 0x03:
            spi_peripheralclk = SystemCoreClock/8;
            break;
    }    
 
    prescale = 8;
    // Find closest clock to target clock
    while (1){
        temp = clock * prescale;
        if (temp >= spi_peripheralclk){
            break;
        }
        prescale += 2;
        if(prescale >= 254){
            break;
        }
    }
    // Write to register
    LPC_SPI->SPCCR = SPI_SPCCR_COUNTER(prescale);
}

//===============Initialize SPI  PINS ==============================

static void SPI_PinInit(SPI_Init_Cfg_Struc *init_cfg)
{
	
	if( init_cfg->Mode == SPI_SLAVE_MODE)
	{
	GPIO_PinFunction( PORT_0, PIN_18, GPIO_PINSEL_FUNCTION_3);
	GPIO_PinDirection( PORT_0, PIN_18, INPUT);
	GPIO_PinFunction( PORT_0, PIN_17, GPIO_PINSEL_FUNCTION_3);
	GPIO_PinDirection( PORT_0, PIN_17, OUTPUT);
	GPIO_PinFunction( PORT_0, PIN_15, GPIO_PINSEL_FUNCTION_3);
	GPIO_PinDirection( PORT_0, PIN_15, INPUT);	
	GPIO_PinFunction( PORT_0, PIN_16, GPIO_PINSEL_FUNCTION_3);
	GPIO_PinDirection( PORT_0, PIN_16, INPUT);
	}
	else
	{
		GPIO_PinFunction( PORT_0, PIN_18, GPIO_PINSEL_FUNCTION_3);
		GPIO_PinDirection( PORT_0, PIN_18, OUTPUT);
		GPIO_PinFunction( PORT_0, PIN_17, GPIO_PINSEL_FUNCTION_3);
		GPIO_PinDirection( PORT_0, PIN_17, INPUT);
		GPIO_PinFunction( PORT_0, PIN_15, GPIO_PINSEL_FUNCTION_3);
		GPIO_PinDirection( PORT_0, PIN_15, OUTPUT);	
		GPIO_PinFunction( PORT_0, PIN_16, GPIO_PINSEL_FUNCTION_0);
		GPIO_PinDirection( PORT_0, PIN_16, OUTPUT);
	}
}

//===============Initialize SPI  Control Register==============================
static void SPI_ControlRegister_Init(SPI_Init_Cfg_Struc *init_cfg)
{
	uint32_t tmp;
	
	tmp = ((init_cfg->CPHA << SPI_CPHA_BIT) | (init_cfg->CPOL << SPI_CPOL_BIT) | (init_cfg->DataOrder << SPI_LSBF_BIT)  | (init_cfg->Mode << SPI_MSTR_BIT) ) & SPI_SPCR_BITMASK;
	LPC_SPI -> SPCR = tmp;

}

//===============ENABLE SPI INTERRUPT ==============================
 void SPI_EnableInterrupt()
{
	LPC_SPI -> SPCR |= SPI_SPCR_SPIE;
}

//===============DISABLE  SPI INTERRUPT ==============================
 void SPI_DisableInterrupt()
{
	LPC_SPI -> SPCR &=  (~SPI_SPCR_SPIE)  & SPI_SPCR_BITMASK;
}


void SPI_Init(SPI_Init_Cfg_Struc *init_cfg)
{
	uint32_t dummy;
	//ENABLE SPI PERIPHERAl
	SPI_Enable();
	
	//Set SPI clock
	SPI_SetClock(init_cfg);
	
	//Initialize SPI0 pins
	SPI_PinInit(init_cfg);
	
	//Control Register
	SPI_ControlRegister_Init(init_cfg);
	
	dummy = LPC_SPI ->SPSR; //clear status reg
	
	if (LPC_SPI ->SPINT & SPI_SPINT_INTFLAG){    ///clear Interrupt
        LPC_SPI->SPINT = SPI_SPINT_INTFLAG;
	} 
	
	if( init_cfg->Mode == SPI_SLAVE_MODE)
		SPI_EnableInterrupt();
}

void SPI_MasterSendData(uint8_t *buffer, uint32_t length)
{
	uint8_t value;
	
	transfer_cfg -> rx_length = 0;
	transfer_cfg -> rx_buffer = 0;
	transfer_cfg -> tx_buffer = buffer;
	transfer_cfg -> tx_length = length;
	transfer_cfg -> tx_counter = 0;
	transfer_cfg -> rx_counter = 0;
	
	value = *(transfer_cfg->tx_buffer);
	transfer_cfg->tx_buffer++;
	
	GPIO_PinWrite(PORT_0,PIN_16,LOW);  					//SLAVE Select
	LPC_SPI -> SPDR = value;
	transfer_cfg->tx_counter++;
	
	SPI_EnableInterrupt();
	
	
}

void SPI_MasterReceiveData(uint8_t *buffer, uint32_t length)
{
	
	uint32_t dummy;
	//	GPIO_PinWrite(PORT_1,PIN_18, LOW);

	transfer_cfg -> tx_length = 0;
	transfer_cfg -> tx_buffer = 0;
	transfer_cfg ->  rx_buffer = buffer;
	transfer_cfg ->  rx_length = length;
	transfer_cfg ->  rx_counter= 0;
	transfer_cfg -> tx_counter = 0;
		
	dummy = LPC_SPI->SPDR;
	
	
	LPC_SPI->SPDR = 0xAA; //dummy BYTE to start CLK
	//GPIO_PinWrite(PORT_1,PIN_18, LOW);
	transfer_cfg->tx_counter++;
	
	SPI_EnableInterrupt();
}

void SPI_SlaveSendData( uint8_t *buffer, uint32_t length)
{
	uint8_t value;
	
	transfer_cfg -> rx_length = 0;
	transfer_cfg -> rx_buffer = 0;
	transfer_cfg -> tx_buffer = buffer;
	transfer_cfg -> tx_length = length;
	transfer_cfg -> tx_counter = 0;
	transfer_cfg -> rx_counter = 0;
	
	value = *(transfer_cfg->tx_buffer);
	transfer_cfg->tx_buffer++;
	
	LPC_SPI -> SPDR = value;
	transfer_cfg->tx_counter++;
	
	SPI_EnableInterrupt();
}

void SPI_SlaveReadData(uint8_t *buffer, uint32_t length)
{
	
	uint32_t data;
	
	transfer_cfg -> tx_length = 0;
	transfer_cfg -> tx_buffer = 0;
	transfer_cfg->rx_buffer = buffer;
	transfer_cfg->rx_length = length;
	transfer_cfg->rx_counter= 0;
	transfer_cfg -> tx_counter = 0;
	
	data = LPC_SPI->SPDR;
	
	*(transfer_cfg -> rx_buffer) =  data;
	
}


void SPI_IRQHandler_func(void)
{
	uint32_t dummy;
	uint8_t data;
	
	LPC_SPI->SPINT = SPI_SPINT_INTFLAG; // clear Interrupt
	dummy = LPC_SPI->SPSR;		//clear status reg
	
	if ( transfer_cfg -> tx_length >= transfer_cfg->tx_counter)
	{
		if(transfer_cfg->tx_counter == transfer_cfg ->tx_length)
			SPI_DisableInterrupt();
		else
		{
		data = *transfer_cfg->tx_buffer;
		transfer_cfg->tx_buffer++;
		LPC_SPI->SPDR = data;
		transfer_cfg->tx_counter++;
		}
	}
	
	if (transfer_cfg->rx_length > transfer_cfg->rx_counter)
	{
		data = LPC_SPI->SPDR;
		*(transfer_cfg->rx_buffer) = data;
		transfer_cfg->rx_buffer++;
		transfer_cfg->rx_counter++;

	
		if(transfer_cfg->rx_counter == transfer_cfg ->rx_length)
			SPI_DisableInterrupt();
		
		LPC_SPI->SPDR = 0xAA;
		transfer_cfg->tx_counter++;
	}
}

