#include "lpc17xx.h"   


// Blocking Delay Function
void delay(unsigned int count)
{
  unsigned int i,j;
  for(i=0;i<count;i++)
  {
    for(j=0;j<20200;j++);
  }
}

/* start the main program */
int main() 
{
  //Clock and PLL configuration:
	SystemInit();
	
	//Enable power to GPIO & IOCON:
	LPC_SC->PCONP     |= (1 << 15);
	
	//Configure the Pins for GPIO:
	LPC_PINCON->PINSEL3 &= ~(1<<2 | 1<<4 | 1<<5 | 1<<7);
	
	//Configure the PORT pins as OUTPUT;
	LPC_GPIO1->FIODIR = ((1<<18) | (1<<20) | (1<<21) | (1<<23));

  while(1)
    {

     /* Turn ON all the leds and wait for ~1  second */ 
       LPC_GPIO1->FIOSET = ((1<<18) | (1<<20) | (1<<21) | (1<<23)); 
       delay(1000);

     /* Turn OFF all the LEDs and wait for ~1 second */ 
       LPC_GPIO1->FIOCLR = ((1<<18) | (1<<20) | (1<<21) | (1<<23));
       delay(1000);
    }
}
