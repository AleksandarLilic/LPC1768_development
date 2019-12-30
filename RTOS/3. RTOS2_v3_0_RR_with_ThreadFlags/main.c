/*----------------------------------------------------------------------------
 * Name:      main.c
 * Purpose: 	CMSIS-RTOS test program v3.0 - Round Robin with no priority and Thread Flags
 * Designer: 	Aleksandar
 *----------------------------------------------------------------------------*/
 
#include "cmsis_os2.h"
#include "LPC17xx.h"

// Declaration of a external functions (thread initialization)
extern int Init_Thread1(void);
extern int Init_Thread2(void);
extern int Init_Thread3(void);
extern int Init_Thread4(void);


void delay(int count){
	unsigned int i,j;
  for(i=0;i<count;i++)
  {
    for(j=0;j<16660;j++); // ~ 1 milisecond delay
  }	
}

uint32_t coreClock_1 = 0;                       /* Variables to store core clock values */
uint32_t coreClock_2 = 0;

int main (void) {
	
	coreClock_1 = SystemCoreClock;
	// System Initialization:
	SystemCoreClockUpdate();
	// Initialize CMSIS-RTOS:
	coreClock_2 = SystemCoreClock;   
	osKernelInitialize();
	// Initialize all Threads:
	Init_Thread1();
	Init_Thread2();
	Init_Thread3();
	Init_Thread4();
	
	LPC_SC->PCONP     |= (1 << 15);  // Enable power to GPIO & IOCON;
  LPC_PINCON->PINSEL3 = 0x000000;  // Configure the Pins for GPIO: 1.16-1.31;
  LPC_GPIO1->FIODIRH = 0x00B4;     // Configure the Pins on GPIO1: 18, 20, 21, 23 as OUTPUT;
	
	LPC_PINCON->PINMODE_OD0 = 0x00018ff3; //Configure the Pins on GPIO0: 0,1,4,5,6,7,8,9,10,11,15,16 as OPEN DRAIN;
	LPC_GPIO0->FIOSET = 0xffffffff;  // Set PORT0 to Vcc
	
	LPC_PINCON->PINSEL0 = 0x000000;  // Configure the Pins for GPIO: 0.0-0.15;
	LPC_PINCON->PINSEL1 = 0x000000;  // Configure the Pins for GPIO: 0.16-0.31;
	LPC_GPIO0->FIODIR = 0x00018ff3;  // Configure the Pins on GPIO0: 0,1,4,5,6,7,8,9,10,11,15,16 as OUTPUT;
		
	// Kernel run function
	osKernelStart();
	
	// If osKernelStart was successful, code after this line will never execute;
	// If osKernelStart was not successful, it will return here, so error code may be written;
	
	// Optional error code here...
	
	// Endless loop
	for(;;){}
			
}
