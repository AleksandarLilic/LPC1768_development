/*----------------------------------------------------------------------------
 * Name:      Thread3.c
 * Purpose: 	CMSIS-RTOS test program v3.0
 * Designer: 	Aleksandar
 *----------------------------------------------------------------------------*/
 
 // CMSIS RTOS header file
#include "cmsis_os2.h"
#include "LPC17xx.h"

/*----------------------------------------------------------------------------
 *      Thread 4 'Thread_StartOS': Sample thread
 *---------------------------------------------------------------------------*/
 
// Thread function declaration
void Thread4 (void *argument);
// Thread id, used for interaction with a thread
osThreadId_t tid_Thread4;
// Thread status
osStatus_t T4_status;

// Externally defined delay function
extern int delay(int);
// Externally defined Thread ID
extern osThreadId_t tid_Thread1;

int Init_Thread4 (void) {

  tid_Thread4 = osThreadNew (Thread4, NULL, NULL);
  if (!tid_Thread4) return(-1);
  
  return(0);
}

__NO_RETURN void Thread4 (void *argument) {

  while (1) {
		LPC_GPIO1->FIOSET = 0x00800000;		// Set LED4
		delay(1000);
		osThreadFlagsSet(tid_Thread1, 0x0001U);
		LPC_GPIO1->FIOCLR = 0x00800000; 	// Clear LED4
		
		osThreadTerminate (tid_Thread4);
  }
}
