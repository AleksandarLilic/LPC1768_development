/*----------------------------------------------------------------------------
 * Name:      Thread3.c
 * Purpose: 	CMSIS-RTOS test program v3.0
 * Designer: 	Aleksandar
 *----------------------------------------------------------------------------*/

// CMSIS RTOS header file
#include "cmsis_os2.h"
#include "LPC17xx.h"

/*----------------------------------------------------------------------------
 *      Thread 3 'LED_three': Sample thread
 *---------------------------------------------------------------------------*/
 
// Thread function declaration
void Thread3 (void *argument);
// Thread id, used for interaction with a thread
osThreadId_t tid_Thread3;
// Thread status
osStatus_t T3_status;

// Externally defined delay function
extern int delay(int);
// Externally defined Thread ID
extern osThreadId_t tid_Thread1;

// Thread initialization function definition:
int Init_Thread3 (void) {

  // Creation of a new thread with Thread ID as a return value if successful:
	tid_Thread3 = osThreadNew (Thread3, NULL, NULL);
  if (!tid_Thread3) return(-1);
  return(0);
}

// Thread definition:
__NO_RETURN void Thread3 (void *argument) {
	
// Priority implementation
//	T3_status = osThreadSetPriority (tid_Thread3, osPriorityNormal1); // Priority = normal + 1 
//  if (T3_status == osOK)  {
//      // Thread priority changed to Normal + 1
//    }
//    else {
//			// Failed to set the priority
//		}	

  while (1) {
    
		// Thread Flag wait
		osThreadFlagsWait(0x0001U, osFlagsWaitAny, osWaitForever);
    
		//uint32_t port = LPC_GPIO1->FIOPIN; // Take current state of a port
		LPC_GPIO1->FIOSET = 0x00200000; // Set LED3
		
		LPC_GPIO0->FIOCLR1 = 0x4;		//Bar3.0 pin0.10
		delay(500); 			// Delay, calculated (simulated workload), in ms
		LPC_GPIO0->FIOCLR1 = 0x8;		//Bar3.1 pin0.11
		delay(500);
		LPC_GPIO0->FIOCLR1 = 0x80;	//Bar3.2 pin0.15
		delay(500);
		LPC_GPIO0->FIOCLR2 = 0x01;	//Bar3.3 pin0.16
		delay(500);
		LPC_GPIO0->FIOSET = 0x18c00;//Bar3 clear all pins (10,11,15,16)
		
		LPC_GPIO1->FIOCLR = 0x00200000; 	// Clear LED3
		osDelay(2000); 		// OsDelay in ms
		
		//Set Thread Flag for Thread1
		osThreadFlagsSet(tid_Thread1, 0x0001U);
		
    //osThreadYield ();// suspend thread
  }
}
