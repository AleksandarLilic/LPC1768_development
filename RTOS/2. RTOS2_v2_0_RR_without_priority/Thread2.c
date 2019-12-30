/*----------------------------------------------------------------------------
 * Name:      Thread2.c
 * Purpose: 	CMSIS-RTOS test program v2.0
 * Designer: 	Aleksandar
 *----------------------------------------------------------------------------*/

// CMSIS RTOS header file
#include "cmsis_os2.h"
#include "LPC17xx.h"

/*----------------------------------------------------------------------------
 *      Thread 2 'LED_two': Sample thread
 *---------------------------------------------------------------------------*/
 
// Thread function declaration
void Thread2 (void *argument);
// Thread id, used for interaction with a thread
osThreadId_t tid_Thread2;
// Thread status
osStatus_t T2_status;

// Externally defined delay function
extern int delay2(int);

// Thread initialization function definition:
int Init_Thread2 (void) {

  // Creation of a new thread with Thread ID as a return value if successful:
	tid_Thread2 = osThreadNew (Thread2, NULL, NULL);
  if (!tid_Thread2) return(-1);
  return(0);
}

// Thread definition:
__NO_RETURN void Thread2 (void *argument) {
	
// Priority implementation
//	T2_status = osThreadSetPriority (tid_Thread2, osPriorityNormal2); // Priority = normal + 2 
//  if (T2_status == osOK)  {
//      // Thread priority changed to Normal + 2
//    }
//    else {
//			// Failed to set the priority
//		}	

  while (1) {
    
		//uint32_t port = LPC_GPIO1->FIOPIN; // Take current state of a port
		LPC_GPIO1->FIOSET = 0x00100000; // Set LED2
		
		LPC_GPIO0->FIOCLR0 = 0x40;	//Bar2.0 pin0.6
		delay2(500); 			// Delay, calculated (simulated workload), in ms
		LPC_GPIO0->FIOCLR0 = 0x80;	//Bar2.1 pin0.7
		delay2(500);
		LPC_GPIO0->FIOCLR1 = 0x01;	//Bar2.2 pin0.8
		delay2(500);
		LPC_GPIO0->FIOCLR1 = 0x02;	//Bar2.3 pin0.9
		delay2(500);
		LPC_GPIO0->FIOSETL = 0x3C0;	//Bar2 clear all pins (6,7,8,9)
		
		LPC_GPIO1->FIOCLR = 0x00100000; 	// Clear LED2
		osDelay(3000); 		// OsDelay in ms
		
    //osThreadYield ();// suspend thread
  }
}
