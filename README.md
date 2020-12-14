# NXP LPC1768 Development

**Description:**  

CMSIS-RTOS2 application with demo switching tasks   
Driver development for peripherals  
Data acquisition project with developed drivers

RTOS application was a part of Real-time systems course. Demo tasks are used as an example.

Driver development and their implementation was done as a project for two courses: Automotive electronics and Data acquisition systems. Data from sensors was sent to Android app via bluetooth for display.

Source files are available for every project and should just slot in IDE of choice.  

**Status:**   
Project is considered finished  

**Further development:**  
No further development is planned  

## **AE & DAS: Driver support and application**

**Description:**  
 

**Status:**   
Finished


## **RTS course: CMSIS-RTOS2 application**

**Description:**  

Project is used as a demonstration of what is possible with RTOS compared to bare-metal programming. Tasks are used in conjuction with four LEDs for each thread to provide visual representation of what happens with running threads in different scenarios.  

First example shows typical use case for RTOS: Round-robin with a thread priority. Thread 1 achives highest determinism as it is a top priority for the execution. Thread 2 is one level lower and thread 3 one lower than that. 

//time graph to be added  

Second example is Round-robin without priorities. It uses system tick of 1000Hz, so each thread is executed 1ms at a time. Results are apparent parallel execution of all three tasks. Slowed down version clearly shows what is actually going on with the execution.

//time graph to be added  

Third and final example uses Round-robin without priorities but with Thread Flags. Reasoning for it's use would be a sensor data processing. It wouldn't make sense to start working while new data has not arrived or is in currently arriving. Three threads are passing one flag in a circular fashion. Fourth thread is used to start the whole process and is then suspended.  

//time graph to be added

**Status:**   
Finished
