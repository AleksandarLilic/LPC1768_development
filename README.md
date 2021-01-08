# NXP LPC1768 Development

**Description:**  

CMSIS-RTOS2 application with demo switching tasks   
Driver development for peripherals  
Data acquisition project with developed drivers

RTOS application was a part of Real-time systems course. Demo tasks are used as an example.

Driver development and their implementation were done as a project for two courses: Automotive electronics and Data acquisition systems. Communication with processor was done through Android app via Bluetooth.

Source files are available for every project and should just slot in IDE of choice.  

**Status:**   
Project is considered finished  

**Further development:**  
No further development is planned  

## **AE & DAS: Driver support and application**

**Description:**  

Driver development for LPC1768 peripherals:
- GPIO
- GPIO extension for onboard LEDs
- Timer
- UART
- SPI
- ADC
- BME280 specific: adaptation of a provided HAL  

Each peripheral has it's own *.h* and *.c* file, so they can be used separately or together.  

Data acquisition application is used as an example of the use case for developed drivers.
In the beginning, all peripherals are initialized and interrupts enabled. The main function is then used just to check raised flags by ISRs. This time can be used to put CPU to sleep if needed.  

The processor is communicating via SPI with Bosch BME280 sensor and via UART to Bluetooth module and/or terminal on PC. Analog sensor TMP36 from Analog Devices is taped to the CPU to measure temperatures under load. Android app is used, via Bluetooth module, for sending refresh rate controls to the microcontroller and receiving sensor data from it. 

![Android Application](https://github.com/AleksandarLilic/LPC1768_development/tree/master/Drivers\images\android_app.png)  


**Status:**   
Finished


## **RTS course: CMSIS-RTOS2 application**

**Description:**  

Project is used as a demonstration of what is possible with RTOS compared to bare-metal programming. Tasks are used in conjunction with four LEDs ( [schematic](https://github.com/AleksandarLilic/LPC1768_development/blob/master/RTOS/Schematic.pdf) ) for each thread to provide a visual representation of what happens with running threads in different scenarios. Every application uses system tick of 1000Hz, so each thread is executed 1ms at a time.  

*Time for scheduler is not diplayed on the graphs below. Although a very small portion of time is used for scheduling at every system tick, the goal of graphs is thread execution pattern.

The first example shows a typical use case for RTOS: Round-robin with a thread priority. Thread 1 achieves the highest determinism as it is a top priority for the execution. Thread 2 is one level lower and thread 3 one lower than that. 

//time graph to be added  

The second example is Round-robin without priorities. As there is no preemption, the end result is a seemingly parallel execution of all three tasks. Slowed down version clearly shows what is actually going on with the workload as threads are executed one at a time.

//time graph to be added  

The third and final example uses Round-robin without priorities but with Thread Flags. The reasoning for its use would be sensor data processing. It wouldn't make sense to start working while new data has not arrived or is in currently arriving. Three threads are circularly passing one flag. Fourth thread is used to start the whole process and is then suspended.  

//time graph to be added

**Status:**   
Finished
