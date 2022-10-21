/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"
#include "queue.h"

/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )


/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/
typedef struct AMessage
{
    char ucMessageID;
    char ucData[ 30];
}xMessage;
QueueHandle_t xStructQueue = NULL;


uint32_t SysTime = 0;														
uint32_t CPU_LOAD = 0;		
uint32_t Task1_InTime = 0, Task1_OutTime = 0, Task1_TotalTime=0;														
uint32_t Task2_InTime = 0, Task2_OutTime = 0, Task2_TotalTime=0;														
uint32_t Task3_InTime = 0, Task3_OutTime = 0, Task3_TotalTime=0;														
uint32_t Task4_InTime = 0, Task4_OutTime = 0, Task4_TotalTime=0;														
uint32_t Task5_InTime = 0, Task5_OutTime = 0, Task5_TotalTime=0;														
uint32_t Task6_InTime = 0, Task6_OutTime = 0, Task6_TotalTime=0;				

	
TaskHandle_t Button_1_Monitor_Handle = NULL;
TaskHandle_t Button_2_Monitor_Handle = NULL;
TaskHandle_t Periodic_Transmitter_Handle = NULL;
TaskHandle_t Uart_Receiver_Handle = NULL;
TaskHandle_t Load_1_Simulation_Handle = NULL;
TaskHandle_t Load_2_Simulation_Handle = NULL;

void vApplicationIdleHook( void )
{
	GPIO_write(PORT_0, PIN8, PIN_IS_HIGH);
	GPIO_write(PORT_0, PIN8, PIN_IS_LOW);
}

void vApplicationTickHook( void )
{
	GPIO_write(PORT_0, PIN9, PIN_IS_HIGH);
	GPIO_write(PORT_0, PIN9, PIN_IS_LOW);
}

void Button_1_Monitor( void * pvParameters )
{		
		xMessage Button_1_rise_edge={'A', "rising edge on button 1     \n"};
		xMessage Button_1_fall_edge={'A', "falling edge on button 1    \n"};
    TickType_t xLastWakeTime;
		uint8_t flag = 2; /*turns to 1 when there is a change*/
		uint8_t currentButton = PIN_IS_HIGH, lastButton = PIN_IS_HIGH;
		const TickType_t xFrequency = 50;
		xLastWakeTime = xTaskGetTickCount();
    configASSERT( ( ( uint32_t ) pvParameters ) == 1 );
		
		vTaskSetApplicationTaskTag( NULL, (void *) 1 );
    for( ;; )
    {
			vTaskDelayUntil( &xLastWakeTime, xFrequency );
			currentButton = GPIO_read(PORT_0, PIN0);
			if (currentButton == lastButton)
				flag = 2;
			else if (currentButton == PIN_IS_HIGH && lastButton == PIN_IS_LOW)
				flag = 1; /*rising edge*/
			else
				flag = 0; /*falling edge*/
			
			if (flag == 0){
				lastButton = PIN_IS_LOW;
				xQueueSend( /* The handle of the queue. */
               xStructQueue,
               /* The address of the xMessage variable.  sizeof( struct AMessage )
               bytes are copied from here into the queue. */
               ( void * ) &Button_1_fall_edge,
               /* Block time of 0 says don't block if the queue is already full.
               Check the value returned by xQueueSend() to know if the message
               was sent to the queue successfully. */
               ( TickType_t ) 0 );
				flag = 2;
			}
			else if (flag == 1)
			{
				lastButton = PIN_IS_HIGH;
				xQueueSend( /* The handle of the queue. */
				 xStructQueue,
				 /* The address of the xMessage variable.  sizeof( struct AMessage )
				 bytes are copied from here into the queue. */
				 ( void * ) &Button_1_rise_edge,
				 /* Block time of 0 says don't block if the queue is already full.
				 Check the value returned by xQueueSend() to know if the message
				 was sent to the queue successfully. */
				 ( TickType_t ) 0 );
				flag = 2;
			}
    }
}

void Button_2_Monitor( void * pvParameters )
{
		xMessage Button_2_rise_edge={'B', "rising edge button 2     \n"};
		xMessage Button_2_fall_edge={'B', "falling edge button 2    \n"};
    TickType_t xLastWakeTime;
		uint8_t flag = 2; /*turns to 1 when there is a change*/
		uint8_t currentButton = PIN_IS_HIGH, lastButton = PIN_IS_HIGH;
		const TickType_t xFrequency = 50;
		xLastWakeTime = xTaskGetTickCount();
    configASSERT( ( ( uint32_t ) pvParameters ) == 1 );
		
		vTaskSetApplicationTaskTag( NULL, (void *) 2 );
    for( ;; )
    {
			vTaskDelayUntil( &xLastWakeTime, xFrequency );
			currentButton = GPIO_read(PORT_0, PIN1);
			if (currentButton == lastButton)
				flag = 2;
			else if (currentButton == PIN_IS_HIGH && lastButton == PIN_IS_LOW)
				flag = 1; /*rising edge*/
			else
				flag = 0; /*falling edge*/
			
			if (flag == 0){
				lastButton = PIN_IS_LOW;
				xQueueSend( /* The handle of the queue. */
               xStructQueue,
               /* The address of the xMessage variable.  sizeof( struct AMessage )
               bytes are copied from here into the queue. */
               ( void * ) &Button_2_fall_edge,
               /* Block time of 0 says don't block if the queue is already full.
               Check the value returned by xQueueSend() to know if the message
               was sent to the queue successfully. */
               ( TickType_t ) 0 );
				flag = 2;
			}
			else if (flag == 1)
			{
				lastButton = PIN_IS_HIGH;
				xQueueSend( /* The handle of the queue. */
				 xStructQueue,
				 /* The address of the xMessage variable.  sizeof( struct AMessage )
				 bytes are copied from here into the queue. */
				 ( void * ) &Button_2_rise_edge,
				 /* Block time of 0 says don't block if the queue is already full.
				 Check the value returned by xQueueSend() to know if the message
				 was sent to the queue successfully. */
				 ( TickType_t ) 0 );
				flag = 2;
			}
    }
}

void Periodic_Transmitter( void * pvParameters )
{
		xMessage Periodic_Transmitter_Message={'C', "Periodic Transmitter Message\n"};	
    TickType_t xLastWakeTime;
		const TickType_t xFrequency = 100;
		xLastWakeTime = xTaskGetTickCount();
    configASSERT( ( ( uint32_t ) pvParameters ) == 1 );
		vTaskSetApplicationTaskTag( NULL, (void *) 3 );

    for( ;; )
    {
			vTaskDelayUntil( &xLastWakeTime, xFrequency );
			xQueueSend( /* The handle of the queue. */
				 xStructQueue,
				 /* The address of the xMessage variable.  sizeof( struct AMessage )
				 bytes are copied from here into the queue. */
				 ( void * ) &Periodic_Transmitter_Message,
				 /* Block time of 0 says don't block if the queue is already full.
				 Check the value returned by xQueueSend() to know if the message
				 was sent to the queue successfully. */
				 ( TickType_t ) 0 );			
    }
}

void Uart_Receiver( void * pvParameters )
{		
		struct AMessage xRxedStructure;
    TickType_t xLastWakeTime;
		const TickType_t xFrequency = 20;
		xLastWakeTime = xTaskGetTickCount();
    configASSERT( ( ( uint32_t ) pvParameters ) == 1 );
		vTaskSetApplicationTaskTag( NULL, (void *) 4 );
    for( ;; )
    {
			vTaskDelayUntil( &xLastWakeTime, xFrequency );
			if( xQueueReceive( xStructQueue,&( xRxedStructure ),( TickType_t ) 10 ) == pdPASS )
      {
         vSerialPutString(xRxedStructure.ucData, sizeof(xRxedStructure.ucData));
      }
    }
}

void Load_1_Simulation( void * pvParameters )
{
    TickType_t xLastWakeTime;
		uint32_t i = 0;
		const TickType_t xFrequency = 10;
		xLastWakeTime = xTaskGetTickCount();
    configASSERT( ( ( uint32_t ) pvParameters ) == 1 );
		vTaskSetApplicationTaskTag( NULL, (void *) 5 );
    for( ;; )
    {
			vTaskDelayUntil( &xLastWakeTime, xFrequency );
			for(i = 0; i<32000; i++)
				i=i;
    }
}

void Load_2_Simulation( void * pvParameters )
{
		uint32_t i = 0;
    TickType_t xLastWakeTime;
		const TickType_t xFrequency = 100;
		xLastWakeTime = xTaskGetTickCount();
    configASSERT( ( ( uint32_t ) pvParameters ) == 1 );
		vTaskSetApplicationTaskTag( NULL, (void *) 6 );
    for( ;; )
    {
			vTaskDelayUntil( &xLastWakeTime, xFrequency );
			for( i = 0; i<72000; i++)
				i=i;

    }
}


/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	xStructQueue = xQueueCreate(
                         /* The number of items the queue can hold. */
                         15,
                         /* Size of each item is big enough to hold the
                         whole structure. */
                         sizeof( xMessage ) );
	
    /* Create Tasks here */
	xTaskPeriodicCreate(
                    Button_1_Monitor,       /* Function that implements the task. */
                    "Button_1_Monitor",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 1,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Button_1_Monitor_Handle,		/* Used to pass out the created task's handle. */
										50													/*periodicity*/										
										);      

	xTaskPeriodicCreate(
                    Button_2_Monitor,       /* Function that implements the task. */
                    "Button_2_Monitor",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 1,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Button_2_Monitor_Handle,		/* Used to pass out the created task's handle. */
										50													/*periodicity*/										
										);
	xTaskPeriodicCreate(
                    Periodic_Transmitter,       /* Function that implements the task. */
                    "Periodic_Transmitter",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 1,    /* Parameter passed into the task. */
                    2,/* Priority at which the task is created. */
                    &Periodic_Transmitter_Handle,		/* Used to pass out the created task's handle. */
										100													/*periodicity*/										
										); 
	xTaskPeriodicCreate(
                    Uart_Receiver,       /* Function that implements the task. */
                    "Uart_Receiver",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 1,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Uart_Receiver_Handle,		/* Used to pass out the created task's handle. */
										20													/*periodicity*/										
										); 
	
	xTaskPeriodicCreate(
                    Load_1_Simulation,       /* Function that implements the task. */
                    "Load_1_Simulation",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 1,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Load_1_Simulation_Handle,		/* Used to pass out the created task's handle. */
										20													/*periodicity*/										
										); 

	xTaskPeriodicCreate(
                    Load_2_Simulation,       /* Function that implements the task. */
                    "Load_2_Simulation",          /* Text name for the task. */
                    100,      /* Stack size in words, not bytes. */
                    ( void * ) 1,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Load_2_Simulation_Handle,		/* Used to pass out the created task's handle. */
										20													/*periodicity*/										
										); 					

	
	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


