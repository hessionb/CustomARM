#ifndef vtUART_H__
#define vtUART_H__

#include <stdlib.h>
#include <stdio.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"
#include "semphr.h"

/* include files. */
#include "lpc17xx_uart.h"
#include "vtUtilities.h"

#include "lpc17xx_libcfg_default.h"
#include "lpc17xx_pinsel.h"

#include "messages.h"

// return codes for UartInit()
#define UartErrInit -1
#define UartInitSuccess 0

// Structure that is used to define the operate of an UART peripheral using the Uart routines
//   It should be initialized by UartInit() and then not changed by anything... ever
//   A user of the API should never change or access it, it should only pass it as a parameter
typedef struct __UARTStruct {
	uint8_t devNum;	  						// Number of the UART peripheral (0,1,2 on the 1768)
	LPC_UART_TypeDef *devAddr;	 			// Memory address of the UART peripheral
	unsigned portBASE_TYPE taskPriority;   	// Priority of the UART task
	xSemaphoreHandle binSemaphore;		   	// Semaphore used between UART task and UART interrupt handler
	xQueueHandle inQ;					   	// Queue used to send messages from other tasks to the UART task
	xQueueHandle outQ;						// Queue used by the UART task to send out results
} UARTStruct;

/* ********************************************************************* */
// The following are the public API calls that other tasks should use to work with the UART task

// Args:
//   dev: pointer to the UARTStruct data structure
//   uartDevNum: The number of the uart device -- 0, 1, or 2
//   taskPriority: At what priority should this task be run?
//   uartCfg: Contains the setting for baudrate, parity, stop bits, and data bits
//	 fifoCfg: Contains the serrtings for fifo operation
// Return:
//   if successful, returns UartInitSuccess
//   if not, should return UartErrInit
// Must be called for each UART device initialized (0, 1, or 2) and used
int uartInit(UARTStruct *devPtr,uint8_t uartDevNum,unsigned portBASE_TYPE taskPriority,UART_CFG_Type* uartCfg, UART_FIFO_CFG_Type* fifoCfg);

// A simple routine to use for filling out and sending a message to the UART thread
// Args
//   dev: pointer to the UARTStruct data structure
//   txLen: The number of bytes you want to send
//   txBuf: The buffer holding the bytes you want to send
// Return:
//   Result of the call to xQueueSend()
portBASE_TYPE sendUartMsg(UARTStruct *dev,uint8_t txLen, uint8_t *txBuf);
#endif