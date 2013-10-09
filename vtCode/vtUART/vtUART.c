#include "vtUART.h"

/* ************************************************ */
// Private definitions used in the Public API
// Structure used to define the messages that are sent to/from the UART thread 
//typedef struct __UARTMsg {
//	uint8_t msgType; // A field you will likely use in your communications between processors (and for debugging)
//	uint8_t	rxLen;	 // Length of the message you *expect* to receive (or, on the way back, the length that *was* received)
//	uint8_t txLen;   // Length of the message you want to sent (or, on the way back, the length that *was* sent)
//	uint8_t status;  // status of the completed operation -- I've not done anything much here, you probably should...
//	uint8_t buf[UART_TX_FIFO_SIZE]; // On the way in, message to be sent, on the way out, message received (if any)
//} UARTMsg;
// Length of the message queues to/from this task
#define UARTQLen 10 /*FIFOs*/

#define UARTTransferFailed -2
#define UARTIntPriority 7

// Here is where we define an array of pointers that lets communication occur between the interrupt handler and the rest of the code in this file
static 	UARTStruct *devStaticPtr[4];

// Variables to control read and write or UART
//static Bool bReadReady[3] = {0,0,0};
//static Bool bWriteReady[3] = {0,0,0};

// I have set this to a large stack size because of (a) using printf() and (b) the depth of function calls
//   for some of the UART operations -- it is possible/very likely these are much larger than needed (see LCDtask.c for how to check the stack size)
#define baseStack 3
#if PRINTF_VERSION == 1
#define uartSTACK_SIZE		((baseStack+5)*configMINIMAL_STACK_SIZE)
#else
#define uartSTACK_SIZE		(baseStack*configMINIMAL_STACK_SIZE)
#endif

// End of private definitions
/* ************************************************ */

/* ************************************************ */
// Public API Functions
//
// Note: This will startup an UART thread, once for each call to this routine
int uartInit(UARTStruct *devPtr,uint8_t uartDevNum,unsigned portBASE_TYPE taskPriority,UART_CFG_Type* uartCfg, UART_FIFO_CFG_Type* fifoCfg)
{
	PINSEL_CFG_Type PinCfg;

	devPtr->devNum = uartDevNum;
	devPtr->taskPriority = taskPriority;

	int retval = UartInitSuccess;
	switch (devPtr->devNum) {
		case 0: {
			devStaticPtr[0] = devPtr; // Setup the permanent variable for use by the interrupt handler
			devPtr->devAddr = LPC_UART0;
			// Start with the interrupts disabled *and* make sure we have the priority correct
			NVIC_SetPriority(UART0_IRQn,UARTIntPriority);	
			NVIC_DisableIRQ(UART0_IRQn);
			// Init UART pin connect
			PinCfg.OpenDrain = 0;
			PinCfg.Pinmode = 0;
			PinCfg.Funcnum = 1;
			PinCfg.Pinnum = 2;
			PinCfg.Portnum = 0;
			PINSEL_ConfigPin(&PinCfg);
			PinCfg.Pinnum = 3;
			PINSEL_ConfigPin(&PinCfg);
			break;
		}
		case 1: {
			devStaticPtr[1] = devPtr; // Setup the permanent variable for use by the interrupt handler
			devPtr->devAddr = LPC_UART1;
			// Start with the interrupts disabled *and* make sure we have the priority correct
			NVIC_SetPriority(UART1_IRQn,UARTIntPriority);	
			NVIC_DisableIRQ(UART1_IRQn);
			// Init UART pin connect
			PinCfg.OpenDrain = 0;
			PinCfg.Pinmode = 0;
			PinCfg.Funcnum = 1;
			PinCfg.Pinnum = 15;
			PinCfg.Portnum = 0;
			PINSEL_ConfigPin(&PinCfg);
			PinCfg.Pinnum = 16;
			PINSEL_ConfigPin(&PinCfg);
			break;
		}
		//TODO: define UART2/3
		default: {
			return(UartErrInit);
			break;
		}
	}

	// Allocate the two queues to be used to communicate with other tasks
	if ((devPtr->inQ = xQueueCreate(UARTQLen*UART_TX_FIFO_SIZE,sizeof(uint8_t))) == NULL) {
		// free up everyone and go home
		return(UartErrInit);
	}
	if ((devPtr->outQ = xQueueCreate(UARTQLen*UART_TX_FIFO_SIZE,sizeof(uint8_t))) == NULL) {
		// free up everyone and go home
		vQueueDelete(devPtr->inQ);
		return(UartErrInit);
	}


	// Initialize  UART peripheral
	UART_Init(devPtr->devAddr, uartCfg);

	//Setup FIFO
	UART_FIFOConfig(devPtr->devAddr, fifoCfg);

	// Enable interrupts
	UART_IntConfig(devPtr->devAddr, UART_INTCFG_THRE, ENABLE);
	UART_IntConfig(devPtr->devAddr, UART_INTCFG_RBR, ENABLE);
	NVIC_EnableIRQ(UART1_IRQn);

	// Enable  UART operation
	UART_TxCmd(devPtr->devAddr, ENABLE);

	return(UartInitSuccess);
}

// A simple routine to use for filling out and sending a message to the UART thread
//   You may want to make your own versions of these as they are not suited to all purposes
portBASE_TYPE sendUartMsg(UARTStruct *dev,uint8_t txLen, uint8_t *txBuf){
	int i=0;
	portBASE_TYPE success = pdTRUE;

	if(UART_CheckBusy(dev->devAddr)==RESET){
		uint8_t len = txLen;
		if( len > UART_TX_FIFO_SIZE ) len = UART_TX_FIFO_SIZE;
		uint32_t bSent = UART_Send(dev->devAddr,txBuf,len,NONE_BLOCKING);
		//printf("Sent %u Bytes.\n",(unsigned int)bSent);
		i = len;
	}

	for (;i<txLen;i++) {
		success &= xQueueSend(dev->inQ,(void *)&(txBuf[i]),portMAX_DELAY);
	}
	return success;
}

// End of public API Functions
/* ************************************************ */

// uart interrupt handler
static __INLINE void UARTIsr(UARTStruct* devPtr) {
	static uint8_t data[UART_TX_FIFO_SIZE];
	vtLEDOff(0xC0);
	uint32_t IIR = UART_GetIntId(devPtr->devAddr);
	if(IIR & UART_IIR_INTID_RDA){
		vtLEDOn(0x80);
		//Grab bytes put in queue
		uint32_t bRecv = UART_Receive(devPtr->devAddr,data,UART_TX_FIFO_SIZE,NONE_BLOCKING);
		int i=0;
		for(i=0;i<bRecv;i++){
			static signed portBASE_TYPE xHigherPriorityTaskWoken;
			xHigherPriorityTaskWoken = pdFALSE;
			if( xQueueSendToBackFromISR(devPtr->outQ,(void*)&(data[i]),&xHigherPriorityTaskWoken) != pdTRUE ){
				//OOPS.....
				VT_HANDLE_FATAL_ERROR(0xC0DE1);
			}
			portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
		}
	}

	if(IIR & UART_IIR_INTID_THRE){
		vtLEDOn(0x40);
		//Load up the FIFO
		unsigned portBASE_TYPE len = uxQueueMessagesWaitingFromISR(devPtr->inQ);
		if( len > UART_TX_FIFO_SIZE) len = UART_TX_FIFO_SIZE;
		int i=0;
		for(i=0;i<len;i++){
			static signed portBASE_TYPE xHigherPriorityTaskWoken;
			xHigherPriorityTaskWoken = pdFALSE;
			if( xQueueReceiveFromISR(devPtr->inQ,(void*)&(data[i]),&xHigherPriorityTaskWoken) != pdTRUE ){
				//OOPS.....
				VT_HANDLE_FATAL_ERROR(0xC0DE2);
			}
			portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
		}
		uint32_t bSent = UART_Send(devPtr->devAddr,data,len,NONE_BLOCKING);
		//printf("TX sent - %u\n",(unsigned int)bSent);
	}
//	UART_MasterHandler(devAddr);
//	if (UART_MasterTransferComplete(devAddr)) {
//		static signed portBASE_TYPE xHigherPriorityTaskWoken;
//		xHigherPriorityTaskWoken = pdFALSE;
//		xSemaphoreGiveFromISR(*binSemaphore,&xHigherPriorityTaskWoken);
//		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
//	}
}
// Simply pass on the information to the real interrupt handler above (have to do this to work for multiple uart peripheral units on the LPC1768
void UART0Isr(void) {
	UARTIsr(devStaticPtr[0]);
}

// Simply pass on the information to the real interrupt handler above (have to do this to work for multiple uart peripheral units on the LPC1768
void UART1Isr(void) {
	UARTIsr(devStaticPtr[1]);
	//NVIC_ClearPendingIRQ(UART1_IRQn);
}

// Simply pass on the information to the real interrupt handler above (have to do this to work for multiple uart peripheral units on the LPC1768
void UART2Isr(void) {
	UARTIsr(devStaticPtr[2]);
}

// Simply pass on the information to the real interrupt handler above (have to do this to work for multiple uart peripheral units on the LPC1768
void UART3Isr(void) {
	UARTIsr(devStaticPtr[3]);
}