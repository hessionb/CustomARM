#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"
#include "semphr.h"

/* include files. */
#include "vtUtilities.h"
#include "vtI2C.h"
#include "LCDtask.h"
#include "i2cVolt.h"
#include "I2CTaskMsgTypes.h"

/* *********************************************** */
// definitions and data structures that are private to this file
// Length of the queue to this task
#define vtVoltQLen 10 
// actual data structure that is sent in a message
typedef struct __vtVoltMsg {
	uint8_t msgType;
	uint8_t	length;	 // Length of the message to be printed
	uint8_t buf[vtVoltMaxLen+1]; // On the way in, message to be sent, on the way out, message received (if any)
} vtVoltMsg;

// I have set this to a large stack size because of (a) using printf() and (b) the depth of function calls
//   for some of the i2c operations	-- almost certainly too large, see LCDTask.c for details on how to check the size
#define baseStack 3
#if PRINTF_VERSION == 1
#define i2cSTACK_SIZE		((baseStack+5)*configMINIMAL_STACK_SIZE)
#else
#define i2cSTACK_SIZE		(baseStack*configMINIMAL_STACK_SIZE)
#endif

// end of defs
/* *********************************************** */

/* The i2cTemp task. */
static portTASK_FUNCTION_PROTO( vi2cVoltUpdateTask, pvParameters );

/*-----------------------------------------------------------*/
// Public API
void vStarti2cVoltTask(vtVoltStruct *params,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c,vtLCDStruct *lcd)
{
	// Create the queue that will be used to talk to this task
	if ((params->inQ = xQueueCreate(vtVoltQLen,sizeof(vtVoltMsg))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	/* Start the task */
	portBASE_TYPE retval;
	params->dev = i2c;
	params->lcdData = lcd;
	if ((retval = xTaskCreate( vi2cVoltUpdateTask, ( signed char * ) "i2cVolt", i2cSTACK_SIZE, (void *) params, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}

portBASE_TYPE SendVoltTimerMsg(vtVoltStruct *voltData,portTickType ticksElapsed,portTickType ticksToBlock)
{
	if (voltData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtVoltMsg voltBuffer;
	voltBuffer.length = sizeof(ticksElapsed);
	if (voltBuffer.length > vtVoltMaxLen) {
		// no room for this message
		VT_HANDLE_FATAL_ERROR(voltBuffer.length);
	}
	memcpy(voltBuffer.buf,(char *)&ticksElapsed,sizeof(ticksElapsed));
	voltBuffer.msgType = VoltMsgTypeTimer;
	return(xQueueSend(voltData->inQ,(void *) (&voltBuffer),ticksToBlock));
}

portBASE_TYPE SendVoltValueMsg(vtVoltStruct *voltData,uint8_t msgType,uint8_t *value,uint8_t size,portTickType ticksToBlock)
{
	vtVoltMsg voltBuffer;

	if (voltData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	voltBuffer.length = sizeof(value);
	if (voltBuffer.length > vtVoltMaxLen) {
		// no room for this message
		VT_HANDLE_FATAL_ERROR(voltBuffer.length);
	}
	memcpy(voltBuffer.buf,(char *)value,size*sizeof(uint8_t)); //### 
	voltBuffer.msgType = msgType;
	return(xQueueSend(voltData->inQ,(void *) (&voltBuffer),ticksToBlock));
}



// End of Public API
/*-----------------------------------------------------------*/
int getMsgType(vtVoltMsg *Buffer)
{
	return(Buffer->msgType);
}

void getValue(uint8_t *source,vtVoltMsg *Buffer,uint8_t size)
{
	memcpy(source,Buffer->buf,size);
}

// I2C commands for the temperature sensor
const uint8_t i2cCmdInit[]= {0xAC,0x00};
const uint8_t i2cCmdStartConvert[]= {0xEE};
const uint8_t i2cCmdStopConvert[]= {0x22};
const uint8_t i2cCmdRead1Vals[]= {0xAA};
// end of I2C command definitions

// State Machine
const uint8_t fsmStateInitSent = 0;
const uint8_t fsmStateVoltRead = 1;
// This is the actual task that is run
static portTASK_FUNCTION( vi2cVoltUpdateTask, pvParameters )
{							
	// Get the parameters
	vtVoltStruct *param = (vtVoltStruct *) pvParameters;
	// Get the I2C device pointer
	vtI2CStruct *devPtr = param->dev;
	// Get the LCD information pointer
	vtLCDStruct *lcdData = param->lcdData;
	// String buffer for printing
	char lcdBuffer[vtLCDMaxLen+1];
	// Buffer for receiving messages
	vtVoltMsg msgBuffer;
	uint8_t currentState;
	// I2C Volt message buffer
	uint8_t data[8];

	// Assumes that the I2C device (and thread) have already been initialized

	// This task is implemented as a Finite State Machine.  The incoming messages are examined to see
	//   whether or not the state should change.
	//
	// Temperature sensor configuration sequence (DS1621) Address 0x4F
	if (vtI2CEnQ(devPtr,vtI2CMsgTypeVoltInit,0x4F,sizeof(i2cCmdInit),i2cCmdInit,0) != pdTRUE) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	currentState = fsmStateInitSent;

	// Like all good tasks, this should never exit
	for(;;)
	{
		// Wait for a message from either a timer or from an I2C operation
		if (xQueueReceive(param->inQ,(void *) &msgBuffer,portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}

		// Now, based on the type of the message and the state, we decide on the new state and action to take
		switch(getMsgType(&msgBuffer)) {

			// Receive Acknowledge
			case vtI2CMsgTypeVoltInit: {
				if (currentState == fsmStateInitSent) {
					currentState = fsmStateVoltRead;
				} else {
					// unexpectedly received this message
					VT_HANDLE_FATAL_ERROR(0);
				}
				break;
			}

			// Send Volt Requests
			case VoltMsgTypeTimer: {
				// Timer messages never change the state, they just cause an action (or not) 
				if (currentState != fsmStateInitSent) {
					if (vtI2CEnQ(devPtr,vtI2CMsgTypeVoltRead,0x4F,sizeof(i2cCmdRead1Vals),i2cCmdRead1Vals,8) != pdTRUE) {
						VT_HANDLE_FATAL_ERROR(0);
					}
				} else {
					// just ignore timer messages until initialization is complete
				} 
				break;
			}

			// Get Volt Data
			case vtI2CMsgTypeVoltRead: {
				if (currentState == fsmStateVoltRead) {
					getValue(data,&msgBuffer,8);
					if (lcdData != NULL) {
						if (SendLCDGraphMsg(lcdData,data,portMAX_DELAY) != pdTRUE) {
							VT_HANDLE_FATAL_ERROR(0);
						}
					}
				} else {
					// unexpectedly received this message
					VT_HANDLE_FATAL_ERROR(0);
				}
				break;
			}
		
			// Error
			default: {
				VT_HANDLE_FATAL_ERROR(getMsgType(&msgBuffer));
				break;
			}
		}
	}
}

