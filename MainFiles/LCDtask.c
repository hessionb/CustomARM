#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

/* include files. */
#include "GLCD.h"
#include "vtUtilities.h"
#include "LCDtask.h"
#include "string.h"

// I have set this to a larger stack size because of (a) using printf() and (b) the depth of function calls
//   for some of the LCD operations
// I actually monitor the stack size in the code to check to make sure I'm not too close to overflowing the stack
//   This monitoring takes place if INPSECT_STACK is defined (search this file for INSPECT_STACK to see the code for this) 
#define INSPECT_STACK 1
#define baseStack 3
#if PRINTF_VERSION == 1
#define lcdSTACK_SIZE		((baseStack+5)*configMINIMAL_STACK_SIZE)
#else
#define lcdSTACK_SIZE		(baseStack*configMINIMAL_STACK_SIZE)
#endif

// definitions and data structures that are private to this file
// Length of the queue to this task
#define vtLCDQLen 10 
// a timer message -- not to be printed
#define LCDMsgTypeTimer 1
// a message to be printed
#define LCDMsgTypePrint 2
// a value to graph
#define LCDMsgTypeGraph 3

// actual data structure that is sent in a message
typedef struct __vtLCDMsg {
	uint8_t msgType;
	uint8_t	length;	 // Length of the message to be printed
	uint16_t buf[vtLCDMaxLen+1]; // On the way in, message to be sent, on the way out, message received (if any)
} vtLCDMsg;
// end of defs

/* definition for the LCD task. */
static portTASK_FUNCTION_PROTO( vLCDUpdateTask, pvParameters );

/*-----------------------------------------------------------*/

void StartLCDTask(vtLCDStruct *ptr, unsigned portBASE_TYPE uxPriority)
{
	if (ptr == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}

	// Create the queue that will be used to talk to this task
	if ((ptr->inQ = xQueueCreate(vtLCDQLen,sizeof(vtLCDMsg))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	/* Start the task */
	portBASE_TYPE retval;
	if ((retval = xTaskCreate( vLCDUpdateTask, ( signed char * ) "LCD", lcdSTACK_SIZE, (void*)ptr, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}

portBASE_TYPE SendLCDTimerMsg(vtLCDStruct *lcdData,portTickType ticksElapsed,portTickType ticksToBlock)
{
	if (lcdData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtLCDMsg lcdBuffer;
	lcdBuffer.length = sizeof(ticksElapsed);
	if (lcdBuffer.length > vtLCDMaxLen) {
		// no room for this message
		VT_HANDLE_FATAL_ERROR(lcdBuffer.length);
	}
	memcpy(lcdBuffer.buf,(char *)&ticksElapsed,sizeof(ticksElapsed));
	lcdBuffer.msgType = LCDMsgTypeTimer;
	return(xQueueSend(lcdData->inQ,(void *) (&lcdBuffer),ticksToBlock));
}

portBASE_TYPE SendLCDPrintMsg(vtLCDStruct *lcdData,int length,char *pString,portTickType ticksToBlock)
{
	if (lcdData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtLCDMsg lcdBuffer;

	if (length > vtLCDMaxLen) {
		// no room for this message
		VT_HANDLE_FATAL_ERROR(lcdBuffer.length);
	}
	lcdBuffer.length = strnlen(pString,vtLCDMaxLen);
	lcdBuffer.msgType = LCDMsgTypePrint;
	strncpy((char *)lcdBuffer.buf,pString,vtLCDMaxLen);
	return(xQueueSend(lcdData->inQ,(void *) (&lcdBuffer),ticksToBlock));
}

portBASE_TYPE SendLCDGraphMsg(vtLCDStruct *lcdData,uint16_t data,portTickType ticksToBlock)
{
	if (lcdData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtLCDMsg lcdBuffer;
	lcdBuffer.length = sizeof(data);
	if (lcdBuffer.length > vtLCDMaxLen) {
		// no room for this message
		VT_HANDLE_FATAL_ERROR(lcdBuffer.length);
	}
	memcpy(lcdBuffer.buf,(uint16_t *)&data,sizeof(uint16_t));
	lcdBuffer.msgType = LCDMsgTypeGraph;
	return(xQueueSend(lcdData->inQ,(void *) (&lcdBuffer),ticksToBlock));
}	 

// Private routines used to unpack the message buffers
//   I do not want to access the message buffer data structures outside of these routines
portTickType unpackTimerMsg(vtLCDMsg *lcdBuffer)
{
	portTickType *ptr = (portTickType *) lcdBuffer->buf;
	return(*ptr);
}

int getMsgType(vtLCDMsg *lcdBuffer)
{
	return(lcdBuffer->msgType);
} 

int getMsgLength(vtLCDMsg *lcdBuffer)
{
	return(lcdBuffer->msgType);
}

void copyMsgString(char *target,vtLCDMsg *lcdBuffer,int targetMaxLen)
{
	strncpy(target,(char *)(lcdBuffer->buf),targetMaxLen);
}

// End of private routines for message buffers

// If LCD_EXAMPLE_OP=0, then accept messages that may be timer or print requests and respond accordingly
// If LCD_EXAMPLE_OP=1, then do a rotating ARM bitmap display
#define LCD_EXAMPLE_OP 0
#if LCD_EXAMPLE_OP==1
// This include the file with the definition of the ARM bitmap
#include "ARM_Ani_16bpp.c"
#endif

static unsigned short hsl2rgb(float H,float S,float L);

#if LCD_EXAMPLE_OP==0
// Buffer in which to store the memory read from the LCD
	#define MAX_RADIUS 15
	#define BUF_LEN (((MAX_RADIUS*2)+1)*((MAX_RADIUS*2)+1))
	static unsigned short int buffer[BUF_LEN];
#endif

#define GRAPHSIZE 157
struct Graph {
	int position;
	int size;
	uint8_t data[GRAPHSIZE];
};

// This is the actual task that is run
static portTASK_FUNCTION( vLCDUpdateTask, pvParameters )
{
	// Initialize graph
	struct Graph g;
	g.position = 0;
	g.size = 0;

	vtLCDMsg msgBuffer;
	vtLCDStruct *lcdPtr = (vtLCDStruct *) pvParameters;

	#ifdef INSPECT_STACK
	// This is meant as an example that you can re-use in your own tasks
	// Inspect to the stack remaining to see how much room is remaining
	// 1. I'll check it here before anything really gets started
	// 2. I'll check during the run to see if it drops below 10%
	// 3. You could use break points or logging to check on this, but
	//    you really don't want to print it out because printf() can
	//    result in significant stack usage.
	// 4. Note that this checking is not perfect -- in fact, it will not
	//    be able to tell how much the stack grows on a printf() call and
	//    that growth can be *large* if version 1 of printf() is used.   
	unsigned portBASE_TYPE InitialStackLeft = uxTaskGetStackHighWaterMark(NULL);
	unsigned portBASE_TYPE CurrentStackLeft;
	float remainingStack = InitialStackLeft;
	remainingStack /= lcdSTACK_SIZE;
	if (remainingStack < 0.10) {
		// If the stack is really low, stop everything because we don't want it to run out
		// The 0.10 is just leaving a cushion, in theory, you could use exactly all of it
		VT_HANDLE_FATAL_ERROR(0);
	}
	#endif

	/* Initialize the LCD and set the initial colors */
	GLCD_Init();
	GLCD_SetTextColor(Red);
	GLCD_SetBackColor(Black);
	GLCD_Clear(Black);

	// Label Axes
	GLCD_DisplayString(0,0,0,(unsigned char *)"T");
	GLCD_DisplayString(1,0,0,(unsigned char *)"i");
	GLCD_DisplayString(2,0,0,(unsigned char *)"m");
	GLCD_DisplayString(3,0,0,(unsigned char *)"e");
	GLCD_DisplayString(4,0,0,(unsigned char *)" ");
	GLCD_DisplayString(5,0,0,(unsigned char *)"s");	

	GLCD_DisplayString(29,20,0,(unsigned char *)"Voltage(V)");

	// This task should never exit
	for(;;)
	{	
		#ifdef INSPECT_STACK   
		CurrentStackLeft = uxTaskGetStackHighWaterMark(NULL);
		float remainingStack = CurrentStackLeft;
		remainingStack /= lcdSTACK_SIZE;
		if (remainingStack < 0.10) {
			// If the stack is really low, stop everything because we don't want it to run out
			VT_HANDLE_FATAL_ERROR(0);
		}
		#endif

		// Wait for a message
		if (xQueueReceive(lcdPtr->inQ,(void *) &msgBuffer,portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}
		
		//Log that we are processing a message -- more explanation of logging is given later on
		vtITMu8(vtITMPortLCDMsg,getMsgType(&msgBuffer));
		vtITMu8(vtITMPortLCDMsg,getMsgLength(&msgBuffer));

		// Take a different action depending on the type of the message that we received
		switch(getMsgType(&msgBuffer)) {
		case LCDMsgTypeGraph: {
		
			// Grab value
			int value = msgBuffer.buf[0];
			if( value > 1024 ) value %= 1024; // Should not happen
		   	value=(value*231)/1024;
		   	value=232-value;
			
			// DEBUGGING I2C
			//unsigned char displayMe[20];
			//sprintf(displayMe,"This:%x",msgBuffer.buf[0]);
			//GLCD_DisplayString(29,20,0,(unsigned char*)displayMe);
			
			if(++g.position >= GRAPHSIZE) g.position = 0;
			if(g.size < GRAPHSIZE) ++g.size;
			g.data[g.position] = (uint8_t)value; 	

			GLCD_ClearWindow(6,0,314,232,Black);

			// Graph values
			int i = g.position;
			int dataCount = 0;
			do {
				int yvalue = g.data[i];
				int xvalue = 320-(dataCount*2);

				GLCD_PutPixel(xvalue,yvalue);
				GLCD_PutPixel(xvalue-1,yvalue);
				GLCD_PutPixel(xvalue,yvalue-1);
				GLCD_PutPixel(xvalue-1,yvalue-1);				

				if(++dataCount >= g.size) break;
				if(--i < 0) i = GRAPHSIZE-1;	
			} while(i != g.position);
			
			break;
		} 
		default: {
			// In this configuration, we are only expecting to receive timer messages
			VT_HANDLE_FATAL_ERROR(getMsgType(&msgBuffer));
			break;
		}
		} // end of switch()
	}
}

// Convert from HSL colormap to RGB values in this weird colormap
// H: 0 to 360
// S: 0 to 1
// L: 0 to 1
// The LCD has a funky bitmap.  Each pixel is 16 bits (a "short unsigned int")
//   Red is the most significant 5 bits
//   Blue is the least significant 5 bits
//   Green is the middle 6 bits
static unsigned short hsl2rgb(float H,float S,float L)
{
	float C = (1.0 - fabs(2.0*L-1.0))*S;
	float Hprime = H / 60;
	unsigned short t = Hprime / 2.0;
	t *= 2;
	float X = C * (1-abs((Hprime - t) - 1));
	unsigned short truncHprime = Hprime;
	float R1, G1, B1;

	switch(truncHprime) {
		case 0: {
			R1 = C; G1 = X; B1 = 0;
			break;
		}
		case 1: {
			R1 = X; G1 = C; B1 = 0;
			break;
		}
		case 2: {
			R1 = 0; G1 = C; B1 = X;
			break;
		}
		case 3: {
			R1 = 0; G1 = X; B1 = C;
			break;
		}
		case 4: {
			R1 = X; G1 = 0; B1 = C;
			break;
		}
		case 5: {
			R1 = C; G1 = 0; B1 = X;
			break;
		}
		default: {
			// make the compiler stop generating warnings
			R1 = 0; G1 = 0; B1 = 0;
			VT_HANDLE_FATAL_ERROR(Hprime);
			break;
		}
	}
	float m = L - 0.5*C;
	R1 += m; G1 += m; B1 += m;
	unsigned short red = R1*32; if (red > 31) red = 31;
	unsigned short green = G1*64; if (green > 63) green = 63;
	unsigned short blue = B1*32; if (blue > 31) blue = 31;
	unsigned short color = (red << 11) | (green << 5) | blue;
	return(color); 
}
