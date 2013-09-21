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

	#if LCD_EXAMPLE_OP==0
	unsigned short screenColor = 0;
	unsigned short tscr;
	unsigned char curLine;
	unsigned timerCount = 0;
	unsigned screenCount = 0;
	int xoffset = 0, yoffset = 0;
	unsigned int xmin=0, xmax=0, ymin=0, ymax=0;
	unsigned int x, y;
	int i, j;
	float hue=0, sat=0.2, light=0.2;
	#elif LCD_EXAMPLE_OP==1
	unsigned char picIndex = 0;
	#else
	Bad definition
	#endif
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
	tscr = Red; // may be reset in the LCDMsgTypeTimer code below
	screenColor = Black; // may be reset in the LCDMsgTypeTimer code below
	GLCD_SetTextColor(tscr);
	GLCD_SetBackColor(screenColor);
	GLCD_Clear(screenColor);

	GLCD_DisplayString(0,0,0,(unsigned char *)"T");
	GLCD_DisplayString(1,0,0,(unsigned char *)"i");
	GLCD_DisplayString(2,0,0,(unsigned char *)"m");
	GLCD_DisplayString(3,0,0,(unsigned char *)"e");
	GLCD_DisplayString(4,0,0,(unsigned char *)" ");
	GLCD_DisplayString(5,0,0,(unsigned char *)"s");
	

	GLCD_DisplayString(29,20,0,(unsigned char *)"Voltage(V)");


	// Note that srand() & rand() require the use of malloc() and should not be used unless you are using
	//   MALLOC_VERSION==1
	#if MALLOC_VERSION==1
	srand((unsigned) 55); // initialize the random number generator to the same seed for repeatability
	#endif



	curLine = 5;
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

		#if LCD_EXAMPLE_OP==0
		// Wait for a message
		if (xQueueReceive(lcdPtr->inQ,(void *) &msgBuffer,portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}
		
		//Log that we are processing a message -- more explanation of logging is given later on
		vtITMu8(vtITMPortLCDMsg,getMsgType(&msgBuffer));
		vtITMu8(vtITMPortLCDMsg,getMsgLength(&msgBuffer));

		// Take a different action depending on the type of the message that we received
		switch(getMsgType(&msgBuffer)) {
		case LCDMsgTypePrint: {
			// This will result in the text printing in the last five lines of the screen
			char   lineBuffer[lcdCHAR_IN_LINE+1];
			copyMsgString(lineBuffer,&msgBuffer,lcdCHAR_IN_LINE);
			// clear the line
			GLCD_ClearLn(curLine,1);
			// show the text
			GLCD_DisplayString(curLine,0,1,(unsigned char *)lineBuffer);
			curLine++;
			if (curLine == lcdNUM_LINES) {
				curLine = 5;
			}
			break;
		}
		case LCDMsgTypeTimer: {
			// Note: if I cared how long the timer update was I would call my routine
			//    unpackTimerMsg() which would unpack the message and get that value
			// Each timer update will cause a circle to be drawn on the top half of the screen
			//   as explained below
			if (timerCount == 0) {
				/* ************************************************** */
				// Find a new color for the screen by randomly (within limits) selecting HSL values
				// This can be ignored unless you care about the color map
				#if MALLOC_VERSION==1
				hue = rand() % 360;
				sat = (rand() % 1024) / 1023.0; sat = sat * 0.5; sat += 0.5;
				light = (rand() % 1024) / 1023.0;	light = light * 0.8; light += 0.10;
				#else
				hue = (hue + 1); if (hue >= 360) hue = 0;
				sat+=0.01; if (sat > 1.0) sat = 0.20;	
				light+=0.03; if (light > 1.0) light = 0.20;
				#endif
				screenColor = hsl2rgb(hue,sat,light);
				// Now choose a complementary value for the text color
				hue += 180;
				if (hue >= 360) hue -= 360;
				tscr = hsl2rgb(hue,sat,light);
				GLCD_SetTextColor(tscr);
				GLCD_SetBackColor(screenColor);
				// End of playing around with figuring out a random color
				/* ************************************************** */

				// clear the top half of the screen
				GLCD_ClearWindow(0,0,320,120,screenColor); 

				// Now we are going to draw a circle in the upper left corner of the screen
				int	count = 200;
				float radius;
				float inc, val, offset = MAX_RADIUS;
				unsigned short circleColor;
				inc = 2*M_PI/count;
				xmax = 0;
				ymax = 0;
				xmin = 50000;
				ymin = 50000;
				val = 0.0;
				for (i=0;i<count;i++) {
					// Make the circle a little thicker
					// by actually drawing three circles w/ different radii
					float cv = cos(val), sv=sin(val);
					circleColor = (val*0xFFFF)/(2*M_PI);
					GLCD_SetTextColor(circleColor);
					for (radius=MAX_RADIUS-2.0;radius<=MAX_RADIUS;radius+=1.0) {
						x = round(cv*radius+offset);
						y = round(sv*radius+offset);
						if (x > xmax) xmax = x;
						if (y > ymax) ymax = y;
						if (x < xmin) xmin = x;
						if (y < ymin) ymin = y;
						GLCD_PutPixel(x,y);	
					}
					val += inc;
				}
				// Now we are going to read the upper left square of the LCD's
				// memory (see its data sheet for details on that) and save
				// that into a buffer for fast re-drawing later
				if (((xmax+1-xmin)*(ymax+1-ymin)) > BUF_LEN) {
					// Make sure we have room for the data
					VT_HANDLE_FATAL_ERROR(0);
				}
				unsigned short int *tbuffer = buffer;
				unsigned int width = (xmax+1-xmin);
				for (j=ymin;j<=ymax;j++) {
					GLCD_GetPixelRow (xmin,j,width,tbuffer);
					tbuffer += width;
				}
				// end of reading in the buffer
				xoffset = xmin;
				yoffset = ymin;
			} else {
				// We are going to write out the data read into the buffer
				//   back onto the screen at a new location
				// This is *very* fast

				// First, clear out where we were
				GLCD_ClearWindow(xoffset,yoffset,xmax+1-xmin,ymax+1-ymin,screenColor);
				// Pick the new location
				#if MALLOC_VERSION==1
				xoffset = rand() % (320-(xmax+1-xmin));
				yoffset = rand() % (120-(ymax+1-ymin));
				#else
				xoffset = (xoffset + 10) % (320-(xmax+1-xmin));
				yoffset = (yoffset + 10) % (120-(ymax+1-ymin));
				#endif
				// Draw the bitmap
				GLCD_Bitmap(xoffset,yoffset,xmax+1-xmin,ymax-1-ymin,(unsigned char *)buffer);
			}
			timerCount++;
			if (timerCount >= 40) {	  
				// every so often, we reset timer count and start again
				// This isn't for any important reason, it is just to for this example code to do "stuff"
				timerCount = 0;
			}
			break;
		}
		case LCDMsgTypeGraph: {
		
			// Grab value
			int value = msgBuffer.buf[0];
			if( value > 1024 ) value %= 1024; // Should not happen
		   	value=(value*231)/1024;
		   	value=232-value;
		   //	unsigned char displayMe[20];
			//sprintf(displayMe,"This:%x",msgBuffer.buf[0]);
		//	GLCD_DisplayString(29,20,0,(unsigned char*)displayMe);
			if(++g.position >= GRAPHSIZE) g.position = 0;
			if(g.size < GRAPHSIZE) ++g.size;
			g.data[g.position] = (uint8_t)value; 	


			GLCD_ClearWindow(6,0,314,232,Black);
			//GLCD_Clear(Black);

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

		// Here is a way to do debugging output via the built-in hardware -- it requires the ULINK cable and the
		//   debugger in the Keil tools to be connected.  You can view PORT0 output in the "Debug(printf) Viewer"
		//   under "View->Serial Windows".  You have to enable "Trace" and "Port0" in the Debug setup options.  This
		//   should not be used if you are using Port0 for printf()
		// There are 31 other ports and their output (and port 0's) can be seen in the "View->Trace->Records"
		//   windows.  You have to enable the prots in the Debug setup options.  Note that unlike ITM_SendChar()
		//   this "raw" port write is not blocking.  That means it can overrun the capability of the system to record
		//   the trace events if you go too quickly; that won't hurt anything or change the program execution and
		//   you can tell if it happens because the "View->Trace->Records" window will show there was an overrun.
		//vtITMu16(vtITMPortLCD,screenColor);

		#elif 	LCD_EXAMPLE_OP==1
		// In this alternate version, we just keep redrawing a series of bitmaps as
		//   we receive timer messages
		// Wait for a message
		if (xQueueReceive(lcdPtr->inQ,(void *) &msgBuffer,portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}
		if (getMsgType(&msgBuffer) != LCDMsgTypeTimer) {
			// In this configuration, we are only expecting to receive timer messages
			VT_HANDLE_FATAL_ERROR(getMsgType(&msgBuffer));
		}
  		/* go through a  bitmap that is really a series of bitmaps */
		picIndex = (picIndex + 1) % 9;
		GLCD_Bmp(99,99,120,45,(unsigned char *) &ARM_Ani_16bpp[picIndex*(120*45*2)]);
		#else
		Bad setting
		#endif	
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
