
#include "predef.h" 
#include <ctype.h>
#include <basictypes.h>
#include <system.h>
#include <constants.h>
#include <ucos.h>
#include <ucosmcfc.h>
#include <sim.h>
#include <bsp.h>
#include <cfinter.h>
#include "servo_drive.h"
#include "filereporter.h" 
REPORT_FILE;


extern "C" 
{
void SetIntc( long func, int vector, int level, int prio );
	void NMI_C_Part();
    void NMI_ASM_Part();
}			 


#define SetServoPin() sim.gpio.settd = 0x04
#define ClrServoPin() sim.gpio.clrtd = ~0x04 


volatile DWORD ServoTimes[8];
volatile int servo_index;
volatile DWORD servo_elapsed;
volatile DWORD servo_isrcnt;


#define SERVO_ZERO (49766)
#define SERVO_MIN (33177)
#define SERVO_MAX (66354)
#define SERVO_DELTA (16588.5)



void Function_Holder()
{
__asm__ (" .global NMI_ASM_Part"); // export the label for ASM part
__asm__ (" .extern NMI_C_Part"); // label for the C part of the ISR
__asm__ ("NMI_ASM_Part:"); // label for the assembly part of the ISR
__asm__ (" move.w #0x2700,%sr "); // set the IRQ mask to mask all
__asm__ (" lea -60(%a7),%a7 "); // make space on the system stack
__asm__ (" movem.l %d0-%d7/%a0-%a6,(%a7) "); // save all registers
__asm__ (" jsr NMI_C_Part "); //
__asm__ (" movem.l (%a7),%d0-%d7/%a0-%a6 ");
__asm__ (" lea 60(%a7),%a7 ");
__asm__ (" rte");
}

volatile bool bServoOff;



/* This is the C/C++ part of the ISR that is called from the assembly code */
void NMI_C_Part()
{
	servo_isrcnt++;
	sim.timer[0].dter=3;
	switch (servo_index)
		{
		case 0:
			//Set pin high */
			sim.timer[0].dtrr=SERVO_ZERO;
			if(!bServoOff) SetServoPin();
			break;
		case 2:
			//Set Pin low
			ClrServoPin();

		case 1:
            
			if(ServoTimes[0]<SERVO_MIN)ServoTimes[0]=SERVO_MIN;
			else
			if(ServoTimes[0]>SERVO_MAX)ServoTimes[0]=SERVO_MAX;
			sim.timer[0].dtrr=ServoTimes[0];
			servo_elapsed+=ServoTimes[0];

			break;
		case 3:
		case 4:
			if(ServoTimes[1]<SERVO_MIN)ServoTimes[1]=SERVO_MIN;
			else
			if(ServoTimes[1]>SERVO_MAX)ServoTimes[1]=SERVO_MAX;
			sim.timer[0].dtrr=ServoTimes[1];
			servo_elapsed+=ServoTimes[1];
			break;
		case 5:
		case 6:
			if(ServoTimes[2]<SERVO_MIN)ServoTimes[2]=SERVO_MIN;
			else
			if(ServoTimes[2]>SERVO_MAX)ServoTimes[2]=SERVO_MAX;

			sim.timer[0].dtrr=ServoTimes[2];
			servo_elapsed+=ServoTimes[2];

			break;
		case 7:
		case 8:
			if(ServoTimes[3]<SERVO_MIN)ServoTimes[3]=SERVO_MIN;
			else
			if(ServoTimes[3]>SERVO_MAX)ServoTimes[3]=SERVO_MAX;
			
			
			sim.timer[0].dtrr=ServoTimes[3];
			servo_elapsed+=ServoTimes[3];

			break;
		case 9:
		case 10:
			if(ServoTimes[4]<SERVO_MIN)ServoTimes[4]=SERVO_MIN;
			else
			if(ServoTimes[4]>SERVO_MAX)ServoTimes[4]=SERVO_MAX;

			sim.timer[0].dtrr=ServoTimes[4];
			servo_elapsed+=ServoTimes[4];

			break;

		case 11:
		case 12:
			if(ServoTimes[5]<SERVO_MIN)ServoTimes[5]=SERVO_MIN;
			else
			if(ServoTimes[5]>SERVO_MAX)ServoTimes[5]=SERVO_MAX;
			sim.timer[0].dtrr=ServoTimes[5];
			servo_elapsed+=ServoTimes[5];

			break;

		case 13:
		case 14:
			if(ServoTimes[6]<SERVO_MIN)ServoTimes[6]=SERVO_MIN;
			else
			if(ServoTimes[6]>SERVO_MAX)ServoTimes[6]=SERVO_MAX;

			sim.timer[0].dtrr=ServoTimes[6];
			servo_elapsed+=ServoTimes[6];

			break;

		case 15:
		case 16:
			if(ServoTimes[7]<SERVO_MIN)ServoTimes[7]=SERVO_MIN;
			else
			if(ServoTimes[7]>SERVO_MAX)ServoTimes[7]=SERVO_MAX;
			sim.timer[0].dtrr=ServoTimes[7];
			servo_elapsed+=ServoTimes[7];

			break;

		case 17:
			sim.timer[0].dtrr=(SERVO_ZERO*26)-servo_elapsed;

			break;



		}
	servo_index++;
	if (servo_index>=18)
		{
		servo_index=0;
		servo_elapsed=0;
		}

}




 

void InitServoDrive()
{
//We use two pins and two pins only.
//DTIN0 pin 36 as time output port TC 0
//Pin function 2
sim.gpio.ptcpar &= ~0x03;
sim.gpio.ptcpar |=0x02;


//Pin 60 PWM5 as servo drive...
//Port PTD2 pin function GPIO ie func 0 s0 xxxxx0xx
sim.gpio.ptdpar&=0xFB; 
sim.gpio.ddrtd |= 0x04; 


	ServoTimes[0]=SERVO_ZERO;
	ServoTimes[1]=SERVO_ZERO;
	ServoTimes[2]=SERVO_ZERO;
	ServoTimes[3]=SERVO_ZERO;
	ServoTimes[4]=SERVO_ZERO;
	ServoTimes[5]=SERVO_ZERO;
	ServoTimes[6]=SERVO_ZERO;
	ServoTimes[7]=SERVO_ZERO;
	servo_index=0;
	servo_elapsed=0;


	sim.timer[0].dtmr=0x0028;   /* 0000 0000 00 1 0 1 0 0 0 */
	sim.timer[0].dtxmr=0; 
	sim.timer[0].dter=3;  
	sim.timer[0].dtrr=SERVO_ZERO; 
	sim.timer[0].dtcr=0; 
	sim.timer[0].dtcn=1; 

	sim.timer[0].dtmr=0x003B;  
	SetIntc((long)NMI_ASM_Part,19,7,4);


}




void RawSetServo(int ch,float v) //Limits -1.0 to +1.0
{
	if ((ch<0) || (ch>7)) return;
	if(v<=-1.0) ServoTimes[ch]=SERVO_MIN;
	else
	if(v>=1.0) ServoTimes[ch]=SERVO_MAX;
	else
	if(v==0.0)ServoTimes[ch]=SERVO_ZERO;
	else
	{
	v*=SERVO_DELTA;
	v+=SERVO_ZERO;
	ServoTimes[ch]=(DWORD)v;
    }
}




