#include "predef.h" 
#include <ctype.h>
#include <basictypes.h>
#include <system.h>
#include <constants.h>
#include <ucos.h>
#include <ucosmcfc.h>
#include <serialirq.h>
#include <stdio.h>
#include <string.h>
#include "dsm2_sub.h"
#include "sensor_config.h"
#include <sim.h>
#include "filereporter.h" 
REPORT_FILE;

#define SM_TASK_STK (256)
#define SmOSSimpleTaskCreate(x,p) { static DWORD   func_##x_Stk[SM_TASK_STK] __attribute__( ( aligned( 4 ) ) ); OSTaskCreate(x,NULL,(void *)&func_##x_Stk[SM_TASK_STK],(void*)func_##x_Stk,p); }

OS_SEM * TheLocalSem;

extern volatile DWORD Pit_Count;
volatile DSM2_READING DSM2_Result;

DWORD LastPit;
volatile DWORD rcrx;

volatile bool bLog;
volatile bool bMode;
volatile int nMode;


unsigned char RCBuf[20];
static int n;
void ParseDSMChar(char c)
{
rcrx++;
if ((LastPit+4)<=Pit_Count) n=0;
LastPit=Pit_Count;

RCBuf[n++]=c;

if(n==16)
	{
	
	OSLock();
	  for(int i=1; i<8; i++)
	  {
		WORD w=RCBuf[i*2];
		w=w<<8;
		w|=RCBuf[(i*2)+1]; 
		int ch=(w>>10) & 0x07;
		DSM2_Result.val[ch]=(w& 0x3FF);
	  }
	  DSM2_Result.ReadingNum++;
    OSUnlock();
	Scaled_DSM2_Result.fElevator=DSM_Con(DSM2_Result.val[2]);
	Scaled_DSM2_Result.fAlieron =DSM_Con(DSM2_Result.val[1]);
	Scaled_DSM2_Result.fRudder  =DSM_Con(DSM2_Result.val[3]);
	Scaled_DSM2_Result.fThrottle=DSM_Con(DSM2_Result.val[0]);
	
	Scaled_DSM2_Result.Gear=DSM_Con(DSM2_Result.val[4]);
	Scaled_DSM2_Result.Aux2=DSM_Con(DSM2_Result.val[6]);
	bLog=(Scaled_DSM2_Result.Gear>0.5);
	bMode=(Scaled_DSM2_Result.Aux2 >0.5);
	if(DSM2_Result.val[5]<400) nMode=0;
	else if(DSM2_Result.val[5]<600) nMode=1;
	else
		nMode=2;



	if (TheLocalSem) {OSSemPost(TheLocalSem);}
	}
if(n>>19) n=18;
}

#define DSMUART (2)



void DSM2Task( void * pd)
{
SimpleUart(DSMUART, 125000 ); 
sim.gpio.pubpar = 0x05;
sim.gpio.pucpar=0x0F;



while(1)
{char c=sgetchar(DSMUART);
    ParseDSMChar(c);
}

}


void InitIDSM2SubSystem(WORD prio,OS_SEM * pDataSem)
{
TheLocalSem=pDataSem;
SmOSSimpleTaskCreate(DSM2Task,prio);
}




float DSM_Con(unsigned short v)
{
float f=v;
f-=512;
f*=(1.0/512);
return f;
}



volatile char Screen[2][16];


volatile SCALE_DSM2_READING Scaled_DSM2_Result;


