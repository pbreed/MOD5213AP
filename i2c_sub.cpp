#include "predef.h" 
#include <ctype.h>
#include <basictypes.h>
#include <system.h>
#include <constants.h>
#include <ucos.h>
#include <ucosmcfc.h>
#include <serialirq.h>
#include <stdio.h>
#include <smarttrap.h>
#include <serialupdate.h>
//#include "i2cmulti.h"             //Used for Multi-Master I2C
#include "i2cmaster.h"          //Used for Master Only I2C
#include <string.h>
#include <Pins.h>
#include <sim.h>
#include <cfinter.h>
#include "i2c_sub.h"
#include "filereporter.h" 
REPORT_FILE;


// Instruct the C++ compiler not to mangle the function name 
extern "C" 
{
void SetIntc( long func, int vector, int level, int prio );
}


volatile bool bNewCompass;
volatile COMPASS_READING Compass_Result;
volatile ALTIMETER_READING Alt_Result;

//volatile DWORD dwPressure;
//volatile DWORD dwTemperature;



OS_SEM Irq3_Sem;


INTERRUPT( irq_3, 0x2300 )
{
sim.eport.epfr |= 0x08; // Clear IRQ3 flag
OSSemPost(&Irq3_Sem);
}


void InitIrq3()
{
OSSemInit(&Irq3_Sem,0);

sim.gpio.pnqpar |= ( 8 );

sim.eport.eppar &= ~0x00C0; // Configure IRQ3 to trigger an falling edge
sim.eport.eppar |= 0x0080; // Configure IRQ3 to trigger an falling edge

sim.eport.epddr &= ~0x08; // Configure IRQ3 as input
sim.eport.epier = 0x08; // Enable IRQ3
SetIntc( ( long ) &irq_3, 3, 3, 7 );
}


 #define HMC588_ADDR (0x1E)
 #define MS5611_ADDR (0x77)

bool InitHMB588()
{
BYTE rv=0;
BYTE buffer[10];
buffer[0]=2;
buffer[1]=0;

rv=I2CSendBuf(HMC588_ADDR,buffer, 2, true);
InitIrq3(); 
if(rv==0) return true;
return false;
}

void ReadHMC588()
{
BYTE rv=0;
BYTE buffer[10];

buffer[0]=3;

rv=I2CSendBuf(HMC588_ADDR,buffer, 1, false);
//iprintf("Send RV=%02X\r\n",rv);

rv=I2CRestart(HMC588_ADDR,true,20);
//iprintf("Restart RV=%02X\r\n",rv);

rv=I2CReadBuf(HMC588_ADDR,buffer, 6, true);

iprintf("at %d RV=%02X %02X %02X %02X %02X %02X %02X\r\n",Secs,rv,buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5]);

}

bool InitMS5611()
{
BYTE rv=0;
BYTE buffer[2];
buffer[0]=0x1E;
buffer[1]=0;

rv=I2CSendBuf(MS5611_ADDR,buffer, 1, true);
if(rv==0) return true;
return false;
}




void ReadMS5611()
{
BYTE rv=0;
BYTE buffer[4];

buffer[0]=0x48;	//Pressure
rv=I2CSendBuf(MS5611_ADDR,buffer, 1, true);
iprintf("Send RV=%02X\r\n",rv);
OSTimeDly(2);

buffer[0]=0x0;
rv=I2CSendBuf(MS5611_ADDR,buffer, 1, true);
iprintf("Send RV=%02X\r\n",rv);

rv=I2CReadBuf(MS5611_ADDR,buffer, 3, true);

iprintf("P At %d RV=%02X %02X %02X %02X \r\n",Secs,rv,buffer[0],buffer[1],buffer[2]);


buffer[0]=0x58;	//Temperaute
rv=I2CSendBuf(MS5611_ADDR,buffer, 1, true);
iprintf("Send RV=%02X\r\n",rv);
OSTimeDly(2);

buffer[0]=0x0;
rv=I2CSendBuf(MS5611_ADDR,buffer, 1, true);
iprintf("Send RV=%02X\r\n",rv);


rv=I2CReadBuf(MS5611_ADDR,buffer, 3, true);

iprintf("T at %d RV=%02X %02X %02X %02X \r\n",Secs,rv,buffer[0],buffer[1],buffer[2]);


}


void HardWarePinsInit()
{
   Pins[4].function( PIN4_SDA );
   Pins[5].function( PIN5_SCL );

   Pins[35].function( 0);
   Pins[38].function( 0);

}



static OS_SEM * pDataSem;
void I2CTask( void * pd)
{
BYTE rv=0;
BYTE buffer[10];
bool bPressure;
COMPASS_READING * pCompass=(COMPASS_READING *)buffer;
static WORD ReadingNumber;

buffer[0]=0x48;	//Pressure
rv=I2CSendBuf(MS5611_ADDR,buffer, 1, true);
OSTimeDly(2);
bPressure=true;

while(1)
	{
	OSSemPend(&Irq3_Sem,0);
	buffer[0]=3;

	rv=I2CSendBuf(HMC588_ADDR,buffer, 1, false);
	rv=I2CRestart(HMC588_ADDR,true,20);
	rv=I2CReadBuf(HMC588_ADDR,buffer, 6, true);
	if(rv<=I2C_MASTER_OK )
		{
		 pCompass->ReadingNum=ReadingNumber++;
		 OSLock();
		 memcpy((void *)&Compass_Result,pCompass,sizeof(COMPASS_READING));
		 OSUnlock();
		 bNewCompass=true;
         }
	buffer[0]=0x0;
	rv=I2CSendBuf(MS5611_ADDR,buffer, 1, true);
    rv=I2CReadBuf(MS5611_ADDR,buffer+1, 3, true);
	buffer[0]=0;
	if(bPressure)
	{
	Alt_Result.dwPressure=*(PDWORD)buffer;
	buffer[0]=0x58; //Temp
	bPressure=false;
	}
	else
	{
	Alt_Result.dwTemperature=*(PDWORD)buffer;
	buffer[0]=0x48; //Pressure
	bPressure=true;
	}
	rv=I2CSendBuf(MS5611_ADDR,buffer, 1, true);
	if(pDataSem) OSSemPost(pDataSem);
	}
}



#define SM_TASK_STK (512)
#define SmOSSimpleTaskCreate(x,p) { static DWORD   func_##x_Stk[SM_TASK_STK] __attribute__( ( aligned( 4 ) ) ); OSTaskCreate(x,NULL,(void *)&func_##x_Stk[SM_TASK_STK],(void*)func_##x_Stk,p); }



void InitI2CSubSystem(WORD prio, OS_SEM * pSem)
{
	pDataSem=pSem;
	HardWarePinsInit(); 

	Master_I2CInit(); 

	HardWarePinsInit(); 


	InitMS5611();
	InitHMB588(); 

	SmOSSimpleTaskCreate(I2CTask,prio);

}

void Testcompass()
{
	HardWarePinsInit(); 

   Master_I2CInit(); 

   HardWarePinsInit(); 

   InitHMB588(); 
   InitMS5611();

while(1)
{
	ReadMS5611();
//	ReadHMC588();
	OSTimeDly(2);

}

}

