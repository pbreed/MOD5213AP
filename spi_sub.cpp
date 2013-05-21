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
#include <string.h>
#include <Pins.h>
#include <sim.h>
#include <cfinter.h>
#include "spi_sub.h"
#include "filereporter.h" 
REPORT_FILE;


// Instruct the C++ compiler not to mangle the function name 
extern "C" 
{
void SetIntc( long func, int vector, int level, int prio );
}

#define SM_TASK_STK (512)
#define SmOSSimpleTaskCreate(x,p) { static DWORD   func_##x_Stk[SM_TASK_STK] __attribute__( ( aligned( 4 ) ) ); OSTaskCreate(x,NULL,(void *)&func_##x_Stk[SM_TASK_STK],(void*)func_##x_Stk,p); }



OS_CRIT Spi_Crit;
OS_SEM  Spi_Sem;

#define QSPI_WRITE_QTxR(n,val) { sim.spi.qar=n; sim.spi.qdr=val; }
#define QSPI_WRITE_QCmdR(n,val) { sim.spi.qar=n+0x20; sim.spi.qdr=val; }
inline WORD QSPI_READ_QRxR(int n)   {sim.spi.qar=n+0x10; return sim.spi.qdr; };


void ShortDelay()
{volatile int i;
for(i=0; i<100; i++) asm(" nop");
}


INTERRUPT( qspi_done, 0x2700 )
{
sim.spi.qir=0x0101;
OSSemPost(&Spi_Sem);
}


 



void Gpio4(bool hi)
{
//Pin 6 of PQFP 100
//port UA 3

if(hi) 
	sim.gpio.setua = 0x08;
else
    sim.gpio.clrua = ~0x08;

sim.gpio.ddrua |= 0x08;


}






bool SpiDone()
{
	return (sim.spi.qdlyr & 0x8000)==0;
}

void WaitForSpi()
{
OSSemPend(&Spi_Sem,0);
while(!SpiDone()) 
	{
	iprintf("Repeat!");
	OSSemPend(&Spi_Sem,0);
	}
}

void SetSend8(int index, BYTE val, BYTE CSMask)
{
QSPI_WRITE_QTxR(index,val); 
QSPI_WRITE_QCmdR(index,0x8000|(CSMask<<8));
}


void SetSend16(int index, WORD val, BYTE CSMask)
{
QSPI_WRITE_QTxR(index,val); 
QSPI_WRITE_QCmdR(index,0xC000|(CSMask<<8));
}


void StartSpi(int qfrom, int qto)
{
//						                    N
//                                      H W W C
//                                      A R R S
//                                      L E T I
//                                      T N 0 V	ENDQ CPQ   STQ
 sim.spi.qwr=0x1000|(qto<<8)|(qfrom); /*1 0 0 1  To 0000 0000 */

sim.spi.qdlyr=0x8000;
}



#define CS_PROM  (0x0E)	 //1110
#define CS_ADIS  (0x0B)	 //1011
#define CS_MPU6K (0x0D)	 //1101

void TestMemory()
{
int index=0;
SetSend8(index++,0x9F,CS_PROM);
SetSend8(index++,0x0 ,CS_PROM);
SetSend8(index++,0x0 ,CS_PROM);
SetSend8(index++,0x0 ,CS_PROM);
SetSend8(index++,0x0 ,CS_PROM);
StartSpi(0,index-1);
WaitForSpi();

for(int i=0; i<index; i++)
{
printf("[%d]=%02X\r\n",i,QSPI_READ_QRxR(i));
}


}


static void  Read28Bytes(DWORD addr)
{
int index=0;
SetSend16(index++,0x0300|((addr>>16)&0xFF),CS_PROM);
SetSend16(index++,addr&0xFFFF,CS_PROM);
SetSend16(index++,0x0,CS_PROM);
SetSend16(index++,0x0,CS_PROM);
SetSend16(index++,0x0,CS_PROM);
SetSend16(index++,0x0,CS_PROM);
SetSend16(index++,0x0,CS_PROM);
SetSend16(index++,0x0,CS_PROM);
SetSend16(index++,0x0,CS_PROM);
SetSend16(index++,0x0,CS_PROM);
SetSend16(index++,0x0,CS_PROM);
SetSend16(index++,0x0,CS_PROM);
SetSend16(index++,0x0,CS_PROM);
SetSend16(index++,0x0,CS_PROM);
SetSend16(index++,0x0,CS_PROM);
SetSend16(index++,0x0,CS_PROM);
StartSpi(0,index-1);
WaitForSpi();
}

int ReadMemory(PBYTE dest, DWORD addr, int len)
{
OSCriticalSectionObj co(Spi_Crit);
int len_left=len;
while(len_left>0)
{
 Read28Bytes(addr);
 addr+=28;
 for(int i=2; i<16; i++)
	{
	 WORD w=QSPI_READ_QRxR(i); 
     if(len_left) { *dest++=((w>>8) & 0xFF); len_left--; }
     if(len_left) { *dest++=(w & 0xFF); len_left--; }
	}
}
	
	
return len;
}

static void SetWEL()
{
int index=0;
OSCriticalSectionObj co(Spi_Crit);
SetSend8(index++,0x06,CS_PROM);
StartSpi(0,index-1);
WaitForSpi();
}


static void ClrWEL()
{
OSCriticalSectionObj co(Spi_Crit);
int index=0;
SetSend8(index++,0x06,CS_PROM);
StartSpi(0,index-1);
WaitForSpi();
}

static BYTE ReadSR()
{
OSCriticalSectionObj co(Spi_Crit);
int index=0;
SetSend8(index++,0x05,CS_PROM);
SetSend8(index++,0x0,CS_PROM);
StartSpi(0,index-1);
WaitForSpi();
return (BYTE) QSPI_READ_QRxR(1);
}

void ChipErase()
{
OSCriticalSectionObj co(Spi_Crit);
SetWEL();
int index=0;
SetSend8(index++,0x60,CS_PROM);
StartSpi(0,index-1);
WaitForSpi();
ClrWEL();
}


bool WriteOrEraseDone()
{
	BYTE sr=ReadSR();
	iprintf("SR = %04X\r\n",sr);
	return ((sr & 1)==0);
}



static void Send16Blk(PWORD & from)
{
for(int i=0; i<16; i++)
	SetSend16(i,*from++,CS_PROM);
StartSpi(0,15);
WaitForSpi();
}

static void SetHardQSPICS(BYTE mask)
{
sim.gpio.setqs=(mask<<3);
sim.gpio.clrqs=(mask<<3);
sim.gpio.pqspar =0x0015; //00 00 00 00 00 01 01 01

}

static void ClearHardQSPICS()
{
sim.gpio.pqspar =0x1555; //00 01 01 01 01 01 01 01
}



static void FastBaud()
{
//						M H		 C C
//					    S Z		 P P
//						T D	 16  O H
//						R O	SIZE L A   Baud		/64 ~ 1Mhz
sim.spi.qmr=0x8306;  /*1 0 0000 1 0 00100000 */
}


static void SlowBaud()
{
//						M H		 C C
//					    S Z		 P P
//						T D	 16  O H
//						R O	SIZE L A   Baud		/64 ~ 1Mhz
sim.spi.qmr=0x8320;  /*1 0 0000 1 0 00100000 */
}

void WriteMemoryBlock(PWORD from, DWORD addr)
{
SetWEL();
OSCriticalSectionObj co(Spi_Crit);
FastBaud();
SetHardQSPICS(CS_PROM);
int index=0;
SetSend16(index++,0x0200|((addr>>16)&0xFF),CS_PROM);//0
SetSend16(index++,addr&0xFFFF,CS_PROM);//1
StartSpi(0,index-1);
WaitForSpi();

for(int i=0; i<8; i++) 
	Send16Blk(from);

SlowBaud();

ClearHardQSPICS();
ClrWEL();
}



static BYTE MPU6000_Read(BYTE reg)
{
OSCriticalSectionObj co(Spi_Crit);
int index=0;
SetSend8(index++,reg|0x80,CS_MPU6K);
SetSend8(index++,0x0,CS_MPU6K);
StartSpi(0,index-1);
WaitForSpi();
return (BYTE) QSPI_READ_QRxR(1);
}


static WORD MPU6000_ReadW(BYTE reg)
{
OSCriticalSectionObj co(Spi_Crit);
int index=0;
SetSend8(index++,reg|0x80,CS_MPU6K);
SetSend16(index++,0x0    ,CS_MPU6K);
StartSpi(0,index-1);
WaitForSpi();
return  QSPI_READ_QRxR(1);
}

static void MPU6000_Write(BYTE reg,BYTE val)
{
OSCriticalSectionObj co(Spi_Crit);
int index=0;
SetSend8(index++,reg,CS_MPU6K);
SetSend8(index++,val,CS_MPU6K);
StartSpi(0,index-1);
WaitForSpi();
}


OS_SEM Irq2_Sem;

volatile DWORD irq2_cnt;

INTERRUPT( irq_2, 0x2400 )
{
sim.eport.epfr |= 0x04; // Clear IRQ3 flag
irq2_cnt++;
OSSemPost(&Irq2_Sem);
}


void InitIrq2()
{
OSSemInit(&Irq2_Sem,0);



sim.gpio.pnqpar |= ( 4 );


sim.eport.eppar &= ~0x0020; // Configure IRQ2 to trigger an riseing edge


sim.eport.eppar |=  0x0010; // Configure IRQ2 to trigger an rising edge

sim.eport.epddr &= ~0x04; // Configure IRQ2 as input

sim.eport.epier |= 0x04; // Enable IRQ2

SetIntc( ( long ) &irq_2, 2, 2, 1 );
}



// MPU 6000 registers

#define MPU_STX  0x0D

#define MPU_STY  0x0DE

#define MPU_STZ  0x0DF

#define MPU_STA2  0x10

#define MPU_WHOAMI 0x75 //
#define	MPU_SMPLRT_DIV 0x19 //
#define MPU_CONFIG 0x1A //
#define MPU_GYRO_CONFIG 0x1B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_INT_PIN_CFG 0x37
#define	MPU_INT_ENABLE 0x38 
#define MPU_ACCEL_XOUT_H 0x3B //
#define MPU_ACCEL_XOUT_L 0x3C //
#define MPU_ACCEL_YOUT_H 0x3D //
#define MPU_ACCEL_YOUT_L 0x3E //
#define MPU_ACCEL_ZOUT_H 0x3F //
#define MPU_ACCEL_ZOUT_L 0x40 //
#define MPU_TEMP_OUT_H 0x41//
#define MPU_TEMP_OUT_L 0x42//
#define MPU_GYRO_XOUT_H 0x43 // 
#define	MPU_GYRO_XOUT_L 0x44 //
#define MPU_GYRO_YOUT_H 0x45 //
#define	MPU_GYRO_YOUT_L 0x46 //
#define MPU_GYRO_ZOUT_H 0x47 //
#define	MPU_GYRO_ZOUT_L 0x48 //
#define MPU_USER_CTRL 0x6A //
#define	MPU_PWR_MGMT_1 0x6B //
#define	MPU_PWR_MGMT_2 0x6C //

// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define	BIT_INT_ANYRD_2CLEAR	    0x10
#define	BIT_RAW_RDY_EN		    0x01
#define	BIT_I2C_IF_DIS              0x10



static BYTE InitMPU()
{
       // Chip reset
    MPU6000_Write(MPU_PWR_MGMT_1, BIT_H_RESET);
	OSTimeDly(3);
    
	// Wake Up device and select GyroZ clock (better performance)
    MPU6000_Write(MPU_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
	OSTimeDly(2);
    // Disable I2C bus (recommended on datasheet)
    MPU6000_Write(MPU_USER_CTRL, BIT_I2C_IF_DIS);
	OSTimeDly(2);
    
	MPU6000_Write(MPU_STX, 0);
	OSTimeDly(2);
	MPU6000_Write(MPU_STY, 0);
	OSTimeDly(2);
 	MPU6000_Write(MPU_STZ, 0);
	OSTimeDly(2);
	MPU6000_Write(MPU_STA2, 0);
    OSTimeDly(2);




	// SAMPLE RATE
    MPU6000_Write(MPU_SMPLRT_DIV,0x04);     // Sample rate = 200Hz    Fsample= 1Khz/(4+1) = 200Hz     
	OSTimeDly(2);
    
	// FS & DLPF   FS=1000º/s, DLPF = 42Hz (low pass filter)
    MPU6000_Write(MPU_CONFIG, BITS_DLPF_CFG_42HZ);  
    OSTimeDly(2);
    MPU6000_Write(MPU_GYRO_CONFIG,BITS_FS_500DPS);  // Gyro scale 1000º/s
    OSTimeDly(2);
    MPU6000_Write(MPU_ACCEL_CONFIG,0x18);//Was 18   // Accel scele 16g (4096LSB/g)
    OSTimeDly(2);
    // INT CFG => Interrupt on Data Ready
    MPU6000_Write(MPU_INT_ENABLE,BIT_RAW_RDY_EN);        // INT: Raw data ready
    OSTimeDly(2);
    
	MPU6000_Write(MPU_INT_PIN_CFG,BIT_INT_ANYRD_2CLEAR);        // INT: Clear on any read
    OSTimeDly(2);
	
	InitIrq2();  

	return MPU6000_Read(MPU_WHOAMI); 

}



static OS_SEM * pDataSem;
volatile IMU_READING ImuResult;
volatile IMU_ZEROS   ImuZeros;


volatile bool bNewImu;

void SPITask( void * pd)
{
	while(1)
	{
	WORD Ax,Ay,Az,Gx,Gy,Gz,T;
	static WORD imu_serial;

	OSSemPend(&Irq2_Sem,0);
	 {
		OSCriticalSectionObj co(Spi_Crit);
		Ax=MPU6000_ReadW(MPU_ACCEL_XOUT_H); 
		Ay=MPU6000_ReadW(MPU_ACCEL_YOUT_H); 
		Az=MPU6000_ReadW(MPU_ACCEL_ZOUT_H); 

		Gx=MPU6000_ReadW(MPU_GYRO_XOUT_H); 
		Gy=MPU6000_ReadW(MPU_GYRO_YOUT_H); 
		Gz=MPU6000_ReadW(MPU_GYRO_ZOUT_H); 

		T=MPU6000_ReadW(MPU_TEMP_OUT_H);  
      
	 }
	OSLock();
	ImuResult.Ax=Ax;
	ImuResult.Ay=Ay;
	ImuResult.Az=Az;

	ImuResult.Gx=Gx;
	ImuResult.Gy=Gy;
	ImuResult.Gz=Gz;

	ImuResult.T=T;
	ImuResult.ReadingNum=imu_serial++;
	OSUnlock();
	if(pDataSem) OSSemPost(pDataSem);
	bNewImu=true;
	}
}


void InitISpiSubSystem(WORD prio,OS_SEM * ps)
{
pDataSem=ps;

OSCritInit(&Spi_Crit);
OSSemInit(&Spi_Sem,0);
	
//Pins 
//QSPI_CS0  20 Data Log
//QSPI_CS1	19 MPU-6000
//QSPI_CS2	13 ADIS16
//QSPI_CS3	12 Unused
//QSPI_CLK	18
//QSPI_DIN	16
//QSPI_DOUT	17
sim.gpio.pqspar =0x1555; //00 01 01 01 01 01 01 01
//Gpio4
sim.gpio.puapar &= ~0xC0;

sim.gpio.ddrqs=0xF8;
sim.gpio.setqs=0xF8;




//						M H		 C C
//					    S Z		 P P
//						T D	 16  O H
//						R O	SIZE L A   Baud		/64 ~ 1Mhz
sim.spi.qmr=0x8320;  /*1 0 0000 1 0 00100000 */
sim.spi.qdlyr=0;
//						   N
//                     H W W C
//                     A R R S
//                     L E T I
//                     T N 0 V	ENDQ CPQ   STQ
sim.spi.qwr=0x1000; /*0 0 0 1  0001 0000 0000 */

 
//Reset the external SPI hardware
Gpio4(false);
ShortDelay(); 
Gpio4(true);

 
SetIntc( ( long ) &qspi_done, 18, 5, 1 );
sim.spi.qir=0x0101; 

BYTE idv=InitMPU(); 
iprintf("MPU ID Value = %02X\r\n",idv);

SmOSSimpleTaskCreate(SPITask,prio);

}


