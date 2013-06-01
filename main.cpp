/* Rev:$Revision: 1.2 $ */
/******************************************************************************
 *
 * Copyright 1998-2006 NetBurner, Inc.  ALL RIGHTS RESERVED
 *   Permission is hereby granted to purchasers of NetBurner Hardware
 *   to use or modify this computer program for any use as long as the
 *   resultant program is only executed on NetBurner provided hardware.
 *
 *   No other rights to use this program or it's derivitives in part or
 *   in whole are granted.
 *
 *   It may be possible to license this or other NetBurner software for
 *   use on non NetBurner Hardware. Please contact Licensing@Netburner.com 
 *   for more information.
 *
 *   NetBurner makes no representation or warranties with respect to the 
 *   performance of this computer program, and specifically disclaims any 
 *   responsibility for any damages, special or consequential, connected 
 *   with the use of this program.
 *
 *   NetBurner, Inc
 *   5405 Morehouse Drive
 *   San Diego Ca, 92121
 *   http://www.netburner.com
 *
 *****************************************************************************/

/******************************************************************************
 Mod5213 I2C2Serial
 This program will illustrate how to use the NetBurner I2C driver to transmit
 multi-master or master-only I2C.  This application should be used with multiple
 NetBurner modules and at most 1 if any configured to be master-only. 

 Please note that jumpers 11 and 12 should be put on the MOD5213 developement 
 board to enable the I2C pull up resistors.     
*****************************************************************************/

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
#include <a2d.h>
#include <cfinter.h>
#include <math.h>
#include <bsp.h>
#include "filereporter.h" 
REPORT_FILE

#include "pitr_sem.h"
#include "i2c_sub.h"
#include "spi_sub.h"
#include "a2d_sub.h"
#include "dsm2_sub.h"
#include "servo_drive.h"
#include "sensor_config.h"
#include "gps.h"
#include "log.h"
#include "prio.h"
#include "imu.h"
#include "exmath.h"

#define PITOT_A2D (4)

// Instruct the C++ compiler not to mangle the function name 
extern "C" 
{
void UserMain( void * pd);
void SetIntc( long func, int vector, int level, int prio );
}

void RawSetServo(int ch,float v);//Limits -1.0 to +1.0


#define SERVO_ALIERON (2)
#define SERVO_ELEVATOR (3)
#define SERVO_RUDDER   (0)
#define SERVO_THROTTLE (1)

 static IMU_Euler imu_state;
 

void SetServo(int ch, float v)
{
float f=0.0;
if(v<0.0)
{
f=(-v*(SensorConfig.servo_neg_lim[ch]-SensorConfig.servo_mid[ch]));
}
else
if(v>0.0)
{
f=(v*(SensorConfig.servo_pos_lim[ch]-SensorConfig.servo_mid[ch]));
}

RawSetServo(ch,SensorConfig.servo_mid[ch]+f);
}

OS_SEM Pit100Hz_Sem;
OS_SEM NewDataSem;



float GzerosF[3];

BOOL bStaticZero;
float gSlew;






void ImuTest();
void MagTest();
void AcelTest();
void GyroTest();
void ServoTest();
void QTest();
void RCTest();
void GPSTest();
void BaroTest();

// Name for development tools to identify this application 
const char * AppName="MOD5213 AP";
extern volatile DWORD irq2_cnt;




void CommandParser()
{
 while(1)
 {
	iprintf("\r\nCMD  (Cap x,d,z,i,m,a,s,q,b,l,v>");
	char c=sgetchar(0);
	if(c=='X') return;
	if(c=='D') LogDump();
	if(c=='Z') LogErase();
	if(c=='I') ImuTest();
	if(c=='M') MagTest();
	if(c=='A') AcelTest();
	if(c=='G') GyroTest();
	if(c=='S') ServoTest();
	if(c=='Q') QTest();
	if(c=='R') RCTest();
	if(c=='N') GPSTest();
	if(c=='B') BaroTest();
	if(c=='L') {ForceReboot();}
	if(c=='V') FileReporter::ShowList();

 }
}



WORD gLastMagSerial;


void ImuTest()
{
WORD LastImuSerial =ImuResult.ReadingNum;

while(!charavail(0))
{
OSSemPend(&NewDataSem,0);
if(LastImuSerial !=ImuResult.ReadingNum)
 {
  LastImuSerial =ImuResult.ReadingNum;
  iprintf("%+5d,%+5d %+5d   %+5d %+5d %+5d\r\n",ImuResult.Ax,ImuResult.Ay,ImuResult.Az,ImuResult.Gx,ImuResult.Gy,ImuResult.Gz);
 }
}
}

double cmag(double * pmags)
{
return sqrt((pmags[0]*pmags[0]+pmags[1]*pmags[1]+pmags[2]*pmags[2]));
}

double LimitTest(short & mv, short v, short & mav)
{
if(mav<v) mav=v;
if(mv>v) mv=v;
short avg=(mav+mv)/2;
short sv=(v-avg);

iprintf("[%5d,%5d,%5d,%5d]",mv,v,sv,mav);
if((mav-mv)>4)
{
 return 2.0*(double(sv))/( ((double)mav)-((double)mv) ) ;
}
return 0.0;

}

void ProcessImu();

void ImuZero()
 {
 WORD LastImuSerial =ImuResult.ReadingNum;
 DWORD ActiveTime=Secs;
 iprintf("Starting IMU Zero \r\n");
 bStaticZero=true;
 bServoOff=true;


 while(Secs<(ActiveTime+12))
 {
  OSSemPend(&NewDataSem,0);
  if(LastImuSerial !=ImuResult.ReadingNum)
  {
	  ProcessImu();
	  LastImuSerial =ImuResult.ReadingNum;
  }
 }

 iprintf("AX avg = %ld \r\n",ImuZeros.Azerosx>>13);
 iprintf("AY avg = %ld \r\n",ImuZeros.Azerosy>>13);
 iprintf("AZ avg = %ld \r\n",ImuZeros.Azerosz>>13);

 }


void EndStatic()
{
		      GzerosF[0]=ImuZeros.Gzerosx;
			  GzerosF[1]=ImuZeros.Gzerosy;
			  GzerosF[2]=ImuZeros.Gzerosz;
			  GzerosF[0]/=(float)(1<<13);
			  GzerosF[1]/=(float)(1<<13);
			  GzerosF[2]/=(float)(1<<13);
		      bStaticZero=false;
}
 


float MagScale(short rv, int index)
{													   
float v=rv;
float fmin=SensorConfig.mag_min[index];
float fmax=SensorConfig.mag_max[index];
v-=(fmin+fmax)*0.5;
v*=2.0/(fmax-fmin);
return v;
}

static float GYawL4[4];
static float GRollL4[4];
static float GPitchL4[4];
static float GYawX4;
static float GRollX4;
static float GPitchX4;
static float Ball;


void DoFourAverage(float & avg, float * Vals, float nv)
{
avg+=nv;
avg-=Vals[0];
Vals[0]=Vals[1];
Vals[1]=Vals[2];
Vals[2]=Vals[3];
Vals[3]=nv;
}


const float Kgx=1.0*(2.663161E-4);	//32768 = 500 dps
const float Kgy=1.0*(2.663161E-4);	//32768 = 500 DPS
const float Kgz=1.0*(2.663161E-4);	//32768= 500 DPS
const float Ka= (1.0/2000.0);

// Need:
//+Gy = + pitch (Nose Up)
//+Gx = + roll  (Right wing down)
//+Gz = + yaw   (Nose right)
//
//
//AX=1.0 = -90 pitch
//AX=-1.0 = 90 pitch
//Ay=1.0  =+ 90 Roll
//Ay -1.0	= -90 Roll
//
//Az -1.0 = Inverted
//Az 1.0 = level
//
//
//Mx 0.5, My=0; Mz=-0.86  -> Heading ~ 0
//Mx 0, My =0.5, Mz=-0.86 ->Heading ~ -90
//Mx -0.5, My=0; Mz=-0.86  -> Heading ~ 180
//Mx 0, My =-0.5, Mz=-0.86 ->Heading ~ +90


void FillInValues(VehicleSense & vs, bool bFillMag )
{           
// Need:
//+Gx = + roll  (Right wing down)
//+Gy = + pitch (Nose Up)
//+Gz = + yaw   (Nose right)
//
//Have
//Gx Pitch + for nose down
//Gy Roll  + for roll left
//Gz Yaw   + for yaw left

			vs.Gx=-Kgy*(((float)ImuResult.Gy)-GzerosF[1])  ;
			vs.Gy=-Kgx*(((float)ImuResult.Gx)-GzerosF[0])  ;
			vs.Gz=-Kgz*(((float)ImuResult.Gz)-GzerosF[2])  ;

			DoFourAverage(GYawX4,GYawL4,vs.Gz);
			DoFourAverage(GRollX4,GRollL4,vs.Gx);
			DoFourAverage(GPitchX4,GPitchL4,vs.Gy);


//Need:
//AX=1.0 = -90 pitch straight down
//AX=-1.0 = 90 pitch straight up
//Ay=1.0  =+ 90 Roll
//Ay -1.0	= -90 Roll
//Az -1.0 = Inverted
//Az 1.0 = level
//Have:
//Level Ax=0,Ay=0, Az=1
//Nose Down Ax=0 Ay=1 Az=0
//Right wing down Ax=1 Ay=0 Az=0

	        vs.Ax=(float)(ImuResult.Ay-SensorConfig.accel_zero[1])*Ka;
										
			vs.Ay=(float)(ImuResult.Ax-SensorConfig.accel_zero[0])*Ka;
	        Ball=vs.Ay;

			vs.Az=(float)(ImuResult.Az-SensorConfig.accel_zero[2])*Ka;

//Need
//Mx 0.5, My=0; Mz=-0.86  -> Heading ~ 0
//Mx 0, My =0.5, Mz=-0.86 ->Heading ~ -90/w/270
//Mx -0.5, My=0; Mz=-0.86  -> Heading ~ 180/s
//Mx 0, My =-0.5, Mz=-0.86 ->Heading ~ +90
//Have
//Level north(0) Ax=0.48 Ay=-0.06 Az=-0.85
//Level south(180) Ax=-0.51 Ay= 0.05  Az=-0.83
//Level West (270) Ax=-0.11 ay=-0.50 az=-0.82
//Level East(90) AX=-0.03 Ay=0.45 az=-0.82


			if(bFillMag)
			{
			 vs.Mx=MagScale(Compass_Result.Mag_X,0);
			 vs.My=-MagScale(Compass_Result.Mag_Y,1); 
			 vs.Mz=MagScale(Compass_Result.Mag_Z,2); 
			}
			else
			{
				vs.Mx=0;
				vs.My=0;
				vs.Mz=0;
			}
		 
}


void QTest()
{
WORD LastImuSerial =ImuResult.ReadingNum;
WORD lclLastMagSerial=Compass_Result.ReadingNum;

IMU_Init();
	
ImuZero();
EndStatic();

	iprintf("Hit X to exit loop\r\n");
	OSTimeDly(20);
	DWORD sSec=Secs;
	float Slew=100;
	while(1)
	{
	while(!charavail(0)) 
	{
		OSSemPend(&NewDataSem,0);
		if(LastImuSerial !=ImuResult.ReadingNum)
		{
	     
		static VehicleSense vs;
		if(lclLastMagSerial!=Compass_Result.ReadingNum)
		{
			lclLastMagSerial=Compass_Result.ReadingNum;
			FillInValues(vs,true);
		}
		else
		{
			FillInValues(vs,false);
		}
		LastImuSerial =ImuResult.ReadingNum;
	    
		static IMU_Euler ie;
		if((Secs-sSec)>100) Slew=1.0;
        else
		if((Secs-sSec)>10) Slew=10.0;
         

	   IMU_Step(vs,ie,Slew);

	    printf("R %6d,P %6d,Y%6d\r\n",(int)(57.295*ie.roll),(int)(57.295*ie.pitch),(int)(57.295*ie.yaw));

	   }
	      }
   char c=sgetchar(0);
   if(c=='X') break;
  }

}



void GyroTest()
{
WORD LastImuSerial =ImuResult.ReadingNum;

	iprintf("Hit X to exit loop\r\n");
	while((!charavail(0)) || (sgetchar(0)!='X'))
	{  int n=0;
	   int sax,say,saz;
	   sax=0;
	   say=0;
	   saz=0;
	   while(n<500)
	   {
		OSSemPend(&NewDataSem,0);
		if(LastImuSerial !=ImuResult.ReadingNum)
		{
		   sax+=(short)ImuResult.Gx;
		   say+=(short)ImuResult.Gy;
		   saz+=(short)ImuResult.Gz;
		   n++;
		   LastImuSerial =ImuResult.ReadingNum;
	   }
	   }//Sample while
	   iprintf("Ax=%5d Ay=%5d Az=%5d\r\n",sax/500,say/500,saz/500);
    }//Outer while
}

void AcelTest()
{
WORD LastImuSerial =ImuResult.ReadingNum;

static short minx,miny,minz;
static short maxx,maxy,maxz;

while(1)
{

iprintf("Accel subsystem Do you want Default,Recall,Cal,Save,Exit,Test?");
char c=sgetchar(0);
switch(c)
{
case 'E': return;
	break;
case 'D':
    minx=-511;
    maxx=453;
    miny=-580;
    maxy=402;
    minz=-446;
    maxz=3635;
	break;
case 'L':
    while((!charavail(0)) || (sgetchar(0)!='X'))
		{
		OSSemPend(&NewDataSem,0);
		if(LastImuSerial !=ImuResult.ReadingNum)
		{
			LastImuSerial =ImuResult.ReadingNum;
		  iprintf("Az=%d\r\n",ImuResult.Az);
		}
		LastImuSerial =ImuResult.ReadingNum;
	}
	break;
case 'C':
	maxx=minx=ImuResult.Ax;
	miny=maxy=ImuResult.Ay;
	minz=maxz=ImuResult.Az;
case 'S':
	{
	
	iprintf("Ax,y,z zero?");
	int i0,i1,i2;
	iscanf("%d,%d,%d",&i0,&i1,&i2);
	SensorConfig.accel_zero[0]=i0;
	SensorConfig.accel_zero[1]=i1;
	SensorConfig.accel_zero[2]=i2;
	SaveSensorConfig();
	}
case 'V':
	 iprintf("\r\nAX zeros:%d,%d,%d\r\n",SensorConfig.accel_zero[0],SensorConfig.accel_zero[1] ,SensorConfig.accel_zero[2]); 
	break;
case 'T':
	iprintf("Hit X to exit loop\r\n");
	while((!charavail(0)) || (sgetchar(0)!='X'))
	{  int n=0;
	   int sax,say,saz;
	   sax=0;
	   say=0;
	   saz=0;
	   while(n<500)
	   {
		OSSemPend(&NewDataSem,0);
		if(LastImuSerial !=ImuResult.ReadingNum)
		{
		   sax+=(short)ImuResult.Ax;
		   say+=(short)ImuResult.Ay;
		   saz+=(short)ImuResult.Az;
		   n++;

			LastImuSerial =ImuResult.ReadingNum;
			{//double mag[3];
				//mag[0]=Limit(minx,ImuResult.Ax,maxx);
				//mag[1]=Limit(miny,ImuResult.Ay,maxy);
				//mag[2]=Limit(minz,ImuResult.Az,maxz);
				//iprintf("Mag: %d\r\n",(int)(1000.0*cmag(mag)));

			}
         }
	   }

	   iprintf("Ax=%5d Ay=%5d Az=%5d\r\n",sax/500,say/500,saz/500);
      }//Test loop
     }//Switch
}//Outer while
}



void MagTest()
{
WORD LastImuSerial =ImuResult.ReadingNum;
WORD lclLastMagSerial = Compass_Result.ReadingNum;

static short minx,miny,minz;
static short maxx,maxy,maxz;

while(1)
{

iprintf("Mag subsystem Do you want Default,Float Test,Recall,Cal,Save,Exit,Test Vals?");
char c=sgetchar(0);
switch(c)
{
case 'E': return;
	break;
case 'F':
    iprintf("Hit X to exit loop\r\n");
	while((!charavail(0)) || (sgetchar(0)!='X'))
	{
		OSSemPend(&NewDataSem,0);
		if(lclLastMagSerial != Compass_Result.ReadingNum)
		 {
			 lclLastMagSerial = Compass_Result.ReadingNum; 
			 float Mx=MagScale(Compass_Result.Mag_X,0);
			 float My=MagScale(Compass_Result.Mag_Y,1); 
			 float Mz=MagScale(Compass_Result.Mag_Z,2); 
			 printf("%d %d %d %5g %5g %5g  Head=%g\r\n",Compass_Result.Mag_X,Compass_Result.Mag_Y,Compass_Result.Mag_Z,Mx,My,Mz, 57.295*atan2(Mx,My));




		}
	}
	break;
case 'D':
    minx=-612;
    maxx=366;
    miny=-591;
    maxy=512;
    minz=-400;
    maxz=488;
	break;
case 'C':
	maxx=minx=Compass_Result.Mag_X;
	miny=maxy=Compass_Result.Mag_Y;
	minz=maxz=Compass_Result.Mag_Z;
case 'T':
	iprintf("Hit X to exit loop\r\n");
	while((!charavail(0)) || (sgetchar(0)!='X'))
	{
		OSSemPend(&NewDataSem,0);
		if(LastImuSerial !=ImuResult.ReadingNum)
		{	
			LastImuSerial =ImuResult.ReadingNum;
			if(lclLastMagSerial != Compass_Result.ReadingNum)
			{double mag[3];
				lclLastMagSerial = Compass_Result.ReadingNum; 
				mag[0]=LimitTest(minx,Compass_Result.Mag_X,maxx);
				mag[1]=LimitTest(miny,Compass_Result.Mag_Y,maxy);
				mag[2]=LimitTest(minz,Compass_Result.Mag_Z,maxz);
				iprintf("Mag: %d\r\n",(int)(1000.0*cmag(mag)));

			  //printf("(%6f,%6f,%6f:%6f\r\n",mag[0],mag[1],mag[2],cmag(mag));
             //iprintf("[%5d,%5d,%5d][%5d,%5d,%5d][%5d,%5d,%5d] \r\n",minx,Compass_Result.Mag_X,maxx,miny,Compass_Result.Mag_Y,maxy,minz,Compass_Result.Mag_Z,maxz);
			}
         }
      }//Test loop
	 iprintf("Results X: [%d,%d]\r\n",maxx,minx);
	 iprintf("Results Y: [%d,%d]\r\n",maxy,miny);
	 iprintf("Results Z: [%d,%d]\r\n",maxz,minz);
	 break;
case 'S':
	iprintf("Save Cal...\r\n");
	SensorConfig.mag_min[0]=minx;
	SensorConfig.mag_min[1]=miny;
	SensorConfig.mag_min[2]=minz;
	SensorConfig.mag_max[0]=maxx;
	SensorConfig.mag_max[1]=maxy;
	SensorConfig.mag_max[2]=maxz;
	SaveSensorConfig();


	break;

case 'V':
	 iprintf("X max: %d Min :%d\r\n",SensorConfig.mag_max[0],SensorConfig.mag_min[0]);
	 iprintf("Y max: %d Min :%d\r\n",SensorConfig.mag_max[1],SensorConfig.mag_min[1]);
 	 iprintf("Z max: %d Min :%d\r\n",SensorConfig.mag_max[2],SensorConfig.mag_min[2]);
	break;
     }//Switch
}//Outer while
}


void GPSTest()
{
    WORD LastGPSFrame=GPS_Result.ReadingNum;
	iprintf("RC Test Hit X to exit loop\r\n");
	while((!charavail(0)) || (sgetchar(0)!='X'))
	{
		OSSemPend(&NewDataSem,0);
		if(LastGPSFrame!=GPS_Result.ReadingNum)
		{
			LastGPSFrame=GPS_Result.ReadingNum;
		   iprintf("Sat=%d Fix = %d Lat=%d Lon =%d  %d,%d\r\n",GPS_Result.numSV,GPS_Result.GPSfix ,GPS_Result.LAT,GPS_Result.LON,GPS_Result.week,GPS_Result.itow);
		}
	}


}


/*extern volatile unsigned char LastRC[256];
extern volatile unsigned char LastPitLsb[256];
extern volatile unsigned char LastRcVal;
extern volatile bool bSuspendRcCap;

*/
void RCTest()
{
 /*   while(sgetchar(0)!='X')
	{
	 bSuspendRcCap=true;
	 int n=0;
	 for(BYTE i=LastRcVal-64; i!=LastRcVal; i++)
	 {
	  iprintf("R:%02X T%02X,",LastRC[i],LastPitLsb[i]);
	  if(n++==8) {iprintf("\r\n"); n=0; }
	  }
	 }
	 bSuspendRcCap=false;
	return;
   */
	 
	WORD LastReadingNum=DSM2_Result.ReadingNum;
	iprintf("RC Test Hit X to exit loop\r\n");
	while((!charavail(0)) || (sgetchar(0)!='X'))
	{
		OSSemPend(&NewDataSem,0);
		if(LastReadingNum !=DSM2_Result.ReadingNum)
		{
			LastReadingNum=DSM2_Result.ReadingNum;
		   // printf("%5d %5d %5d %5d %5d %5d %5d %5d\r\n",DSM2_Result.val[0],DSM2_Result.val[1],DSM2_Result.val[2],DSM2_Result.val[3],DSM2_Result.val[4],DSM2_Result.val[5],DSM2_Result.val[6],DSM2_Result.val[7]);
		   // iprintf("%4d,%4d,%4d,%4d %04X %04X %04X %04X %04X\r\n",Xbee_Result.nElevator,Xbee_Result.nAlieron ,Xbee_Result.nRudder,Xbee_Result.nThrottle,Xbee_Result.nN1,Xbee_Result.nN2,Xbee_Result.nN3,Xbee_Result.nN4,Xbee_Result.switches);
		   //printf("E:%5g,A:%5g,R:%5g,T:%5g\r\n",Scaled_DSM2_Result.fElevator,Scaled_DSM2_Result.fAlieron ,Scaled_DSM2_Result.fRudder,Scaled_DSM2_Result.fThrottle);
		   for(int i=0; i<10; i++) iprintf("%5d ",DSM2_Result.val[i]);
		   iprintf("\r\n");
		}
	}

}


void BaroTest()
{
	while((!charavail(0)) || (sgetchar(0)!='X'))
	 {
		OSTimeDly(5);
		iprintf("Press %8ld   Temp %8ld  Pitot %5d\r\n",Alt_Result.dwPressure,Alt_Result.dwTemperature,ReadA2D(PITOT_A2D) );
	}
}




void ProcessImu()
{
if((bStaticZero) && (ImuZeros.Azerosz==0))
{
ImuZeros.Gzerosx=ImuResult.Gx;
ImuZeros.Gzerosy=ImuResult.Gy;
ImuZeros.Gzerosz=ImuResult.Gz;
ImuZeros.Azerosx=ImuResult.Ax;
ImuZeros.Azerosy=ImuResult.Ay;
ImuZeros.Azerosz=ImuResult.Az;


ImuZeros.Gzerosx=ImuZeros.Gzerosx<<13;
ImuZeros.Gzerosy=ImuZeros.Gzerosy<<13;
ImuZeros.Gzerosz=ImuZeros.Gzerosz<<13;
ImuZeros.Azerosx=ImuZeros.Azerosx<<13;
ImuZeros.Azerosy=ImuZeros.Azerosy<<13;
ImuZeros.Azerosz=ImuZeros.Azerosz<<13;
}

if(bStaticZero)
{
ImuZeros.Gzerosx+=ImuResult.Gx;
ImuZeros.Gzerosy+=ImuResult.Gy;
ImuZeros.Gzerosz+=ImuResult.Gz;
ImuZeros.Azerosx+=ImuResult.Ax;
ImuZeros.Azerosy+=ImuResult.Ay;
ImuZeros.Azerosz+=ImuResult.Az;
ImuZeros.Gzerosx-=(ImuZeros.Gzerosx>>13);
ImuZeros.Gzerosy-=(ImuZeros.Gzerosy>>13);
ImuZeros.Gzerosz-=(ImuZeros.Gzerosz>>13);
ImuZeros.Azerosx-=(ImuZeros.Azerosx>>13);
ImuZeros.Azerosy-=(ImuZeros.Azerosy>>13);
ImuZeros.Azerosz-=(ImuZeros.Azerosz>>13);
}
else
{

      static VehicleSense vs;
	   if(gLastMagSerial!=Compass_Result.ReadingNum)
	   {
		   gLastMagSerial=Compass_Result.ReadingNum;
		   FillInValues(vs,true);
		   IMU_Step(vs,imu_state,gSlew);
		   
		   if(bLog) 
		    {
			static StateRec	sr;
	        sr.Ax=         vs.Ax;
           	sr.Ay=         vs.Ay;
           	sr.Az=         vs.Az;
           	sr.Gx=         vs.Gx;
           	sr.Gy=         vs.Gy;
           	sr.Gz=         vs.Gz;
           	sr.Mag_X=      vs.Mx;
           	sr.Mag_Z=      vs.My;
           	sr.Mag_Y=      vs.Mz;
           	sr.roll=       imu_state.roll;
           	sr.pitch=      imu_state.pitch;
           	sr.yaw=        imu_state.yaw;
           	sr.dwPressure= Alt_Result.dwPressure;    
           	sr.pitot=ReadA2D(PITOT_A2D);
			LogState(sr);
		    }
		   
		
	   }
	   else
	   {
		   FillInValues(vs,false);
	       IMU_Step(vs,imu_state,gSlew);
	   }

}
}




void ServoTest()
{
bServoOff=false; 
double v=0.0;
int n=-1;
bool bRaw=false;
while(1)
{	
	iprintf("Z,h,H,f,F,X>");
	char c=sgetchar(0);

	if(c=='X') break;
	if(c=='Z') v=0;
	if(c=='h') v=-0.5;
	if(c=='H') v=0.5;
	if(c=='f') v=-1;
	if(c=='F') v=1;
	if(c=='R') bRaw=true;
	if(c=='r') bRaw=false;


	if(c=='.') v+=0.02;
	if(c==',') v-=0.02;
	if(c=='>') v+=0.005;
	if(c=='<') c-=0.005;
	if(c=='0') n=0;
	if(c=='1') n=1;
	if(c=='2') n=2;
    if(c=='3') n=3;
	if(c=='4') n=4;
	if(c=='5') n=5;
	if(c=='A') n=-1;
	if(c=='a') n=-1;
	if(bRaw) printf("Raw:"); else printf("Cal:");
	 printf("%d V=%g\r\n",n,v);
	 if(bRaw)
	{
	 if((n==0) || (n==-1)) RawSetServo(0,v);
	 if((n==1) || (n==-1)) RawSetServo(1,v);
	 if((n==2) || (n==-1)) RawSetServo(2,v);
	 if((n==3) || (n==-1)) RawSetServo(3,v);
 	 if((n==4) || (n==-1)) RawSetServo(4,v);
 	 if((n==5) || (n==-1)) RawSetServo(5,v);
	}
	else
	{
	if((n==0) || (n==-1)) SetServo(0,v);
	if((n==1) || (n==-1)) SetServo(1,v);
	if((n==2) || (n==-1)) SetServo(2,v);
	if((n==3) || (n==-1)) SetServo(3,v);
 	if((n==4) || (n==-1)) SetServo(4,v);
 	if((n==5) || (n==-1)) SetServo(5,v);
	}
}

bServoOff=true; 
}



const float RadX4toDps=(45/M_PI);


void NewSwitchState(WORD sw, WORD old)
{
}

inline void DoBipolarLimit(float & f, float lim)
{
if(f>lim) f=lim;
else
if(f<-lim) f=-lim;
}

const float KHeadingCorrection = 1.0;
const float MaxTurnRate=30.0;
const float RudderGain=0.12;


static float CurGPSHead_deg;
static float CurGPSAlt_m; 
static float CurGPSVV_mps;
static float CurGPSGS_mps;
static float PitchZeroElevator;


float AutoRudder()
{
return RudderGain*Ball;
}


const float KAlg = (1.0/45);

float estimated_roll_error;


float AutoAlieron(float target_head)
{

//Ok we have turning roll error...
//Given GPS velocity... 
//And rate of turn we have estimated actual roll error
//float cur_velocity=GPS_Result.Speed*0.01;
//float cur_rate=GYawX4*RadX4toDps; //Rate is deg per second of yaw added to roll rate
//float cent_accell_g=(cur_rate*cur_velocity)*0.001779133;
//float best_roll_guess=atan(cent_accell_g)*57.29577951;


float best_roll_guess=atanf(GYawX4*((float)GPS_Result.Speed)*0.000254842);

float croll=imu_state.roll*DEGREES_PER_RADIAN;

//This should be called at 50Hz so 0.005 is 100% corection in 2 sec
estimated_roll_error+=0.005*(croll-best_roll_guess);  //real roll 30 reading is 45  error is -15

	
	

float chead=(float)GPS_Result.Heading*1.0E-5;
float herr=(target_head-chead);
while(herr>180) herr-=360.0;
while(herr<-180) herr+=360.0;
float target_roll=herr;


if(target_roll>45.0) target_roll=45.0;
if(target_roll<-45.0) target_roll=-45.0;

//Adjust for centrifugal
croll+=estimated_roll_error; //reading 45+-15 = 30 which is real
return (target_roll-croll)*KAlg;
}




float XAutoAlieron(float target_head)
{


float herr=(target_head-CurGPSHead_deg);
while(herr>180) herr-=360.0;
while(herr<-180) herr+=360.0;


float cur_rate=(GYawX4+GRollX4)*RadX4toDps; //Rate is deg per second of yaw added to roll rate
float target_rate=herr*KHeadingCorrection;
DoBipolarLimit(target_rate,MaxTurnRate);

//float target_roll=herr;
//if(target_roll>45.0) target_roll=45.0;
//if(target_roll<-45.0) target_roll=-45.0;
//return (target_roll-croll)*KAlg;

return (target_rate-cur_rate)*KAlg;
}

/*
const float KAltToVVGain =0.5;
const float MaxVV=(10.0);
const float PitchRateConstant = //Pitch rate is 4X Rad/sec to VV adjustment
const float ElevatorGain = 0.025;//From vvError in m/sec to pitch 10 = 0.25 pitch.
const float LevelPitch=5.0;

float AutoElevator(float target_alt)
{
float alt_err=(target_alt-CurGPSAlt_m);
float target_vv=(alt_err * KAltToVVGain);
DoBipolarLimit(target_vv,MaxVV);
float vvError=(target_vv-CurGPSVV_mps);
//vvError is positive if we want maore altitude...
float TargetPitch

vvError-=GPitchX4*PitchRateConstant;  //Positive for nose up...pitch rate
return vvError*ElevatorGain+PitchZeroElevator;
}

*/

// Main task 
void UserMain( void * pd)
{
DWORD cmd_t=0;
OSChangePrio( MAIN_PRIO );
EnableSmartTraps();



SimpleUart( 0, SystemBaud );
assign_stdio( 0 );
   
OSSemInit(&Pit100Hz_Sem,0);
OSSemInit(&NewDataSem,0);

LoadSensorConfig();

PiterSem(&Pit100Hz_Sem,200); 
iprintf("Start servo..");
InitServoDrive();
SetServo(SERVO_THROTTLE,-1.0);


//iprintf("Xbeew...");
//InitIXbeeSubSystem(XBEE_PRIO, &NewDataSem); //0x32 - 4 = 2E
InitIDSM2SubSystem(XBEE_PRIO,&NewDataSem);


iprintf("I2C...");
InitI2CSubSystem(I2C_PRIO,&NewDataSem); 	 //0x32 -1 = 31
iprintf("SPI...");

InitISpiSubSystem(SPI_PRIO,&NewDataSem);  //0x32-3 = 2F
iprintf("A2D\r\n");
Inita2DSub();

iprintf("Init GPS\r\n");
InitGpsSubSystem(GPS_PRIO,&NewDataSem); 	 //0x32-2 = 0x30

int state=0;

siprintf((char *)Screen[0],"Initialize");
 
Screen[1][0]='W';

while(charavail(0))
{
    char c=sgetchar(0);
    if ((c=='c') && (state==0))	state=1;
	else
	if ((c=='m') && (state==1))	state=2;
	else
	if ((c=='d') && (state==2))	
		{
		bServoOff=true;
		CommandParser();
		bServoOff=false;
		iprintf("CMD Done\r\n");
	    }
	 else
	   state=0;

}

Screen[1][0]='L';

iprintf("Starting up finding log start\r\n");

LogInit();

iprintf("Log init done (lower case C----M----D\r\n");
iprintf("Battery = %d\r\n",ReadBVx1000());


cmd_t=Secs;
while(Secs<(cmd_t+5))
{
if(charavail(0))
 {
	char c=sgetchar(0);
    if ((c=='c') && (state==0))	state=1;
	else
	if ((c=='m') && (state==1))	state=2;
	else
	if ((c=='d') && (state==2))	
		{
		bServoOff=true;
		CommandParser();
		bServoOff=false;
		iprintf("CMD Done\r\n");
	    }
	 else
	   state=0;
 }

}

LogConfig(SensorConfig);






 WORD LastImuSerial =ImuResult.ReadingNum;
 WORD LastRCFrame =DSM2_Result.ReadingNum;
 WORD LastGPSFrame=GPS_Result.ReadingNum;
 DWORD LastStatusUpTime=Secs;

Screen[1][0]='Z';

 ImuZero();
 

 iprintf("Starting run mode battery = %d\r\n",ReadBVx1000());
  bStaticZero=true;
 IMU_Init( ); //Zero the IMU
 EndStatic();
 LogZeros(*((IMU_ZEROS *)&ImuZeros));
 
 siprintf((char *)Screen[0],"              ");
 bServoOff=false;

 iprintf("Running!");
 gSlew=100.0;
 
 static DWORD LastRcFrameTime;
 while(1)
 {
	 OSSemPend(&NewDataSem,0);
	 
	 if(LastImuSerial !=ImuResult.ReadingNum)
	 {
	   LastImuSerial =ImuResult.ReadingNum;
	   ProcessImu();
	 }
	 LogService();
	 
	 
	 if(LastRCFrame != DSM2_Result.ReadingNum)
	 { 
	   if ((bLog) && ((DSM2_Result.ReadingNum &15)==15))
		    LogRC(*((DSM2_READING*)&DSM2_Result));
		 LastRCFrame= DSM2_Result.ReadingNum;
		 
		 static bool LastbMode;
		 static int LastnMode;
		 if(!bMode)
		 {
		 SetServo(SERVO_ELEVATOR,Scaled_DSM2_Result.fElevator);
		 SetServo(SERVO_ALIERON ,Scaled_DSM2_Result.fAlieron);
		 SetServo(SERVO_RUDDER,Scaled_DSM2_Result.fRudder);
		 SetServo(SERVO_THROTTLE,Scaled_DSM2_Result.fThrottle*(1/0.65));
		 }
		 else
		 {static float TargetHead;
		  static float TargetAlt;
		  if(!LastbMode)
		  {
			TargetHead=CurGPSHead_deg;
			TargetAlt=CurGPSAlt_m;
			PitchZeroElevator=Scaled_DSM2_Result.fElevator;
		  }
		  else
		  {
		   if((nMode==2) && (LastnMode==1)) TargetHead+=90.0;
		   if((nMode==0) && (LastnMode==1)) TargetHead-=90.0;
		   if(TargetHead>180.0) TargetHead-=360.0;
		   if(TargetHead<-180) TargetHead+=360.0;
		   }

		  SetServo(SERVO_ALIERON, AutoAlieron(TargetHead));
          //SetServo(SERVO_ALIERON ,Scaled_DSM2_Result.fAlieron);
		  
		  
		  SetServo(SERVO_ELEVATOR,Scaled_DSM2_Result.fElevator);
		  //SetServo(SERVO_ELEVATOR,AutoElevator(TargetAlt);
		  
		  SetServo(SERVO_RUDDER,AutoRudder());
		  //SetServo(SERVO_RUDDER,Scaled_DSM2_Result.fRudder);
		  
		  
		  SetServo(SERVO_THROTTLE,Scaled_DSM2_Result.fThrottle*(1/0.65));
		 }
		 LastbMode=bMode;
		 LastnMode=nMode;

		// if(LastXbeeSwitch!=Xbee_Result.switches) NewSwitchState(Xbee_Result.switches,LastXbeeSwitch);
		// LastXbeeSwitch=Xbee_Result.switches;
		 LastRcFrameTime=TimeTick;
	 }
	 else
	 {
	   if((LastRcFrameTime+20) < TimeTick)
		{
	     SetServo(SERVO_ELEVATOR,0.0);
	     SetServo(SERVO_ALIERON,0.0);
	     SetServo(SERVO_RUDDER,0.0);
	     SetServo(SERVO_THROTTLE,-1.0);
		 iprintf("Fail Safe\r\n");
		 LastRcFrameTime=TimeTick;

		}

	 }
	 LogService();

	 if(LastGPSFrame!=GPS_Result.ReadingNum)
	 {
		 LastGPSFrame=GPS_Result.ReadingNum;
		 CurGPSHead_deg=(float)GPS_Result.Heading*1.0E-5;
		 CurGPSAlt_m   =(float)GPS_Result.HMSL * 0.001; //mm to m
		 CurGPSVV_mps =(float)GPS_Result.VEL_D* 0.01; //cm to m 
		 CurGPSGS_mps =(float)GPS_Result.Speed*0.01; //cm to m
	     if(bLog) LogGps(*((GPS_READING *)&GPS_Result));
	   

	 }
	 LogService();
	 
	  if(LastStatusUpTime!=Secs)
        { 
		  //int delta=Xbee_Result.ReadingNum-LastXbeeRx;
		  // siprintf((char*)Screen[0],"G:%02d e:%3d l:%d",GPS_Result.numSV,delta,XbeeFrameLoss);
		  // siprintf((char *)Screen[1],"LOG %08X",LogPagesWritten);
		   LastStatusUpTime=Secs;
		   if(gSlew>1.0) gSlew-=1.0;
		   printf("Log=%dMode=%d nMode=%d r%6g p%6g y%6g lt%g rt%g\r\n",
				  bLog,
				  bMode,
				  nMode,
				  imu_state.roll*57.0,
				  imu_state.pitch*57.0,
				  imu_state.yaw*57.0,
				  Scaled_DSM2_Result.ltrim,
				  Scaled_DSM2_Result.rtrim
				  );
		}


	 //LogService();
}


 }
 
