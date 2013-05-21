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
#include "i2c_sub.h"
#include "spi_sub.h"
//#include "xbee_sub.h"
#include "DSM2_sub.h"
#include "gps.h"
#include "log.h"
#include "sensor_config.h"
#include "filereporter.h" 
REPORT_FILE;

/*int ReadMemory(PBYTE dest, DWORD addr, int len);
void ChipErase();
void WriteMemoryBlock(PWORD from, DWORD addr);
bool WriteOrEraseDone();
*/


volatile DWORD LogPagesWritten;
																
typedef enum tNames
{
 tu8=1,
 ti8=2,
 tu16=3,
 ti16=4,
 tu32=5,
 ti32=6,
 tpu8=7,
 tpi8=8,
 tf32=9,
 td64=10
};


static BYTE LogBuf[512];
static int wPtr;
static int NextBufPtr;
static WORD ReadBuf[128];

DWORD LogErrors;

//Log Descriptor Type...
#define LOG_TYPE_STRUCT (1)
#define LOG_TYPE_ITEM (2)

//Log message types   
#define MSG_TYPE  (0x11)
#define IMU_TYPE  (0x12)
#define ZERO_TYPE (0x13)
#define MAG_TYPE  (0x14)
#define ALT_TYPE  (0x15)
#define XBE_TYPE  (0x16)
#define GPS_TYPE  (0x17)
#define CFG_TYPE  (0x18)
#define RC_TYPE   (0x19)
#define CSTATE_TYPE (0x20)

OS_CRIT LogShiftCrit;

#define LOG_REC_START  (0xEB)
#define LOG_REC_END    (0xEE)
#define LOG_REC_ESC  (0xEC)
#define LOG_REC_TICK (0xED)

static BYTE nSputs;

void SPutRawByte(BYTE b)
{
iprintf("%02X",b);
nSputs++;
if((nSputs&1)==0) iprintf(", ");
if((nSputs & 0x0F)==0) iprintf("\r\n");
}



void PutRawByte(BYTE b)
{
 LogBuf[wPtr++]=b;
 if((wPtr>256))
  {
   LogService();
  }
}


void PutEscapedByte(BYTE b)
{
if(b==LOG_REC_START) {PutRawByte(LOG_REC_ESC); PutRawByte(0); }
else
if(b==LOG_REC_ESC) {PutRawByte(LOG_REC_ESC); PutRawByte(1); }
else
if(b==LOG_REC_END) {PutRawByte(LOG_REC_ESC); PutRawByte(2); }
else
if(b==LOG_REC_TICK) {PutRawByte(LOG_REC_ESC); PutRawByte(3); }
else
PutRawByte(b);
}


void SPutEscapedByte(BYTE b)
{
if(b==LOG_REC_START) {SPutRawByte(LOG_REC_ESC); SPutRawByte(0); }
else
if(b==LOG_REC_ESC) {SPutRawByte(LOG_REC_ESC); SPutRawByte(1); }
else
if(b==LOG_REC_END) {SPutRawByte(LOG_REC_ESC); SPutRawByte(2); }
else
if(b==LOG_REC_TICK) {SPutRawByte(LOG_REC_ESC); SPutRawByte(3); }
else
SPutRawByte(b);
}




inline void StartLogRecord() { PutRawByte(LOG_REC_START); }
inline void EndLogRecord()   { PutRawByte(LOG_REC_END); }

void LogRawRecord(BYTE msg, const unsigned char * data, int len)
{
StartLogRecord();
PutRawByte(msg);
for(int i=0; i<len; i++)
{
 PutEscapedByte(((PBYTE)data)[i]);
}
EndLogRecord();
}






void StartItemIntro(BYTE id, int siz, const char * name)
{
SPutRawByte(LOG_REC_START);
SPutRawByte(LOG_TYPE_STRUCT);
SPutRawByte(id);
SPutEscapedByte((BYTE)(siz>>8)&0xFF);
SPutEscapedByte((BYTE)(siz&0xFF));
	while(*name) SPutEscapedByte(*name++);
SPutRawByte(LOG_REC_END); 

}


//Log the base element of a structure */
void ShowBaseElement(BYTE id,PBYTE p1, PBYTE p2,int siz, enum  tNames tn, const char * el_name)
{
//iprintf("Offset : %d  size :%d  type %d  name %s\r\n",(p2-p1),siz,tn ,el_name);
int offset=(p2-p1);
SPutRawByte(LOG_REC_START); 
SPutRawByte(LOG_TYPE_ITEM );
SPutEscapedByte((BYTE)id);
SPutEscapedByte((BYTE)tn);
SPutEscapedByte((BYTE)(siz>>8)&0xFF);
SPutEscapedByte((BYTE)(siz&0xFF));
SPutEscapedByte((BYTE)(offset>>8)&0xFF);
SPutEscapedByte((BYTE)(offset&0xFF));
 	while(*el_name) SPutEscapedByte(*el_name++);
SPutRawByte(LOG_REC_END); 
}


//Generic message class...
void LogMessage(const char * msg)
{
	LogRawRecord(MSG_TYPE,( const unsigned char * )msg,strlen(msg));
}







inline static void ShowElement(BYTE id,void * vp,void * vp2, short el,const char * name)  
{
ShowBaseElement(id,(PBYTE)vp,(PBYTE)vp2,sizeof(el),ti16,name);
}


inline static void ShowElement(BYTE id,void * vp,void * vp2, int el,const char * name)
{

ShowBaseElement(id,(PBYTE)vp,(PBYTE)vp2,sizeof(el),ti32,name);
}
										

inline static void ShowElement(BYTE id,void * vp,void * vp2, long el,const char * name)
{

ShowBaseElement(id,(PBYTE)vp,(PBYTE)vp2,sizeof(el),ti32,name);
}
 


inline static void ShowElement(BYTE id,void * vp,void * vp2, unsigned short el,const char * name)
{
ShowBaseElement(id,(PBYTE)vp,(PBYTE)vp2,sizeof(el),tu16,name);
}


inline static void ShowElement(BYTE id,void * vp,void * vp2, unsigned long  el,const char * name)
{
ShowBaseElement(id,(PBYTE)vp,(PBYTE)vp2,sizeof(el),tu32,name);

}



inline static void ShowElement(BYTE id,void * vp,void * vp2, char el,const char * name)
{
ShowBaseElement(id,(PBYTE)vp,(PBYTE)vp2,sizeof(el),ti8,name);
}


inline static void ShowElement(BYTE id,void * vp,void * vp2, unsigned char el,const char * name)
{
ShowBaseElement(id,(PBYTE)vp,(PBYTE)vp2,sizeof(el),tu8,name);

}

inline static void ShowElement(BYTE id,void * vp,void * vp2, float el,const char * name)
{
ShowBaseElement(id,(PBYTE)vp,(PBYTE)vp2,sizeof(el),tf32,name);

}

/*
inline static void ShowElement(BYTE id,void * vp,void * vp2, char * el,const char * name)
{
ShowBaseElement(id,(PBYTE)vp,(PBYTE)vp2,sizeof(el),tpi8,name);
}


inline static void ShowElement(BYTE id,void * vp,void * vp2, unsigned char * el,const char * name)
{
ShowBaseElement(id,(PBYTE)vp,(PBYTE)vp2,sizeof(el),tpu8,name);
}

*/





void DumpRecords();

void LogDump()
{int i=0;
int n=0;
int cZero;
bool bZero;
cZero=0;

DumpRecords();
while(1)
{
ReadMemory((PBYTE)ReadBuf,i,256);
bZero=true;
for(n=0; n<128; n++) if(ReadBuf[n]!=0xFFFF) bZero=false;
if(bZero)
 {
   cZero++;
   if(cZero>10)
	   { iprintf("\r\nDone \r\n");
	     return;
       }
 }
for(n=0; n<128; n++)
 {
  iprintf("%04X ,",ReadBuf[n]);
  if((n%8)==7) iprintf("\r\n");
 }
i+=256;
}
}


void LogErase()
{
int i=80;
iprintf("Eraseing log\r\n");
ChipErase();
while(1)
{
OSTimeDly(20);
iprintf("%d\r\n",i--);
if(WriteOrEraseDone())
{
	iprintf("Erase done\r\n");
	NextBufPtr=0;
	return;
}
}
iprintf("Finish error\r\n");
}


void LogInit()
{int i=0;
int n=0;
bool bZero;
OSCritInit(&LogShiftCrit);

while(1)
{
ReadMemory((PBYTE)ReadBuf,i,256);
bZero=true;
for(n=0; n<128; n++) if(ReadBuf[n]!=0xFFFF) bZero=false;
if(bZero)
 {
	NextBufPtr=i;
	iprintf("Blank at loc %d\r\n",i);
	return;
 }
i+=256;
}

}			  


/* Individual log classes... */

#define LogStart(id ,name) BYTE tid=id;   StartItemIntro(id,sizeof(item),name); 
#define LogElement(x,y) ShowElement(tid,&item,&item.x,item.x,y)










void LogImu( IMU_READING & item)
{
 LogRawRecord(IMU_TYPE,(const unsigned char *)&item,sizeof(item)); 
}




void LogMag( COMPASS_READING & item)
{
LogRawRecord(MAG_TYPE,(const unsigned char *) &item,sizeof(item));
}


void LogZeros( IMU_ZEROS & item)
{
LogRawRecord(ZERO_TYPE,(const unsigned char *)&item,sizeof(item));
}

void LogAlt( ALTIMETER_READING & item)
{
LogRawRecord(ALT_TYPE,(const unsigned char *)&item,sizeof(item));
}
 

/*void LogXbee( RAW_XBEE_READING &item)
{
LogRawRecord(XBE_TYPE,(const unsigned char *)&item,sizeof(item));
}
*/

void LogRC(DSM2_READING & item)
{
LogRawRecord(RC_TYPE,(const unsigned char *)&item,sizeof(item));
}



void LogGps(GPS_READING  &item)
{
	LogRawRecord(GPS_TYPE,(const unsigned char *)&item,sizeof(item));
}


void LogConfig(sensor_saved_config &item)
{
	LogRawRecord(CFG_TYPE,(const unsigned char *)&item,sizeof(item));

}


void LogState(StateRec & item)
{
 LogRawRecord(CSTATE_TYPE ,(const unsigned char *)&item,sizeof(item));
}


void LogService()
{

if(wPtr>256)
 {
	WriteMemoryBlock((PWORD)LogBuf,NextBufPtr);
	LogPagesWritten++;
    OSCritEnter( &LogShiftCrit,0);

	for(int i=256; i<wPtr; i++)LogBuf[i-256]=LogBuf[i];
	wPtr-=256;
	NextBufPtr+=256;
	OSCritLeave(&LogShiftCrit);

 }
}




void ShowConfig(sensor_saved_config &item)
{
	LogStart(CFG_TYPE,"Config");
	LogElement(mag_max[0]             ,"mag_max[0]");
    LogElement(mag_max[1]             ,"mag_max[1]");
	LogElement(mag_max[2]             ,"mag_max[2]");
	LogElement(mag_min[0]             ,"mag_min[0]");
	LogElement(mag_min[1]             ,"mag_min[1]");
 	LogElement(mag_min[2]             ,"mag_min[2]");
	LogElement(accel_zero[0]          ,"accel_zero[0]");
	LogElement(accel_zero[1]          ,"accel_zero[1]");
 	LogElement(accel_zero[2]          ,"accel_zero[2]");
	LogElement(default_gyro_zero[0]   ,"default_gyro_zero[0]");
    LogElement(default_gyro_zero[1]   ,"default_gyro_zero[1]");
	LogElement(default_gyro_zero[2]   ,"default_gyro_zero[2]");
	LogElement(rx_rc_zeros_el         ,"rx_rc_zeros_el");
	LogElement(rx_rc_gain_el          ,"rx_rc_gain_el" );      
	LogElement(rx_rc_zeros_al         ,"rx_rc_zeros_al");      
	LogElement(rx_rc_gain_al          ,"rx_rc_gain_al" );      
	LogElement(rx_rc_zeros_rd         ,"rx_rc_zeros_rd");      
	LogElement(rx_rc_gain_rd          ,"rx_rc_gain_rd" );      
	LogElement(rx_rc_zeros_th         ,"rx_rc_zeros_th");      
	LogElement(rx_rc_gain_th          ,"rx_rc_gain_th" );      
	LogElement(servo_neg_lim[0]       ,"servo_neg_lim[0]");    
	LogElement(servo_neg_lim[1]       ,"servo_neg_lim[1]");    
 	LogElement(servo_neg_lim[2]       ,"servo_neg_lim[2]");    
 	LogElement(servo_neg_lim[3]       ,"servo_neg_lim[3]");    
 	LogElement(servo_neg_lim[4]       ,"servo_neg_lim[4]");    
 	LogElement(servo_neg_lim[5]       ,"servo_neg_lim[5]");    
 	LogElement(servo_neg_lim[6]       ,"servo_neg_lim[6]");    
	LogElement(servo_neg_lim[7]       ,"servo_neg_lim[7]");    
	LogElement(servo_pos_lim[0]       ,"servo_pos_lim[0]");    
	LogElement(servo_pos_lim[1]       ,"servo_pos_lim[1]");    
	LogElement(servo_pos_lim[2]       ,"servo_pos_lim[2]");    
	LogElement(servo_pos_lim[3]       ,"servo_pos_lim[3]");    
	LogElement(servo_pos_lim[4]       ,"servo_pos_lim[4]");    
	LogElement(servo_pos_lim[5]       ,"servo_pos_lim[5]");    
	LogElement(servo_pos_lim[6]       ,"servo_pos_lim[6]");    
	LogElement(servo_pos_lim[7]       ,"servo_pos_lim[7]");    
	LogElement(servo_mid[0]           ,"servo_mid[0]");        
	LogElement(servo_mid[1]           ,"servo_mid[1]");        
	LogElement(servo_mid[2]           ,"servo_mid[2]");        
	LogElement(servo_mid[3]           ,"servo_mid[3]");        
	LogElement(servo_mid[4]           ,"servo_mid[4]");        
	LogElement(servo_mid[5]           ,"servo_mid[5]");        
	LogElement(servo_mid[6]           ,"servo_mid[6]");        
	LogElement(servo_mid[7]           ,"servo_mid[7]");        
}


/*
void ShowImuRec(IMU_READING & item)
{
	LogStart(IMU_TYPE,"IMU Record");
	LogElement(Ax,"Ax");
	LogElement(Ay,"Ay");
	LogElement(Az,"Az");
	LogElement(Gx,"Gx");
	LogElement(Gy,"Gy");
	LogElement(Gz,"Gz");
	LogElement(T,"T");
	LogElement(ReadingNum,"RN");
}
 */

/*
void ShowMag( COMPASS_READING & item)
{
	LogStart(MAG_TYPE,"Magnatometer");
	LogElement(Mag_X,"Mx");
	LogElement(Mag_Z,"Mz");   
	LogElement(Mag_Y,"My");    
	LogElement(ReadingNum,"RN");
}

  */
void ShowZeros( IMU_ZEROS & item)
{
 	LogStart(ZERO_TYPE,"IMU Zeros");
	LogElement(Gzerosx, "Gzx");
	LogElement(Gzerosy, "Gzy"); 
	LogElement(Gzerosz, "Gzz");
	LogElement(Azerosx, "Azx"); 
	LogElement(Azerosy, "Azy"); 
	LogElement(Azerosz, "Azz"); 
}


/*
void ShowAlt( ALTIMETER_READING & item)
{
	LogStart(ALT_TYPE,"Altimeter");
	LogElement(dwPressure,  "Baro");
	LogElement(dwTemperature,"Temp");
    LogElement(pitot,"Pitot");
}
  */
/*void ShowXbee( RAW_XBEE_READING &item)
{
	LogStart(XBE_TYPE,"Xbee Frame");
	LogElement(nElevator,"Elev");
	LogElement(nAlieron,"Alie");
	LogElement(nRudder,"Rud");
	LogElement(nThrottle," Thro");
	LogElement(nN1,"N1");
	LogElement(nN2,"N2");
	LogElement(nN3,"N3");
	LogElement(nN4,"N4");
    LogElement(switches,"switches");
	LogElement(ReadingNum,"RN");
}
*/




void ShowRC( DSM2_READING &item)
{
	LogStart(RC_TYPE,"DSM2 Frame");
	LogElement(val[0],"V[0]");
	LogElement(val[1],"V[1]");
 	LogElement(val[2],"V[2]");
 	LogElement(val[3],"V[3]");
 	LogElement(val[4],"V[4]");
 	LogElement(val[5],"V[5]");
 	LogElement(val[6],"V[6]");
 	LogElement(val[7],"V[7]");
	LogElement(ReadingNum,"RN");
}


void ShowState(StateRec & item)
{
LogStart(CSTATE_TYPE,"State");
LogElement(Ax,"Ax");
LogElement(Ay,"Ay");
LogElement(Az,"Az");
LogElement(Gx,"Gx");
LogElement(Gy,"Gy");
LogElement(Gz,"Gz");
LogElement(Mag_X,"Mx");
LogElement(Mag_Z,"My");
LogElement(Mag_Y,"Mz");
LogElement(roll,"Roll");
LogElement(pitch,"Pitch");
LogElement(yaw,"Yaw");
LogElement(dwPressure,"Press");    
LogElement(pitot,"AS");

}




void ShowGps(GPS_READING  &item)
{
  LogStart(GPS_TYPE,"GPS Frame");
  LogElement(  itow   , "itow");         // ms GPS Millisecond Time of Week            
  LogElement(ECEF_X , "ECEF_X");       // cm ECEF X coordinate                       
  LogElement(ECEF_Y , "ECEF_Y");       // cm ECEF Y coordinate                       
  LogElement(ECEF_Z , "ECEF_Z");       // cm ECEF Z coordinate                       
  LogElement(PAcc   , "PAcc");         // cm 3D Position Accuracy Estimate           
  LogElement(ECEFVX , "ECEFVX");       // cm/s ECEF X velocity                       
  LogElement(ECEFVY , "ECEFVY");       // cm/s ECEF Y velocity                       
  LogElement(ECEFVZ , "ECEFVZ");       // cm/s ECEF Z velocity                       
  LogElement(LON    , "LON");          // 1e-07 deg Longitude                        
  LogElement(LAT    , "LAT");          // 1e-07 deg Latitude                         
  LogElement(HEIGHT , "HEIGHT");       // mm Height above Ellipsoid                  
  LogElement(HMSL   , "HMSL");         // mm Height above mean sea level             
  LogElement(VEL_N  , "VEL_N");        // cm/s  NED north velocity                   
  LogElement(VEL_E  , "VEL_E");        // cm/s  NED east velocity                    
  LogElement(VEL_D  , "VEL_D");        // cm/s  NED down velocity                    
  LogElement(Speed  , "Speed");        // cm/s  Speed (3-D)                          
  LogElement(GSpeed , "GSpeed");       // cm/s  Ground Speed (2-D)                   
  LogElement(Heading, "Heading");       // 1e-05 deg  Heading 2-D                     
  LogElement(SAcc   , "SAcc");         // cm/s  Speed Accuracy Estimate             
  LogElement(CAcc   , "CAcc");         // deg  Course / Heading Accuracy Estimate   
  LogElement(Hacc   , "Hacc");         // mm Horizontal Accuracy Estimate           
  LogElement(Vacc   , "Vacc");         // mm Vertical Accuracy Estimate             
  LogElement(numSV  , "numSV");        // Number of SVs used in navigation solution 
  LogElement(GPSfix , "GPSfix");       // GPSfix Type, range 0..6                   
  LogElement(week   , "week");         // GPS week                                  
  LogElement(ReadingNum,"RN");
}




void ShowFileVer(const char * cp)
{
SPutRawByte(LOG_REC_START); 
SPutRawByte(MSG_TYPE );
 	while(*cp) SPutEscapedByte(*cp++);
SPutRawByte(LOG_REC_END); 
}

void DumpRecords()
{
BYTE item[16];
nSputs=0;
ShowConfig((*((sensor_saved_config* )&item))) ;
//ShowImuRec((*((IMU_READING * )& item))) ;
//ShowMag((*(( COMPASS_READING* ) & item)));
ShowZeros((*(( IMU_ZEROS * )& item)));
//ShowAlt((*(( ALTIMETER_READING * )& item)));
//ShowXbee((*(( RAW_XBEE_READING * )&item)));

ShowRC((*((DSM2_READING *)&item)));
ShowGps((*((GPS_READING  * )&item)));
ShowState((*((StateRec*)&item)));
FileReporter::DumpList();


if(nSputs & 1) SPutRawByte(LOG_REC_END); 
}




FileReporter* FileReporter::pRoot;

void FileReporter::DumpList()
{
 FileReporter * pF =FileReporter::pRoot;
 while(pF)
 {
	 ShowFileVer(pF->m_pName);
	 pF=pF->m_pNext;
 }

}

void FileReporter::ShowList()
{
 FileReporter * pF =FileReporter::pRoot;
 while(pF)
 {
	 iprintf("%s\r\n",pF->m_pName);
	 pF=pF->m_pNext;
 }

}


