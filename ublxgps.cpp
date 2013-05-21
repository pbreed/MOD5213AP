#include "predef.h" 
#include <basictypes.h>
#include <constants.h>
#include <ucos.h>
#include <ucosmcfc.h>
#include <serialirq.h>
#include <math.h>
#include "gps.h"
#include "filereporter.h" 
REPORT_FILE;



volatile GPS_READING GPS_Result;



static OS_SEM * pDataSem;


#define INVALID 0
#define NEWDATA 1
#define DATAUSED 2

// message sync bytes
#define UBX_SYNC1_CHAR  0xB5
#define UBX_SYNC2_CHAR  0x62
// protocoll identifier
#define UBX_CLASS_NAV   0x01
// message id
#define UBX_ID_POSLLH   0x02
#define UBX_ID_SOL              0x06
#define UBX_ID_VELNED   0x12

// ------------------------------------------------------------------------------------------------
// typedefs
#define uint32_t DWORD
#define int32_t int
#define int16_t short
#define uint8_t BYTE
#define uint16_t WORD



// ubx parser state
typedef enum
{
        UBXSTATE_IDLE,
        UBXSTATE_SYNC1,
        UBXSTATE_SYNC2,
        UBXSTATE_CLASS,
        UBXSTATE_LEN1,
        UBXSTATE_LEN2,
        UBXSTATE_DATA,
        UBXSTATE_CKA,
        UBXSTATE_CKB
} ubxState_t;


#define uint32_t DWORD
#define int32_t int
#define int16_t short
#define uint8_t BYTE
#define uint16_t WORD


DWORD ConLittleEnd(DWORD dw) 
{
volatile DWORD rdw;
PBYTE pb= (PBYTE)&rdw;
PBYTE pbf=(PBYTE)&dw;
pb[0]=pbf[3];
pb[1]=pbf[2];
pb[2]=pbf[1];
pb[3]=pbf[0];
return rdw;
}


int ConLittleEnd(int i) 
{
volatile int ri;
PBYTE pb= (PBYTE)&ri;
PBYTE pbf=(PBYTE)&i;
pb[0]=pbf[3];
pb[1]=pbf[2];
pb[2]=pbf[1];
pb[3]=pbf[0];
return ri;
}


short ConLittleEnd(short s )
{
volatile short rs;
PBYTE pb= (PBYTE)&rs;
PBYTE pbf=(PBYTE)&s;
pb[0]=pbf[1];
pb[1]=pbf[0];
return rs;
}


WORD ConLittleEnd(WORD w)
{
volatile WORD rw;
PBYTE pb= (PBYTE)&rw;
PBYTE pbf=(PBYTE)&w;
pb[0]=pbf[1];
pb[1]=pbf[0];
return rw;
}







typedef struct
{
        uint32_t        itow;           // ms GPS Millisecond Time of Week
        int32_t         frac;           // ns remainder of rounded ms above
        int16_t         week;           // GPS week
        uint8_t         GPSfix;         // GPSfix Type, range 0..6
        uint8_t         Flags;          // Navigation Status Flags
        int32_t         ECEF_X;         // cm ECEF X coordinate
        int32_t         ECEF_Y;         // cm ECEF Y coordinate
        int32_t         ECEF_Z;         // cm ECEF Z coordinate
        int32_t         PAcc;           // cm 3D Position Accuracy Estimate
        int32_t         ECEFVX;         // cm/s ECEF X velocity
        int32_t         ECEFVY;         // cm/s ECEF Y velocity
        int32_t         ECEFVZ;         // cm/s ECEF Z velocity
        uint32_t        SAcc;           // cm/s Speed Accuracy Estimate
        uint16_t        PDOP;           // 0.01 Position DOP
        uint8_t         res1;           // reserved
        uint8_t         numSV;          // Number of SVs used in navigation solution
        uint32_t        res2;           // reserved
        uint8_t         Status;     // invalid/newdata/processed
} __attribute__((packed)) ubx_nav_sol_t;


typedef struct
{
        uint32_t        itow;           // ms  GPS Millisecond Time of Week
        int32_t         VEL_N;          // cm/s  NED north velocity
        int32_t         VEL_E;          // cm/s  NED east velocity
        int32_t         VEL_D;          // cm/s  NED down velocity
        int32_t         Speed;          // cm/s  Speed (3-D)
        int32_t         GSpeed;         // cm/s  Ground Speed (2-D)
        int32_t         Heading;        // 1e-05 deg  Heading 2-D
        uint32_t        SAcc;           // cm/s  Speed Accuracy Estimate
        uint32_t        CAcc;           // deg  Course / Heading Accuracy Estimate
        uint8_t         Status;         // invalid/newdata/processed
} __attribute__((packed)) ubx_nav_velned_t;

typedef struct
{
        uint32_t        itow;           // ms GPS Millisecond Time of Week
        int32_t         LON;            // 1e-07 deg Longitude
        int32_t         LAT;            // 1e-07 deg Latitude
        int32_t         HEIGHT;         // mm Height above Ellipsoid
        int32_t         HMSL;           // mm Height above mean sea level
        uint32_t        Hacc;           // mm Horizontal Accuracy Estimate
        uint32_t        Vacc;           // mm Vertical Accuracy Estimate
        uint8_t         Status;         // invalid/newdata/processed
} __attribute__((packed)) ubx_nav_posllh_t;



//------------------------------------------------------------------------------------
// global variables

// local buffers for the incomming ubx messages
volatile ubx_nav_sol_t          UbxSol    = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, INVALID};
volatile ubx_nav_posllh_t       UbxPosLlh = {0,0,0,0,0,0,0, INVALID};
volatile ubx_nav_velned_t       UbxVelNed = {0,0,0,0,0,0,0,0,0, INVALID};
volatile BYTE PacketType;

uint16_t CheckGPSOkay = 0;




        static ubxState_t ubxState = UBXSTATE_IDLE;


/********************************************************/
/*                   UBX Parser                         */
/********************************************************/
void UBX_Parser(uint8_t c)
{
        static uint16_t msglen;
        static uint8_t cka, ckb;
        static uint8_t *ubxP, *ubxEp, *ubxSp; // pointers to data currently transfered
		 //iprintf(".");
        //state machine
        switch (ubxState)       // ubx message parser
        {
                case UBXSTATE_IDLE: // check 1st sync byte
                        if (c == UBX_SYNC1_CHAR) ubxState = UBXSTATE_SYNC1;
                        else ubxState = UBXSTATE_IDLE; // out of synchronization
                        break;

                case UBXSTATE_SYNC1: // check 2nd sync byte
                        if (c == UBX_SYNC2_CHAR) ubxState = UBXSTATE_SYNC2;
                        else ubxState = UBXSTATE_IDLE; // out of synchronization
                        break;

                case UBXSTATE_SYNC2: // check msg class to be NAV
                        if (c == UBX_CLASS_NAV) ubxState = UBXSTATE_CLASS;
                        else ubxState = UBXSTATE_IDLE; // unsupported message class
                        break;

				case UBXSTATE_CLASS: // check message identifier
					   PacketType=c;
                        switch(c)
                        {
                                case UBX_ID_POSLLH: // geodetic position
                                        ubxP =  (uint8_t *)&UbxPosLlh; // data start pointer
                                        ubxEp = (uint8_t *)(&UbxPosLlh + 1); // data end pointer
                                        ubxSp = (uint8_t *)&UbxPosLlh.Status; // status pointer
									
                                        break;

                                case UBX_ID_SOL: // navigation solution
                                        ubxP =  (uint8_t *)&UbxSol; // data start pointer
                                        ubxEp = (uint8_t *)(&UbxSol + 1); // data end pointer
                                        ubxSp = (uint8_t *)&UbxSol.Status; // status pointer
                                        break;

                                case UBX_ID_VELNED: // velocity vector in tangent plane
                                        ubxP =  (uint8_t *)&UbxVelNed; // data start pointer
                                        ubxEp = (uint8_t *)(&UbxVelNed + 1); // data end pointer
                                        ubxSp = (uint8_t *)&UbxVelNed.Status; // status pointer
                                        break;

                                default:                        // unsupported identifier
                                        ubxState = UBXSTATE_IDLE;
                                        break;
                        }
                        if (ubxState != UBXSTATE_IDLE)
                        {
                                ubxState = UBXSTATE_LEN1;
                                cka = UBX_CLASS_NAV + c;
                                ckb = UBX_CLASS_NAV + cka;
                        }
                        break;

                case UBXSTATE_LEN1: // 1st message length byte
                        msglen = (uint16_t)c; // lowbyte first
                        cka += c;
                        ckb += cka;
                        ubxState = UBXSTATE_LEN2;
                        break;

                case UBXSTATE_LEN2: // 2nd message length byte
                        msglen += ((uint16_t)c)<<8; // high byte last
                        cka += c;
                        ckb += cka;
                        // if the old data are not processed so far then break parsing now
                        // to avoid writing new data in ISR during reading by another function
                        if ( *ubxSp == NEWDATA )
                        {
                                ubxState = UBXSTATE_IDLE;
                        }
                        else // data invalid or allready processd
                        {
                                *ubxSp = INVALID; // mark invalid during buffer filling
                                ubxState = UBXSTATE_DATA;
                        }
                        break;

                case UBXSTATE_DATA: // collecting data
                        if (ubxP < ubxEp)
                        {
                                *ubxP++ = c; // copy curent data byte if any space is left
                                cka += c;
                                ckb += cka;
                                if (--msglen == 0)      ubxState = UBXSTATE_CKA; // switch to next state if all data was read
                        }
                        else // rx buffer overrun
                        {
                                ubxState = UBXSTATE_IDLE;
                        }
                        break;

                case UBXSTATE_CKA:
                        if (c == cka) ubxState = UBXSTATE_CKB;
                        else
                        {
                                *ubxSp = INVALID;
                                ubxState = UBXSTATE_IDLE;
                        }
                        break;

                case UBXSTATE_CKB:
                        if (c == ckb)
                        {
                                *ubxSp = NEWDATA; // new data are valid
								if((UbxSol.Status == NEWDATA) && 
								   (UbxPosLlh.Status==NEWDATA) && 
								   (UbxVelNed.Status ==NEWDATA))
								{
									OSLock();
									GPS_Result.itow   =ConLittleEnd(UbxSol.itow   );
									GPS_Result.ECEF_X =ConLittleEnd(UbxSol.ECEF_X );      
									GPS_Result.ECEF_Y =ConLittleEnd(UbxSol.ECEF_Y );      
									GPS_Result.ECEF_Z =ConLittleEnd(UbxSol.ECEF_Z );      
									GPS_Result.PAcc   =ConLittleEnd(UbxSol.PAcc   );       
									GPS_Result.ECEFVX =ConLittleEnd(UbxSol.ECEFVX );      
									GPS_Result.ECEFVY =ConLittleEnd(UbxSol.ECEFVY );      
									GPS_Result.ECEFVZ =ConLittleEnd(UbxSol.ECEFVZ );      
									GPS_Result.LON    =ConLittleEnd(UbxPosLlh.LON    );       
									GPS_Result.LAT    =ConLittleEnd(UbxPosLlh.LAT    );       
									GPS_Result.HEIGHT =ConLittleEnd(UbxPosLlh.HEIGHT );      
									GPS_Result.HMSL   =ConLittleEnd(UbxPosLlh.HMSL   );       
									GPS_Result.VEL_N  =ConLittleEnd(UbxVelNed.VEL_N  );       
									GPS_Result.VEL_E  =ConLittleEnd(UbxVelNed.VEL_E  );       
									GPS_Result.VEL_D  =ConLittleEnd(UbxVelNed.VEL_D  );       
									GPS_Result.Speed  =ConLittleEnd(UbxVelNed.Speed  );       
									GPS_Result.GSpeed =ConLittleEnd(UbxVelNed.GSpeed );      
									GPS_Result.Heading=ConLittleEnd(UbxVelNed.Heading);     
									GPS_Result. SAcc  =ConLittleEnd(UbxVelNed.SAcc  );       
									GPS_Result. CAcc  =ConLittleEnd(UbxVelNed.CAcc  );       
									GPS_Result. Hacc  =ConLittleEnd(UbxPosLlh.Hacc  );       
									GPS_Result. Vacc  =ConLittleEnd(UbxPosLlh.Vacc  );       
									GPS_Result. numSV =UbxSol.numSV;      
									GPS_Result. GPSfix=UbxSol.GPSfix;     
									GPS_Result. week  =ConLittleEnd(UbxSol.week );       
									GPS_Result.ReadingNum++;
									OSUnlock();
								  if(pDataSem) OSSemPost(pDataSem);
                                  UbxSol.Status=DATAUSED;   
								  UbxPosLlh.Status=DATAUSED;    
								  UbxVelNed.Status=DATAUSED;    
								}
                        }
                        else
                        {       // if checksum not match then set data invalid
                                *ubxSp = INVALID;
								iprintf("Got invalid data\r\n");

                        }
                        ubxState = UBXSTATE_IDLE; // ready to parse new data
                        break;

                default: // unknown ubx state
                        ubxState = UBXSTATE_IDLE;
                        break;

        }
}


void GpsTask(void *p)
{

	SimpleUart( 1, 38400 ); //4800,9600,19200,38400,57600
   
    ubxState = UBXSTATE_IDLE; 
           
   while ( 1 )
   {
	  char c=sgetchar(1);
	  UBX_Parser((BYTE)c);
   }
}  


#define SM_TASK_STK (512)
#define SmOSSimpleTaskCreate(x,p) { static DWORD   func_##x_Stk[SM_TASK_STK] __attribute__( ( aligned( 4 ) ) ); OSTaskCreate(x,NULL,(void *)&func_##x_Stk[SM_TASK_STK],(void*)func_##x_Stk,p); }




void InitGpsSubSystem(WORD prio,OS_SEM * pSem)
{

	pDataSem=pSem;
	SmOSSimpleTaskCreate(GpsTask,prio);

}



