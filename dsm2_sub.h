struct DSM2_READING
{
/*short nElevator;
short nAlieron;
short nRudder;
short nThrottle;
short nGear;
short nAux2;
*/
unsigned short val[10];//Up to 10 channels 
WORD  ReadingNum;
};


struct SCALE_DSM2_READING
{
float fElevator;
float fAlieron;
float fRudder;
float fThrottle;
float Gear;
float Aux2;
float n1;
float n2;
float rtrim;  //Ch 8 o->1024
float ltrim;  //Ch 9 0 ->512
};



extern volatile char Screen[2][16];


float DSM_Con(unsigned short v);
extern volatile DSM2_READING DSM2_Result;
extern volatile SCALE_DSM2_READING Scaled_DSM2_Result;

void InitIDSM2SubSystem(WORD prio,OS_SEM * pDataSem);
extern volatile bool bLog;
extern volatile bool bMode;
extern volatile int  nMode;

