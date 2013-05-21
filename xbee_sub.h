struct RAW_XBEE_READING
{
short nElevator;
short nAlieron;
short nRudder;
short nThrottle;
WORD  nN1;
WORD  nN2;
WORD  nN3;
WORD  nN4;
WORD  switches;
WORD  ReadingNum;
};


struct SCALE_XBEE_READING
{
float fElevator;
float fAlieron;
float fRudder;
float fThrottle;
float fN1;
float fN2;
float fN3;
float fN4;
};

extern volatile BYTE bLog;
extern volatile RAW_XBEE_READING Xbee_Result;
extern volatile SCALE_XBEE_READING Scaled_Xbee_Result;

extern volatile WORD XbeeFrameLoss;

void InitIXbeeSubSystem(WORD prio,OS_SEM * pDataSem);

void XBeeSend(const char * s);


extern volatile char Screen[2][16];

