void InitI2CSubSystem(WORD prio,OS_SEM * pDataSem);

extern volatile bool bNewCompass;

struct COMPASS_READING
{
short Mag_X;
short Mag_Z;
short Mag_Y;
WORD ReadingNum;
};


struct ALTIMETER_READING
{
 DWORD dwPressure;    
 DWORD dwTemperature; 
 int   pitot;
};

extern volatile COMPASS_READING Compass_Result;
extern volatile ALTIMETER_READING Alt_Result;
