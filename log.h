void LogDump();
void LogErase();
void LogInit();
void LogMessage(const char * msg);
void LogService();


struct sensor_saved_config ; //forward

struct StateRec
{
short Ax;
short Ay;
short Az;
short Gx;
short Gy;
short Gz;
short Mag_X;
short Mag_Z;
short Mag_Y;
float roll;
float pitch;
float yaw;
DWORD dwPressure;    
int pitot;
};


void LogState(StateRec & item);
void LogImu  (IMU_READING & item);
void LogMag  (COMPASS_READING & item);
void LogZeros(IMU_ZEROS & item);
void LogAlt  (ALTIMETER_READING & item);
//void LogXbee (RAW_XBEE_READING &item);
void LogRC(DSM2_READING & item);
void LogGps(GPS_READING  & gps);
void LogConfig(sensor_saved_config &sc);

extern volatile DWORD LogPagesWritten;
extern DWORD LogErrors; 
