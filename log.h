void LogDump();
void LogErase();
void LogInit();
void LogMessage(const char * msg);
void LogService();


struct sensor_saved_config ; //forward

struct Aloop
{
float th;
float brg;
float tr;
float rv;
float ere;
};


struct StateRec
{
float Ax;
float Ay;
float Az;
float Gx;
float Gy;
float Gz;
float Mag_X;
float Mag_Z;
float Mag_Y;
float roll;
float pitch;
float yaw;
DWORD dwPressure;    
int pitot;
};

void LogALoop(Aloop & a);
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
