void InitGpsSubSystem(WORD prio,OS_SEM * pDataSem);

struct GPS_READING
{
DWORD       itow;           // ms GPS Millisecond Time of Week
int         ECEF_X;         // cm ECEF X coordinate
int         ECEF_Y;         // cm ECEF Y coordinate
int         ECEF_Z;         // cm ECEF Z coordinate
int         PAcc;           // cm 3D Position Accuracy Estimate
int         ECEFVX;         // cm/s ECEF X velocity
int         ECEFVY;         // cm/s ECEF Y velocity
int         ECEFVZ;         // cm/s ECEF Z velocity
int         LON;            // 1e-07 deg Longitude
int         LAT;            // 1e-07 deg Latitude
int         HEIGHT;         // mm Height above Ellipsoid
int         HMSL;           // mm Height above mean sea level
int         VEL_N;          // cm/s  NED north velocity
int         VEL_E;          // cm/s  NED east velocity
int         VEL_D;          // cm/s  NED down velocity
int         Speed;          // cm/s  Speed (3-D)
int         GSpeed;         // cm/s  Ground Speed (2-D)
int         Heading;        // 1e-05 deg  Heading 2-D
DWORD        SAcc;           // cm/s  Speed Accuracy Estimate
DWORD        CAcc;           // deg  Course / Heading Accuracy Estimate
DWORD        Hacc;           // mm Horizontal Accuracy Estimate
DWORD        Vacc;           // mm Vertical Accuracy Estimate
BYTE         numSV;          // Number of SVs used in navigation solution
BYTE         GPSfix;         // GPSfix Type, range 0..6
WORD         week;           // GPS week
WORD         ReadingNum;  //Serial number of GPS reading
}; 

extern volatile GPS_READING GPS_Result;








