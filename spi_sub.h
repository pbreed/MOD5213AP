void InitISpiSubSystem(WORD prio,OS_SEM * pDataSem);

void TestMemory();
int ReadMemory(PBYTE dest, DWORD addr, int len);
void ChipErase();
void WriteMemoryBlock(PWORD from, DWORD addr);
bool WriteOrEraseDone();



extern volatile DWORD irq2_cnt;
extern volatile bool bNewImu;

struct IMU_READING
{
short Ax;
short Ay;
short Az;
short Gx;
short Gy;
short Gz;
WORD T;
WORD ReadingNum;
};

struct IMU_ZEROS
{
long Gzerosx;
long Gzerosy;
long Gzerosz;
long Azerosx;
long Azerosy;
long Azerosz;
float pitotzero;
};



extern volatile IMU_READING ImuResult;
extern volatile IMU_ZEROS   ImuZeros;


