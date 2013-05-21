struct  VehicleSense
{
float Gx;
float Gy;
float Gz;
float Ax;
float Ay;
float Az;
float Mx;
float My;
float Mz;
};


struct IMU_Euler
{
float roll;
float pitch;
float yaw;
};

void IMU_Init( );

void IMU_Step(VehicleSense & vs, IMU_Euler & result, float SlewScale);
