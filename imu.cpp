////////////////////////////////////////////////////////////////////////////////////////////////////
// "FreeSpace IMU" - By Terence J. Bordelon (www.bordelon.net/tj@bordelon.net)
//  
// This code was removed from my working IMU and stripped down for your use and to help you 
// understand the algorithm presented in the artical. 
//
// A few notes on the coordinate system used:
//
//   Theta = Pitch = (Circle with horizonal line)
//   Phi = Roll = (Circle with vertical line)
//   Psi = Yaw = Y with vertical line down the center
//
// Rotations (point your thumb down +axis, fingers curl to + rotation):
//   About X = Roll = P = positive rotations to the right
//   About Y = Pitch = Q = positive raises the nose
//   About Z = Yaw = R = positive turns to the right
//
// Aircraft coordinate system:
//   X axis is forward, Y is to the right, Z is down.
//
//	Visualizing Quaternions:
//
//	Pretty much axis/angle, specifies displacement from identy (system described above).
//	the quaternion will rotate world space to aircraft local space. If you draw the vector,
//  it will tell you what axis to rotate to get to the aircraft space.
//	
////////////////////////////////////////////////////////////////////////////////////////////////////

//---[Includes]------------------------------------------------------


#include "exmath.h"
#include <math.h>
#include "imu.h"
#include <stdio.h>
#include "filereporter.h" 
REPORT_FILE;


//---[Defines]--------------------------------------------------------


#define DECLINATION_DEGREES (0)
#define DECLINATION_MINUTES	(0)

#define g_rps_scale (0.0314)

#define ERECT_RPS (0.314)
#define NORTH_RPS (0.314)


//---[Static]--------------------------------------------------------

quat 		g_gyroFrame;

vec3		g_world_down, g_world_north;

//---[Functions]------------------------------------------------------


void IMU_SetNorthVector( float declination_degrees, float declination_minutes)
// Set magnetometer "north" vector, set from NOAA's geomag data.
// We flatten this vector to the X-Y plane (no Z!)
// http://www.ngdc.noaa.gov/geomagmodels/struts/igrfWmmZip
{
	quat q;
	vec3 rot;
	float declination = (declination_degrees  + declination_minutes / 60.0f) * (float) RADIANS_PER_DEGREE;

	// Start vector pointing "real" north, along the X axis.
	v3set( &g_world_north, 1.0f, 0.0f, 0.0f);

	// Rotate according to the declination	
	v3set(&rot, 0.0f, 0.0f, 1.0f);	
	Quat_SetAxisAndAngle( &q, &rot, declination);
	Quat_RotateVec3(&q, &g_world_north, &g_world_north);
}


void IMU_Init( )
// Initialize
{
	// Start off with no orientation. 
	//
	// TODO: This is most certainly a bad estimate on powerup, and it will take time to settle.
	// You can create a better estimate by temporarily running with high values passed into IMU_GetErectorQuat for rads_sec
	// and bring them back to normal after a few seconds pass. For simplicity, we omit this step.

	Quat_Set( &g_gyroFrame, 1.0f, 0.0f, 0.0f, 0.0f );	
	v3set(&g_world_down, 0.0f,0.0f, 1.0f);
	
	// TODO: These are set for your location. See the comments in this function.
	IMU_SetNorthVector( DECLINATION_DEGREES, DECLINATION_MINUTES );
}
	

#define NANTest(n,x) if(isnan(x) || isinf(x)) {/*iprintf("%s Nan!",n);*/ return; }


void IMU_IntegrateGyros(float dt, vec3 *rates, quat *pGyroFrame)
// Integrate gyros into our gyro frame. 
{
	// Get rotation axis from gyros, and rate in rads/sec	
   
	if ((rates->x==0.0f) && (rates->y==0.0f) && (rates->z==0.0f)) return;

	vec3 axis;	
	float rate = v3normalize(&axis,rates);

	NANTest("rate",rate);

	// Get a quat for the displacement this timestep did.
	quat q_dot;
	Quat_SetAxisAndAngle( &q_dot, &axis, rate * dt);


	NANTest("z",q_dot.z);
	NANTest("x",q_dot.x);
	NANTest("y",q_dot.y);
	NANTest("z",q_dot.z);
	Quat_Multiply(pGyroFrame, pGyroFrame, &q_dot);
	Quat_Normalize( pGyroFrame);
}


static float lGx,lGy,lGz;
static float lAx,lAy,lAz;
static float lMx,lMy,lMz;
static bool bDoMag;



void IMU_GetAircraftSpaceState( vec3 *imu_north, vec3 *imu_down, vec3 *rates )
// Fill in this function and return sensor values for the IMU algorithm to use:
//		imu_north - Normalized magnetometer vector
//		imu_down  - Normalized accelerometer, with optional centrepetal force correction applied.
//		rates	  - rads/sec gyro rates
//
//+Gy = + pitch (Nose Up)
//+Gx = + roll  (Right wing down)
//+Gz = + yaw   (Nose right)
//
//
//AX=1.0 = -90 pitch
//AX=-1.0 = 90 pitch
//Ay=1.0  =+ 90 Roll
//Ay -1.0	= -90 Roll
//
//Az -1.0 = Inverted
//Az 1.0 = level
//
//
//Mx 0.5, My=0; Mz=-0.86  -> Heading ~ 0
//Mx 0, My =0.5, Mz=-0.86 ->Heading ~ -90
//Mx -0.5, My=0; Mz=-0.86  -> Heading ~ 180
//Mx 0, My =-0.5, Mz=-0.86 ->Heading ~ +90
//
{
	// TODO: You need to read your ADC, apply calibration tables, and return these values.	

	vec3 tmp;
	v3set(rates,lGx,lGy,lGz);
	v3set(&tmp,lAx,lAy,lAz);
	v3normalize(imu_down,&tmp);
	if (
		 (lMy!=0) || 
		 (lMx!=0) || 
		 (lMz!=0)
		)
	 {v3set(&tmp,lMx,lMy,lMz);
	  v3normalize(imu_north,&tmp);
	  bDoMag=true;
	 }
	else
	{
	 bDoMag=false;
	}
}

bool IMU_GetErectorQuat( quat *q_erect_out, quat *frame, vec3 *v_measured, vec3 *v_reference, float rads_sec, float dt)
// Calculates the quaternion needed to rotate the measured vector to the reference vector (world space) at a fixed correction rate
// Returns TRUE if correction is necessary, or FALSE if it is below the threshold of caring.
{
	// Get the rotation axis we'll use to erect.
	// (Normalize returns the length. We do a lower limit. No sense in correcting if we're close)

	vec3 c;
	v3cross(&c,v_measured, v_reference);	
	if( v3normalize(&c,&c) > 1.0f * RADIANS_PER_DEGREE)			
	{
		// Get the angle between the two vectors, and clamp to that limit. We don't want to overshoot.
		// Angles are always positive since the rotation angle flips appropriately.
		float rads =  rads_sec * g_rps_scale * dt;
		
		float a = fabs(clamped_acos(v3dot(v_measured, v_reference)));
		
		if((rads>a)  && (rads>0))
			{
			rads=a;
			}
		else
		if((rads<-a) && (rads<0))
		   {
	        rads=-a;
		   }
		//ULIMIT(rads,a);			
		
		// Get the quat that rotates our sensor toward the world vector by the specified amount
		Quat_SetAxisAndAngle( q_erect_out, &c, rads); 		

		return true;
	}

	return false;
}




//---------------------------------------------------------------------------
// EulerAngles::fromInertialToObjectQuaternion
//
// Setup the Euler angles, given an inertial->object rotation quaternion
//
// See 10.6.6 for more information.

void	fromInertialToObjectQuaternion(quat &qn, float & pitch, float & heading, float & bank ) {

	// Extract sin(pitch)
	quat q;
	q.z=qn.x;
	q.x=qn.y;
	q.y=-qn.z;
	q.w=qn.w;



	float sp = -2.0f * (q.y*q.z + q.w*q.x);

	// Check for Gimbel lock, giving slight tolerance for numerical imprecision

	if (fabs(sp) > 0.9999f) {

		// Looking straight up or down

		pitch =-PI* sp/2.0;

		// Compute heading, slam bank to zero

		heading = atan2(-q.x*q.z - q.w*q.y, 0.5f - q.y*q.y - q.z*q.z);
		bank = 0.0f;

	} else {

		// Compute angles.  We don't have to use the "safe" asin
		// function because we already checked for range errors when
		// checking for Gimbel lock

		pitch	= -asin(sp);
		heading	= atan2(q.x*q.z - q.w*q.y, 0.5f - q.x*q.x - q.y*q.y);
		bank	= -atan2(q.x*q.y - q.w*q.z, 0.5f - q.x*q.x - q.z*q.z);
	}
}




void IMU_Update( float dt ,float slew_scale)
// Main IMU sensor fusion function. Call frequently.
// R_estimate - Estimated vehicle orientation (in/out)
// dt - timestep, in seconds.
{
	quat erector;
	vec3 imu_north, imu_down, rates;

	// Read our sensors and get engineering units...
	IMU_GetAircraftSpaceState( &imu_north, &imu_down, &rates);

	// Euler Integrate our angluar rate gyros into R
	IMU_IntegrateGyros( dt, &rates, &g_gyroFrame);


	// Get the down vector in world space.	
	//IE rotate from measured IMU down to what it would be in world space
	//If the gryo frame was without error.
	
	vec3 v_measured_down_world;
	Quat_RotateVec3( &g_gyroFrame, &v_measured_down_world, &imu_down);
 
	

	// Get erector quaternion from measured down and "reference" down.
	//G World down shoudl probably be adjusted with GPS measured acceleratiosn to give
	//a proper value accouting for accelearations etc...
	if( IMU_GetErectorQuat( &erector, &g_gyroFrame, &v_measured_down_world, &g_world_down , ERECT_RPS*slew_scale, dt) )
	{
		// If a correction is needed, apply the erector rotation to our estimate.
		Quat_Multiply( &g_gyroFrame, &erector, &g_gyroFrame );
	}	
	
	if(bDoMag)
	{
	// Get the north vector in world space, flattened on the X-Y plane
	vec3 v_measured_north_world;
	Quat_RotateVec3( &g_gyroFrame, &v_measured_north_world, &imu_north);
	v_measured_north_world.z = 0.0f;
	v3normalize(&v_measured_north_world,&v_measured_north_world);

	// Get erector quaternion from measured north and "reference" north.
	if( IMU_GetErectorQuat( &erector, &g_gyroFrame, &v_measured_north_world, &g_world_north , NORTH_RPS*slew_scale, dt) )
	 {
		// If a correction is needed, apply the erector rotation to our estimate.
		Quat_Multiply( &g_gyroFrame, &erector, &g_gyroFrame );
	 }	
	}
    
}




		 
void IMU_Step(VehicleSense & vs, IMU_Euler & result, float SlewScale)
{

lGx=vs.Gx;
lGz=vs.Gz;
lGy=vs.Gy;

lAx=vs.Ax;
lAy=vs.Ay;
lAz=vs.Az;

lMx=vs.Mx;
lMy=vs.My;
lMz=vs.Mz;

IMU_Update(0.005,SlewScale);

float  pitch, heading,bank;
fromInertialToObjectQuaternion(g_gyroFrame,pitch, heading, bank );

result.roll=bank;//*DEGREES_PER_RADIAN;
result.pitch=pitch;//*DEGREES_PER_RADIAN;
result.yaw=heading;//*DEGREES_PER_RADIAN;
}


/*
#include <stdio.h>

void fromInertialToObjectQuaternion(quat &q, float & pitch, float & heading, float & bank );

void DoImuStep
int main()
{
IMU_Init();

Gx=0;
Gz=0;
Gx=0;

Gy=(PI/180)*1;
for(int i=0; i<900; i++)
{
IMU_Update(0.005);

float  pitch, heading,bank;
fromInertialToObjectQuaternion(g_gyroFrame,pitch, heading, bank );
printf("T=%d P=%+2.2f H=%+2.2f B=%+2.2f\n",i,pitch/RADIANS_PER_DEGREE ,heading/RADIANS_PER_DEGREE ,bank/RADIANS_PER_DEGREE );

}

	return 0;
}
*/




