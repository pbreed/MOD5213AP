////////////////////////////////////////////////////////////////////////////////////////////////////
// "FreeSpace IMU" - By Terence J. Bordelon (www.bordelon.net/tj@bordelon.net)
// Extra Math functions
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _EXMATH_H
#define _EXMATH_H

// Includes

//---[Defines]----------------------------------------------------------------------------

#define DEGREES_PER_RADIAN  57.29578
#define RADIANS_PER_DEGREE  0.0174532925199432957692
#define PI                  3.1415926535897932384626433832795

//----[ Structs ]---------------------------------------------------------------------------------------

typedef struct {
	float w,x,y,z;
} quat;

typedef struct {
	float x,y,z;
} vec3;

//---[Prototypes]-------------------------------------------------------------------------

// Vector math

typedef float F32;

void v3set( vec3 *v, F32 x, F32 y, F32 z);
void v3sub( vec3 *out, vec3 *v1, vec3 *v2);
void v3add( vec3 *out, vec3 *v1, vec3 *v2);
void v3mul( vec3 *out, vec3 *v1, vec3 *v2);
F32 v3normalize( vec3 *out, vec3 *in);
void v3scale( vec3 *out, vec3 *in, F32 s);
F32 v3length( vec3 *v );
void v3setLength( vec3 *v, F32 l);
void v3cross( vec3 *out, vec3 *v1, vec3 *v2);
F32 v3dot( vec3 *v1, vec3 *v2);
void v3invert( vec3 *out, vec3 *in);
F32 v3dist( vec3 *v1, vec3 *v2 );
float v3index( vec3 *v, int idx);


float clamped_acos(float angle);

// Quaternion Math

void Quat_Normalize( quat *q);
void Quat_Invert( quat *q);
void Quat_Multiply ( quat *res, quat *q1, quat *q2 );
void Quat_RotateVec3( quat *q, vec3 *vOut, vec3 *vIn);
void Quat_Set( quat *q, float w, float x, float y, float z);
void Quat_SetAxisAndAngle( quat *q, vec3 *axis, float angle);


#endif //_EXMATH_H

