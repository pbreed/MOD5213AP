#include "predef.h" 
#include <ctype.h>
#include <basictypes.h>
#include <system.h>
#include <string.h>
#include "sensor_config.h"

#include "filereporter.h" 
REPORT_FILE;



sensor_saved_config SensorConfig;

#define DEF_VERSION (0x12340003)


void LoadSensorConfig()
{
sensor_saved_config * pcfg=(sensor_saved_config *)GetUserParameters();
if((pcfg->version==DEF_VERSION) && (pcfg->version_2==DEF_VERSION))
	{
	memcpy(&SensorConfig,pcfg,sizeof(SensorConfig));
    }	
	else
	{//Default values 
		SensorConfig.version=DEF_VERSION;
		for(int i=0; i<3; i++)
		{
		SensorConfig.mag_max[i]=128;          
		SensorConfig.mag_min[i]=-128;          
		SensorConfig.accel_zero[i]=0;       
		SensorConfig.default_gyro_zero[i]=0;
		}
		for(int i=0; i<8; i++)
		{
		  SensorConfig.servo_neg_lim[i]=-1.04;    
		  SensorConfig.servo_pos_lim[i]=0.96;    
		  SensorConfig.servo_mid[i]=-0.04;        
		}

		SensorConfig.rx_rc_zeros_el=0;
		SensorConfig.rx_rc_gain_el=1/(16000.0);
		SensorConfig.rx_rc_zeros_al=0;
		SensorConfig.rx_rc_gain_al=1/(16000.0);
		SensorConfig.rx_rc_zeros_rd=0;
		SensorConfig.rx_rc_gain_rd=1/(16000.0);
		SensorConfig.rx_rc_zeros_th=0;
		SensorConfig.rx_rc_gain_th=1/(16000.0);
		SensorConfig.version_2=DEF_VERSION;	
	}//Defaults 


}
void SaveSensorConfig()
{
 SaveUserParameters(&SensorConfig,sizeof(SensorConfig));
}




