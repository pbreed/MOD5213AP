struct sensor_saved_config
{
DWORD version;
short mag_max[3];
short mag_min[3];
short accel_zero[3];
short default_gyro_zero[3];
short rx_rc_zeros_el;
float rx_rc_gain_el;
short rx_rc_zeros_al;
float rx_rc_gain_al;
short rx_rc_zeros_rd;
float rx_rc_gain_rd;
short rx_rc_zeros_th;
float rx_rc_gain_th;
float servo_neg_lim[8];
float servo_pos_lim[8];
float servo_mid[8];
DWORD version_2;
};

void LoadSensorConfig();
void SaveSensorConfig();

extern sensor_saved_config SensorConfig;



