void InitServoDrive();
void SetServo(int ch,float v); //Limits -1.0 to +1.0
extern volatile DWORD servo_isrcnt;
extern volatile bool bServoOff;

