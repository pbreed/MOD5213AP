#include "predef.h"
#include <basictypes.h>
#include <a2d.h>
#include <Pins.h>
#include "filereporter.h" 
REPORT_FILE;


void Inita2DSub()
{   // Configure the A2D pins as analog inputs 
   Pins[11].function( PIN11_AN2 );
   Pins[12].function( PIN12_AN1 );
   Pins[13].function( PIN13_AN0 );
   Pins[14].function( PIN14_AN3 );
   Pins[15].function( PIN15_AN7 );
   Pins[16].function( PIN16_AN6 );
   Pins[17].function( PIN17_AN5 );
   Pins[18].function( PIN18_AN4 );

   /*
     Enable the A2D. The A2D subsystem will run in the background
     doing samples at 98.742Khz per channel.  This is all done in 
     hardware with no CPU overhead.
   */
 EnableAD(); 

}

int ReadA2D(int ch)
{
 return ( ReadA2DResult( ch ) >> 3); 
}


int ReadBVx1000()
{
//7.66v =2972
//6.46= 2510
//8.09 =3146
return (ReadA2D(0)*168703)>>16;
}


