#include <iostream>
#include <string>
#include <stdlib.h>
#define _USE_MATH_DEFINES (1)
#include <math.h>
using namespace std;

//Operators
// +
// -
// *
// /
// ^ //Power
// # //Enter
// ** Square
// sin
// cos
// tan
// abs
// asin
// acos
// atan
// atan2
// exp
// log
// rad2deg
// deg2rad
// >   
// <
// >=
// <=
// =
// : swap bottom of stack


#define STACK_SIZE (10)

#ifndef M_PI  
#define M_PI (3.141592653589793238462)
#endif
void popop(double * d)
{
for(int i=1; i<(STACK_SIZE-1); i++) d[i]=d[i+1];
d[STACK_SIZE-1]=0;
}


void DoRpn(string & s, double * d)
{
while (isspace(s[0]) && (s.length())) s=s.substr(1);
while (isspace(s[s.length()-1]) && (s.length())) s=s.substr(0,s.length()-1);

//cout<<"Doing ["<<s<<"]" <<d[0]<<" "<<d[1]<<" "<<d[2]<<" " <<d[3];

double neg=1.0;


if((s[0]=='+') && (s.length()>1)) s=s.substr(1);  //Handle +/- before numbers...
if((s[0]=='-') && (s.length()>1)) {s=s.substr(1);  neg=-1.0; }

switch(s[0])
{
case '+': d[0]=(d[0]+d[1]); popop(d); break;
case '-': d[0]=(d[1]-d[0]); popop(d); break;
case '^': d[0]=pow(d[1],d[0]); popop(d); break;
case '/': d[0]=(d[1]/d[0]); popop(d); break;
case '=': d[0]=(d[0]==d[1]); popop(d); break;
case '>': {
		   if((s.length()>1) && (s[1]=='=')) 
				d[0]=(d[1]>=d[0]); 
		       else
				   d[0]=(d[1]>d[0]); 
			popop(d); 
		  }
			break;
case '<': {
		   if(s.length()>1) 
			  {
			   if (s[1]=='=')
				d[0]=(d[1]<=d[0]); 
			   else if (s[1]=='>')
				d[0]=(d[1]!=d[0]); 
		       else
				   d[0]=(d[1]<d[0]); 
				}
				else
				  d[0]=(d[1]<d[0]); 

			popop(d); 
		  }
			break;

case '*': if((s.length()>1) && (s[1]=='*'))
			{
	         d[0]=(d[0]*d[0]);
			}
			else
			{
			d[0]=(d[0]*d[1]); popop(d); 
			}
			break;
case ':' :  {
			double t=d[0];
			d[0]=d[1];
			d[1]=t;
			}
			break;
case '#': 	{
			for (int i=(STACK_SIZE-1); i>0; i--)
				 d[i]=d[i-1];
			}
			break;
case 'i': 
	   		 d[0]=(int)d[0];
			break;
case 's':
			 d[0]=sin(d[0]);
			break;
case 'c':
			 d[0]=cos(d[0]);
			break;
case 't':
			 d[0]=tan(d[0]);
			break;

case 'l':   
		   d[0]=log(d[0]);
		   break;
case 'e':   
		   d[0]=exp(d[0]);
		   break;
case 'r':
			d[0]=d[0]*180.0/M_PI;
		   break;
case 'd':
			d[0]=d[0]*M_PI/180.0;
		   break;

case 'a':
		switch(s[1])
		{
		case 's': /*asin*/ d[0]=asin(d[0]);  break;
		case 'c': /*acod*/ d[0]=acos(d[0]);  break;
		case 't': /*atan or atan2*/ 
				if ((s.length()>3) && (s[4]=='2'))
						{
						d[0]=atan2(d[0],d[1]);
						popop(d);
						}
				else
					d[0]=atan(d[0]);

				break;
		case 'b': /*abs*/ d[0]=fabs(d[0]);  break;

		}
		   break;
		
default:
	   {
		double v=atof(s.c_str());
		//cout<<"PArse num:"<<v;
		v*=neg;
		for(int i=(STACK_SIZE-1); i>0; i--) d[i]=d[i-1];
		d[0]=v;
	   }
	   break;

}


//cout<<"After:"<<d[0]<<" "<<d[1]<<" "<<d[2]<<" " <<d[3]<<endl;


}


double eval(double di,const string si)
{
double d[STACK_SIZE];
//cerr<<"Eval "<<di<<" for "<<si;
//for(int i=0; i<STACK_SIZE; i++)d[i]=0.0;
	 d[0]=di;
string s=si;
int pos=s.find(",");
while(pos!=string::npos)
{
  string e=s.substr(0,pos);
  s=s.substr(pos+1,string::npos);
  DoRpn(e,d);
  pos=s.find(",");
}
  DoRpn(s,d);

//cerr<<d[0]<<endl;
return d[0];
}




