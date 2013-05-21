#include <stdio.h>
#include <winsock.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <string>


#define LOG_REC_START  (0xEB)
#define LOG_REC_END  (0xEE)
#define LOG_REC_ESC  (0xEC)
#define LOG_REC_TICK (0xED)


#define PI       3.14159265358979323846

										   
typedef unsigned char   uint8_t; 
typedef  unsigned short uint16_t;
typedef  unsigned int   uint32_t;
typedef int             int32_t; 
typedef short           int16_t; 


FILE * fout;

std::string GetAllocName(const unsigned char * p, int len)
{
std::string s((const char *)p,len);
return s;
}


double extract_double(const unsigned char * p)
{
volatile double dd;
unsigned char *pt=(unsigned char* )&dd;
pt[7]=p[0];
pt[6]=p[1];
pt[5]=p[2];
pt[4]=p[3];
pt[3]=p[4];
pt[2]=p[5];
pt[1]=p[6];
pt[0]=p[7];
return dd;
}


double extract_float(const unsigned char * p)
{
volatile float ff;
unsigned char *pt=(unsigned char* )&ff;
double d;

pt[3]=p[0];
pt[2]=p[1];
pt[1]=p[2];
pt[0]=p[3];

d=ff;
//printf("\n%02X,%02X,%02X,%02X : %08X   %g\n",p[0],p[1],p[2],p[3],*((unsigned int *)&ff),d);

return d;
}






unsigned int getuint(const unsigned char * buf, int offset, int len)
{
unsigned int u=0;
int n=0;

while(len)
{
 u=(u<<8)+buf[offset+n];
 len--;
 n++;
}
return u;
}


int getint(const unsigned char * buf, int offset, int len)
{
int i=0;
int n=0;

if(buf[offset]&0x80) i=-1;

while(len)
{
 i=(i<<8)+buf[offset+n];
 len--;
 n++;
}
return i;
}



double getdouble(const unsigned char * buf, int offset)
{
return extract_double(buf+offset);
}


double getfloat(const unsigned char * buf, int offset) 
{
return extract_float(buf+offset);
}





static bool bError;


volatile unsigned long global_time_marks;

int GetRecord(unsigned char * buf, int len, int & pos, unsigned char* dest_buffer,int dest_len,int &rx_len)
{
 int rv;
 bool bLast_Escape=false;
 BYTE csum=0;

 while ((pos<len) && (buf[pos]!=LOG_REC_START)) 
	 {
	 if(buf[pos]==LOG_REC_TICK) global_time_marks++;;
	 pos++;
	 }
 
 if(buf[pos]!=LOG_REC_START) 
	 {
	  //printf("Log exit %02X\n",buf[pos]);
	  return -1;
     }
 
 pos++;
 if(pos>=len) return -1;
 if(buf[pos]==LOG_REC_TICK)
 {
  global_time_marks++;;
  pos++;
 }
 csum=buf[pos];

 rv=buf[pos++]; //Record type 

 rx_len=0;
 while ((pos<len) && (buf[pos]!=LOG_REC_END)) 
	 {
	 if(buf[pos]==LOG_REC_ESC)
	 {
		 bLast_Escape=true;
	 }
	 else
	  if(buf[pos]==LOG_REC_TICK)
	  {
		  global_time_marks++;;
	  }
	 else
	 {
	  if(bLast_Escape)
	  {
	   switch(buf[pos])
		{
		   	case 0: dest_buffer[rx_len++]=LOG_REC_START; break;
			case 1: dest_buffer[rx_len++]=LOG_REC_ESC; break;
			case 2: dest_buffer[rx_len++]=LOG_REC_END; break;
			case 3: dest_buffer[rx_len++]=LOG_REC_TICK; break;
			case 4: dest_buffer[rx_len++]=0X11; break;
			case 5: dest_buffer[rx_len++]=0X13; break;
	   default:
		   break;
		}
	   csum+=dest_buffer[rx_len-1];

	  }
	  else
	  {
	  csum+=buf[pos];
	  dest_buffer[rx_len++]=buf[pos];
	  }
      if(rx_len>=dest_len) rx_len=dest_len-1;
	  bLast_Escape=false;
	 }
	 pos++;
     }
if(buf[pos]==LOG_REC_END) 
{
  if(csum!=buf[pos+1]) 
  {
	  bError=true;
	  if(buf[pos+1]==LOG_REC_ESC)
	  {	BYTE cv;
		  switch(buf[pos+2])
		{
		   	case 0: cv=LOG_REC_START; break;
			case 1: cv=LOG_REC_ESC; break;
			case 2: cv=LOG_REC_END; break;
			case 3: cv=LOG_REC_TICK; break;

	   }
		if(cv==csum) bError=false;
	  }
  }
  else
	bError=false;
}
return rv;
}


#define LOG_TYPE_MSG (0x11)
#define LOG_TYPE_STRUCT (1)
#define LOG_TYPE_ITEM  (2)



#define RXBUF_SIZE (10000)


class StructureElement
{
public:
	int m_data_type;
	int m_siz;
	int m_offset;
	std::string m_sName;
    
	StructureElement * pNext;
	bool bShow;
	StructureElement(int siz,int offset, int data_type, std::string &sName) {m_data_type=data_type; m_offset=offset; m_sName=sName; m_siz=siz;  bShow=false;};
	const char * GetTypeName();
	int Render(const unsigned char * buf, char * outbuf, int maxlen); 

    
};

class StructureElement; //forward

class StructureRecord
{
protected:
	int num;
	int siz;
	std::string m_sStructName;
	StructureElement * pElHead;
	bool bShowAll;


 StructureRecord() {num=0; pElHead=0; bShowAll=false;};

public:
 virtual void HandleRecord(const unsigned char * buildbuf,int len)=0;
 virtual void ShowRecord(const unsigned char * buildbuf,int len){ /* Do nothing by default*/ };
 virtual void ShowLabels(){ /* Do nothing by default*/ };
 void HangElement(StructureElement *pel);
 int GetIndex() {return num; };
 std::string GetName() {return m_sStructName;};
 void Show(bool bshow) {bShowAll=bshow; }
 void TestShow(const char * testv);
};


void StructureRecord::TestShow(const char * test)
{
 std::string s=test;
 if(s.compare(m_sStructName)==0) Show(true);
}

void StructureRecord::HangElement(StructureElement *pel)
 {
 
	 pel->pNext=0;
	if(pElHead) 
	{
	 StructureElement * pfind=pElHead;
	 while(pfind->pNext) pfind=pfind->pNext;
	 pfind->pNext=pel;
    } else 
	 pElHead=pel;
};





class MessageProcess :public StructureRecord
{
public:
   MessageProcess() {num=1; m_sStructName="Message"; siz=-1;}
   virtual void HandleRecord(const unsigned char * buildbuf,int len){/* Do nothing for handle */};
   virtual void ShowRecord(const unsigned char * buildbuf,int len);
};


class NewStructure :public StructureRecord
{
public:
   NewStructure() {num=2; m_sStructName="New Structure"; siz=-1;}
   virtual void HandleRecord(const unsigned char * buildbuf,int len);

};

class NewElement :public StructureRecord
{
public:
   NewElement() {num=2; m_sStructName="New Element"; siz=-1;}
   virtual void HandleRecord(const unsigned char * buildbuf,int len);
};


class DataStructure : public StructureRecord
{
public:
	DataStructure(int id, int ssize, std::string &name) {num=id, siz=ssize; m_sStructName=name;}
	virtual void HandleRecord(const unsigned char * buildbuf,int len);
	virtual void ShowRecord(const unsigned char * buildbuf,int len);
	virtual void ShowLabels();
};



StructureRecord * pRecs[256];
unsigned long RecsReceived[256];

/* tu8=1,
 ti8=2,
 tu16=3,
 ti16=4,
 tu32=5,
 ti32=6,
 tpu8=7,
 tpi8=8,
 tf32=9,
 tf64=10,
*/




const char * StructureElement::GetTypeName()
{
 switch (m_data_type) 
 {
 case 1: return "u8";
 case 2: return "i8";
 case 3: return "u16";
 case 4: return "i16";
 case 5: return "u32";
 case 6: return "i32";
 case 7: return "pu8";
 case 8: return "pi8";
 case 9: return "float";
 case 10: return "double";
 }
 return "???";
}

int StructureElement::Render(const unsigned char * buf, char * outbuf, int maxlen)
{
int i;
double d;

	switch (m_data_type) 
	{
	case 1: return sprintf(outbuf,"%u",getuint(buf,m_offset,1));//u8
	case 2: return sprintf(outbuf,"%d", getint(buf,m_offset,1));//"i8";
	case 3: return sprintf(outbuf,"%u",getuint(buf,m_offset,2));//"u16";
	case 4: return sprintf(outbuf,"%d", getint(buf,m_offset,2));//"i16";
	case 5: return sprintf(outbuf,"%u",getuint(buf,m_offset,4));//"u32";
	case 6: return sprintf(outbuf,"%d", getint(buf,m_offset,4));//"i32";
	case 7: break; //"pu8";
	case 8: break; //"pi8";
	case 9: return  sprintf(outbuf,"%0.10g",  getfloat(buf,m_offset));//"float";
	case 10: return sprintf(outbuf,"%0.20g", getdouble(buf,m_offset));//"double";
	}

outbuf[0]=0;

return 0;
}


void MessageProcess::ShowRecord(const unsigned char * buildbuf,int len)
{
if(!bShowAll) return;
char tmpbuf[1024];
memcpy(tmpbuf,buildbuf,len);
tmpbuf[len]=0;
 fprintf(fout,"%s\n",tmpbuf);
}



void NewStructure::HandleRecord(const unsigned char * buildbuf,int len) 
{
//printf("New Structure\n");
// id
//size msb
//size lsb
//name ...
std::string sName=GetAllocName(buildbuf+3,len-3);

int id=buildbuf[0];
int ilen=(buildbuf[1]<<8)|(buildbuf[2]);

//printf("building for %d\n id:%d slen %d\n%s\n",len,id,ilen,buildbuf+3);
DataStructure *ds= new DataStructure(id,ilen,sName);
if(pRecs[id]) delete pRecs[id];
pRecs[id]=ds;
//printf("Found structure id %d len %d %s\n",id,ilen,pName);
}




void NewElement::HandleRecord(const unsigned char * buildbuf,int len) 
{
int id =buildbuf[0];
int tn =buildbuf[1];
int elesize=(buildbuf[2]<<8)|(buildbuf[3]); 
int offset=(buildbuf[4]<<8)|(buildbuf[5]);

if(len<6) return;
std::string sName=GetAllocName(buildbuf+6,len-6); 

if(pRecs[id])
 {
   StructureElement * pel= new StructureElement(elesize,offset,tn,sName); 
   pRecs[id]->HangElement(pel);
   //printf("EL[%d] def %s->(%s)%s\n",offset,pRecs[id]->GetName().c_str(),pel->GetTypeName(),sName.c_str());
 }
else
 {
   fprintf(stderr,"Got element for id %d name %s unknown\n",id,sName.c_str());
 }
}


void DataStructure::HandleRecord(const unsigned char * buildbuf,int len)
{
}

void DataStructure::ShowLabels()
{
if(!bShowAll) return;
StructureElement * pEl=pElHead;
char obuf[256];
while(pEl)
{
 fprintf(fout,"%s,",pEl->m_sName.c_str());
 pEl=pEl->pNext;
}
if(pElHead) fprintf(fout,"\n");
}


void DataStructure::ShowRecord(const unsigned char * buildbuf,int len)
{
if(!bShowAll) return;

StructureElement * pEl=pElHead;
char obuf[256];
while(pEl)
{
 pEl->Render(buildbuf,obuf,256);
 fprintf(fout,"%s,",obuf);
 pEl=pEl->pNext;
}
if(pElHead) fprintf(fout,"\n");
}




void HexBufCon(unsigned char* buf,size_t & len)
{
size_t ml=len;
size_t nl=0;
unsigned char * put=buf;
unsigned char tmp;
bool half=false;;

for(int i=0; i<ml; i++)
{
char c=buf[i];
if ((c>='0') && (c<='9'))
{
	tmp=(tmp<<4)+(c-'0');
    if(half) 
		{
		 half=false;
		 *put++=tmp;
		 nl++;
		 tmp=0;
		}
		else
		half=true;
}
else
if((c>='A') && (c<='F'))
{
	tmp=(tmp<<4)+(10+c-'A');
    if(half) 
		{
		 half=false;
		 *put++=tmp;
		 nl++;
		 tmp=0;
		}
		else
		half=true;
}
}

len=nl;

}


int main(int argc, char** argv)
{
	if ( argc < 2 )
{
   printf( "Usage is :\n" );
   printf( "read_log filename <options>\n" );
   printf( "options:\n");
   printf( "-o outputfile\n");
   printf( "-e use error records as well\n");
   printf( "-s for show record contents\n");
   printf( "-l <list of records>\n");
   printf( "-b nameofrecord  name of record to break line around\n");
   printf( "example read_log in.bin -o out.csv -l Mode,GPS,IMU -b GPS\n\n this would outpug mode,gps and imu records one a single line every time a GPS record is seen");

   return -1;
}

char *infile = argv[1];

int out_file_argv=-1;
int bShowRecords =0;
int list_argv =-1;
int break_argv=-1;
bool bShowMessages =false;
bool bUseError =false;


for(int i=0; i<argc; i++)
{
if(argv[i][0]=='-')
	{
	switch (argv[i][1]) 
	  {
		case 's': bShowRecords=1; break;
		case 'l': list_argv=i+1; break;
		case 'b': break_argv=i+1; break;
		case 'o': out_file_argv=i+1; break;
        case 'm': bShowMessages=true;
		case 'e': bUseError=true;

      }
    }
}




if(out_file_argv>0) 
fout	= fopen( argv[out_file_argv], "w" ); 
else
fout=stdout;


FILE *fp = fopen( infile, "rb" );
int rv = 0;

if ( fp != NULL )
{
   fseek( fp, 0, SEEK_END );
   size_t len = ftell( fp );
   fseek( fp, 0, SEEK_SET );
   unsigned char * buf = new unsigned char[len];
   int n = fread( buf, 1, len, fp );
   fclose( fp );
   int pos=0;
   int rx_len;
   int rec_type;
   unsigned char dest_buffer[RXBUF_SIZE];

   bool bHexMode=true;

   for(int i=0; i<256; i++)
   {
    char c=buf[i];
	   if  (
			(c!='\r') && 
		    (c!='\n') && 
		    (c!=',') && 
			(c!=' ') && 
		     !(((c>='0') &&  (c<='9')) || ((c>='A') && (c<='F')))
			)
		{
		   bHexMode=false;
		   fprintf(stderr,"Hex mode fails for %02X %c\n",c,c);
		   break;
	    }
   }

   if (bHexMode) 
   {
	HexBufCon(buf,len);
   }

   
   pRecs[LOG_TYPE_MSG ]=new MessageProcess();
   pRecs[LOG_TYPE_STRUCT]=new NewStructure();
   pRecs[LOG_TYPE_ITEM]=new NewElement();


   
    while ((rec_type=GetRecord(buf,len,pos,dest_buffer,(int)RXBUF_SIZE, rx_len))>=0)
	{
		RecsReceived[rec_type]++;
       // printf("Got record of len %d id %d\n",rx_len,rec_type);
		 if(pRecs[rec_type]) pRecs[rec_type]->HandleRecord(dest_buffer,rx_len);
		 else
		 fprintf(stderr,"Got Undefined Record type %d of len %d\n",rec_type,rx_len);
	   //printf("Done\n");
	 }


	if(bShowRecords)
	{
    for(int i=0; i<256; i++)
	{
		if((RecsReceived[i]) && ((i==1) || (i>3)))
		{
		if(pRecs[i]) fprintf(fout,"%s: Received %d records\n",pRecs[i]->GetName().c_str(),RecsReceived[i]);
		else
		fprintf(stderr,"Recieved %d orphans at %d\n",RecsReceived[i],i);
		}
	}
	if(bShowMessages)
		{
		pos=0;
		pRecs[LOG_TYPE_MSG]->Show(true); 
		while ((rec_type=GetRecord(buf,len,pos,dest_buffer,(int)RXBUF_SIZE, rx_len))>=0)
		{
			
			if(rec_type==LOG_TYPE_MSG ) 
				{
				 pRecs[rec_type]->ShowRecord(dest_buffer,rx_len);
				}


		}
	  }
	 }
	else
	{
	if(bShowMessages) pRecs[LOG_TYPE_MSG]->Show(true);
	
	if(list_argv!=-1) 
		{
		 for(int i=0; i<256; i++) if(pRecs[i]) pRecs[i]->TestShow(argv[list_argv]);
		 fprintf(stderr,"Showing records %s\n",argv[list_argv]);
		}
	else
		{
		for(int i=0; i<256; i++) if(pRecs[i]) pRecs[i]->Show(true);
		fprintf(stderr,"Showing all records\n");
		}



	 for(int i=0; i<256; i++) if(pRecs[i]) pRecs[i]->ShowLabels(); 



	pos=0;
    while ((rec_type=GetRecord(buf,len,pos,dest_buffer,(int)RXBUF_SIZE, rx_len))>=0)
	{
		if((bUseError) ||(!bError))
		{
		
		if(pRecs[rec_type]) pRecs[rec_type]->ShowRecord(dest_buffer,rx_len);
		 else
		 fprintf(stderr,"Got Undefined Record type %d of len %d\n",rec_type,rx_len);
		}
	 }
	}



}
if(out_file_argv) 
fclose(fout);

return 0;


}
