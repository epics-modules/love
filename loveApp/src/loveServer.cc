// loveServer.cc

// Author: Jim Kowalkowski
// Date: 7/94
// Changes for MPF: Marty Kraimer
// 27AUG98
// Changed Again to reflect Love interface same as useing Bitbus. 
//  Revised by Mohan Ramanathan
// Date: May 14, 1999
//
// 28-Aug-2001 Mark Rivers
//     Added (int) cast to call to writeRead to avoid compiler error


extern "C" {
#include <vxWorks.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <taskLib.h>
#include <semLib.h>
}
 
#include "Message.h"
#include "Int32Message.h"
#include "Char8ArrayMessage.h"
#include "loveServer.h"
#include "WatchDog.h"
#include "SerialPort.h"
 
static const int readBufSize = 20;

int loveServerDebug=0;
 
enum readState {readIdle, readStart, readData};

class Love
{
public:
    Love(const char *name,const char *portName, int queueSize);
    static void loveServer(Love *);
    bool getStartOk() const {return(startOk);}
    static byteHandlerRC byteHandler(void*,unsigned char);
    void talk(Int32Message *pmessage);
private:
    static int setUpRead(unsigned char* msg,unsigned char addr,short cmd);
    static int checkSumFailure(unsigned char* msg,int response_len);
    static unsigned long getValue(unsigned char* data,int pos);

    MessageServer *pMessageServer;
    SerialPort* pSerialPort;
    bool startOk;
    readState state;

    int nextRead;
    unsigned char readBuf[readBufSize];
};

enum { STX=0x02, ETX=0x03, ACK=0x06 };

static char codes[] = "0123456789ABCDEF";
static int hex[] = { 0,1,2,3,4,5,6,7,8,9,0,0,0,0,0,0,0,10,11,12,13,14,15,16 };

/*
    message formats:
                                   user stuff
                | header        | checksum data (ascii) | tail
    -----------------------------------------------------------
    error	| <stx> <Baddr> | <addr> N <error code> | <ack>
    instr	| <stx> <Baddr> | <addr> <data>         | <ack>
    host	| <stx> <Baddr> | <addr> <data>         | <cksm> <etx>

    <stx>=02 <ack>=06 <etx>=03
    <cksm>=lower 8 bits of sum of checksum data
    <Baddr>  = Currently for Love controller it is "L" (0x4C)
    <addr>=binary address of destination 0x01 to 0xff, 0x00 invalid

    all user data encoded as ascii hex - 0x12FD sent as 31 32 46 44 (4 bytes)
*/

// error codes:
#define ERR_UNDEFINED_CMD1	01
#define ERR_CHECK_SUM		02
#define ERR_CMD_NOT_PERFORMED	03
#define ERR_BAD_ASCII_CHAR_RECV 04
#define ERR_DATA_FIELD_ERROR	05
#define ERR_UNDEFINED_CMD2	06
#define ERR_HARDWARE_FAULT1	08
#define ERR_HARDWARE_FAULT2	09
#define ERR_UNDEFINED_CMD3	10

static char taskname[] = "love";
int initLoveServer(char *serverName,char *portName, int queueSize)
{
    Love *pLove = new Love(serverName,portName, queueSize);
    if(!pLove->getStartOk()) return(0);
    int taskId = taskSpawn(taskname,100,VX_FP_TASK,2000,
	(FUNCPTR)Love::loveServer,(int)pLove,0,0,0,0,0,0,0,0,0);
    if(taskId==ERROR) printf("loveServer taskSpawn Failure\n");
    return(0);
}

Love:: Love(const char *name,const char *portName, int queueSize)
:  startOk(false), state(readIdle), nextRead(0)
  
{
    pSerialPort = SerialPort::bind(portName,this,byteHandler);
    if(pSerialPort==0) {
	printf("%s: could not bind to SerialPort %s\n",name,portName);
        return;
    }
    pMessageServer = new MessageServer(name,queueSize);
    startOk = true;
}

byteHandlerRC Love::byteHandler(void* v, unsigned char data)
{
    Love *pLove = (Love *)v;
    switch(pLove->state) {
    case readIdle:
	return(byteHandlerError);
    case readStart:
	if(data!=STX) break; // just skip character
	pLove->state = readData;
	break;
    case readData:
	if(data==ACK) {
	    pLove->state=readIdle;
	    return(byteHandlerEndRead);
	}
        if((pLove->nextRead)>=readBufSize) {
	    pLove->state=readIdle;
	    return(byteHandlerError);
        }
	pLove->readBuf[pLove->nextRead++] = data;
	break;
    default:
	break;
    }
    return(byteHandlerOK);
}

void Love::loveServer(Love *pLove)
{
    while(true) {
        pLove->pMessageServer->waitForMessage();
	Message *message;
        while((message = pLove->pMessageServer->receive())) {
            if(message->getType()!=messageTypeInt32) {
                printf("loveServer got illegal message type %d\n",
		     message->getType());
		continue;
            }
            Int32Message  *preceive = (Int32Message *)message;
	    pLove->talk(preceive);
	   
	    pLove->pMessageServer->reply(preceive);
	}
    }
    return;
}

void Love::talk(Int32Message* preceive)
{
    unsigned char out_data[readBufSize];
    unsigned long sign;
    int value;
    ::memset(out_data,0,readBufSize);
    int len = setUpRead(out_data,preceive->address,preceive->cmd);
    out_data[len]=0;
    nextRead = 0;
    state = readStart;
    ::memset(readBuf,0,readBufSize);

    if (loveServerDebug) 
        printf(" sending data (%d) : |%s|\n",len, out_data);
    
    int status = pSerialPort->writeRead(out_data,len,(int)preceive->timeout);
    readBuf[nextRead] = 0;

    if (loveServerDebug) 
        printf(" receiving data (%d): |%s|\n",status, readBuf);
    
    if(status!=0) {
        preceive->status = -1;
        return;
    }
    
#if 0
    if(readBuf[4]=='N') { 
	int error = (hex[readBuf[5]-'0'])<<4 | hex[readBuf[6]-'0'];
#endif
    if(readBuf[3]=='N') { 
	int error = (hex[readBuf[4]-'0'])<<4 | hex[readBuf[5]-'0'];
	printf("love: error %x from controller %c%c\n",
		error,readBuf[1],readBuf[2] );
	preceive->status = -1;
	return;
    } else {
        if( checkSumFailure(readBuf,nextRead) ) {
	    if(loveServerDebug) 
	        printf("checksum failure\n");
	    preceive->status = -1;
	    return;
	}
        switch(preceive->cmd) {
        case R_DPT:
            value = (unsigned long)(readBuf[4]-'0');
	    preceive->status = 0;
            break;
        case R_PV:
            // determine sign from the 4 th char and least bit
            sign =(unsigned long)(hex[readBuf[6]-'0']);

	    value = (long)getValue(readBuf,7);
            if(sign) {
                // adjust sign
                value *= -1;
            }
            break;

        case R_SP1_VALUE:
        case R_SP2_VALUE:
        case R_AL_LOW:
        case R_AL_HIGH:

           // determine sign from the 1st & 2nd char
            sign =
                (unsigned long)(hex[readBuf[3]-'0'])+
                (unsigned long)(hex[readBuf[4]-'0']);

	    value = (int)getValue(readBuf,5);
            if(sign != 0) {
                // adjust sign
                value *= -1;
            }
            break;

        case R_STATUS:
           // determine setpoint bit which is 4th char(document is wrong!)
	    value = (int)(hex[readBuf[6]-'0']);
            break;

        default:
	    preceive->status = -1;
	    return;
        }
        preceive->value = (int32)value;
        preceive->status = 0;
        if (loveServerDebug) 
            printf(" Sending back Value: %d \n",preceive->value);
        
    }
}

int Love::setUpRead(unsigned char* data, unsigned char addr,short cmd)
{
    int total=0;
    int pos=0;
    unsigned char sum;

    data[pos++]=STX;
    data[pos++]='L';	// binary addr, no 'L'

    // ascii version of address
    data[pos]=codes[(addr>>4)];		total+=(int)data[pos++];
    data[pos]=codes[(addr&0x0f)];	total+=(int)data[pos++];

    // command to execute
    if(cmd <= 0xff) {
        data[pos]=codes[(cmd>>4)];	total+=(int)data[pos++];
        data[pos]=codes[(cmd&0x0f)];	total+=(int)data[pos++];
    } else {
        data[pos]=codes[(cmd>>12)];		total+=(int)data[pos++];
        data[pos]=codes[(cmd>>8&0x000f)];	total+=(int)data[pos++];
        data[pos]=codes[(cmd>>4&0x000f)];	total+=(int)data[pos++];
        data[pos]=codes[(cmd&0x000f)];		total+=(int)data[pos++];
    }
    sum=(unsigned char)total; // checksum for ascii address + command
    data[pos++]=codes[(sum>>4)];
    data[pos++]=codes[(sum&0x0f)];
    data[pos++]=ETX;
    return pos;
}

int Love::checkSumFailure(unsigned char* data,int data_len)
{
    int total=0;
    int i,sum;
/*    unsigned char* pos=&data[1];
  Subtract 3 for start char & checksum... & 
    for(i=0;i<(data_len-3);i++) total+=(int)pos[i];
    total&=0xff;
    sum=(hex[pos[i]-'0'])<<4 | hex[pos[i+1]-'0'];
*/
//  Subtract 2 for checksum... 
    for(i=0;i<(data_len-2);i++) 
        total+=(int)data[i];
    total&=0xff;
    sum=(hex[data[i]-'0'])<<4 | hex[data[i+1]-'0'];
    return (sum==total)?0:-1;
}

unsigned long Love::getValue(unsigned char* data,int pos)
{
    unsigned long x;

    x=(unsigned long)(data[pos+0]-'0')*1000+
      (unsigned long)(data[pos+1]-'0')*100+
      (unsigned long)(data[pos+2]-'0')*10+
      (unsigned long)(data[pos+3]-'0');

    return x;
}
