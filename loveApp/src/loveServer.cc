// loveServer.cc
// Author: Jim Kowalkowski
// Date: 7/94
// Changes for MPF: Marty Kraimer
// 27AUG98
// Changed Again to reflect Love interface same as useing Bitbus. 
//  Revised by Mohan Ramanathan
// Date: May 14, 1999
//  Revised to accomadate both 1600 series and 16A series.
//  August 31, 2000
//  Added Port and Address specific debugs.
//  October 24, 2001
//  Revised to add more functionality.  -Mohan Ramanathan
//  March 5, 2002
/*
 *****************************************************************
 *                         COPYRIGHT NOTIFICATION
 *****************************************************************
 * THE FOLLOWING IS A NOTICE OF COPYRIGHT, AVAILABILITY OF THE CODE,
 * AND DISCLAIMER WHICH MUST BE INCLUDED IN THE PROLOGUE OF THE CODE
 * AND IN ALL SOURCE LISTINGS OF THE CODE.
 
 * (C)  COPYRIGHT 2000 UNIVERSITY OF CHICAGO
 
 * Argonne National Laboratory (ANL), with facilities in the States of 
 * Illinois and Idaho, is owned by the United States Government, and
 * operated by the University of Chicago under provision of a contract
 * with the Department of Energy.

 * Portions of this material resulted from work developed under a U.S.
 * Government contract and are subject to the following license:  For
 * a period of five years from August 31, 2000, the Government is
 * granted for itself and others acting on its behalf a paid-up,
 * nonexclusive, irrevocable worldwide license in this computer
 * software to reproduce, prepare derivative works, and perform
 * publicly and display publicly.  With the approval of DOE, this
 * period may be renewed for two additional five year periods. 
 * Following the expiration of this period or periods, the Government
 * is granted for itself and others acting on its behalf, a paid-up,
 * nonexclusive, irrevocable worldwide license in this computer
 * software to reproduce, prepare derivative works, distribute copies
 * to the public, perform publicly and display publicly, and to permit
 * others to do so.

 *****************************************************************
 *                               DISCLAIMER
 *****************************************************************
 * NEITHER THE UNITED STATES GOVERNMENT NOR ANY AGENCY THEREOF, NOR
 * THE UNIVERSITY OF CHICAGO, NOR ANY OF THEIR EMPLOYEES OR OFFICERS,
 * MAKES ANY WARRANTY, EXPRESS OR IMPLIED, OR ASSUMES ANY LEGAL
 * LIABILITY OR RESPONSIBILITY FOR THE ACCURACY, COMPLETENESS, OR
 * USEFULNESS OF ANY INFORMATION, APPARATUS, PRODUCT, OR PROCESS
 * DISCLOSED, OR REPRESENTS THAT ITS USE WOULD NOT INFRINGE PRIVATELY
 * OWNED RIGHTS.  
 *****************************************************************
 * LICENSING INQUIRIES MAY BE DIRECTED TO THE INDUSTRIAL TECHNOLOGY
 * DEVELOPMENT CENTER AT ARGONNE NATIONAL LABORATORY (708-252-2000).
 *****************************************************************
*/


#include <vxWorks.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <taskLib.h>
#include <semLib.h>
 
#include "Message.h"
#include "Int32Message.h"
#include "Char8ArrayMessage.h"
#include "loveServer.h"
#include "WatchDog.h"
#include "SerialPort.h"
 
static const int readBufSize = 30;
double loveTimeout = 0.15;

int loveSeverDelay=8000;  // used to delay in quick succession.
/*
loveServeDelay has a value of 8000 for PPC604 controller. All mv167, mv172 
can have values of 0.  this is needed to delay the transmission of data
on 485.  The devices need time to switch from transmitting to listening.
*/
 
enum readState {readIdle, readStart, readData};

long loveServerDebug=0;
/*
loveServerDebug=0x1	:  Track only Errors from Device
loveServerDebug=0x2	:  Track all activities both to and from Device
loveServerDebug=0x4	:  Track all messages to the Device
loveServerDebug=0x8	:  Track all messages from the Device
*/
int debugAddr;
char debugPort[30];
/*
Due to the numerous lovecontrollers on the same link debuging is a problem.
So if you set loveDebugPort(char *port, int addr) before you set the
loveServerDebug to be nonzero you can get selective debug messages.
*/
class Love
{
public:
    Love(const char *name,const char *portName, int queueSize);
    static void loveServer(Love *);
    bool getStartOk() const {return(startOk);}
    static byteHandlerRC byteHandler(void*, unsigned char);
    void talk(Int32Message *pmessage);
private:
    static void setUpRead(const char* port, int address, int cmd, char* cmdBuf);
    static void setUpWrite(const char* port, int address, int cmd, int dpt,
    						 int value,char* cmdBuf);
    bool xact(const char* port, int address, const char* cmdBuf);
    bool checkSumFailure(char* ,int);
	
    MessageServer *pMessageServer;
    SerialPort* pSerialPort;
    bool startOk;
    readState state;
    char cmdBuf[readBufSize];
    int nextRead;
    char readBuf[readBufSize];
};

enum { STX=0x02, ETX=0x03, ACK=0x06 };

/*
    message formats:
                                   user stuff
                | header        | checksum data (ascii) | tail
    -----------------------------------------------------------
    host	| <stx> <Baddr> | <addr> <data>         | <cksm> <etx>
    instr	| <stx> <Baddr> | <addr> <data>         | <cksm> <ack>

    error       | <stx> <Baddr> | <addr> N <error code> | <ack>

    <stx>=02 <ack>=06 <etx>=03
    <cksm>=lower 8 bits of sum of checksum data
    <Baddr>  = Currently for Love controller it is "L" (0x4C)
    <addr>=binary address of destination 0x01 to 0xff, 0x00 invalid

    all user data encoded as ascii hex - 0x12FD sent as 31 32 46 44 (4 bytes)
    
    The data from the instument depends on the command:
    00 - Read Value and Alarm status   - 8 chars.
   	5 thru 8 is the value.
   	4th char least bit is sign. 0 = + 1= -
   	2nd char highest (4th) bit Alarm status 0 = Deenergized 1 = Energized
   	for Model 16A:
   	3rd char least 2 bits is decimal place 0 = None 1 = 0.0
   		0 = None 1=0.0 2=0.00 3=0.000
    0324 - Read decimal place for Model 1600.  - 2 chars.
         2nd char :  0 = None 1=0.0 2=0.00 3=0.000

for Model 1600:
    0104 - Read Alarm Lo
    0105 - Read Alarm Hi
    0100 - Read SP1 value
    0102 - Read SP2 value
    	All of the above use 6 char. last 4 char is data.
    	first 2 char is sign if both zero positive else negative.
    All the above commands also need 0324 for decimal place.

for Model 16A:    
    0106 - Read Alarm Lo
    0107 - Read Alarm Hi
    0101 - Read SP1 value
    0105 - Read SP2 value
    	All of the above use 6 char. last 4 char is data.
   	2nd char least bit is sign. 0 = + 1= -
   	1st char least 2 bits is decimal place 0 = None 1 = 0.0
   		0 = None 1=0.0 2=0.00 3=0.000
*/

void loveDebugPort(char *port, int addr)
{
    strcpy(debugPort,port);
    debugAddr= addr;
    printf ("Love Server Debug is for Port %s and Address %d Now\n",
    		debugPort,  debugAddr);
    return;
}

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
    char * pvalue;
    int cmd =0;
    int dpt=0; 
    int sign=0;
    int value=0;
    
//  extra carries the valve of 0 for Model 1600 and 1 for models 16A,32A,....

// model 1600 get the decimal point location if the command is for values...
// for write commands of values the decimal point is also required
//  Commands 0 to 6 are read values and 11 to 14 are write values
    if ( (preceive->extra == 0  && preceive->cmd < 7)  
    		|| (preceive->cmd > 10 && preceive->cmd < 15)) {

        if ( preceive->extra )
	    cmd = R1_DPT;
	else
	    cmd = R0_DPT;

	setUpRead(pMessageServer->getName(),preceive->address, cmd, cmdBuf);
        if(!xact(pMessageServer->getName(),preceive->address,cmdBuf)) {
            preceive->status = -1;
            return;
        }
	pvalue = &readBuf[4];
        ::sscanf(pvalue,"%1d",&dpt);
    }

// Setup the correct type of command to send to get data ....    
    switch (preceive->cmd) {
    case GetSP1:
        if (preceive->extra)
            cmd = R1_SP1V;
        else
            cmd = R0_SP1V;
        break;
    case GetSP2:
        if (preceive->extra)
            cmd = R1_SP2V;
        else
            cmd = R0_SP2V;
        break;
    case GetALLo:
        if (preceive->extra)
            cmd = R1_ALLO;
        else
            cmd = R0_ALLO;
        break;
    case GetALHi:
        if (preceive->extra)
            cmd = R1_ALHI;
        else
            cmd = R0_ALHI;
        break;
    case GetPeak:
        if (preceive->extra)
            cmd = R1_PEAK;
        else
            cmd = R0_PEAK;
        break;
    case GetValley:
        if (preceive->extra)
            cmd = R1_VALY;
        else
            cmd = R0_VALY;
        break;
    case GetALType:
        if (preceive->extra)
            cmd = R1_ALTY;
        else
            cmd = R0_ALTY;
        break;
    case GetALStatus:
    case GetValue:
        cmd = R_PV;
        break;
    case GetInpType:
        if (preceive->extra)
            cmd = R1_INTY;
        else
            cmd = R0_INTY;
        break;
    case GetCommStatus:
        if (preceive->extra)
            cmd = R1_COMM;
        else
            cmd = R0_COMM;
        break;
    case PutSP1:
        if (preceive->extra)
            cmd = W1_SP1V;
        else
            cmd = W0_SP1V;
        break;
    case PutSP2:
        if (preceive->extra)
            cmd = W1_SP2V;
        else
            cmd = W0_SP2V;
        break;
    case PutALLo:
        if (preceive->extra)
            cmd = W1_ALLO;
        else
            cmd = W0_ALLO;
        break;
    case PutALHi:
        if (preceive->extra)
            cmd = W1_ALHI;
        else
            cmd = W0_ALHI;
        break;
    case ResetPeak:
        if (preceive->extra)
            cmd = W1_PEAK;
        else
            cmd = W0_PEAK;
        break;
    case ResetValley:
        if (preceive->extra)
            cmd = W1_VALY;
        else
            cmd = W0_VALY;
        break;
    case SetRemote:
        if (preceive->value)
            cmd = W_REM;
        else
            cmd = W_LOC;
        break;    
    }

//  pack the send buffer properly and send it....  
//  depending upon whether read or write command..  
    if (preceive->cmd < 11 || preceive->cmd >14)
        setUpRead(pMessageServer->getName(),preceive->address, cmd, cmdBuf);
    else
        setUpWrite(pMessageServer->getName(),preceive->address, dpt, 
    				preceive->value, cmd, cmdBuf);
    

    if(!xact(pMessageServer->getName(),preceive->address,cmdBuf)) {
        preceive->status = -1;
        return;
    }

// In all returned data the first three char are : L <addr> 
//  Useful data starts only at readBuf[3]
    
    if (preceive->cmd == GetValue) {
        if (preceive->extra) {
	    pvalue = &readBuf[5];
            ::sscanf(pvalue,"%1X",&dpt);
            dpt &=  0x03;
        }
        // determine value from the 5th thru 8th char
	pvalue = &readBuf[7];
        ::sscanf(pvalue,"%4d",&value);
        // determine sign from the 4th char and least bit
	pvalue = &readBuf[6];
        ::sscanf(pvalue,"%1X",&sign);
        sign &= 0x01;
        if (sign) 
            value *= -1; // adjust sign    

    } else if (preceive->cmd > 0 &&  preceive->cmd  < 7) {
        if (preceive->extra) {
            // determine the decimal point from 1nd char and least 2 bits
	    pvalue = &readBuf[3];
            ::sscanf(pvalue,"%1X",&dpt);
            dpt &=  0x03;
            // determine sign from the 2nd char least bit
	    pvalue = &readBuf[4];
            ::sscanf(pvalue,"%1X",&sign);
            sign &= 0x01;
              
        } else {
            // determine sign from the 1st & 2nd char
	    pvalue = &readBuf[3];
            ::sscanf(pvalue,"%2d",&sign);
        }
        // determine value from the 3th thru 6th char
	pvalue = &readBuf[5];
        ::sscanf(pvalue,"%4d",&value);
         if (sign != 0)
             value *= -1;

    } else if (preceive->cmd == GetALStatus ) {
	pvalue = &readBuf[4];
        ::sscanf(pvalue,"%1X",&value);
        value >>=3;

    } else if (preceive->cmd == GetALType) {
	pvalue = &readBuf[3];
        if (preceive->extra)
            ::sscanf(pvalue,"%2d",&value);
        else 
            ::sscanf(pvalue,"%1d",&value);

    } else if (preceive->cmd == GetInpType ) {
        if (preceive->extra) {
            pvalue = &readBuf[3];
            ::sscanf(pvalue,"%2X",&value);
        } else {
            pvalue = &readBuf[4];
            ::sscanf(pvalue,"%1X",&value);
        }
        

    } else if (preceive->cmd > 9 &&  preceive->cmd  < 18 ) {
	pvalue = &readBuf[3];
        ::sscanf(pvalue,"%d",&value);
    }

    preceive->value = (int32)value;
    preceive->extra = (int32)dpt;
    preceive->status = 0;
    if (loveServerDebug & 0x8 ) {
 	if(debugPort && debugAddr) {
  	    if ( (!strcmp(debugPort,pMessageServer->getName())) && 
	    			debugAddr==preceive->address ) 
                printf(" (%s : %d) Sending back Value and dpt: %d  (%d)\n",
        		pMessageServer->getName(),preceive->address,
        		preceive->value, preceive->extra);
        } else {
            printf(" (%s : %d) Sending back Value and dpt: %d  (%d)\n",
        		pMessageServer->getName(),preceive->address,
        		preceive->value, preceive->extra);
    	}
    }

}

void Love::setUpRead(const char *port, int addr, int cmd, char* cmdBuf)
{
    char temp[5];
    int chkSum=0;
    unsigned int i=0;
    ::memset(cmdBuf,0,readBufSize);

    sprintf(cmdBuf,"%cL%02X",STX,addr);

    // command to execute
    if(cmd == R_PV)
        sprintf(temp,"%02X",cmd);
    else
        sprintf(temp,"%04X",cmd);
    
    ::strcat(cmdBuf,temp);

    for(i=2; i< ::strlen(cmdBuf); i++)
        chkSum+=cmdBuf[i];
    chkSum &= 0xff;
    sprintf(&cmdBuf[i], "%02X%c",chkSum,ETX);
    
    if (loveServerDebug & 0x4){
 	if(debugPort && debugAddr) {
	    if ( (!strcmp(debugPort,port) ) && debugAddr==addr )
                printf(" (%s : %d) Data Encoded as  (%d) |%s|\n",
           		port,addr,::strlen(cmdBuf),cmdBuf);
        } else {
            printf(" (%s : %d) Data Encoded as  (%d) |%s|\n",
           	port,addr,::strlen(cmdBuf),cmdBuf);
    	}
    }    
    return;
}

void Love::setUpWrite(const char *port, int addr, int dpt, int value,
						int cmd, char* cmdBuf)
{
    char temp[10];
    int sign=0;
    int chkSum=0;
    unsigned int i=0;
    ::memset(cmdBuf,0,readBufSize);

    sprintf(cmdBuf,"%cL%02X",STX,addr);

    // command to execute
    sprintf(temp,"%04X",cmd);
    ::strcat(cmdBuf,temp);
    
    if (value < 0) {
        sign=10;
        value = abs(value);
    }   
 
    switch (dpt) {
    case 0:
        value /= 1000;
        break;
    case 1:
        value /= 100;
        break;
    case 2:
        value /= 10;
        break;
    case 3:
         break;
    }
    sprintf(temp,"%04d%02d",value,sign);
    ::strcat(cmdBuf,temp);


    for(i=2; i< ::strlen(cmdBuf); i++)
        chkSum+=cmdBuf[i];
    chkSum &= 0xff;
    sprintf(&cmdBuf[i], "%02X%c",chkSum,ETX);
    
    if (loveServerDebug & 0x4){
 	if(debugPort && debugAddr) {
	    if ( (!strcmp(debugPort,port) ) && debugAddr==addr )
                printf(" (%s : %d) Data Encoded as  (%d) |%s|\n",
           		port,addr,::strlen(cmdBuf),cmdBuf);
        } else {
            printf(" (%s : %d) Data Encoded as  (%d) |%s|\n",
           	port,addr,::strlen(cmdBuf),cmdBuf);
    	}
    }    
    return;
}

bool Love::xact(const char *port, int addr, const char* cmdBuf)
{
    ::memset(readBuf,0,readBufSize);
    int error=0;
    state = readStart;
    nextRead = 0;
//   Put a delay due to timing problem when using the PPC controller.
    int y=0;
    for (int x =0; x<= loveSeverDelay; x++) {
	y++;
    }	
    serialStatus status = pSerialPort->write(
        (unsigned char *)cmdBuf,::strlen(cmdBuf),loveTimeout);
    status = pSerialPort->read(loveTimeout);
    readBuf[nextRead] = 0;
    if (nextRead < 5) {
    	status = -1;
    } else {
        if (readBuf[3]=='N') {
            char * pvalue = &readBuf[4];
            ::sscanf(pvalue,"%2X",&error);
	    if(loveServerDebug) 
	        printf("(%s : %d) love: error from controller %x\n",
	           port, addr,error);
	    status = -1;
        } else {
            if( checkSumFailure(readBuf,nextRead) ) {
	        if(loveServerDebug) 
	            printf("(%s : %d) checksum failure from Love controller\n",
	            	port,addr);
	        status = -1;
	    } else 
                readBuf[nextRead-2] = 0; // remove the 2 char checksum data 
        }
    }
    if((loveServerDebug & 0x2) || (status && loveServerDebug)) { 
 	if(debugPort && debugAddr) {
	    if ( (!strcmp(debugPort,port) ) && debugAddr==addr )
                printf("(%s : %d) cmd: %s\n  reply: %s\n",
            		port, addr, cmdBuf, readBuf);
        } else {
            printf("(%s : %d) cmd: %s\n  reply: %s\n",
            	port, addr, cmdBuf, readBuf);

    	}
    }
    return((status==0) ? true : false);
}


bool Love::checkSumFailure(char* data,int data_len)
{
    int total=0;
    int i;
    int status=-1;
    char chksum[3];
//  If there was no read dont bother checking the checksum
    if (data_len < 2)
        return(status);
//  Do not use the last two char in checksum calculation
    for(i=0;i<(data_len-2);i++) 
        total+=(int)data[i];
    total&=0xff;
    sprintf(chksum,"%02X",total);
    if (data[i] == chksum[0] && data[i+1] == chksum[1])
    	status=0;
    return (status);
}

