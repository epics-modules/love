//devLove.cc
// Author: Jim Kowalkowski
// Revised: 2/15/95
// Changes for MPF: Marty Kraimer
// Date: 27OCT98
// Changed Again to reflect Love interface same as useing Bitbus. 
//  Revised by Mohan Ramanathan
// Date: May 14, 1999

extern "C" {
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
 
#include <semLib.h>
#include <tickLib.h>
#include <taskLib.h>
#include <math.h>
 
#include "dbAccess.h"
#include "dbDefs.h"
#include "dbScan.h"
#include "link.h"
#include "epicsPrint.h"
#include "dbCommon.h"
#include "aiRecord.h"
#include "biRecord.h"
#include "recSup.h"
}

#include "Message.h"
#include "Int32Message.h"
#include "DevMpf.h"
#include "loveServer.h"

/*
INP has form Cx Sx @server,address
S0 - Read value       - AI
S1 - Read SP1 value   - AI
S2 - Read SP2 value   - AI
S3 - Read AlHi value  - AI
S4 - Read AlLo value  - AI
S5 - SP1 - ON/OFF     - BI
S6 - SP2 - ON/OFF     - BI
S7 - AL - ON/OFF      - BI
*/

long devLoveDebug=0;

enum getState {getStart, getDecimalPlaces, getData};

class DevLove : public DevMpf
{
public:
    DevLove(dbCommon*,DBLINK*);
    virtual long startIO(dbCommon* pr);
    virtual long completeIO(dbCommon* pr,Message* m);
protected:
    int value;
    int cmd;
    int32 address;
    int signal;
    double cvtFactor;
    getState state;
};

class DevAiLove : public DevLove
{
public:
    DevAiLove( dbCommon*,DBLINK*);
    long startIO(dbCommon* pr);
    long completeIO(dbCommon* pr,Message* m);
    static long dev_init(void*);
   
};

class DevBiLove : public DevLove
{
public:
    DevBiLove( dbCommon*,DBLINK*);
    long startIO(dbCommon* pr);
    long completeIO(dbCommon* pr,Message* m);
    static long dev_init(void*);
   
};

MAKE_LINCONV_DSET(devAiLove,DevAiLove::dev_init)
MAKE_DSET(devBiLove,DevBiLove::dev_init)

long DevAiLove::dev_init(void* v)
{
    aiRecord* pr = (aiRecord*)v;
    DevLove* pDevLove = new DevAiLove((dbCommon*)pr,&(pr->inp));
    return(pDevLove->getStatus());
}

long DevBiLove::dev_init(void* v)
{
    biRecord* pr = (biRecord*)v;
    DevLove* pDevLove = new DevBiLove((dbCommon*)pr,&(pr->inp));
    return(pDevLove->getStatus());
}

DevLove::DevLove(dbCommon* pr,DBLINK* l)
: DevMpf(pr,l,false), value(0), cmd(0), address(0), signal(0), 
  cvtFactor(1.0), state(getStart)
{
    vmeio* io = (vmeio*)&(l->value);
    const char* pv = getUserParm();
    if(pv==0) {
	epicsPrintf("%s DevLove Illegal INP field\n",pr->name);
        pr->pact = TRUE;
        return;
    }
    ::sscanf(pv,"%x",&address);    
    signal = io->signal;
    if (signal <0 || signal > 7) {
        epicsPrintf("%s devLove Illegal INP field\n",pr->name);
        pr->pact = TRUE;
        return;
    }
    if (devLoveDebug)
        printf(" %s address : %d  signal : %d\n",pr->name,address,signal);
}

DevAiLove::DevAiLove(dbCommon* pr,link* l)
: DevLove(pr,l)
{
}

DevBiLove::DevBiLove(dbCommon* pr,link* l)
: DevLove(pr,l)
{
}

long DevLove::startIO(dbCommon *)
{
    Int32Message *message = new Int32Message;
    message->address = address;
    message->cmd = cmd;
    message->timeout = 1;
    return(sendReply(message));
}

long DevAiLove::startIO(dbCommon *pr)
{
    if(state==getStart) {
        cmd = R_DPT;
        state = getDecimalPlaces;
    } else if(state==getData) {
         if (signal == 0)
              cmd = R_PV;
         else if (signal == 1)
             cmd = R_SP1_VALUE;
         else if (signal == 2)
             cmd = R_SP2_VALUE;
         else if (signal == 3)
             cmd = R_AL_LOW;
         else if (signal == 4)
             cmd = R_AL_HIGH;
         else {
	     epicsPrintf("%s DevLove Invalid Signal %d\n",pr->name,signal);
             return(-1);
         }
    } else {
		epicsPrintf("%s (%d) DevLove illegal state in startIO\n",
			pr->name,state);
		state = getStart;
        return(-1);
    }
    int status = DevLove::startIO((dbCommon*) pr);
    return(status);
}

long DevBiLove::startIO(dbCommon *pr)
{
    if(state==getStart || state ==getData) {
         state = getData;
         if (signal > 4 && signal < 8)
             cmd = R_STATUS;         
         else {
	     epicsPrintf("%s DevLove Invalid Signal %d\n",pr->name,signal);
             return(-1);
         }
    } else {
		epicsPrintf("%s (%d) DevLove illegal state in startIO\n",
			pr->name,state);
		state = getStart;
        return(-1);
    }
    int status = DevLove::startIO((dbCommon*) pr);
    return(status);
}

long DevLove::completeIO(dbCommon* pr,Message* pmessage)
{
    if((pmessage->getType()) != messageTypeInt32) {
	state = getStart;
	epicsPrintf("%s DevLove::completeIO illegal message.\n", pr->name);
	recGblSetSevr(pr,READ_ALARM,INVALID_ALARM);
	return(0);
    }
    Int32Message *pInt32Message = (Int32Message *)pmessage;
    if(pInt32Message->status!=0) {
        recGblSetSevr(pr,READ_ALARM,INVALID_ALARM);
        if (devLoveDebug)
            printf(" Status (%d) Alarm Set !\n",pInt32Message->status);
        state = getStart;
    }else if(state==getDecimalPlaces) {
        double decimal_places = (double)pInt32Message->value;
        cvtFactor = 1.0/pow(10.0,decimal_places);
        state = getData;
        scanOnce((void *)pr);
        if (devLoveDebug)
            printf(" Convert Factor is %f \n",cvtFactor);
    } else if (state == getData){
        value  = (int) pInt32Message->value;
    } else  {
        recGblSetSevr(pr,READ_ALARM,INVALID_ALARM);
        state = getStart;
	epicsPrintf("%s DevLove illegal state in completeIO\n",pr->name);
    }
    delete pInt32Message;
    return(0);
}

long DevAiLove::completeIO(dbCommon* pr,Message* pmessage)
{
    aiRecord* precord = (aiRecord*)pr;
    DevLove::completeIO((dbCommon*) pr, pmessage);
    
    precord->val = (double) value * cvtFactor;
    precord->udf = 0;
    if (devLoveDebug)
        printf(" Message value is : %d and Value : %f \n",value,precord->val);

    return(MPF_NoConvert);
}

long DevBiLove::completeIO(dbCommon* pr,Message* pmessage)
{
    biRecord* precord = (biRecord*)pr;
    DevLove::completeIO((dbCommon*) pr, pmessage);
    switch (signal) {
        case 5:
            precord->rval = value & 0x04;
            break;
        case 6:
            precord->rval = value & 0x02;
            break;
        case 7:
            precord->rval = value & 0x01;
            break;
        default:
            break;
    }
    precord->udf = 0;
    if (devLoveDebug) {
        printf(" Message value is : %d and Value : %ld \n",value,precord->rval);
    }
    return(MPF_OK);
}
