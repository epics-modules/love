//devLoveMpf.cc
// Author: Jim Kowalkowski
// Revised: 2/15/95
// Changes for MPF: Marty Kraimer
// Date: 27OCT98
// Changed Again to reflect Love interface same as using Bitbus. 
//  Revised by Mohan Ramanathan
// Date: May 14, 1999
//  Revised to accomadate both 1600 series and 16A/32A/2600/8600 series.
//  August 31, 2000
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
#include "mbbiRecord.h"
#include "aoRecord.h"
#include "boRecord.h"
#include "recSup.h"
}

#include "Message.h"
#include "Int32Message.h"
#include "DevMpf.h"
#include "loveServer.h"

/*
INP has form Cx Sx @server,address,model
S0  - Read value	- AI
S1  - Read SP1 value	- AI
S2  - Read SP2 value	- AI
S3  - Read AlLo value	- AI
S4  - Read AlHi value	- AI
S5  - Read Peak value	- AI
S6  - Read Valley value - AI
S7  - Alarm ON/OFF 	- BI
S8  - Type of Alarm set	- MBBI
S9  - Input Type	- MBBI
S10 - Communication 	- BI

S11 - Write SP1 value	- AO
S12 - Write SP2 value	- AO
S13 - Write AlLo value	- AO
S14 - Write AlHi value	- AO
S15 - Reset Peak value	- BO
S16 - Reset Valley value- BO
S17 - Set Comm Type	- BO

address is in the range of 01 to ff
model is 0 for 1600 and 1 for 16A/32A, 2600 and 8600
Only models 1600 and 16A have been tested.
*/

long devLoveDebug=0;

class DevLove : public DevMpf
{
public:
    DevLove(dbCommon*,DBLINK*);
    virtual long startIO(dbCommon* pr);
    virtual long completeIO(dbCommon* pr,Message* m);
protected:
    int value;
    int address;
    int signal;
    int model;
    double cvtFactor;
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

class DevMbbiLove : public DevLove
{
public:
    DevMbbiLove( dbCommon*,DBLINK*);
    long startIO(dbCommon* pr);
    long completeIO(dbCommon* pr,Message* m);
    static long dev_init(void*);
   
};

class DevAoLove : public DevLove
{
public:
    DevAoLove( dbCommon*,DBLINK*);
    long startIO(dbCommon* pr);
    long completeIO(dbCommon* pr,Message* m);
    static long dev_init(void*);
   
};

class DevBoLove : public DevLove
{
public:
    DevBoLove( dbCommon*,DBLINK*);
    long startIO(dbCommon* pr);
    long completeIO(dbCommon* pr,Message* m);
    static long dev_init(void*);
   
};

MAKE_LINCONV_DSET(devAiLove,DevAiLove::dev_init)
MAKE_DSET(devBiLove,DevBiLove::dev_init)
MAKE_DSET(devMbbiLove,DevMbbiLove::dev_init)
MAKE_LINCONV_DSET(devAoLove,DevAoLove::dev_init)
MAKE_DSET(devBoLove,DevBoLove::dev_init)

long DevAiLove::dev_init(void* v)
{
    aiRecord* pr = (aiRecord*)v;
    DevLove* pDevLove = new DevAiLove((dbCommon*)pr,&(pr->inp));
    pDevLove->bind();
    return(pDevLove->getStatus());
}

long DevBiLove::dev_init(void* v)
{
    biRecord* pr = (biRecord*)v;
    DevLove* pDevLove = new DevBiLove((dbCommon*)pr,&(pr->inp));
    pDevLove->bind();
    return(pDevLove->getStatus());
}

long DevMbbiLove::dev_init(void* v)
{
    mbbiRecord* pr = (mbbiRecord*)v;
    DevLove* pDevLove = new DevMbbiLove((dbCommon*)pr,&(pr->inp));
    pDevLove->bind();
    return(pDevLove->getStatus());
}

long DevAoLove::dev_init(void* v)
{
    aoRecord* pr = (aoRecord*)v;
    DevLove* pDevLove = new DevAoLove((dbCommon*)pr,&(pr->out));
    pDevLove->bind();
    return(pDevLove->getStatus());
}

long DevBoLove::dev_init(void* v)
{
    boRecord* pr = (boRecord*)v;
    DevLove* pDevLove = new DevBoLove((dbCommon*)pr,&(pr->out));
    pDevLove->bind();
    return(pDevLove->getStatus());
}

DevLove::DevLove(dbCommon* pr,DBLINK* l)
: DevMpf(pr,l,false), address(0), signal(0), model(0)
{
    vmeio* io = (vmeio*)&(l->value);
    const char* pv = getUserParm();
    if(pv==0) {
	epicsPrintf("%s DevLove Illegal INP field\n",pr->name);
        pr->pact = TRUE;
        return;
    }
    ::sscanf(pv,"%x,%d",&address,&model);    
    signal = io->signal;
    if (signal <0 || signal > 17) {
        epicsPrintf("%s devLove Illegal INP field\n",pr->name);
        pr->pact = TRUE;
        return;
    }
    if (devLoveDebug)
        printf(" %s address : %d  signal : %d model : %d\n",
        	pr->name,address,signal,model);
}

DevAiLove::DevAiLove(dbCommon* pr,link* l)
: DevLove(pr,l)
{
}

DevBiLove::DevBiLove(dbCommon* pr,link* l)
: DevLove(pr,l)
{
}

DevMbbiLove::DevMbbiLove(dbCommon* pr,link* l)
: DevLove(pr,l)
{
}

DevAoLove::DevAoLove(dbCommon* pr,link* l)
: DevLove(pr,l)
{
}

DevBoLove::DevBoLove(dbCommon* pr,link* l)
: DevLove(pr,l)
{
}


long DevLove::startIO(dbCommon *)
{
    Int32Message *message = new Int32Message;
    message->address = address;
    message->extra = model;
    message->cmd = (cmdType) signal;
    message->value = value;
    if (devLoveDebug & 0x02) {
        printf(" Address : %d Extra: %d Cmd : %d Value: %d\n",
        	address,model,signal,value);
    }
    return(sendReply(message));
}


long DevAiLove::startIO(dbCommon *pr)
{
    if ( signal < 0 || signal > 6) {
	epicsPrintf("%s DevAiLove Invalid Signal %d\n",pr->name,signal);
	return(-1);
    }
    int status = DevLove::startIO((dbCommon*) pr);
    return(status);
}


long DevBiLove::startIO(dbCommon *pr)
{
    if (signal != 7 && signal != 10) {
	epicsPrintf("%s DevBiLove Invalid Signal %d\n",pr->name,signal);
	return(-1);
    }
    int status = DevLove::startIO((dbCommon*) pr);
    return(status);
}


long DevMbbiLove::startIO(dbCommon *pr)
{
    if (signal != 8 && signal != 9) {
	epicsPrintf("%s DevMbbiLove Invalid Signal %d\n",pr->name,signal);
	return(-1);
    }
    int status = DevLove::startIO((dbCommon*) pr);
    return(status);
}


long DevAoLove::startIO(dbCommon *pr)
{
    aoRecord* precord = (aoRecord*)pr;
    if ( signal < 11 || signal > 14) {
	epicsPrintf("%s DevAoLove Invalid Signal %d\n",pr->name,signal);
	return(-1);
    }
    value = (int) precord->val * 1000;
    int status = DevLove::startIO((dbCommon*) pr);
    return(status);
}


long DevBoLove::startIO(dbCommon *pr)
{
    boRecord* precord = (boRecord*)pr;
    if ( signal < 15 || signal > 17) {
	epicsPrintf("%s DevBoLove Invalid Signal %d\n",pr->name,signal);
	return(-1);
    }
    value = precord->val;
    int status = DevLove::startIO((dbCommon*) pr);
    return(status);
}


long DevLove::completeIO(dbCommon* pr,Message* pmessage)
{
    if ((pmessage->getType()) != messageTypeInt32) {
	epicsPrintf("%s DevLove::completeIO illegal message.\n", pr->name);
	recGblSetSevr(pr,READ_ALARM,INVALID_ALARM);
	return(0);
    }
    Int32Message *pInt32Message = (Int32Message *)pmessage;
    if (pInt32Message->status!=0) {
        recGblSetSevr(pr,READ_ALARM,INVALID_ALARM);
        if (devLoveDebug)
            printf(" Status (%d) Alarm Set !\n",pInt32Message->status);
        delete pInt32Message;
        return(-1);

    } else  {
	int decimal = (int) pInt32Message->extra;
	switch (decimal) {
	    case 1:
	        cvtFactor = 0.1;
	    break;
	    case 2:
	        cvtFactor = 0.01;
	    break;
	    case 3:
	        cvtFactor = 0.001;
	    break;
	    default:
	        cvtFactor = 1.0;
	    break;
	}
	    
        value  = (int) pInt32Message->value;
        if (devLoveDebug)
            printf(" Convert Factor is %f and Value %d \n",cvtFactor,value);
    }
    delete pInt32Message;
    return(0);
}

long DevAiLove::completeIO(dbCommon* pr,Message* pmessage)
{
    aiRecord* precord = (aiRecord*)pr;
    int status = DevLove::completeIO((dbCommon*) pr, pmessage);
    if (status!=0) {
        precord->val = 0;
        precord->udf =1;
    } else {
        precord->val = (double) value * cvtFactor;
        precord->udf = 0;
    }
    if (devLoveDebug)
        printf(" Message value is : %f \n",precord->val);

    return(MPF_NoConvert);
}


long DevBiLove::completeIO(dbCommon* pr,Message* pmessage)
{
    biRecord* precord = (biRecord*)pr;
    int status = DevLove::completeIO((dbCommon*) pr, pmessage);
    if (status!=0) {
        precord->val = 0;
        precord->udf =1;
    } else {
        precord->rval = value;
        precord->udf = 0;
    }
    if (devLoveDebug) {
        printf(" Message value is : %ld \n",precord->rval);
    }
    return(MPF_OK);
}


long DevMbbiLove::completeIO(dbCommon* pr,Message* pmessage)
{
    mbbiRecord* precord = (mbbiRecord*)pr;
    int status = DevLove::completeIO((dbCommon*) pr, pmessage);
    
    if (status!=0) {
        precord->val = 0;
        precord->udf =1;
    } else {
        precord->rval = value;
        precord->udf = 0;
    }
    if (devLoveDebug) {
        printf(" Message value is : %ld  \n",precord->rval);
    }
    return(MPF_OK);
}


long DevAoLove::completeIO(dbCommon* pr,Message* pmessage)
{
    aoRecord* precord = (aoRecord*)pr;
    int status = DevLove::completeIO((dbCommon*) pr, pmessage);
    if (status!=0) {
        precord->rbv = 0;
        precord->udf =1;
    } else {
        precord->rbv = (int) (precord->val / cvtFactor);
        precord->udf = 0;
    }
    if (devLoveDebug)
        printf(" Message value is : %ld \n",precord->rbv);

    return(MPF_NoConvert);
}


long DevBoLove::completeIO(dbCommon* pr,Message* pmessage)
{
    boRecord* precord = (boRecord*)pr;
    int status = DevLove::completeIO((dbCommon*) pr, pmessage);
    if (status!=0) {
        precord->rbv = 0;
        precord->udf =1;
    } else {
        precord->rbv = precord->val;
        precord->udf = 0;
    }
    if (devLoveDebug) {
        printf(" Message value is : %ld \n",precord->rbv);
    }
    return(MPF_OK);
}
