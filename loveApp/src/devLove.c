/*

                          Argonne National Laboratory
                            APS Operations Division
                     Beamline Controls and Data Acquisition

                         Love Controller Device Support



 -----------------------------------------------------------------------------
                                COPYRIGHT NOTICE
 -----------------------------------------------------------------------------
   Copyright (c) 2002 The University of Chicago, as Operator of Argonne
      National Laboratory.
   Copyright (c) 2002 The Regents of the University of California, as
      Operator of Los Alamos National Laboratory.
   Synapps Versions 4-5
   and higher are distributed subject to a Software License Agreement found
   in file LICENSE that is included with this distribution.
 -----------------------------------------------------------------------------

 Description
    This module represents the device support for the Love controllers. It
    resides below record support and interacts with the ifaceLove interface.

    The database definition file (.dbd) provides the mapping between the
    record type and the device support table entries. The following
    provides this mapping.
        device(ai,INST_IO,devAiLove,"devLove")
        device(ao,INST_IO,devAoLove,"devLove")
        device(bi,INST_IO,devBiLove,"devLove")
        device(bo,INST_IO,devBoLove,"devLove")
        device(mbbi,INST_IO,devMbbiLove,"devLove")

    The "INST_IO" address type is used to indicate the format of the INP and
    OUT field in the database (.db) file. The format is defined by Asyn.
        field(INP,"@asyn(port,addr) cmd")
        field(OUT,"@asyn(port,addr) cmd")

        Where:
            port - Port name (i.e. "L0")
            addr - Love controller address in hex (i.e. 0x32)
            cmd  - Love controller string command (listed in ifaceLove.c)

    The DTYP field specifies the device support as shown in the here.
        field(DTYP, "devLove" )

 Developer notes:

 Source control info:
    Modified by:    $Author: dkline $
                    $Date: 2005-04-12 17:40:30 $
                    $Revision: 1.1 $

 =============================================================================
 History:
 Author: David M. Kline
 -----------------------------------------------------------------------------
 2005-Apr-08  DMK  Initial development for device support.
 -----------------------------------------------------------------------------

*/


/* EPICS base version-specific definitions (must be performed first) */
#include <epicsVersion.h>
#define LT_EPICSBASE(v,r,l)  \
   ((EPICS_VERSION<=(v)) && (EPICS_REVISION<=(r)) && (EPICS_MODIFICATION<(l)))

#if LT_EPICSBASE(3,14,6)
    #error "EPICS base must be 3.14.6 or greater"
#endif


/* System related include files */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* EPICS system related include files */
#include <link.h>
#include <alarm.h>
#include <devLib.h>
#include <devSup.h>
#include <recSup.h>
#include <recGbl.h>
#include <epicsPrint.h>
#include <epicsThread.h>
#include <epicsExport.h>
#include <cantProceed.h>
#include <asynEpicsUtils.h>


/* EPICS Asyn related include files */
#include <asynDriver.h>


/* EPICS record processing related include files */
#include <dbLock.h>
#include <dbCommon.h>
#include <aoRecord.h>
#include <aiRecord.h>
#include <boRecord.h>
#include <biRecord.h>
#include <mbbiRecord.h>


/* Love controller related include files */
#include "ifaceLove.h"


/* Structure forward references */
typedef struct Love Love;
typedef struct Dset Dset;


/* Define general symbolic constants */
#define STS__OK         (     0 )   /* OK */
#define STS__OKNOVAL    (     2 )   /* OK - do not modify VAL */
#define STS__ERROR      ( ERROR )   /* FAILURE */


/* Define record type enum */
typedef enum {ai,ao,bi,bo,mbbi} Rec;


/* Declare record instance struct */
struct Love
{
    Love*      pnext;

    asynStatus sts;
    int        count;
    double     value;
    dbCommon*  prec;
    Rec        rec;

    int        addr;
    char*      pport;
    char*      pcmd;

    ifaceLove* piface;
    void*      pifacePvt;
    asynUser*  pasynUser;
};


/* Declare device support struct */
struct Dset
{
    long      number;
    DEVSUPFUN report;
    DEVSUPFUN init;
    DEVSUPFUN init_record;
    DEVSUPFUN get_ioint_info;
    DEVSUPFUN method;
    DEVSUPFUN specialLinconv;
};


/* Define local variants */
static Love* ploves;
static unsigned long readCount;
static unsigned long writCount;
static unsigned long instCount;


/* Forward references for support methods */
static asynStatus recordInit(dbCommon* prec,DBLINK* pio,void (*callback)(asynUser* pasynUser),Rec rec);
static asynStatus queueIt(dbCommon* prec);
static void myCallback(asynUser* pasynUser);


/* Forward references for record-processing methods */
static long report(int level);
static long ai__init(aiRecord* pai);
static long ai__read(aiRecord* pai);
static long ao__init(aoRecord* pao);
static long ao__write(aoRecord* pao);
static long bi__init(biRecord* pbi);
static long bi__read(biRecord* pbi);
static long bo__init(boRecord* pbo);
static long bo__write(boRecord* pbo);
static long mbbi__init(mbbiRecord* pmbbi);
static long mbbi__read(mbbiRecord* pmbbi);


/* Define Dset's */
static Dset devAiLove = {6,report,NULL,ai__init,NULL,ai__read,NULL};
static Dset devAoLove = {6,NULL,NULL,ao__init,NULL,ao__write,NULL};
static Dset devBiLove = {5,NULL,NULL,bi__init,NULL,bi__read,NULL};
static Dset devBoLove = {5,NULL,NULL,bo__init,NULL,bo__write,NULL};
static Dset devMbbiLove = {5,NULL,NULL,mbbi__init,NULL,mbbi__read,NULL};


/* Publish Dset's to EPICS */
epicsExportAddress(dset,devAiLove);
epicsExportAddress(dset,devAoLove);
epicsExportAddress(dset,devBiLove);
epicsExportAddress(dset,devBoLove);
epicsExportAddress(dset,devMbbiLove);


/* Define macros */
#define ISOK(s)      (asynSuccess==(s))
#define ISNOTOK(s)   (!ISOK(s))
#define ISREADREC(r) ((r)==ai||(r)==bi||(r)==mbbi)


/****************************************************************************
 * Define private support methods
 ****************************************************************************/
static asynStatus recordInit(dbCommon* prec,DBLINK* pio,void (*callback)(asynUser* pasynUser),Rec rec)
{
    Love* plove;
    asynStatus sts;
    asynUser* pasynUser;
    asynInterface* pasynIface;


    plove = callocMustSucceed(1,sizeof(Love),"devLove::recordInit");

    pasynUser = pasynManager->createAsynUser(callback,NULL);
    if( pasynUser == NULL )
    {
        asynPrint(pasynUser,ASYN_TRACE_ERROR,"devLove::recordInit, failure to create asynUser - %s\n",prec->name);
        free(plove);
        return( asynError );
    }

    /* Parse INP/OUT string */
    sts = pasynEpicsUtils->parseLink(pasynUser,pio,&plove->pport,&plove->addr,&plove->pcmd);
    if( ISNOTOK(sts) )
    {
        asynPrint(pasynUser,ASYN_TRACE_ERROR,"devLove::recordInit, parse link failure - %s\n",prec->name);
        pasynManager->freeAsynUser(pasynUser);
        free(plove);
        return( sts );
    }

    sts = pasynManager->connectDevice(pasynUser,plove->pport,plove->addr);
    if( ISNOTOK(sts) )
    {
        asynPrint(pasynUser,ASYN_TRACE_ERROR,"devLove::recordInit, connect device failure - %s\n",prec->name);
        pasynManager->freeAsynUser(pasynUser);
        free(plove);
        return( sts );
    }

    pasynIface = pasynManager->findInterface(pasynUser,ifaceLoveType,1);
    if( pasynIface )
    {
        plove->piface = (ifaceLove*)pasynIface->pinterface;
        plove->pifacePvt = pasynIface->drvPvt;
    }
    else
    {
        asynPrint(pasynUser,ASYN_TRACE_ERROR,"devLove::recordInit, find ifaceLoveType interface failure - %s\n",prec->name);
        pasynManager->freeAsynUser(pasynUser);
        free(plove);
        return( sts );
    }

    /* Setup interface */
    sts = plove->piface->setupUser(plove->pifacePvt,pasynUser,plove->pcmd);
    if( ISNOTOK(sts) )
    {
        asynPrint(pasynUser,ASYN_TRACE_ERROR,"devLove::recordInit, failure to setup ifaceLoveType interface for cmd %s - %s\n",plove->pcmd,prec->name);
        pasynManager->freeAsynUser(pasynUser);
        free(plove);
        return( sts );
    }

    ++instCount;

    prec->dpvt = plove;
    pasynUser->userPvt = prec;
    plove->sts = asynSuccess;
    plove->count = 0;
    plove->rec = rec;
    plove->prec = prec;
    plove->pasynUser = pasynUser;

    if( ploves )
        plove->pnext = ploves;
    ploves = plove;

    asynPrint(pasynUser,ASYN_TRACE_FLOW,"devLove::recordInit, name=\"%s\", port=\"%s\", addr=%d, cmd=\"%s\"\n",prec->name,plove->pport,plove->addr,plove->pcmd);
    return( asynSuccess );
}


static asynStatus queueIt(dbCommon* prec)
{
    Love* plove = (Love*)prec->dpvt;

    asynPrint(plove->pasynUser,ASYN_TRACE_FLOW,"devLove::queueIt\n");
    prec->pact = 1;
    return( pasynManager->queueRequest(plove->pasynUser,0,0) );
}


static void myCallback(asynUser* pasynUser)
{
    Love* plove;
    dbCommon* prec;


    asynPrint(pasynUser,ASYN_TRACE_FLOW,"devLove::myCallback\n");

    prec = (dbCommon*)pasynUser->userPvt;
    plove = (Love*)prec->dpvt;

    if( ISREADREC(plove->rec) )
        plove->sts = plove->piface->read(plove->pifacePvt,plove->pasynUser,&plove->value);
    else
        plove->sts = plove->piface->write(plove->pifacePvt,plove->pasynUser,plove->value);

    if( ISNOTOK(plove->sts) )
    {
        asynPrint(plove->pasynUser,ASYN_TRACE_ERROR,"devLove::myCallback failure\n");
        return;
    }

    dbScanLock(prec);
    prec->rset->process(prec);
    dbScanUnlock(prec);
}


/****************************************************************************
 * Define private Dset methods
 ****************************************************************************/
static long report(int level)
{
    Love* plove;
    static char* state[] = {"Y","N"};
    static char* rectyp[] = {"ai  ","ao  ","bi  ","bo  ","mbbi"};
    static char* status[] = {"OK  ","TMO ","OVR ","ERR "};


    printf("\nLove Controller Configuration\n" );
    printf("\tInterest level        - %d\n",level);
    printf("\tRecord read count     - %d\n",(int)readCount);
    printf("\tRecord write count    - %d\n",(int)writCount);
    printf("\tRecord instance count - %d\n",(int)instCount);
    printf("\tEPICS release version - %s\n",epicsReleaseVersion);

    printf("\n\tPort  Addr  Command        Sts  Rec  PACT  Count  PV\n");
    for(plove=ploves; plove; plove=plove->pnext)
    {
        printf("\t%-4.4s  0x%2.2X  %-14.14s ",plove->pport,plove->addr,plove->pcmd);
        printf("%s %s %s ",status[plove->sts],rectyp[plove->rec],state[plove->prec->pact]);
        printf("    %06d %s\n",plove->count,plove->prec->name);
    }

    return( 0 );
}


static long ai__init(aiRecord* pai)
{
    return( recordInit((dbCommon*)pai,&pai->inp,myCallback,ai) );
}


static long ai__read(aiRecord* pai)
{
    Love* plove = (Love*)pai->dpvt;


    if( plove == NULL )
    {
       pai->pact = 1;
       asynPrint(plove->pasynUser,ASYN_TRACE_ERROR,"devLove::ai__read dpvt is NULL \"%s\"\n",pai->name);

       return( STS__OKNOVAL );
    }

    if( pai->pact == 0 )
    {
        queueIt((dbCommon*)pai);
        return( STS__OKNOVAL );
    }

    if( pai->pact == 1 )
    {
        if( ISOK(plove->sts) )
        {
            pai->val = plove->value;
            pai->udf = 0;
        }
        else
        {
            pai->val = 0.0;
            pai->udf = 1;

            recGblSetSevr(pai,READ_ALARM,INVALID_ALARM);
            asynPrint(plove->pasynUser,ASYN_TRACE_ERROR,"devLove::ai__read failure in \"%s\"\n",pai->name);
        }

        ++readCount;
        ++plove->count;
    }

    return( STS__OKNOVAL );
}


static long ao__init(aoRecord* pao)
{
    return( recordInit((dbCommon*)pao,&pao->out,myCallback,ao) );
}


static long ao__write(aoRecord* pao)
{
    Love* plove = (Love*)pao->dpvt;


    if( plove == NULL )
    {
       pao->pact = 1;
       asynPrint(plove->pasynUser,ASYN_TRACE_ERROR,"devLove::ao__write dpvt is NULL \"%s\"\n",pao->name);

       return( STS__OKNOVAL );
    }

    if( pao->pact == 0 )
    {
        plove->value = pao->val;
        queueIt((dbCommon*)pao);

        return( STS__OKNOVAL );
    }

    if( pao->pact == 1 )
    {
        if( ISOK(plove->sts) )
        {
            pao->rbv = (epicsInt32)plove->value;
            pao->udf = 0;
        }
        else
        {
            pao->rbv = 0;
            pao->udf = 1;

            recGblSetSevr(pao,WRITE_ALARM,INVALID_ALARM);
            asynPrint(plove->pasynUser,ASYN_TRACE_ERROR,"devLove::ao__write failure in \"%s\"\n",pao->name);
        }

        ++writCount;
        ++plove->count;
    }

    return( STS__OKNOVAL );
}


static long bi__init(biRecord* pbi)
{
    return( recordInit((dbCommon*)pbi,&pbi->inp,myCallback,bi) );
}


static long bi__read(biRecord* pbi)
{
    Love* plove = (Love*)pbi->dpvt;


    if( plove == NULL )
    {
       pbi->pact = 1;
       asynPrint(plove->pasynUser,ASYN_TRACE_ERROR,"devLove::bi__read dpvt is NULL \"%s\"\n",pbi->name);

       return( STS__OKNOVAL );
    }

    if( pbi->pact == 0 )
    {
        queueIt((dbCommon*)pbi);
        return( STS__OK );
    }

    if( pbi->pact == 1 )
    {
        if( ISOK(plove->sts) )
        {
            pbi->rval = (unsigned long)plove->value;
            pbi->udf  = 0;
        }
        else
        {
            pbi->val  = 0;
            pbi->udf  = 1;

            recGblSetSevr(pbi,READ_ALARM,INVALID_ALARM);
            asynPrint(plove->pasynUser,ASYN_TRACE_ERROR,"devLove::bi__read failure in \"%s\"\n",pbi->name);
        }

        ++readCount;
        ++plove->count;
    }

    return( STS__OK );
}


static long bo__init(boRecord* pbo)
{
    return( recordInit((dbCommon*)pbo,&pbo->out,myCallback,bo) );
}


static long bo__write(boRecord* pbo)
{
    Love* plove = (Love*)pbo->dpvt;


    if( plove == NULL )
    {
       pbo->pact = 1;
       asynPrint(plove->pasynUser,ASYN_TRACE_ERROR,"devLove::bo__write dpvt is NULL \"%s\"\n",pbo->name);

       return( STS__OKNOVAL );
    }

    if( pbo->pact == 0 )
    {
        queueIt((dbCommon*)pbo);
        return( STS__OK );
    }

    if( pbo->pact == 1 )
    {
        if( ISOK(plove->sts) )
        {
            pbo->rbv = (unsigned long)plove->value;
            pbo->udf  = 0;
        }
        else
        {
            pbo->rbv  = 0;
            pbo->udf  = 1;

            recGblSetSevr(pbo,WRITE_ALARM,INVALID_ALARM);
            asynPrint(plove->pasynUser,ASYN_TRACE_ERROR,"devLove::bo__write failure in \"%s\"\n",pbo->name);
        }

        ++writCount;
        ++plove->count;
    }

    return( STS__OK );
}


static long mbbi__init(mbbiRecord* pmbbi)
{
    return( recordInit((dbCommon*)pmbbi,&pmbbi->inp,myCallback,mbbi) );
}


static long mbbi__read(mbbiRecord* pmbbi)
{
    Love* plove = (Love*)pmbbi->dpvt;


    if( plove == NULL )
    {
       pmbbi->pact = 1;
       asynPrint(plove->pasynUser,ASYN_TRACE_ERROR,"devLove::mbbi__read dpvt is NULL \"%s\"\n",pmbbi->name);

       return( STS__OKNOVAL );
    }

    if( pmbbi->pact == 0 )
    {
        queueIt((dbCommon*)pmbbi);
        return( STS__OK );
    }

    if( pmbbi->pact == 1 )
    {
        if( ISOK(plove->sts) )
        {
            pmbbi->rval = (unsigned long)plove->value;
            pmbbi->udf  = 0;
        }
        else
        {
            pmbbi->val = 0;
            pmbbi->udf = 1;

            recGblSetSevr(pmbbi,READ_ALARM,INVALID_ALARM);
            asynPrint(plove->pasynUser,ASYN_TRACE_ERROR,"devLove::mbbi__read failure in \"%s\"\n",pmbbi->name);
        }

        ++readCount;
        ++plove->count;
    }

    return( STS__OK );
}
