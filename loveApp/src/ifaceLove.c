/*

                          Argonne National Laboratory
                            APS Operations Division
                     Beamline Controls and Data Acquisition

                           Love Controller Interface


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
    Refer to ifaceLove.h for documentation.

 Source control info:
    Modified by:    $Author: dkline $
                    $Date: 2005-04-12 17:40:30 $
                    $Revision: 1.1 $

 =============================================================================
 History:
 Author: David M. Kline
 -----------------------------------------------------------------------------
 2005-Apr-01  DMK  Begin.
 -----------------------------------------------------------------------------

*/


/* EPICS base version-specific definitions (must be performed first) */
#include <epicsVersion.h>
#define LT_EPICSBASE(v,r,l) ((EPICS_VERSION<=(v)) && (EPICS_REVISION<=(r)) && (EPICS_MODIFICATION<(l)))

#if LT_EPICSBASE(3,14,6)
    #error "EPICS base must be 3.14.6 or greater"
#endif


/* System related include files */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* EPICS system related include files */
#include <iocsh.h>
#include <cantProceed.h>
#include <epicsStdio.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <epicsExport.h>


/* EPICS synApps/Asyn related include files */
#include <asynDriver.h>
#include <asynOctet.h>


/* Love controller related include files */
#include "ifaceLove.h"


/* Forward struct declations */
typedef struct Port Port;
typedef struct Inst Inst;
typedef struct Instr Instr;
typedef struct CmdTbl CmdTbl;
typedef union Readback Readback;


/* Define model enum */
typedef enum {model1600,model16A} Model;


/* Define command strings struct */
struct CmdTbl
{
    char* pname;
    asynStatus (*process)(Inst* pinst,double* value);
    const char* strings[2];
};


/* Define instrument struct */
struct Instr
{
    int decpts;
    Model modidx;
};


/* Define instance struct */
struct Inst
{
    Instr* pinfo;
    Port* pport;
    const char* pcmd;
    asynStatus (*process)(Inst* pinst,double* value);
};


/* Define port struct */
struct Port
{
    Port* pport;

    char* name;
    Instr instr[256];

    asynInterface iface;

    asynUser*     pasynUser;
    asynOctet*    pasynOctet;
    void*         pasynOctetPvt;

    char wrBuf[20];
    char rdBuf[20];
};


/* Define readback struct */
union Readback
{
    struct
    {
        char data[2];
    } State;

    struct
    {
        char info[2];
        char data[4];
    } Signed;

    struct
    {
        char stat[4];
        char data[4];
    } Value;

};


/* Forward references for command response methods */
static asynStatus getStatusValue(Inst* pinst,double* value);
static asynStatus getAlarmStatus(Inst* pinst,double* value);
static asynStatus getSignedValue(Inst* pinst,double* value);
static asynStatus getCommStatus(Inst* pinst,double* value);
static asynStatus getDecpts(Inst* pinst,double* value);
static asynStatus getData(Inst* pinst,double* value);
static asynStatus putData(Inst* pinst,double* value);
static asynStatus putCmd(Inst* pinst,double* value);


/* Define local variants */
static const CmdTbl CmdTable[] =
{
    /*Command         Process Method   1600     16A   */
    {"getValue",      getStatusValue, {  "00",   "00"}},
    {"getSP1",        getSignedValue, {"0100", "0101"}},
    {"getSP2",        getSignedValue, {"0102", "0105"}},
    {"getAlLo",       getSignedValue, {"0104", "0106"}},
    {"getAlHi",       getSignedValue, {"0105", "0107"}},
    {"getPeak",       getSignedValue, {"011A", "011D"}},
    {"getValley",     getSignedValue, {"011B", "011E"}},
    {"getAlStatus",   getAlarmStatus, {  "00",   "00"}},
    {"getAlMode",     getData,        {"0337", "031D"}},
    {"getInpType",    getData,        {"0323", "0317"}},
    {"getCommStatus", getCommStatus,  {"032A", "0324"}},
    {"getDecpts",     getDecpts,      {"0324", "031A"}},
    {"putSP1",        putData,        {"0200", "0200"}},
    {"putSP2",        putData,        {"0202", "0204"}},
    {"putAlLo",       putData,        {"0204", "0207"}},
    {"putAlHi",       putData,        {"0205", "0208"}},
    {"resetPeak",     putCmd,         {"0407", "040A"}},
    {"resetValley",   putCmd,         {"0408", "040B"}},
    {"setRemote",     putCmd,         {"0400", "0400"}},
    {"setLocal",      putCmd,         {"0401", "0401"}}
};

static Port* pports = NULL;
static const int cmdCount = (sizeof(CmdTable) / sizeof(CmdTbl));


/* Public forward references */
int ifaceLoveInit(const char* port);
int ifaceLoveRead(const char* port,int addr,const char* cmd);
int ifaceLoveWrite(const char* port,int addr,const char* cmd,double value);
int ifaceLoveConfig(const char* port,int addr,const char* model,int decpts);


/* Forward references for support methods */
static asynStatus executeCommand(Port* pport,asynUser* pasynUser);
static asynStatus processWriteResponse(Port* pport);


/* Forward references for ifaceLove interface methods */
static asynStatus setupUser(void* drvPvt,asynUser* pasynUser,const char* cmd);
static asynStatus read(void* drvPvt,asynUser* pasynUser,double* value);
static asynStatus write(void* drvPvt,asynUser* pasynUser,double value);
static ifaceLove iface = {read,write,setupUser};


/* Define macros */
#define ISOK(s)    ( asynSuccess == (s) )
#define ISNOTOK(s) ( !ISOK(s) )


/****************************************************************************
 * Define public interface methods
 ****************************************************************************/
int ifaceLoveInit(const char* port)
{
    asynStatus sts;
    int        len;
    Port*      pport;
    asynUser*  pasynUser;
    asynInterface* pasynIface;


    len = sizeof(Port) + strlen(port) + 1;
    pport = callocMustSucceed(len,sizeof(char),"ifaceLoveInit");
    pport->name = (char*)(pport + 1);
    strcpy(pport->name,port);

    pasynUser = pasynManager->createAsynUser(NULL,NULL);
    if( pasynUser == NULL )
    {
        printf("ifaceLoveInit::create asynUser failure\n");
        return( -1 );
    }

    sts = pasynManager->connectDevice(pasynUser,port,-1);
    if( ISNOTOK(sts) )
    {
        printf("ifaceLoveInit::connect device to %s failure\n",port);
        return( -1 );
    }

    pport->iface.interfaceType = ifaceLoveType;
    pport->iface.pinterface    = &iface;
    pport->iface.drvPvt        = pport;

    sts = pasynManager->interposeInterface(port,-1,&pport->iface,NULL);
    if( ISNOTOK(sts) )
    {
        printf("ifaceLoveInit::failure to interpose ifaceLove interface on %s\n", port);
        return( -1 );
    }

    pasynIface = pasynManager->findInterface(pasynUser,asynOctetType,1);
    if( pasynIface )
    {
        pport->pasynOctet    = (asynOctet*)pasynIface->pinterface;
        pport->pasynOctetPvt = pasynIface->drvPvt;
    }
    else
    {
        printf("ifaceLoveInit::failure to find asynOctet interface in %s\n",port);
        return( -1 );
    }

    pport->pasynUser = pasynUser;
    if( pports )
        pport->pport = pport;
    pports = pport;

    return( 0 );
}


int ifaceLoveRead(const char* port,int addr,const char* cmd)
{
    double value;
    asynStatus sts;
    asynUser* pasynUser;
    void* pifaceLovePvt;
    ifaceLove* pifaceLove;
    asynInterface* pasynIface;


    pasynUser = pasynManager->createAsynUser(NULL,NULL);
    if( pasynUser == NULL )
    {
        printf("ifaceLoveRead::create asynUser failure\n");
        return( -1 );
    }

    sts = pasynManager->connectDevice(pasynUser,port,addr);
    if( ISNOTOK(sts) )
    {
        printf("ifaceLoveRead::connect device to %s failure\n",port);
        return( -1 );
    }

    pasynIface = pasynManager->findInterface(pasynUser,ifaceLoveType,1);
    if( pasynIface )
    {
        pifaceLove = (ifaceLove*)pasynIface->pinterface;
        pifaceLovePvt = pasynIface->drvPvt;
    }
    else
    {
        printf("ifaceLoveRead::failure to find ifaceLoveType interface in %s\n",port);
        return( -1 );
    }

    sts = pifaceLove->setupUser(pifaceLovePvt,pasynUser,cmd);
    if( ISNOTOK(sts) )
    {
        printf("ifaceLoveRead::failure to setup user\n");
        return( -1 );
    }

    sts = pifaceLove->read(pifaceLovePvt,pasynUser,&value);
    if( ISOK(sts) )
    {
        printf("ifaceLoveRead::command %s returned %f\n",cmd,value);
    }
    else
    {
        printf("ifaceLoveRead::read command %s failure\n",cmd);
        return( -1 );
    }

    sts = pasynManager->disconnect(pasynUser);
    if( ISNOTOK(sts) )
    {
        printf("ifaceLoveRead::disconnect failure\n");
        return( -1 );
    }

    sts = pasynManager->freeAsynUser(pasynUser);
    if( ISNOTOK(sts) )
    {
        printf("ifaceLoveRead::freeAsynUser failure\n");
        return( -1 );
    }

    return( 0 );
}


int ifaceLoveWrite(const char* port,int addr,const char* cmd,double value)
{
    asynStatus sts;
    asynUser* pasynUser;
    void* pifaceLovePvt;
    ifaceLove* pifaceLove;
    asynInterface* pasynIface;


    pasynUser = pasynManager->createAsynUser(NULL,NULL);
    if( pasynUser == NULL )
    {
        printf("ifaceLoveWrite::create asynUser failure\n");
        return( -1 );
    }

    sts = pasynManager->connectDevice(pasynUser,port,addr);
    if( ISNOTOK(sts) )
    {
        printf("ifaceLoveWrite::connect device to %s failure\n",port);
        return( -1 );
    }

    pasynIface = pasynManager->findInterface(pasynUser,ifaceLoveType,1);
    if( pasynIface )
    {
        pifaceLove = (ifaceLove*)pasynIface->pinterface;
        pifaceLovePvt = pasynIface->drvPvt;
    }
    else
    {
        printf("ifaceLoveWrite::failure to find ifaceLoveType interface in %s\n",port);
        return( -1 );
    }

    sts = pifaceLove->setupUser(pifaceLovePvt,pasynUser,cmd);
    if( ISNOTOK(sts) )
    {
        printf("ifaceLoveWrite::failure to setup user\n");
        return( -1 );
    }

    sts  = pifaceLove->write(pifaceLovePvt,pasynUser,value);
    if( ISOK(sts) )
        printf("ifaceLoveWrite::command %s wrote %f\n",cmd,value);
    else
    {
        printf("ifaceLoveWrite::read command %s failure\n",cmd);
        return( -1 );
    }

    sts = pasynManager->disconnect(pasynUser);
    if( ISNOTOK(sts) )
    {
        printf("ifaceLoveWrite::disconnect failure\n");
        return( -1 );
    }

    sts = pasynManager->freeAsynUser(pasynUser);
    if( ISNOTOK(sts) )
    {
        printf("ifaceLoveWrite::freeAsynUser failure\n");
        return( -1 );
    }

    return( 0 );
}


int ifaceLoveConfig(const char* port,int addr,const char* model,int decpts)
{
    Port* pport;

    for( pport = pports; pport; pport = pport->pport )
        if( epicsStrCaseCmp(pport->name,port) == 0 )
        {
            pport->instr[addr-1].decpts = decpts;
            if( epicsStrCaseCmp("1600",model) == 0 )
                pport->instr[addr-1].modidx = model1600;
            else if( epicsStrCaseCmp("16A",model) == 0 )
                pport->instr[addr-1].modidx = model16A;
            else
            {
                printf("ifaceLoveConfig::unsupported model \"%s\"",model);
                return( -1 );
            }

            return( 0 );
        }

    printf("ifaceLoveConfig::failure to locate port %s\n",port);
    return( -1 );
}


/****************************************************************************
 * Define private interface ifaceLove methods
 ****************************************************************************/
static asynStatus read(void* drvPvt,asynUser* pasynUser,double* value)
{
    asynStatus sts;
    Port* pport = (Port*)drvPvt;
    Inst* pinst = (Inst*)pasynUser->drvUser;


    asynPrint(pasynUser,ASYN_TRACE_FLOW,"ifaceLove::read\n");

    sprintf(pport->wrBuf,"%s",pinst->pcmd);

    sts = executeCommand(pport,pasynUser);
    if( ISNOTOK(sts) )
    {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,"%s error %s",pport->name,pport->pasynUser->errorMessage);
        return( sts );
    }

    sts = pinst->process(pinst,value);
    if( ISNOTOK(sts) )
    {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,"%s error %s",pport->name,pport->pasynUser->errorMessage);
        return( sts );
    }

    return( asynSuccess );
}


static asynStatus write(void* drvPvt,asynUser* pasynUser,double value)
{
    asynStatus sts;
    Port* pport = (Port*)drvPvt;
    Inst* pinst = (Inst*)pasynUser->drvUser;


    asynPrint(pasynUser,ASYN_TRACE_FLOW,"ifaceLove::write\n");

    sts = pinst->process(pinst,&value);
    if( ISNOTOK(sts) )
    {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,"%s error %s",pport->name,pport->pasynUser->errorMessage);
        return( sts );
    }

    sts = executeCommand(pport,pasynUser);
    if( ISNOTOK(sts) )
    {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,"%s error %s",pport->name,pport->pasynUser->errorMessage);
        return( sts );
    }

    sts = processWriteResponse(pport);
    if( ISNOTOK(sts) )
    {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,"%s error %s",pport->name,pport->pasynUser->errorMessage);
        return( sts );
    }

    return( asynSuccess );
}


static asynStatus setupUser(void* drvPvt,asynUser* pasynUser,const char* cmd)
{
    int i,addr;
    asynStatus sts;
    Inst* pinst;
    Port* pport = (Port*)drvPvt;


    asynPrint(pasynUser,ASYN_TRACE_FLOW,"ifaceLove::setup\n");

    sts = pasynManager->getAddr(pasynUser,&addr);
    if( ISNOTOK(sts) )
        return( sts );

    for( i = 0; i < cmdCount; ++i )
    {
        if( epicsStrCaseCmp(CmdTable[i].pname,cmd) == 0 )
        {
            pinst = callocMustSucceed(sizeof(Inst),sizeof(char),"ifaceLove::setup");
            pinst->pport = pport;
            pinst->pinfo = &pport->instr[addr-1];
            pinst->process = CmdTable[i].process;
            pinst->pcmd = CmdTable[i].strings[pinst->pinfo->modidx];

            pasynUser->drvUser = (void*)pinst;

            return( asynSuccess );
        }
    }

    epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,"failure to find command %s",cmd);
    return( asynError );
}


/****************************************************************************
 * Define private support methods
 ****************************************************************************/
static asynStatus executeCommand(Port* pport,asynUser* pasynUser)
{
    int i;
    asynStatus sts;
    size_t eomReason;
    size_t bytesRead;
    size_t bytesToWrite;
    size_t bytesWritten;


    asynPrint(pasynUser,ASYN_TRACE_FLOW,"ifaceLove::executeCommand\n" );

    bytesToWrite = strlen(pport->wrBuf);
    pasynUser->timeout = 1.0;

    for( i = 0; i < 3; ++i )
    {
        epicsThreadSleep( 0.04 );

        sts = pport->pasynOctet->write(pport->pasynOctetPvt,pasynUser,pport->wrBuf,bytesToWrite,&bytesWritten);
        if( ISOK(sts) )
            asynPrint(pasynUser,ASYN_TRACEIO_FILTER,"ifaceLove::executeCommand write \"%*s\"\n",bytesToWrite,pport->wrBuf);
        else
        {
            if( sts == asynTimeout )
            {
                asynPrint(pasynUser,ASYN_TRACE_ERROR,"ifaceLove::executeCommand write timeout, retrying\n");
                continue;
            }

            asynPrint(pasynUser,ASYN_TRACE_ERROR,"ifaceLove::executeCommand write failure\n");
            return( sts );
        }

        sts = pport->pasynOctet->read(pport->pasynOctetPvt,pasynUser,pport->rdBuf,sizeof(pport->rdBuf),&bytesRead,&eomReason);
        if( ISOK(sts) )
            asynPrint(pasynUser,ASYN_TRACEIO_FILTER,"ifaceLove::executeCommand read \"%*s\"\n",bytesRead,pport->rdBuf);
        else
        {
            if( sts == asynTimeout )
            {
                asynPrint(pasynUser,ASYN_TRACE_ERROR,"ifaceLove::executeCommand read timeout, retrying\n");
                continue;
            }

            asynPrint(pasynUser,ASYN_TRACE_ERROR,"ifaceLove::executeCommand read failure\n");
            return( sts );
        }

        pport->rdBuf[bytesRead] = '\0';
        return( asynSuccess );
    }

    asynPrint(pasynUser,ASYN_TRACE_ERROR,"ifaceLove::executeCommand retries exceeded\n");
    return( asynError );
}


static asynStatus processWriteResponse(Port* pport)
{
    int resp;


    asynPrint(pport->pasynUser,ASYN_TRACE_FLOW,"ifaceLove::processWriteResponse\n" );

    sscanf(pport->rdBuf,"%2d",&resp);
    if( resp )
    {
        asynPrint(pport->pasynUser,ASYN_TRACE_ERROR,"ifaceLove::processWriteResponse write command failed\n" );
        return( asynError );
    }

    return( asynSuccess );
}


/****************************************************************************
 * Define private command / response methods
 ****************************************************************************/
static asynStatus getStatusValue(Inst* pinst,double* value)
{
    int sign,data;
    Port* pport = pinst->pport;
    Readback* prb = (Readback*)pport->rdBuf;


    asynPrint(pport->pasynUser,ASYN_TRACE_FLOW,"ifaceLove::getStatusValue\n" );

    sscanf(prb->Value.stat,"%4x",&sign);
    sscanf(prb->Value.data,"%4d",&data);

    *value = (double)data * (1 / pow(10,pinst->pinfo->decpts));
    if( sign & 0x0001 )
        *value *= -1;

    return( asynSuccess );
}


static asynStatus getAlarmStatus(Inst* pinst,double* value)
{
    int data;
    Port* pport = pinst->pport;
    Readback* prb = (Readback*)pport->rdBuf;


    asynPrint(pport->pasynUser,ASYN_TRACE_FLOW,"ifaceLove::getAlarmStatus\n" );

    sscanf(prb->Value.stat,"%4x",&data);
    if( data & 0x0800 )
        *value = 1.0;
    else
        *value = 0.0;

    return( asynSuccess );
}


static asynStatus getSignedValue(Inst* pinst,double* value)
{
    int sts,data;
    Port* pport = pinst->pport;
    Readback* prb = (Readback*)pport->rdBuf;


    asynPrint(pport->pasynUser,ASYN_TRACE_FLOW,"ifaceLove::getSignedValue\n" );

    sscanf(prb->Signed.data,"%4d",&data);
    *value = (double)data * (1 / pow(10,pinst->pinfo->decpts));

    if( pinst->pinfo->modidx == model1600 )
    {
        sscanf(prb->Signed.info,"%2d",&sts);
        if( sts )
            *value *= -1;
    }
    else
    {
        sscanf(prb->Signed.info,"%2x",&sts);
        if( sts & 0x0001 )
            *value *= -1;
    }

    return( asynSuccess );
}


static asynStatus getCommStatus(Inst* pinst,double* value)
{
    int data;
    Port* pport = pinst->pport;
    Readback* prb = (Readback*)pport->rdBuf;


    asynPrint(pport->pasynUser,ASYN_TRACE_FLOW,"ifaceLove::getCommStatus\n" );

    sscanf(prb->State.data,"%2x",&data);
    if( data )
        *value = 1.0;
    else
        *value = 0.0;

    return( asynSuccess );
}


static asynStatus getData(Inst* pinst,double* value)
{
    int data;
    Port* pport = pinst->pport;
    Readback* prb = (Readback*)pport->rdBuf;


    asynPrint(pport->pasynUser,ASYN_TRACE_FLOW,"ifaceLove::getData\n" );

    sscanf(prb->State.data,"%2x",&data);
    *value = (double)data;

    return( asynSuccess );
}


static asynStatus getDecpts(Inst* pinst,double* value)
{
    int addr;
    asynStatus sts;
    Port* pport = pinst->pport;


    asynPrint(pport->pasynUser,ASYN_TRACE_FLOW,"ifaceLove::getDecpts\n" );

    sts = getData(pinst,value);
    if( ISNOTOK(sts) )
        return( sts );

    sts = pasynManager->getAddr(pport->pasynUser,&addr);
    if( ISNOTOK(sts) )
        return( sts );

    pport->instr[addr-1].decpts = (int)*value;
    return( asynSuccess );
}


static asynStatus putData(Inst* pinst,double* value)
{
    int sign,data;
    Port* pport = pinst->pport;


    asynPrint(pport->pasynUser,ASYN_TRACE_FLOW,"ifaceLove::putData\n" );

    if( *value < 0 )
    {
        sign = 0xFF;
        *value *= -1;
    }
    else
        sign = 0;

    data = (int)(*value * pow(10,pinst->pinfo->decpts));
    sprintf(pport->wrBuf,"%s%4.4d%2.2X",pinst->pcmd,data,sign);

    return( asynSuccess );
}


static asynStatus putCmd(Inst* pinst,double* value)
{
    Port* pport = pinst->pport;


    asynPrint(pport->pasynUser,ASYN_TRACE_FLOW,"ifaceLove::putCmd\n" );
    sprintf(pport->wrBuf,"%s",pinst->pcmd);
    return( asynSuccess );
}


/****************************************************************************
 * Register public methods
 ****************************************************************************/

/* Definitions for ifaceLoveInit method */
static const iocshArg ifaceLoveInitArg0 = {"port",iocshArgString};
static const iocshArg* ifaceLoveInitArgs[] = {&ifaceLoveInitArg0};
static const iocshFuncDef ifaceLoveInitFuncDef = {"ifaceLoveInit",1,ifaceLoveInitArgs};
static void ifaceLoveInitCallFunc(const iocshArgBuf* args)
{
    ifaceLoveInit(args[0].sval);
}

/* Definitions for ifaceLoveRead method */
static const iocshArg ifaceLoveReadArg0 = {"port",iocshArgString};
static const iocshArg ifaceLoveReadArg1 = {"addr",iocshArgInt};
static const iocshArg ifaceLoveReadArg2 = {"cmd",iocshArgString};
static const iocshArg* ifaceLoveReadArgs[] = {&ifaceLoveReadArg0,&ifaceLoveReadArg1,&ifaceLoveReadArg2};
static const iocshFuncDef ifaceLoveReadFuncDef = {"ifaceLoveRead",3,ifaceLoveReadArgs};
static void ifaceLoveReadCallFunc(const iocshArgBuf* args)
{
    ifaceLoveRead(args[0].sval,args[1].ival,args[2].sval);
}

/* Definitions for ifaceLoveWrite method */
static const iocshArg ifaceLoveWriteArg0 = {"port",iocshArgString};
static const iocshArg ifaceLoveWriteArg1 = {"addr",iocshArgInt};
static const iocshArg ifaceLoveWriteArg2 = {"cmd",iocshArgString};
static const iocshArg ifaceLoveWriteArg3 = {"cmd",iocshArgDouble};
static const iocshArg* ifaceLoveWriteArgs[] = {&ifaceLoveWriteArg0,&ifaceLoveWriteArg1,&ifaceLoveWriteArg2,&ifaceLoveWriteArg3};
static const iocshFuncDef ifaceLoveWriteFuncDef = {"ifaceLoveRead",4,ifaceLoveWriteArgs};
static void ifaceLoveWriteCallFunc(const iocshArgBuf* args)
{
    ifaceLoveWrite(args[0].sval,args[1].ival,args[2].sval,args[3].dval);
}

/* Definitions for ifaceLoveConfig method */
static const iocshArg ifaceLoveConfigArg0 = {"port",iocshArgString};
static const iocshArg ifaceLoveConfigArg1 = {"addr",iocshArgInt};
static const iocshArg ifaceLoveConfigArg2 = {"mod",iocshArgString};
static const iocshArg ifaceLoveConfigArg3 = {"decpts",iocshArgInt};
static const iocshArg* ifaceLoveConfigArgs[] = {&ifaceLoveConfigArg0,&ifaceLoveConfigArg1,&ifaceLoveConfigArg2,&ifaceLoveConfigArg3};
static const iocshFuncDef ifaceLoveConfigFuncDef = {"ifaceLoveConfig",4,ifaceLoveConfigArgs};
static void ifaceLoveConfigCallFunc(const iocshArgBuf* args)
{
    ifaceLoveConfig(args[0].sval,args[1].ival,args[2].sval,args[3].ival);
}

/* Registration method */
static void registerLoveIfaces(void)
{
    static int firstTime = 1;

    if( firstTime )
    {
        firstTime = 0;
        iocshRegister(&ifaceLoveInitFuncDef,ifaceLoveInitCallFunc);
        iocshRegister(&ifaceLoveReadFuncDef,ifaceLoveReadCallFunc);
        iocshRegister(&ifaceLoveWriteFuncDef,ifaceLoveWriteCallFunc);
        iocshRegister(&ifaceLoveConfigFuncDef,ifaceLoveConfigCallFunc);
    }
}
epicsExportRegistrar( registerLoveIfaces );
