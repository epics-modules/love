/*

                          Argonne National Laboratory
                            APS Operations Division
                     Beamline Controls and Data Acquisition

                          Love Controller Port Driver



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
    This module provides support for a multiple device port driver. To
    initialize the driver, the method drvLoveInit is called from the
    startup script with the following calling sequence.

        drvLoveInit( lovPort, serPort, serAddr )

        Where:
            lovPort - Love port driver name (i.e. "L0" )
            serPort - Serial port driver name (i.e. "S0" )
            serAddr - Serial port driver address

    Prior to initializing the drvLove driver, the serial port driver
    (drvAsynSerialPort) must be initialized.

    The method dbior can be called from the IOC shell to display the current
    status of the driver and well as individual controllers.


 Developer notes:
    1) Command from device support:
       Read  - <ADDR><COMMAND>
       Write - <ADDR><COMMAND><DATA>
    2) Command from Interpose layer to Love controller:
       Read  - <STX>L<ADDR><COMMAND><CS><ETX>
       Write - <STX>L<ADDR><COMMAND><DATA><CS><ETX>
    3) Love controller command response to Interpose layer:
       Read  - <STX>L<ADDR><COMMAND><DATA><CS><ACK>
       Write - <STX>L<ADDR><COMMAND><CS><ACK>
       Error - <STX>L<ADDR>N<CODE><ACK>

 Source control info:
    Modified by:    $Author: dkline $
                    $Date: 2005-04-12 17:40:30 $
                    $Revision: 1.1 $

 =============================================================================
 History:
 Author: David M. Kline
 -----------------------------------------------------------------------------
 2005-Mar-25  DMK  Derived from lovelink interpose interface.
 2005-Mar-29  DMK  Initial development of the port driver complete.
 -----------------------------------------------------------------------------

*/


/* EPICS base version-specific definitions (must be performed first) */
#include <epicsVersion.h>
#define LT_EPICSBASE(v,r,l) ((EPICS_VERSION<=(v)) && (EPICS_REVISION<=(r)) && (EPICS_MODIFICATION<(l)))


/* Evaluate EPICS base */
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
#include <epicsExport.h>


/* EPICS synApps/Asyn related include files */
#include <asynDriver.h>
#include <asynOctet.h>


/* Define symbolic constants */
#define K_INSTRMAX ( 256 )
#define K_COMTMO   ( 1.0 )


/* Forward struct declarations */
typedef struct Serport Serport;
typedef struct Lovport Lovport;
typedef struct Instrinfo Instrinfo;


/* Declare instrument info structure */
struct Instrinfo
{
    int isConnected;
};


/* Declare serial port structure */
struct Serport
{
    char*       name;
    int         addr;
    int         multiDevice;
    int         canBlock;
    int         autoConnect;
    int         isConnected;
    asynUser*   pasynUser;
    asynCommon* pasynCommon;
    void*       commonPvt;
    asynOctet*  pasynOctet;
    void*       octetPvt;
};


/* Declare love port structure */
struct Lovport
{
    Lovport*      pport;

    char*         name;
    int           isConnected;
    asynInterface octet;
    asynInterface common;
    asynUser*     pasynUser;
    Serport*      pserport;
    char          outMsg[20];
    char          inpMsg[20];
    char          tmpMsg[20];
    Instrinfo     instr[K_INSTRMAX];
};


/* Define local variants */
static Lovport* prList = NULL;
static char inpEos = '\006';    /* ACK (^B) */
static char outEos = '\003';    /* ETX (^C) */

static char* errCodes[] =
{
/* 00 */  "Not used.",
/* 01 */  "Undefined command. Command not within acceptable range.",
/* 02 */  "Checksum error on received data from Host.",
/* 03 */  "Command not performed by instrument.",
/* 04 */  "Illegal ASCII characters received.",
/* 05 */  "Data field error. Not enough, too many, or improper positioning.",
/* 06 */  "Undefined command. Command not within acceptable range.",
/* 07 */  "Not used.",
/* 08 */  "Hardware fault. Return to Factory for service.",
/* 09 */  "Hardware fault. Return to Factory for service.",
/* 10 */  "Undefined command. Command not within acceptable range."
};

/* Public forward references */
int drvLoveInit(const char *lovPort,const char* serPort,int serAddr);


/* Forward references for support methods */
static asynStatus initSerialPort(Lovport* plov,const char* serPort,int serAddr);
static void exceptCallback(asynUser* pasynUser,asynException exception);
static asynStatus lockPort(Lovport* plov,asynUser* pasynUser);
static void unlockPort(Lovport* plov,asynUser* pasynUser);

static asynStatus setDefaultEos(Lovport* plov);
static asynStatus showFailure(asynUser* pasynUser,const char* pmethod);
static asynStatus evalMessage(size_t* pcount,char* pinp,asynUser* pasynUser,char* pout);
static void calcChecksum(size_t count,const char* pdata,unsigned char* pcs);


/* Forward references for asynCommon methods */
static void reportIt(void* ppvt,FILE* fp,int details);
static asynStatus connectIt(void* ppvt,asynUser* pasynUser);
static asynStatus disconnectIt(void* ppvt,asynUser* pasynUser);
static asynCommon common = {reportIt,connectIt,disconnectIt};


/* Forward references for asynOctet methods */
static asynStatus flushIt(void* ppvt,asynUser* pasynUser);
static asynStatus setInpEos(void* ppvt,asynUser* pasynUser,const char* eos,int eoslen);
static asynStatus getInpEos(void* ppvt,asynUser* pasynUser,char* eos,int eossize,int* eoslen);
static asynStatus setOutEos(void* ppvt,asynUser* pasynUser,const char* eos,int eoslen);
static asynStatus getOutEos(void* ppvt,asynUser* pasynUser,char* eos,int eossize,int* eoslen);
static asynStatus writeIt(void* ppvt,asynUser* pasynUser,const char* data,size_t numchars,size_t* pnbytesTransfered);
static asynStatus writeRaw(void* ppvt,asynUser* pasynUser,const char* data,size_t numchars,size_t* pnbytesTransfered);
static asynStatus readIt(void* ppvt,asynUser* pasynUser,char* data,size_t maxchars,size_t* pnbytesTransfered,int* peomReason);
static asynStatus readRaw(void* ppvt,asynUser* pasynUser,char* data,size_t maxchars,size_t* pnbytesTransfered,int* peomReason);
static asynOctet octet = {writeIt,writeRaw,readIt,readRaw,flushIt,NULL,NULL,setInpEos,getInpEos,setOutEos,getOutEos};


/* Define macros */
#define ISOK(s) (asynSuccess==(s))
#define ISNOTOK(s) (!ISOK(s))


/****************************************************************************
 * Define public interface methods
 ****************************************************************************/
int drvLoveInit(const char *lovPort,const char* serPort,int serAddr)
{
    asynStatus sts;
    int len,attr;
    Lovport* plov;
    Serport* pser;
    asynUser* pasynUser;


    len = sizeof(Lovport) + sizeof(Serport) + strlen(lovPort) + strlen(serPort) + 2;
    plov = callocMustSucceed(len,sizeof(char),"drvLoveInit");
    pser = (Serport*)(plov + 1);
    plov->name = (char*)(pser + 1);
    pser->name = plov->name + strlen(lovPort) + 1;

    plov->pserport = pser;
    strcpy(plov->name,lovPort);

    sts = initSerialPort(plov,serPort,serAddr);
    if( ISNOTOK(sts) )
    {
        printf("drvLoveInit::failure to initialize serial port %s\n",serPort);
        free(plov);

        return( -1 );
    }

    attr = ASYN_MULTIDEVICE;
    attr |= (pser->canBlock ? ASYN_CANBLOCK : 0);

    sts = pasynManager->registerPort(lovPort,attr,pser->autoConnect,0,0);
    if( ISNOTOK(sts) )
    {
        printf("drvLoveInit::failure to register love port %s\n",lovPort);
        pasynManager->disconnect(pser->pasynUser);
        pasynManager->freeAsynUser(pser->pasynUser);
        free(plov);
        return( -1 );
    }

    plov->common.interfaceType = asynCommonType;
    plov->common.pinterface = &common;
    plov->common.drvPvt = plov;

    sts = pasynManager->registerInterface(lovPort,&plov->common);
    if( ISNOTOK(sts) )
    {
        printf("drvLoveInit::failure to register asynCommon\n");
        return( -1 );
    }

    plov->octet.interfaceType = asynOctetType;
    plov->octet.pinterface = &octet;
    plov->octet.drvPvt = plov;

    sts = pasynOctetBase->initialize(lovPort,&plov->octet,0,0,0);
    if( ISNOTOK(sts) )
    {
        printf("drvLoveInit::failure to initialize asynOctetBase\n");
        return( -1 );
    }

    pasynUser = pasynManager->createAsynUser(NULL,NULL);
    if( pasynUser )
    {
        plov->pasynUser = pasynUser;
        pasynUser->userPvt = plov;
        pasynUser->timeout = K_COMTMO;
    }
    else
    {
        printf("drvLoveInit::create asynUser failure\n");
        return( -1 );
    }

    sts = pasynManager->connectDevice(pasynUser,lovPort,-1);
    if( ISOK(sts) )
        plov->isConnected = 1;
    else
    {
        printf("drvLoveInit::failure to connect with device %s\n",lovPort);
        plov->isConnected = 0;
        return( -1 );
    }

    pasynManager->exceptionCallbackAdd(pser->pasynUser,exceptCallback);

    if( prList )
        plov->pport = prList;
    prList = plov;

    sts = setDefaultEos(plov);
    if( ISNOTOK(sts) )
    {
        printf("drvLoveInit::failure to set %s EOS\n",lovPort);
        return( -1 );
    }

    return( 0 );
}


static asynStatus initSerialPort(Lovport* plov,const char* serPort,int serAddr)
{
    asynStatus sts;
    asynUser* pasynUser;
    Serport* pser = plov->pserport;
    asynInterface* pasynIface;


    pasynUser = pasynManager->createAsynUser(NULL,NULL);
    if( pasynUser == NULL )
        return( asynError );

    sts = pasynManager->connectDevice(pasynUser,serPort,serAddr);
    if( ISNOTOK(sts) )
    {
        printf("initSerialPort::failure to connect with device %s @ %d\n",serPort,serAddr);
        pasynManager->freeAsynUser(pasynUser);
        return( asynError );
    }

    sts = pasynManager->isMultiDevice(pasynUser,serPort,&pser->multiDevice);
    if( ISNOTOK(sts) )
    {
        printf("initSerialPort::failure to determine if %s @ %d is multi device\n",serPort,serAddr);
        pasynManager->disconnect(pasynUser);
        pasynManager->freeAsynUser(pasynUser);
        return( asynError );
    }

    sts = pasynManager->canBlock(pasynUser,&pser->canBlock);
    if( ISNOTOK(sts) )
    {
        printf("initSerialPort::failure to determine if %s @ %d can block\n",serPort,serAddr);
        pasynManager->disconnect(pasynUser);
        pasynManager->freeAsynUser(pasynUser);
        return( asynError );
    }

    sts = pasynManager->isAutoConnect(pasynUser,&pser->autoConnect);
    if( ISNOTOK(sts) )
    {
        printf("initSerialPort::failure to determine if %s @ %d is autoconnect\n",serPort,serAddr);
        pasynManager->disconnect(pasynUser);
        pasynManager->freeAsynUser(pasynUser);
        return( asynError );
    }

    pasynIface = pasynManager->findInterface(pasynUser,asynCommonType,1);
    if( pasynIface )
    {
        pser->pasynCommon = (asynCommon*)pasynIface->pinterface;
        pser->commonPvt = pasynIface->drvPvt;
    }
    else
    {
        printf("initSerialPort::failure to find interface %s\n",asynCommonType);
        pasynManager->disconnect(pasynUser);
        pasynManager->freeAsynUser(pasynUser);
        return( asynError );
    }

    pasynIface = pasynManager->findInterface(pasynUser,asynOctetType,1);
    if( pasynIface )
    {
        pser->pasynOctet = (asynOctet*)pasynIface->pinterface;
        pser->octetPvt = pasynIface->drvPvt;
    }
    else
    {
        printf("initSerialPort::failure to find interface %s\n",asynOctetType);
        pasynManager->disconnect(pasynUser);
        pasynManager->freeAsynUser(pasynUser);
        return( asynError );
    }

    pser->addr = serAddr;
    pser->pasynUser = pasynUser;
    pser->isConnected = 1;
    strcpy(pser->name,serPort);
    pasynUser->userPvt = plov;
    pasynUser->timeout = K_COMTMO;

    return( asynSuccess );
}


/****************************************************************************
 * Define private interface suppport methods
 ****************************************************************************/
static asynStatus evalMessage(size_t* pcount,char* pinp,asynUser* pasynUser,char* pout)
{
    size_t len;
    asynStatus sts;


    asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::evalMessage\n");

    /* Process message given size */
    if( *pcount == 7 )
    {
        int errNum;

        len = *pcount - 4;      /* Minus STX, FILTER, ADDR */
        errNum = atol( pinp + len );
        sts = asynError;

        asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::evalMessage error message received \"%s\"\n",errCodes[errNum]);
    }
    else
    {
        unsigned int  csMsg;
        unsigned char csPos, csVal;

        len   = *pcount - 3;    /* Minus STX and CHECKSUM */
        csPos = *pcount - 2;    /* Checksum position */
        calcChecksum(len,&pinp[1],&csVal);
        sscanf(&pinp[csPos],"%2x",&csMsg);

        if( (unsigned int)csMsg != (unsigned int)csVal )
        {
            asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::evalMessage checksum failed\n");
            return( asynError );
        }

        len = len - 3;
        sts = asynSuccess;

        asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::evalMessage message received\n");
    }

    memcpy(pout,&pinp[4],len);
    pout[len] = '\0';
    *pcount = len;

    return( sts );
}


static void calcChecksum(size_t count,const char* pdata,unsigned char* pcs)
{
    int i;
    unsigned long cs;

    cs = 0;
    for( i = 0; i < count; ++i )
        cs += pdata[i];
    *pcs = (unsigned char)(cs & 0xFF);

    return;
}


static asynStatus setDefaultEos(Lovport* plov)
{
    asynStatus sts;


    sts = setInpEos((void*)plov,plov->pasynUser,&inpEos,1);
    if( ISOK(sts) )
        printf("drvLove::setDefaultEos Input EOS set to \\0%d\n",inpEos);
    else
        printf("drvLove::setDefaultEos Input EOS set failed to \\0%d\n",inpEos);

    sts = setOutEos((void*)plov,plov->pasynUser,&outEos,1);
    if( ISOK(sts) )
        printf("drvLove::setDefaultEos Output EOS set to \\0%d\n",outEos);
    else
        printf("drvLove::setDefaultEos Output EOS set failed to \\0%d\n",outEos);

    return( sts );
}


static asynStatus showFailure(asynUser* pasynUser,const char* pmethod)
{
    asynStatus sts;
    const char* name;


    asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::showFailure\n");

    sts = pasynManager->getPortName(pasynUser,&name);
    if( ISNOTOK(sts) )
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,"%s, %s unsupported\n",name,pmethod);
    else
        asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::showFailure failure to acquire port name\n");

    return( asynError );
}


static void exceptCallback(asynUser* pasynUser,asynException exception)
{
    asynStatus sts;
    int isConnected;
    Lovport* plov = pasynUser->userPvt;
    Serport* pser = plov->pserport;


    asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::exceptionCallback\n");

    sts = pasynManager->isConnected(pasynUser,&isConnected);
    if( ISNOTOK(sts) )
    {
        asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::exceptionCallback failure to determine if %s is connected to %s\n",plov->name,pser->name);
        return;
    }

    if( isConnected )
        return;

    if( plov->isConnected == 0 )
        return;

    plov->isConnected = 0;
    pasynManager->exceptionDisconnect(plov->pasynUser);
}


static asynStatus lockPort(Lovport* plov,asynUser* pasynUser)
{
    asynStatus sts;
    Serport* pser = plov->pserport;


    asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::lockPort\n");

    sts = pasynManager->lockPort(pser->pasynUser,1);
    if( ISNOTOK(sts) )
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,"%s error %s",pser->name,pser->pasynUser->errorMessage);

    return( sts );
}


static void unlockPort(Lovport* plov,asynUser* pasynUser)
{
    asynStatus sts;
    Serport* pser = plov->pserport;


    asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::unlockPort\n");

    sts = pasynManager->unlockPort(pser->pasynUser);
    if( ISNOTOK(sts) )
        asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::unlockPort %s error %s\n",pser->name,pser->pasynUser->errorMessage);

    return;
}


/****************************************************************************
 * Define private interface asynCommon methods
 ****************************************************************************/
static void reportIt(void* ppvt,FILE* fp,int details)
{
    int i;
    Lovport* plov = (Lovport*)ppvt;
    Serport* pser = plov->pserport;


    fprintf(fp, "    %s is connected to %s\n",plov->name,pser->name);

    for( i = 0; i < K_INSTRMAX; ++i )
        if( plov->instr[i].isConnected )
            fprintf(fp, "        Addr %d is connected\n",(i + 1));
}


static asynStatus connectIt(void* ppvt,asynUser* pasynUser)
{
    asynStatus sts;
    int addr,isConnected;
    Lovport* plov = (Lovport*)ppvt;
    Serport* pser = plov->pserport;


    asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::connectIt\n");

    sts = pasynManager->isConnected(pser->pasynUser,&isConnected);
    if( ISNOTOK(sts) )
    {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,"port %s isConnected error %s\n",pser->name,pser->pasynUser->errorMessage);
        return( asynError );
    }

    if( isConnected == 0 )
    {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize, "port %s not connected to %s\n",plov->name,pser->name);
        return( asynError );
    }

    sts = pasynManager->getAddr(pasynUser,&addr);
    if( ISNOTOK(sts) )
        return( sts );

    if( (addr == 0) || (addr > K_INSTRMAX) )
    {
        asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::connectIt %s illegal addr %d\n",plov->name,addr);
        return( asynError );
    }

    if( addr > 0 )
    {
        Instrinfo* prInstr = &plov->instr[addr - 1];

        if( prInstr->isConnected )
        {
            asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::connectIt %s device %d already connected\n",plov->name,addr);
            return( asynError );
        }

        prInstr->isConnected = 1;
    }
    else
        plov->isConnected = 1;

    pasynManager->exceptionConnect(pasynUser);

    return( asynSuccess );
}


static asynStatus disconnectIt(void* ppvt,asynUser* pasynUser)
{
    asynStatus sts;
    int addr;
    Instrinfo* prInstr;
    Lovport* plov = (Lovport*)ppvt;


    asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::disconnectIt\n");

    sts = pasynManager->getAddr(pasynUser,&addr);
    if( ISNOTOK(sts) )
        return( sts );

    if( (addr == 0) || (addr > K_INSTRMAX) )
    {
        asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::disconnectIt %s illegal addr %d\n",plov->name,addr);
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,"illegal addr %d",addr);
        return( asynError );
    }

    if( addr < 0 )
    {
        if( plov->isConnected == 0 )
        {
            epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,"not connected");
            return( asynError );
        }

        pasynManager->exceptionDisconnect(pasynUser);
        plov->isConnected = 0;
        return( asynSuccess );
    }

    prInstr = &plov->instr[addr - 1];
    if( prInstr->isConnected )
        prInstr->isConnected = 0;
    else
    {
        epicsSnprintf(pasynUser->errorMessage,pasynUser->errorMessageSize,"not connected");
        return( asynError );
    }

    pasynManager->exceptionDisconnect(pasynUser);

    return( asynSuccess );
}


/****************************************************************************
 * Define private interface asynOctet methods
 ****************************************************************************/
static asynStatus writeIt(void* ppvt,asynUser* pasynUser,const char* data,size_t numchars,size_t* pnbytesTransfered)
{
    unsigned char cs;
    asynStatus sts;
    int addr;
    size_t len,bytesXfer;
    size_t* pbytesXfer;
    Lovport* plov = (Lovport*)ppvt;


    asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::writeIt\n");

    if( pnbytesTransfered == NULL )
        pbytesXfer = &bytesXfer;
    else
        pbytesXfer = pnbytesTransfered;

    sts = pasynManager->getAddr(pasynUser,&addr);
    if( ISNOTOK(sts) )
        return( sts );

    sprintf(plov->tmpMsg,"%02X%*s",addr,(int)numchars,data);
    calcChecksum(strlen(plov->tmpMsg),plov->tmpMsg,&cs);
    sprintf(plov->outMsg,"\002L%s%2X",plov->tmpMsg,cs);
    len = strlen(plov->outMsg);

    sts = lockPort(plov,pasynUser);
    if( ISNOTOK(sts) )
        return( sts );

    sts = writeRaw(ppvt,pasynUser,plov->outMsg,len,pbytesXfer);
    if( ISOK(sts) )
    {
        if( len == *pbytesXfer )
            *pbytesXfer = numchars;
        asynPrint(pasynUser,ASYN_TRACEIO_FILTER,"drvLove::writeIt \"%*s\"\n",numchars,data);
    }
    unlockPort(plov,pasynUser);

    return( sts );
}


static asynStatus writeRaw(void* ppvt,asynUser* pasynUser,const char* data,size_t numchars,size_t* pnbytesTransfered)
{
    asynStatus sts;
    size_t bytesXfer;
    size_t* pbytesXfer;
    Lovport* plov = (Lovport*)ppvt;
    Serport* pser = plov->pserport;


    asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::writeRaw\n");

    if( pnbytesTransfered == NULL )
        pbytesXfer = &bytesXfer;
    else
        pbytesXfer = pnbytesTransfered;

    sts = lockPort(plov,pasynUser);
    if( ISNOTOK(sts) )
        return( sts );

    sts = pser->pasynOctet->write(pser->octetPvt,pasynUser,data,numchars,pbytesXfer);
    if( ISOK(sts) )
        asynPrint(pasynUser,ASYN_TRACEIO_FILTER,"drvLove::writeRaw \"%*s\"\n",numchars,data);
    else
    {
        if( sts == asynTimeout )
            asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::writeRaw asynTimeout\n");
        else if( sts == asynOverflow )
            asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::writeRaw asynOverflow\n");
        else if( sts == asynError )
            asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::writeRaw asynError\n");
        else
            asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::writeRaw failed - unknown Asyn error\n");
    }
    unlockPort(plov,pasynUser);

    return( sts );
}


static asynStatus readIt(void* ppvt,asynUser* pasynUser,char* data,size_t maxchars,size_t* pnbytesTransfered,int* peomReason)
{
    asynStatus sts;
    int eom;
    int* peom;
    size_t bytesXfer;
    size_t* pbytesXfer;
    Lovport* plov = (Lovport*)ppvt;


    asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::readIt\n");

    if( pnbytesTransfered == NULL )
        pbytesXfer = &bytesXfer;
    else
        pbytesXfer = pnbytesTransfered;

    if( peomReason == NULL )
        peom = &eom;
    else
        peom = peomReason;

    sts = lockPort(plov,pasynUser);
    if( ISNOTOK(sts) )
        return( sts );

    sts = readRaw(ppvt,pasynUser,plov->inpMsg,maxchars,pbytesXfer,peom);
    if( ISOK(sts) )
    {
        sts = evalMessage(pbytesXfer,plov->inpMsg,pasynUser,data);
        asynPrint(pasynUser,ASYN_TRACEIO_FILTER,"drvLove::readIt %d \"%s\"\n",*pbytesXfer,data);
    }
    unlockPort(plov,pasynUser);

    return( sts );
}


static asynStatus readRaw(void* ppvt,asynUser* pasynUser,char* data,size_t maxchars,size_t* pnbytesTransfered,int* peomReason)
{
    asynStatus sts;
    int eom;
    int* peom;
    size_t bytesXfer;
    size_t* pbytesXfer;
    Lovport* plov = (Lovport*)ppvt;
    Serport* pser = plov->pserport;


    asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::readRaw\n");

    if( pnbytesTransfered == NULL )
        pbytesXfer = &bytesXfer;
    else
        pbytesXfer = pnbytesTransfered;

    if( peomReason == NULL )
        peom = &eom;
    else
        peom = peomReason;

    sts = lockPort(plov,pasynUser);
    if( ISNOTOK(sts) )
        return( sts );

    sts = pser->pasynOctet->read(pser->octetPvt,pasynUser,data,maxchars,pbytesXfer,peom);
    if( ISOK(sts) )
    {
        data[*pbytesXfer] = '\0';

        if( (*peom & ASYN_EOM_EOS) == 0 )
        {
            sts = asynError;
            asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::readRaw invalid EOM reason %d\n",*peom);
        }

        asynPrint(pasynUser,ASYN_TRACEIO_FILTER,"drvLove::readRaw %d \"%s\"\n",*pbytesXfer,data);
    }
    else
    {
        if( sts == asynTimeout )
            asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::readRaw asynTimeout\n");
        else if( sts == asynOverflow )
            asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::readRaw asynOverflow\n");
        else if( sts == asynError )
            asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::readRaw asynError\n");
        else
            asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::readRaw failed - unknown Asyn error\n");
    }
    unlockPort(plov,pasynUser);

    return( sts );
}


static asynStatus flushIt(void* ppvt,asynUser* pasynUser)
{
    asynStatus sts;
    Lovport* plov = (Lovport*)ppvt;
    Serport* pser = plov->pserport;


    asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::flushIt\n" );

    sts = lockPort(plov,pasynUser);
    if( ISNOTOK(sts) )
        return( sts );

    sts = pser->pasynOctet->flush(pser->octetPvt,pasynUser);
    if( ISOK(sts) )
        asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::flushIt done\n");
    else
        asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::flushIt failed\n");
    unlockPort(plov,pasynUser);

    return( sts );
}


static asynStatus setInpEos(void* ppvt,asynUser* pasynUser,const char* eos,int eoslen)
{
    asynStatus sts;
    Lovport* plov = (Lovport*)ppvt;
    Serport* pser = plov->pserport;
    static char inpEosSet = 0;


    asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::setInpEos\n");

    if( inpEosSet == 1 )
    {
        sts = showFailure(pasynUser,"setInpEos");
        return( sts );
    }

    sts = lockPort(plov,pasynUser);
    if( ISNOTOK(sts) )
        return( sts );

    sts = pser->pasynOctet->setInputEos(pser->octetPvt,pasynUser,eos,eoslen);
    if( ISOK(sts) )
    {
        inpEosSet = 1;
        asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::Input EOS set to \\0%d\n",*eos);
    }
    else
    {
        inpEosSet = 0;
        asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::Input EOS set failed to \\0%d\n",*eos);
    }
    unlockPort(plov,pasynUser);

    return( sts );
}


static asynStatus getInpEos(void* ppvt,asynUser* pasynUser,char* eos,int eossize,int* eoslen)
{
    asynStatus sts;
    Lovport* plov = (Lovport*)ppvt;
    Serport* pser = plov->pserport;


    asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::getInpEos\n");

    sts = lockPort(plov,pasynUser);
    if( ISNOTOK(sts) )
        return( sts );

    sts = pser->pasynOctet->getInputEos(pser->octetPvt,pasynUser,eos,eossize,eoslen);
    if( ISOK(sts) )
        asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::getInpEos done\n");
    else
        asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::getInpEos failed\n");
    unlockPort(plov,pasynUser);

    return( sts );
}


static asynStatus setOutEos(void* ppvt,asynUser* pasynUser,const char* eos,int eoslen)
{
    asynStatus sts;
    Lovport* plov = (Lovport*)ppvt;
    Serport* pser = plov->pserport;
    static char outEosSet = 0;


    asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::setOutEos\n");

    if( outEosSet == 1 )
    {
        sts = showFailure(pasynUser,"setOutEos");
        return( sts );
    }

    sts = lockPort(plov,pasynUser);
    if( ISNOTOK(sts) )
        return( sts );

    sts = pser->pasynOctet->setOutputEos(pser->octetPvt,pasynUser,eos,eoslen);
    if( ISOK(sts) )
    {
        outEosSet = 1;
        asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::setOutEos Output EOS set to \\0%d\n",*eos);
    }
    else
    {
        outEosSet = 0;
        asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::setOutEos Output EOS set failed to \\0%d\n",*eos);
    }
    unlockPort(plov,pasynUser);

    return( sts );
}


static asynStatus getOutEos(void* ppvt,asynUser* pasynUser,char* eos,int eossize,int* eoslen)
{
    asynStatus sts;
    Lovport* plov = (Lovport*)ppvt;
    Serport* pser = plov->pserport;


    asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::getOutEos\n");

    sts = lockPort(plov,pasynUser);
    if( ISNOTOK(sts) )
        return( sts );

    sts = pser->pasynOctet->getOutputEos(pser->octetPvt,pasynUser,eos,eossize,eoslen);
    if( ISOK(sts) )
        asynPrint(pasynUser,ASYN_TRACE_FLOW,"drvLove::getOutEos done\n");
    else
        asynPrint(pasynUser,ASYN_TRACE_ERROR,"drvLove::getOutEos failed\n");
    unlockPort(plov,pasynUser);

    return( sts );
}


/****************************************************************************
 * Register public methods
 ****************************************************************************/

/* Initialization method definitions */
static const iocshArg drvLoveInitArg0 = {"lovPort",iocshArgString};
static const iocshArg drvLoveInitArg1 = {"serPort",iocshArgString};
static const iocshArg drvLoveInitArg2 = {"serAddr",iocshArgInt};
static const iocshArg* drvLoveInitArgs[]= {&drvLoveInitArg0,&drvLoveInitArg1,&drvLoveInitArg2};
static const iocshFuncDef drvLoveInitFuncDef = {"drvLoveInit",3,drvLoveInitArgs};
static void drvLoveInitCallFunc(const iocshArgBuf* args)
{
    drvLoveInit(args[0].sval,args[1].sval,args[2].ival);
}

/* Registration method */
static void drvLoveRegister(void)
{
    static int firstTime = 1;

    if( firstTime )
    {
        firstTime = 0;
        iocshRegister( &drvLoveInitFuncDef, drvLoveInitCallFunc );
    }
}
epicsExportRegistrar( drvLoveRegister );
