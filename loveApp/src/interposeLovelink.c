/*+ interposeLovelink.c

                          Argonne National Laboratory
                            APS Operations Division
                     Beamline Controls and Data Acquisition

                       Love Controller Lovelink Protocol
                             Interpose Interface



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
    This module implements the interface that is 'interposed' between the
    ASYN device support and ASYN port driver layers. It provides the mechanism
    to transport messages between ASYN device support and the Love controllers
    on a serial network link (either RS232 or RS485).

    The Love controllers implement a master/slave communication where the
    master IOC sends a command to read or write the the Love controller. The
    controller responds with either readback data or a positive or negative
    acknowledgement to the command.

    The command recieved from device support consists of the controller address,
    command, and optional data for write operations. The interpose interface
    encapsulates the command with the control characters necessary for
    communications as well as calculates the checksum of the message. The
    control characters and checksum are striped from the response, then
    presented to devive support leaving the controller address and data or
    status.

    To initialize the interface, the method interposeLovelink is called from
    the startup script with the following calling sequence.

    interposeLovelink( portName, addr )

    Where:
        portName - ASYN port driver name (i.e. "L0" )
        addr     - Controller address (unused, for future functionality)

    The method interposeLovelinkReport can be called from the IOC shell to
    display the current status of the interface, such as the ASYN port name
    and associated Love controller address.


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
    4) asynStatus includes: asynSuccess, asynTimeout, asynOverflow,
       and asynError.


 =============================================================================
 History:
 Author: David M. Kline
 -----------------------------------------------------------------------------
 2005-Feb-17  DMK  Taken from the existing echoServer interpose interface.
 2005-Feb-21  DMK  Initial version complete. Private code review ready.
 -----------------------------------------------------------------------------

-*/


/* EPICS base version-specific definitions (must be performed first) */
#include <epicsVersion.h>
#define ILL__IS_EPICSBASE(v,r,l)  \
   ((EPICS_VERSION==(v)) && (EPICS_REVISION==(r)) && (EPICS_MODIFICATION==(l)))
#define ILL__GT_EPICSBASE(v,r,l)  \
   ((EPICS_VERSION>=(v)) && (EPICS_REVISION>=(r)) && (EPICS_MODIFICATION>(l)))
#define ILL__LT_EPICSBASE(v,r,l)  \
   ((EPICS_VERSION<=(v)) && (EPICS_REVISION<=(r)) && (EPICS_MODIFICATION<(l)))


/* Evaluate EPICS base */
#if ILL__LT_EPICSBASE(3,14,6)
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


/* EPICS synApps/ASYN related include files */
#include <asynDriver.h>
#include <asynOctet.h>


/* Define symbolic constants */
#define ILL__M_CHECKSUM         (   0xFF )


/* Define Lovelink symbolic constants */
#define ILL__S_MSG              (    100 )

#define ILL__K_STX              ( '\002' )
#define ILL__K_ETX              ( '\003' )
#define ILL__K_ACK              ( '\006' )
#define ILL__K_FILTER           (    'L' )
#define ILL__K_ERROR            (    'N' )


/* Define message index symbolic constants */
#define ILL__K_INDEX_STX        ( 0 )
#define ILL__K_INDEX_FILTER     ( 1 )
#define ILL__K_INDEX_ADDR       ( 2 )
#define ILL__K_INDEX_DATA       ( 4 )


/* Define message index symbolic constants */
#define ILL__K_INDEX_ERRID      ( 4 )
#define ILL__K_INDEX_ERRCODE    ( 5 )


/* Declare Love data structure */
typedef struct rILL
{
    int           addr;             /* Controller address (unused) */
    char*         strPortname;      /* Port name string */
    asynInterface octet;            /* Pointer to Octet structure */
    asynOctet*    pasynOctet;       /* Pointer to method pointer structure */
    void*         pasynOctetPvt;    /* Low level driver */
    struct rILL*  pnext;            /* Pointer to rILL next structure */
} rILL;


/* Define local variants */
static rILL* prLoveList       = NULL;           /* List of LOVE instances */

static char inputEosSet       = 0;              /* Input EOS indicator */
static char outputEosSet      = 0;              /* Output EOS indicator */

static char loveLinkinputEOS  = ILL__K_ACK;     /* Input EOS */
static char loveLinkoutputEOS = ILL__K_ETX;     /* Output EOS */
static char loveLinkFilter    = ILL__K_FILTER;  /* Filter character */

static char* loveErrorCodes[] =
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

/* Declare public forward references */
int interposeLovelink(const char*, int );


/* Declare local forward references for 'helper' methods */
static asynStatus setDefaultEos(rILL*);
static void calcChecksum(size_t, const char*, unsigned char*);
static asynStatus evalMessage(size_t*, char*, asynUser*);


/* Declare local forward references for asynOctet methods */
static asynStatus flushIt(void*, asynUser*);

static asynStatus setInputEos(void*, asynUser*, const char*,int);
static asynStatus getInputEos(void*, asynUser*, char*, int, int*);
static asynStatus setOutputEos(void*, asynUser*, const char*, int);
static asynStatus getOutputEos(void*, asynUser*, char*, int, int*);

static asynStatus writeIt(void*, asynUser*, const char*, size_t, size_t*);
static asynStatus writeRaw(void*, asynUser*, const char*, size_t, size_t*);

static asynStatus readIt(void*, asynUser*, char*, size_t,size_t*,int*);
static asynStatus readRaw(void*, asynUser*, char*, size_t, size_t*, int*);

static asynStatus cancelInterruptUser(void*, asynUser*);
static asynStatus registerInterruptUser(void*,asynUser*,interruptCallbackOctet, void*, void**);


/* Publish interpose-interface methods to asynOctet interface */
static asynOctet octet =
{
    writeIt,
    writeRaw,
    readIt,
    readRaw,
    flushIt,
    registerInterruptUser,
    cancelInterruptUser,
    setInputEos,
    getInputEos,
    setOutputEos,
    getOutputEos
};


/* Define macros */
#define ASYN__IS_OK(s)      ( asynSuccess == (s) )
#define ASYN__IS_NOTOK(s)   ( !ASYN__IS_OK(s) )


/****************************************************************************
 * Define public interface methods
 ****************************************************************************/


/*
 * interposeLovelink()
 *
 * Description:
 *    This method is called from the startup script to 'interpose' the
 *    Lovelink protocol layer between the device support and port driver
 *    layers. In addition, the input and output Eos characters are set.
 *
 * Input Parameters:
 *    pname     - Asyn port name.
 *    addr      - Controller address (unused).
 *
 * Output Parameters:
 *    None.
 *
 * Returns:
 *    0 for success, -1 for Failure.
 *
 * Developer notes:
 *
 */
int interposeLovelink( const char *pname, int addr )
{
    int             len;
    rILL*           prLov;
    asynStatus      sts;
    asynInterface*  poaIface;


    /* Allocate dynamic memory for LOVE data structure */
    len   = sizeof( rILL ) + strlen( pname ) + 1;
    prLov = callocMustSucceed( len, sizeof(char), "interposeLovelink" );

    /* Assign LOVE data structure members */
    prLov->addr        = addr;
    prLov->pnext       = NULL;
    prLov->strPortname = (char*)(prLov + 1);
    strcpy( prLov->strPortname, pname );

    /* Assign LOVE data structure Octet members */
    prLov->octet.interfaceType = asynOctetType;
    prLov->octet.pinterface    = &octet;
    prLov->octet.drvPvt        = prLov;

    /* Perform interface interpose */
    sts = pasynManager->interposeInterface( pname, addr, &prLov->octet, &poaIface );

    /* Evaluate completion status */
    if( ASYN__IS_NOTOK(sts) || (poaIface == NULL) )
    {
        printf( "interposeLovelink(): %s interpose failed.\n", pname );
        free( prLov );

        return( -1 );
    }

    /* Assign LOVE data structure Octet and driver pointers */
    prLov->pasynOctet    = (asynOctet*)poaIface->pinterface;
    prLov->pasynOctetPvt = poaIface->drvPvt;

    /* Link into list */
    if( prLoveList )
    {
        prLov->pnext = prLoveList;
    }
    prLoveList = prLov;

    /* Set both input and output EOS characters */
    sts = setDefaultEos( prLov );

    /* Evaluate completion status */
    if( ASYN__IS_NOTOK(sts) )
    {
        printf( "interposeLovelink(): %s set EOS failed.\n", pname );
        free( prLov );

        return( -1 );
    }

    /* Return completion status */
    return( 0 );

} /* end-method: interposeLovelink */


/*
 * interposeLovelinkReport()
 *
 * Description:
 *    This method reports the current status of the interpose interface
 *    Lovelink layer.
 *
 * Input Parameters:
 *    level - Interest level.
 *
 * Output Parameters:
 *    None.
 *
 * Returns:
 *    Always zero.
 *
 * Developer notes:
 *
 */
int interposeLovelinkReport( int level )
{
    rILL* prLov;


    /* Evaluate interest level */
    if( level <= 0 )
    {
        return( 0 );
    }

    /* Output LOVE structure information */
    printf( "\nLovelink Interpose Layer Report\n" );
    printf( "-------------------------------\n" );

    for( prLov = prLoveList; prLov; prLov = prLov->pnext )
    {

        if( level > 0 )
        {
            printf( "0x%2.2X\t\"%s\"\n", prLov->addr, prLov->strPortname );
        }

    }

    /* Return completion status */
    return( 0 );

} /* end-method: interposeLovelinkReport */


/****************************************************************************
 * Define private interface asynOctet methods
 ****************************************************************************/

/*
 * writeIt()
 *
 * Description:
 *    Send a message to the device. *pnbytesTransfered is the number of 8-bit
 *    bytes sent to the device. Interpose or driver code may add end of string
 *    terminators to the message but the extra characters are not included in
 *    pnbytesTransfered.
 *
 * Input Parameters:
 *    ppvt              - Pointer to LOVE data structure.
 *    pasynUser         - Pointer to ASYN user.
 *    data              - Pointer to write data.
 *    numchars          - Size of data (in bytes).
 *
 * Output Parameters:
 *    nBytesTransferred - Pointer to byte count transmitted to controller.
 *
 * Returns:
 *    Completion status
 *
 * Developer notes:
 *    asynStatus
 *
 */
static asynStatus writeIt( void*       ppvt,
                           asynUser*   pasynUser,
                           const char* data,
                           size_t      numchars,
                           size_t*     pnbytesTransfered
                         )
{
    size_t        len;
    unsigned char cs;
    char          msg[ILL__S_MSG];
    asynStatus    sts;


    /* Output flow message */
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "interposeLovelink::writeIt\n" );

    /* Calculate checksum */
    calcChecksum( numchars, data, &cs );

    /* Build message (STX, FILTER, ADDRESS, DATA) */
    msg[ILL__K_INDEX_STX]    = ILL__K_STX;
    msg[ILL__K_INDEX_FILTER] = loveLinkFilter;
    sprintf( &msg[ILL__K_INDEX_ADDR], "%*.*s%2.2X", (int)numchars, (int)numchars, data, cs );

    /* Calculate message length */
    len = strlen( msg );

    /* Include EOS (must be included if not previously set) */
    if( outputEosSet == 0 )
    {
        /* Append EOS */
        msg[len] = loveLinkoutputEOS;

        /* Recalculate message length */
        ++len;
    }

    /* Execute write */
    sts = writeRaw( ppvt, pasynUser, msg, len, pnbytesTransfered );

    /* Evaluate completion status */
    if( ASYN__IS_OK(sts) )
    {

        /* Make assignment to avoid 'write error' */
        if( len == *pnbytesTransfered )
        {
            *pnbytesTransfered = numchars;
        }

    }

    /* Return completion status */
    return( sts );

} /* end-method: writeIt */


/*
 * writeRaw()
 *
 * Description:
 *    Send a message to the device. *pnbytesTransfered is the number of 8-bit
 *    bytes sent to the device. Interpose or driver code must not add end of
 *    string terminators to the message.
 *
 * Input Parameters:
 *    ppvt              - Pointer to LOVE data structure.
 *    pasynUser         - Pointer to ASYN user.
 *    data              - Pointer to write data.
 *    numchars          - Size of data (in bytes).
 *
 * Output Parameters:
 *    nBytesTransferred - Pointer to byte count transmitted to controller.
 *
 * Returns:
 *    asynStatus
 *
 * Developer notes:
 *
 */
static asynStatus writeRaw( void*       ppvt,
                            asynUser*   pasynUser,
                            const char* data,
                            size_t      numchars,
                            size_t*     pnbytesTransfered
                          )
{
    asynStatus sts;
    rILL*      prLov = (rILL*)ppvt;


    /* Output flow message */
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "interposeLovelink::writeRaw\n" );

    /* Execute write */
    sts = prLov->pasynOctet->write( prLov->pasynOctetPvt, pasynUser, data, numchars, pnbytesTransfered );

    /* Evaluate completion status */
    if( ASYN__IS_OK(sts) )
    {
        asynPrintIO( pasynUser, ASYN_TRACEIO_FILTER, data, *pnbytesTransfered, "interposeLovelink::writeRaw\n" );
    }
    else
    {

        /* Evaluate return status */
        if( sts == asynTimeout )
        {
            asynPrint( pasynUser, ASYN_TRACE_ERROR, "interposeLovelink::writeRaw asynTimeout\n" );
        }
        else if( sts == asynOverflow )
        {
            asynPrint( pasynUser, ASYN_TRACE_ERROR, "interposeLovelink::writeRaw asynOverflow\n" );
        }
        else if( sts == asynError )
        {
            asynPrint( pasynUser, ASYN_TRACE_ERROR, "interposeLovelink::writeRaw asynError\n" );
        }
        else
        {
            asynPrint( pasynUser, ASYN_TRACE_ERROR, "interposeLovelink::writeRaw failed - unknown ASYN error\n" );
        }

    }

    /* Return completion status */
    return( sts );

} /* end-method: writeRaw */


/*
 * readIt()
 *
 * Description:
 *    Read a message from the device. *pnbytesTransfered is the number of 8-bit
 *    bytes read from the device. If read returns asynSuccess, then eomReason
 *    tells why the read completed. Interpose or driver code may strip end of
 *    string terminators from the message. If it does the first EOS character
 *    will be replaced by null and the eos characters will not be included in
 *    *pnbytesTransfered.
 *
 * Input Parameters:
 *    ppvt              - Pointer to LOVE data structure.
 *    pasynUser         - Pointer to ASYN user.
 *    data              - Pointer to read data.
 *    maxchars          - Max. size of read data (in bytes).
 *
 * Output Parameters:
 *    nBytesTransferred - Number of bytes read.
 *    peomReason        - Pointer to reason for End-Of-Message.
 *
 * Returns:
 *    asynStatus
 *
 * Developer notes:
 *
 */
static asynStatus readIt( void*     ppvt,
                          asynUser* pasynUser,
                          char*     data,
                          size_t    maxchars,
                          size_t*   pnbytesTransfered,
                          int*      peomReason
                        )
{
    char       msg[ILL__S_MSG];
    asynStatus sts;


    /* Output message */
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "interposeLovelink::readIt\n" );

    /* Execute read */
    sts = readRaw( ppvt, pasynUser, msg, maxchars, pnbytesTransfered, peomReason );

    /* Evaluate completion status */
    if( ASYN__IS_OK(sts) )
    {
        /* Evaluate message */
        sts = evalMessage( pnbytesTransfered, msg, pasynUser );

        /* Copy return data */
        memcpy( data, &msg[ILL__K_INDEX_ADDR], *pnbytesTransfered );
    }

    /* Return completion status */
    return( sts );

} /* end-method: readIt */


/*
 * readRaw()
 *
 * Description:
 *    Read a message from the device. *pnbytesTransfered is the number of 8-bit
 *    bytes read from the device. If read returns asynSuccess, then eomReason
 *    tells why the read completed. Interpose or driver code must not strip end
 *    of string terminators from the message.
 *
 * Input Parameters:
 *    ppvt              - Pointer to LOVE data structure.
 *    pasynUser         - Pointer to ASYN user.
 *    data              - Pointer to read data.
 *    maxchars          - Max. size of read data (in bytes).
 *
 * Output Parameters:
 *    nBytesTransferred - Number of bytes read.
 *    eomReason         - Pointer to reason for End-Of-Message.
 *
 * Returns:
 *    asynStatus
 *
 * Developer notes:
 *
 */
static asynStatus readRaw( void*     ppvt,
                           asynUser* pasynUser,
                           char*     data,
                           size_t    maxchars,
                           size_t*   pnbytesTransfered,
                           int*      eomReason
                         )
{
    asynStatus sts;
    rILL*      prLov = (rILL*)ppvt;


    /* Output flow message */
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "interposeLovelink::readRaw\n" );

    /* Execute read */
    sts = prLov->pasynOctet->read( prLov->pasynOctetPvt, pasynUser, data, maxchars, pnbytesTransfered, eomReason );

    /* Evaluate completion status */
    if( ASYN__IS_OK(sts) )
    {
        asynPrintIO( pasynUser, ASYN_TRACEIO_FILTER, data, *pnbytesTransfered, "interposeLovelink::readRaw success\n" );

        /* Evaluate EOM response */
        if( (*eomReason & ASYN_EOM_EOS) == 0 )
        {
            sts = asynError;
            asynPrint( pasynUser, ASYN_TRACE_ERROR, "interposeLovelink::readRaw invalid EOM reason %d\n", *eomReason );
        }

    }
    else
    {

        /* Evaluate completion status */
        if( sts == asynTimeout )
        {
            asynPrint( pasynUser, ASYN_TRACE_ERROR, "interposeLovelink::readRaw asynTimeout\n" );
        }
        else if( sts == asynOverflow )
        {
            asynPrint( pasynUser, ASYN_TRACE_ERROR, "interposeLovelink::readRaw asynOverflow\n" );
        }
        else if( sts == asynError )
        {
            asynPrint( pasynUser, ASYN_TRACE_ERROR, "interposeLovelink::readRaw asynError\n" );
        }
        else
        {
            asynPrint( pasynUser, ASYN_TRACE_ERROR, "interposeLovelink::readRaw failed - unknown ASYN error\n" );
        }

    }

    /* Return completion status */
    return( sts );

} /* end-method: readRaw */


/*
 * flushIt()
 *
 * Description:
 *    Flush the input buffer.
 *
 * Input Parameters:
 *    ppvt      - Pointer to LOVE data structure.
 *    pasynUser - Pointer to ASYN user.
 *
 * Output Parameters:
 *    None.
 *
 * Returns:
 *    asynstatus
 *
 * Developer notes:
 *
 */
static asynStatus flushIt( void* ppvt, asynUser* pasynUser )
{
    asynStatus sts;
    rILL*      prLov = (rILL*)ppvt;


    /* Output message */
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "interposeLovelink::flushIt\n" );

    /* Execute flush */
    sts = prLov->pasynOctet->flush( prLov->pasynOctetPvt, pasynUser );

    /* Evaluate completion status */
    if( ASYN__IS_OK(sts) )
    {
        asynPrint( pasynUser, ASYN_TRACE_FLOW, "interposeLovelink::flush done\n" );
    }
    else
    {
        asynPrint( pasynUser, ASYN_TRACE_ERROR, "interposeLovelink::flushIt failed\n" );
    }

    /* Return completion status */
    return( sts );

} /* end-method: flushIt */


/*
 * registerInterruptUser()
 *
 * Description:
 *    Register a user that will be called whenever a new message is received.
 *    NOTE: The callback must not block and must not call registerInterruptUser
 *    or cancelInterruptUser.
 *
 * Input Parameters:
 *    ppvt          - Pointer to LOVE data structure.
 *    pasynUser     - Pointer to ASYN user.
 *    callbaack     - Pointer to callback method.
 *    userPvt       - Pointer to user parameter.
 *
 * Output Parameters:
 *    registrarPvt  - Pointer to registrar.
 *
 * Returns:
 *    asynStatus
 *
 * Developer notes:
 *
 */
static asynStatus registerInterruptUser( void*                  ppvt,
                                         asynUser*              pasynUser,
                                         interruptCallbackOctet callback,
                                         void*                  userPvt,
                                         void**                 registrarPvt
                                       )
{
    asynStatus sts;
    rILL*      prLov = (rILL*)ppvt;


    /* Output message */
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "interposeLovelink::registerInterruptUser\n" );

    /* Execute registration */
    sts = prLov->pasynOctet->registerInterruptUser( prLov->pasynOctetPvt, pasynUser, callback, userPvt, registrarPvt );

    /* Evaluate completion status */
    if( ASYN__IS_OK(sts) )
    {
        asynPrint( pasynUser, ASYN_TRACE_FLOW, "interposeLovelink::registerInterruptUser done\n" );
    }
    else
    {
        asynPrint( pasynUser, ASYN_TRACE_ERROR, "interposeLovelink::registerInterruptUser failed\n" );
    }

    /* Return completion status */
    return( sts );

} /* end-method: registerInterruptUser */


/*
 * cancelInterruptUser()
 *
 * Description:
 *    Cancel a registered user.
 *
 * Input Parameters:
 *    ppvt      - Pointer to LOVE data structure.
 *    asynUser  - Pointer to ASYN user.
 *
 * Output Parameters:
 *    None.
 *
 * Returns:
 *    asynstatus
 *
 * Developer notes:
 *
 */
static asynStatus cancelInterruptUser( void* ppvt, asynUser* pasynUser )
{
    asynStatus sts;
    rILL*      prLov = (rILL*)ppvt;


    /* Output message */
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "interposeLovelink::cancelInterruptUser\n" );

    /* Execute cancellation */
    sts = prLov->pasynOctet->cancelInterruptUser( prLov->pasynOctetPvt, pasynUser );

    /* Evaluate completion status */
    if( ASYN__IS_OK(sts) )
    {
        asynPrint( pasynUser, ASYN_TRACE_FLOW, "interposeLovelink::cancelInterruptUser done\n" );
    }
    else
    {
        asynPrint( pasynUser, ASYN_TRACE_ERROR, "interposeLovelink::cancelInterruptUser failed\n" );
    }

    /* Return completion status */
    return( sts );

} /* end-method: cancelInterruptUser */


/*
 * setInputEos()
 *
 * Description:
 *    Set End Of String for input.
 *
 * Input Parameters:
 *    ppvt      - Pointer to LOVE data structure.
 *    pasynUser - Pointer to ASYN user.
 *    eos       - Pointer to EOS character.
 *    eoslen    - Size of EOS (in bytes).
 *
 * Output Parameters:
 *    None.
 *
 * Returns:
 *    asynStatus
 *
 * Developer notes:
 *
 */
static asynStatus setInputEos( void*       ppvt,
                               asynUser*   pasynUser,
                               const char* eos,
                               int         eoslen
                             )
{
    asynStatus sts;
    rILL*      prLov = (rILL*)ppvt;


    /* Output message */
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "interposeLovelink::setInputEos\n" );

    /* Call driver to set input EOS */
    sts = prLov->pasynOctet->setInputEos( prLov->pasynOctetPvt, pasynUser, eos, eoslen );

    /* Evaluate completion status */
    if( ASYN__IS_OK(sts) )
    {
        inputEosSet = 1;
        asynPrint( pasynUser, ASYN_TRACE_FLOW, "interposeLovelink::Input EOS set to \\0%d\n", *eos );
    }
    else
    {
        inputEosSet = 0;
        asynPrint( pasynUser, ASYN_TRACE_ERROR, "interposeLovelink::Input EOS set failed to \\0%d\n", *eos );
    }

    /* Return completion status */
    return( sts );

} /* end-method: setInputEos */


/*
 * getInputEos()
 *
 * Description:
 *    Get the current End of String for input.
 *
 * Input Parameters:
 *    ppvt      - Pointer to LOVE data structure.
 *    pasynUser - Pointer to ASYN user.
 *    eossize   - Size of EOS (in bytes).
 *
 * Output Parameters:
 *    eos       - Pointer to EOS character.
 *    eoslen    - Size of EOS (in bytes).
 *
 * Returns:
 *    asynStatus
 *
 * Developer notes:
 *
 */
static asynStatus getInputEos( void*     ppvt,
                               asynUser* pasynUser,
                               char*     eos,
                               int       eossize,
                               int*      eoslen
                             )
{
    asynStatus sts;
    rILL*      prLov = (rILL*)ppvt;


    /* Output message */
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "interposeLovelink::getInputEos\n" );

    /* Acquire input EOS */
    sts = prLov->pasynOctet->getInputEos( prLov->pasynOctetPvt, pasynUser, eos, eossize, eoslen );

    /* Evaluate completion status */
    if( ASYN__IS_OK(sts) )
    {
        asynPrint( pasynUser, ASYN_TRACE_FLOW, "interposeLovelink::getInputEos done\n" );
    }
    else
    {
        asynPrint( pasynUser, ASYN_TRACE_ERROR, "interposeLovelink::getInputEos failed\n" );
    }

    /* Return completion status */
    return( sts );

} /* end-method: getInputEos */


/*
 * setOutputEos()
 *
 * Description:
 *    Set End Of String for output.
 *
 * Input Parameters:
 *    ppvt      - Pointer to LOVE data structure.
 *    pasynUser - Pointer to ASYN user.
 *    eos       - Pointer to EOS character.
 *    eoslen    - Size of EOS (in bytes).
 *
 * Output Parameters:
 *    None.
 *
 * Returns:
 *    asynStatus
 *
 * Developer notes:
 *
 */
static asynStatus setOutputEos( void*       ppvt,
                                asynUser*   pasynUser,
                                const char* eos,
                                int         eoslen
                              )
{
    asynStatus sts;
    rILL*      prLov = (rILL*)ppvt;


    /* Output message */
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "interposeLovelink::setOutputEos\n" );

    /* Call driver to set output EOS */
    sts = prLov->pasynOctet->setOutputEos( prLov->pasynOctetPvt, pasynUser, eos, eoslen );

    /* Evaluate completion status */
    if( ASYN__IS_OK(sts) )
    {
        outputEosSet = 1;
        asynPrint( pasynUser, ASYN_TRACE_FLOW, "interposeLovelink::Output EOS set to \\0%d\n", *eos );
    }
    else
    {
        outputEosSet = 0;
        asynPrint( pasynUser, ASYN_TRACE_ERROR, "interposeLovelink::Output EOS set failed to \\0%d\n", *eos );
    }

    /* Return completion status */
    return( sts );

} /* end-method: setOutputEos */


/*
 * getOutputEos()
 *
 * Description:
 *    Get the current End of String for output.
 *
 * Input Parameters:
 *    ppvt      - Pointer to LOVE data structure.
 *    pasynUser - Pointer to ASYN user.
 *    eossize   - Size of EOS (in bytes).
 *
 * Output Parameters:
 *    eos       - Pointer to EOS character.
 *    eoslen    - Size of EOS (in bytes).
 *
 * Returns:
 *    asynStatus
 *
 * Developer notes:
 *
 */
static asynStatus getOutputEos( void*     ppvt,
                                asynUser* pasynUser,
                                char*     eos,
                                int       eossize,
                                int*      eoslen
                              )
{
    asynStatus sts;
    rILL*      prLov = (rILL*)ppvt;


    /* Output message */
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "interposeLovelink::getOutputEos\n" );

    /* Acquire output EOS */
    sts = prLov->pasynOctet->getOutputEos( prLov->pasynOctetPvt, pasynUser, eos, eossize, eoslen );

    /* Evaluate completion status */
    if( ASYN__IS_OK(sts) )
    {
        asynPrint( pasynUser, ASYN_TRACE_FLOW, "interposeLovelink::getOutputEos done\n" );
    }
    else
    {
        asynPrint( pasynUser, ASYN_TRACE_ERROR, "interposeLovelink::getOutputEos failed\n" );
    }

    /* Return completion status */
    return( sts );

} /* end-method: getOutputEos */


/*
 * evalMessage()
 *
 * Description:
 *    This method evaluates the input message components and verifies
 *    the checksum. It modifies the message count ('pcount') subtracting
 *    the STX, FILTER, CHECKSUM, and ACK message components.
 *
 * Input Parameters:
 *    pcount    - Pointer to size of input data in bytes.
 *    pdata     - Input data.
 *    pasynUser - Pointer to ASYN user information
 *
 * Output Parameters:
 *    None.
 *
 * Returns:
 *    asynStatus
 *
 * Developer notes:
 *    1) This method evaluates the error response code so that messages
 *       are displayed when the asynRecord is used (bypassing Love device
 *       support.
 *    2) An error response does not include a checksum.
 *    3) Number of bytes for a:
 *       STX      = 1
 *       FILTER   = 1
 *       CHECKSUM = 2
 *       ACK      = 1
 *
 */
static asynStatus evalMessage( size_t* pcount, char* pdata, asynUser* pasynUser )
{
    size_t        len;
    asynStatus    sts;
    unsigned int  msgcs;
    unsigned char cspos;
    unsigned char cs;


    /* Output message */
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "interposeLovelink:evalMessage\n" );

    /* NULL terminate message */
    pdata[ *pcount ] = '\0';

    /* Evaluate message STX */
    if( pdata[ILL__K_INDEX_STX] != ILL__K_STX )
    {
        asynPrint( pasynUser, ASYN_TRACE_ERROR, "interposeLovelink::evalMessage invalid STX\n" );
        return( asynError );
    }

    /* Evaluate message FILTER */
    if( pdata[ILL__K_INDEX_FILTER] != loveLinkFilter )
    {
        asynPrint( pasynUser, ASYN_TRACE_ERROR, "interposeLovelink::evalMessage invalid FILTER\n" );
        return( asynError );
    }

    /* Assume success status */
    sts = asynSuccess;

    /* Evaluate for error message ('N' character) */
    if( pdata[ILL__K_INDEX_ERRID] == ILL__K_ERROR )
    {
        unsigned long errorCode;

        /* Extract error code */
        errorCode = atol( &pdata[ILL__K_INDEX_ERRCODE] );

        /* Output banner */
        asynPrint( pasynUser, ASYN_TRACE_ERROR, "interposeLovelink::evalMessage error message received \"%s\"\n", loveErrorCodes[errorCode] );

        /* Calc. message length (account for presense or ACK) */
        if( inputEosSet == 1 )
        {
            len = *pcount - 2;      /* Subtract for STX, FILTER */
        }
        else
        {
            len = *pcount - 3;      /* Subtract for STX, FILTER, ACK */
        }

        /* Indicate failure status */
        sts = asynError;

    }
    else
    {

        /* Output banner */
        asynPrint( pasynUser, ASYN_TRACE_FLOW, "interposeLovelink::evalMessage message received\n" );

        /* Calc. message length (account for ACK presense) */
        if( inputEosSet == 1 )
        {
            len   = *pcount - 4;    /* Subtract for STX, FILTER, CHECKSUM */
            cspos = *pcount - 2;
        }
        else
        {
            len   = *pcount - 5;    /* Subtract for STX, FILTER, CHECKSUM, ACK */
            cspos = *pcount - 3;
        }

        /* Evaluate checksum (include filter) */
        calcChecksum( (len + 1), &pdata[ILL__K_INDEX_FILTER], &cs );

        /* Convert checksum */
        sscanf( &pdata[cspos], "%2x", &msgcs );

        /* Evaluate checksum */
        if( (unsigned int)msgcs != (unsigned int)cs )
        {
            asynPrint( pasynUser, ASYN_TRACE_ERROR, "interposeLovelink::evalMessage checksum failed\n" );
            return( asynError );
        }

    }

    /* Assign data length */
    *pcount = len;

    /* Return completion status */
    return( sts );

} /* end-method: evalMessage */


/*
 * calcChecksum()
 *
 * Description:
 *    This method is responsible for calculating the checksum by
 *    adding sequential bytes and using only the lower byte.
 *
 * Input Parameters:
 *    count - Size of input data in bytes.
 *    pdata - Input data to calculate checksum over.
 *
 * Output Parameters:
 *    pcs   - Result checksum.
 *
 * Returns:
 *    None.
 *
 * Developer notes:
 *
 */
static void calcChecksum( size_t count, const char* pdata, unsigned char* pcs )
{
    int           i;
    unsigned long cs;


    /* Initialize checksum */
    cs = 0;

    /* Generate checksum */
    for( i = 0; i < count; ++i )
    {
        cs += pdata[i];
    }

    /* Truncate and assign checksum */
    *pcs = (unsigned char)(cs & ILL__M_CHECKSUM);

    /* Return to caller */
    return;

} /* end-method: calcChecksum */


/*
 * setDefaultEos()
 *
 * Description:
 *    Set the default End Of String (EOS) characters for both input and output
 *    during interface initialization.
 *
 * Input Parameters:
 *    prLov  - Pointer to LOVE data structure.
 *
 * Output Parameters:
 *    None.
 *
 * Returns:
 *    asynStatus
 *
 * Developer notes:
 *    This method should be called only from interposeLoveLink. Subsequent
 *    changes to the EOS characters should be made through the ASYN record.
 *
 */
static asynStatus setDefaultEos( rILL* prLov )
{
    asynStatus sts;
    asynUser*  pasynUser;


    /* Create an asynUser */
    pasynUser = pasynManager->createAsynUser( NULL, NULL );
    if( pasynUser == NULL )
    {
        printf( "interposeLovelink::create asynUser failure\n" );
        return( asynError );
    }

    /* Set input EOS */
    sts = setInputEos( (void*)prLov, pasynUser, &loveLinkinputEOS, 1 );

    /* Evaluate completion status */
    if( ASYN__IS_OK(sts) )
    {
        printf( "interposeLovelink::setDefaultEos Input EOS set to \\0%d\n", loveLinkinputEOS );
    }
    else
    {
        printf( "interposeLovelink::setDefaultEos Input EOS set failed to \\0%d\n", loveLinkinputEOS );
    }

    /* Set output EOS */
    sts = setOutputEos( (void*)prLov, pasynUser, &loveLinkoutputEOS, 1 );

    /* Evaluate completion status */
    if( ASYN__IS_OK(sts) )
    {
        printf( "interposeLovelink::setDefaultEos Output EOS set to \\0%d\n", loveLinkoutputEOS );
    }
    else
    {
        printf( "interposeLovelink::setDefaultEos Output EOS set failed to \\0%d\n", loveLinkoutputEOS );
    }

    /* Free the asynUser */
    sts = pasynManager->freeAsynUser( pasynUser );

    /* Return completion status */
    return( sts );

} /* end-method: setDefaultEos */


/****************************************************************************
 * Register public methods
 ****************************************************************************/

/* Initialization method definitions */
static const iocshArg interposeLovelinkArg0 =
    { "portName", iocshArgString };
static const iocshArg interposeLovelinkArg1 =
    { "addr", iocshArgInt };
static const iocshArg* interposeLovelinkArgs[] =
    { &interposeLovelinkArg0, &interposeLovelinkArg1 };
static const iocshFuncDef interposeLovelinkFuncDef =
    { "interposeLovelink", 2, interposeLovelinkArgs };

/* Report method definitions */
static const iocshArg interposeLovelinkReportArg0 =
    { "level", iocshArgInt };
static const iocshArg* interposeLovelinkReportArgs[] =
    { &interposeLovelinkReportArg0 };
static const iocshFuncDef interposeLovelinkReportFuncDef =
    { "interposeLovelinkReport", 1, interposeLovelinkReportArgs };

static void interposeLovelinkReportCallFunc( const iocshArgBuf* args )
{
    interposeLovelinkReport( args[0].ival );
}

static void interposeLovelinkCallFunc( const iocshArgBuf* args )
{
    interposeLovelink( args[0].sval, args[2].ival );
}

static void registerLovelink( void )
{
    static int firstTime = 1;

    if( firstTime )
    {
        firstTime = 0;

        iocshRegister( &interposeLovelinkFuncDef,       interposeLovelinkCallFunc );
        iocshRegister( &interposeLovelinkReportFuncDef, interposeLovelinkReportCallFunc );
    }
}
epicsExportRegistrar( registerLovelink );

