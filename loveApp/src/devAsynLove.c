/*+ devAsynLove.c

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
    This module provides ASYN-based device support for the Love controllers,
    specifically models 1600 and 16A and using EPICS base 3.14.6 or greater.
    Previous versions for Love support employed MPF and are implemented for
    eariler versions of EPICS base. Refer to the documentation directory
    for specific information related to the communication protocol, and
    Love controller setup and functionality.

    The communication layer imposed by the interposeLovelink module is employed
    to transport commands to the Love controllers and receive command responses.
    Refer to interposeLovelink.c for additional information regarding the
    implementation.

    The database definition file (.dbd) provides the mapping between the
    record type and the device support table entries. The current device
    support includes the ao, ai, bo, bi, and mbbi record types. The mapping
    between record and device support is as follows.

    device(ai,   INST_IO, devAiAsynLove,   "devAsynLove")
    device(ao,   INST_IO, devAoAsynLove,   "devAsynLove")
    device(bi,   INST_IO, devBiAsynLove,   "devAsynLove")
    device(bo,   INST_IO, devBoAsynLove,   "devAsynLove")
    device(mbbi, INST_IO, devMbbiAsynLove, "devAsynLove")

    The "INST_IO" address type is used to indicate the format of the INP and
    OUT field in the database (.db) file. Both fields have a format that is
    defined by ASYN.

    field(INP," @asyn(portName,contAddr) Cmd Model")
    field(OUT," @asyn(portName,contAddr) Cmd Model")

    Where:
        portName - ASYN port name (i.e. "L0")
        contAddr - Controller address in hex (i.e. 0x32)
        Cmd      - Controller command 0..19 (see TABLE 1)
        Model    - Controller model (i.e. 16A or 1600 only)

    The DTYP field specifies the device support as shown in
    the here.

    field(DTYP, "devAsynLove" )

    Record instance initialization creates an asynUser and connects to the
    asynPort as indicated in the INP or OUT fields. Then, it finds the
    asynOctetType interface and processes the command parameters. The command
    parameters are used to lookup model specific information in the
    lov__supModels database.


 Developer notes:
    - Mapping between old and new commands:

      Rec     Old     New
      Typ     Cmd     Cmd     Description
      -------------------------------------------------
      AI      0       0       - Get current value
      AI      1       1       - Get SP1 value
      AI      2       2       - Get SP2 value
      AI      3       3       - Get AlLo value
      AI      4       4       - Get AlHi value
      AI      5       5       - Get Peak value
      AI      6       6       - Get Valley
      BI      7       7       - Get Alarm status
      MBBI    8       8       - Get Alarm type
      MBBI    9       9       - Get Input type
      BI     10      10       - Get Comm status
      AI             11       - Get decimal points
      AO     11      12       - Put SP1 value
      AO     12      13       - Put SP2 value
      AO     13      14       - Put AlLo value
      AO     14      15       - Put AlHi value
      BO     15      16       - Reset Peak value
      BO     16      17       - Reset Valley value
      BO     17               - Set Comm Type (REM/LOC)
      BO             18       - Set Comm Remote
      BO             19       - Set Comm Local
                          TABLE 1


 =============================================================================
 History:
 Author: David M. Kline
 -----------------------------------------------------------------------------
 2005-Feb-22  DMK  Initial development for ASYN-based device support.
 -----------------------------------------------------------------------------

-*/


/* EPICS base version-specific definitions (must be performed first) */
#include <epicsVersion.h>
#define MRD__IS_EPICSBASE(v,r,l)  \
   ((EPICS_VERSION==(v)) && (EPICS_REVISION==(r)) && (EPICS_MODIFICATION==(l)))
#define MRD__GT_EPICSBASE(v,r,l)  \
   ((EPICS_VERSION>=(v)) && (EPICS_REVISION>=(r)) && (EPICS_MODIFICATION>(l)))
#define LOV__LT_EPICSBASE(v,r,l)  \
   ((EPICS_VERSION<=(v)) && (EPICS_REVISION<=(r)) && (EPICS_MODIFICATION<(l)))


/* Evaluate EPICS base */
#if LOV__LT_EPICSBASE(3,14,6)
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


/* EPICS asyn related include files */
#include <asynDriver.h>
#include <asynOctet.h>
#include <asynEpicsUtils.h>


/* EPICS record processing related include files */
#include <dbLock.h>
#include <dbCommon.h>
#include <aoRecord.h>
#include <aiRecord.h>
#include <boRecord.h>
#include <biRecord.h>
#include <mbbiRecord.h>


/* EPICS base version-specific include files */
#if ( MRD__GT_EPICSBASE(3,14,6) )
    #include <epicsExit.h>
#else

    #ifdef vxWorks
        #include <rebootLib.h>
    #endif

#endif


/* Structure forward references */
struct rMODEL;
struct rLOVREC;


/* Define general symbolic constants */
#define LOV__K_ACTIVE           (     1 )   /* Record active */
#define LOV__K_INACTIVE         (     0 )   /* Record inactive */

#define LOV__S_BUFMAX           (    20 )   /* Max. buffer size */
#define LOV__K_TIMEOUT          (   1.0 )   /* Read / Write timeout */
#define LOV__K_DECPTS           (     1 )   /* Default decimal pts */
#define LOV__K_COMMRETRY        (     3 )   /* Communication retry */
#define LOV__K_SLEEPDELAY       (  0.04 )   /* Thread sleep delay */

#define LOV__STS_OK             (     0 )   /* OK */
#define LOV__STS_OKNOVAL        (     2 )   /* OK - do not modify VAL */
#define LOV__STS_ERROR          ( ERROR )   /* FAILURE */


/* Define symbolic constants for model 1600 status and data */
#define LOV__V_STS_AUTO_1600    (    15 )
#define LOV__M_STS_AUTO_1600    ( 0x1 << LOV__V_STS_AUTO_1600  )

#define LOV__V_STS_COMM_1600    (    14 )
#define LOV__M_STS_COMM_1600    ( 0x1 << LOV__V_STS_COMM_1600  )

#define LOV__V_STS_ERROR_1600   (    12 )
#define LOV__M_STS_ERROR_1600   ( 0x1 << LOV__V_STS_ERROR_1600 )

#define LOV__V_STS_ALM_1600     (    11 )
#define LOV__M_STS_ALM_1600     ( 0x1 << LOV__V_STS_ALM_1600   )

#define LOV__V_STS_SPTYP_1600   (     9 )
#define LOV__M_STS_SPTYP_1600   ( 0x1 << LOV__V_STS_SPTYP_1600 )

#define LOV__V_STS_ACT_1600     (     1 )
#define LOV__M_STS_ACT_1600     ( 0x1 << LOV__V_STS_ACT_1600   )

#define LOV__V_STS_SIGN_1600    (     0 )
#define LOV__M_STS_SIGN_1600    ( 0x1 << LOV__V_STS_SIGN_1600  )


/* Define symbolic constants for status and data (non-1600 models) */
#define LOV__V_DPTS             (     4 )
#define LOV__M_DPTS             ( 0x3 << LOV__V_DPTS  )

#define LOV__V_SIGN             (     0 )
#define LOV__M_SIGN             ( 0x1 << LOV__V_SIGN  )

#define LOV__V_STS_AUTO         (    15 )
#define LOV__M_STS_AUTO         ( 0x1 << LOV__V_STS_AUTO  )

#define LOV__V_STS_COMM         (    14 )
#define LOV__M_STS_COMM         ( 0x1 << LOV__V_STS_COMM  )

#define LOV__V_STS_ERROR        (    12 )
#define LOV__M_STS_ERROR        ( 0x1 << LOV__V_STS_ERROR )

#define LOV__V_STS_ALM          (    11 )
#define LOV__M_STS_ALM          ( 0x1 << LOV__V_STS_ALM   )

#define LOV__V_STS_SPTYP        (     8 )
#define LOV__M_STS_SPTYP        ( 0x3 << LOV__V_STS_SPTYP )

#define LOV__V_STS_ACT          (     7 )
#define LOV__M_STS_ACT          ( 0x1 << LOV__V_STS_ACT   )

#define LOV__V_STS_DPTS         (     4 )
#define LOV__M_STS_DPTS         ( 0x3 << LOV__V_STS_DPTS  )

#define LOV__V_STS_SIGN         (     0 )
#define LOV__M_STS_SIGN         ( 0x1 << LOV__V_STS_SIGN  )


/* Define enumerators */
typedef enum
{
    model16A,
    model32A,
    model1600,
    model2600,
    model8600,
    maxModel

} eModelType;

typedef enum
{
    noSup,
    yaSup,
    maxSup

} eSupType;

typedef enum
{
    inpFunc,
    outFunc,
    maxFunc

} eFuncType;

typedef enum
{
    aiRec,
    aoRec,
    biRec,
    boRec,
    mbbiRec,
    maxRec

} eRecType;

typedef enum
{
                    /*         TABLE 2         */
                    /* Rec   Cmd     Model     */
                    /* Type  Num  1600  16A    */
                    /* ----------------------- */
    getValue,       /*   AI    0    00    00   */
    getSP1,         /*   AI    1  0100  0101   */
    getSP2,         /*   AI    2  0102  0105   */
    getAlLo,        /*   AI    3  0104  0106   */
    getAlHi,        /*   AI    4  0105  0107   */
    getPeak,        /*   AI    5  011A  011D   */
    getValley,      /*   AI    6  011B  011E   */
    getAlStatus,    /*   BI    7    00    00   */
    getAlMode,      /* MBBI    8  0337  031D   */
    getInpType,     /* MBBI    9  0323  0317   */
    getCommStatus,  /*   BI   10  032A  0324   */
    getDecPt,       /*   AI   11  0324  031A   */
    putSP1,         /*   AO   12  0200  0200   */
    putSP2,         /*   AO   13  0202  0204   */
    putAlLo,        /*   AO   14  0204  0207   */
    putAlHi,        /*   AO   15  0205  0208   */
    resetPeak,      /*   BO   16  0407  040A   */
    resetValley,    /*   BO   17  0408  040B   */
    setRemote,      /*   BO   18  0400  0400   */
    setLocal,       /*   BO   19  0401  0401   */
    maxCmd

} eCmdType;


/* Declare datatypes union */
typedef union uDATATYPE
{
    double dData;
    float  fData;
    short  sData;
    long   lData;
    char   cData;

    unsigned char  ucData;
    unsigned short usData;
    unsigned long  ulData;

} uDATATYPE;


/* Declare 1600 message structures */
typedef struct rREADBACK_VALUE_1600
{
    char addr[2];
    char stat[4];
    char data[4];

} rREADBACK_VALUE_1600;

typedef struct rREADBACK_STATUS_1600
{
    char addr[2];
    char stat[10];

} rREADBACK_STATUS_1600;

typedef struct rREADBACK_SIGNED_1600
{
    char addr[2];
    char sign[2];
    char data[4];

} rREADBACK_SIGNED_1600;

typedef struct rREADBACK_UNSIGNED_1600
{
    char addr[2];
    char pad[2];
    char data[4];

} rREADBACK_UNSIGNED_1600;

typedef struct rREADBACK_CONFIG_1600
{
    char addr[2];
    char config[2];

} rREADBACK_CONFIG_1600;

typedef struct rREADBACK_INPTYP_1600
{
    char addr[2];
    char type[2];

} rREADBACK_INPTYP_1600;

typedef struct rREADBACK_DECPT_1600
{
    char addr[2];
    char decpt[2];

} rREADBACK_DECPT_1600;

typedef struct rREADBACK_ALMODE_1600
{
    char addr[2];
    char mode[2];

} rREADBACK_ALMODE_1600;

typedef struct rREADBACK_TUNEMODE_1600
{
    char addr[2];
    char mode[2];

} rREADBACK_TUNEMODE_1600;

typedef struct rREADBACK_WRITERESP_1600
{
    char addr[2];
    char resp[2];

} rREADBACK_WRITERESP_1600;

typedef union uREADBACK_1600
{
    char                     rawMsg[LOV__S_BUFMAX];
    rREADBACK_VALUE_1600     rValue;
    rREADBACK_STATUS_1600    rStatus;
    rREADBACK_SIGNED_1600    rSigned;
    rREADBACK_UNSIGNED_1600  rUnsigned;
    rREADBACK_CONFIG_1600    rConfig;
    rREADBACK_INPTYP_1600    rInpTyp;
    rREADBACK_DECPT_1600     rDecPt;
    rREADBACK_ALMODE_1600    rAlMode;
    rREADBACK_TUNEMODE_1600  rTuneMode;
    rREADBACK_WRITERESP_1600 rWriteResp;

} uREADBACK_1600;


/* Declare message structures */
typedef struct rREADBACK_VALUE      /* 00 */
{
    char addr[2];
    char stat[4];
    char data[4];

} rREADBACK_VALUE;

typedef struct rREADBACK_STATUS     /* 05 */
{
    char addr[2];
    char stat[4];
    char unused[6];

} rREADBACK_STATUS;

typedef struct rREADBACK_SIGNED     /* 01XX */
{
    char addr[2];
    char info[2];
    char data[4];

} rREADBACK_SIGNED;

typedef struct rREADBACK_UNSIGNED   /* 03XX */
{
    char addr[2];
    char data[2];

} rREADBACK_UNSIGNED;

typedef struct rREADBACK_WRITERESP
{
    char addr[2];
    char resp[2];

} rREADBACK_WRITERESP;

typedef union uREADBACK
{
    char                rawMsg[LOV__S_BUFMAX];
    rREADBACK_VALUE     rValue;
    rREADBACK_STATUS    rStatus;
    rREADBACK_SIGNED    rSigned;
    rREADBACK_UNSIGNED  rUnsigned;
    rREADBACK_WRITERESP rWriteResp;

} uREADBACK;


/* Declare controller models structure */
typedef struct rMODEL
{
    char*      strID;
    eSupType   isSup;
    eModelType modelType;
    void       (*preProcess)(struct rLOVREC*);
    void       (*ioCompletion)(struct rLOVREC*);
    char*      strCmds[maxCmd];

} rMODEL;


/* Declare record instance structure */
typedef struct rLOVREC
{
    asynStatus      sts;
    dbCommon*       prRec;
    int             iAddr;
    char*           pportName;
    asynUser*       pasynUser;
    asynOctet*      pasynOctet;
    void*           pasynOctetPvt;
    eCmdType        cmdType;
    eRecType        recType;
    eFuncType       funcType;
    int             decPts;
    uDATATYPE       rawData;
    int             procCount;
    size_t          bytesRead;
    size_t          bytesWritten;
    char            wrBuf[LOV__S_BUFMAX];
    char            rdBuf[LOV__S_BUFMAX];
    const rMODEL*   prModel;
    struct rLOVREC* prNext;

} rLOVREC;


/* Declare DSET data structure */
typedef struct rLOV__DSET
{
    long      number;           /* # of method pointers */
    DEVSUPFUN report;           /* Reports device support information */
    DEVSUPFUN init;             /* Device support initialization */
    DEVSUPFUN init_record;      /* Record support initialization */
    DEVSUPFUN get_ioint_info;   /* Associate interrupt source with record */
    DEVSUPFUN method;           /* Read/Write method */
    DEVSUPFUN specialLinconv;   /* Special processing for AO/AI records */

} rLOV__DSET;


/* Forward reference pre/post processing methods */
static void lov__preProcess( rLOVREC* );
static void lov__ioCompletion( rLOVREC* );
static void lov__preProcess1600( rLOVREC* );
static void lov__ioCompletion1600( rLOVREC* );


/* Define local variants */
static unsigned long lov__recReadCount;     /* Record read count */
static unsigned long lov__recWritCount;     /* Record write count */
static unsigned long lov__recInstCount;     /* Record instance count */

static rLOVREC* lov__prInstances = NULL;    /* List of instances */

static double lov__convFactor[maxFunc][4] =
{
    {1.0, 0.1, 0.01, 0.001},                /* inpFunc  */
    {  1,  10,  100,  1000}                 /* outFunc */
};

static const rMODEL lov__supModels[] =      /* Supported models */
{
    {
        "16A",  yaSup, model16A,  lov__preProcess, lov__ioCompletion,
            {  "00",    /*  0 - getValue      */
               "0101",  /*  1 - getSP1        */
               "0105",  /*  2 - getSP2        */
               "0106",  /*  3 - getAlLo       */
               "0107",  /*  4 - getAlHi       */
               "011D",  /*  5 - getPeak       */
               "011E",  /*  6 - getValley     */
               "00",    /*  7 - getAlStatus   */
               "031D",  /*  8 - getAlMode     */
               "0317",  /*  9 - getInpType    */
               "0324",  /* 10 - getCommStatus */
               "031A",  /* 11 - getDecPt      */
               "0200",  /* 12 - putSP1        */
               "0204",  /* 13 - putSP2        */
               "0207",  /* 14 - putAlLo       */
               "0208",  /* 15 - putAlHi       */
               "040A",  /* 16 - resetPeak     */
               "040B",  /* 17 - resetValley   */
               "0400",  /* 18 - setRemote     */
               "0401"   /* 19 - setLocal      */
            },
    },

    {
        "32A",  noSup, model32A,  lov__preProcess, lov__ioCompletion,
            {  "00",    /*  0 - getValue      */
               "0101",  /*  1 - getSP1        */
               "0105",  /*  2 - getSP2        */
               "0106",  /*  3 - getAlLo       */
               "0107",  /*  4 - getAlHi       */
               "011D",  /*  5 - getPeak       */
               "011E",  /*  6 - getValley     */
               "00",    /*  7 - getAlStatus   */
               "031D",  /*  8 - getAlMode     */
               "0317",  /*  9 - getInpType    */
               "0324",  /* 10 - getCommStatus */
               "031A",  /* 11 - getDecPt      */
               "0200",  /* 12 - putSP1        */
               "0204",  /* 13 - putSP2        */
               "0207",  /* 14 - putAlLo       */
               "0208",  /* 15 - putAlHi       */
               "040A",  /* 16 - resetPeak     */
               "040B",  /* 17 - resetValley   */
               "0400",  /* 18 - setRemote     */
               "0401"   /* 19 - setLocal      */
            },
    },

    {
        "1600", yaSup, model1600, lov__preProcess1600, lov__ioCompletion1600,
            {  "00",    /*  0 - getValue      */
               "0100",  /*  1 - getSP1        */
               "0102",  /*  2 - getSP2        */
               "0104",  /*  3 - getAlLo       */
               "0105",  /*  4 - getAlHi       */
               "011A",  /*  5 - getPeak       */
               "011B",  /*  6 - getValley     */
               "00",    /*  7 - getAlStatus   */
               "0337",  /*  8 - getAlMode     */
               "0323",  /*  9 - getInpType    */
               "032A",  /* 10 - getCommStatus */
               "0324",  /* 11 - getDecPt      */
               "0200",  /* 12 - putSP1        */
               "0202",  /* 13 - putSP2        */
               "0204",  /* 14 - putAlLo       */
               "0205",  /* 15 - putAlHi       */
               "0407",  /* 16 - resetPeak     */
               "0408",  /* 17 - resetValley   */
               "0400",  /* 18 - setRemote     */
               "0401"   /* 19 - setLocal      */
            },
    },

    {
        "2600", noSup, model2600, lov__preProcess, lov__ioCompletion,
            {  "00",    /*  0 - getValue      */
               "0101",  /*  1 - getSP1        */
               "0105",  /*  2 - getSP2        */
               "0106",  /*  3 - getAlLo       */
               "0107",  /*  4 - getAlHi       */
               "011D",  /*  5 - getPeak       */
               "011E",  /*  6 - getValley     */
               "00",    /*  7 - getAlStatus   */
               "031D",  /*  8 - getAlMode     */
               "0317",  /*  9 - getInpType    */
               "0324",  /* 10 - getCommStatus */
               "031A",  /* 11 - getDecPt      */
               "0200",  /* 12 - putSP1        */
               "0204",  /* 13 - putSP2        */
               "0207",  /* 14 - putAlLo       */
               "0208",  /* 15 - putAlHi       */
               "040A",  /* 16 - resetPeak     */
               "040B",  /* 17 - resetValley   */
               "0400",  /* 18 - setRemote     */
               "0401"   /* 19 - setLocal      */
            },
    },

    {
        "8600", noSup, model8600, lov__preProcess, lov__ioCompletion,
            {  "00",    /*  0 - getValue      */
               "0101",  /*  1 - getSP1        */
               "0105",  /*  2 - getSP2        */
               "0106",  /*  3 - getAlLo       */
               "0107",  /*  4 - getAlHi       */
               "011D",  /*  5 - getPeak       */
               "011E",  /*  6 - getValley     */
               "00",    /*  7 - getAlStatus   */
               "031D",  /*  8 - getAlMode     */
               "0317",  /*  9 - getInpType    */
               "0324",  /* 10 - getCommStatus */
               "031A",  /* 11 - getDecPt      */
               "0200",  /* 12 - putSP1        */
               "0204",  /* 13 - putSP2        */
               "0207",  /* 14 - putAlLo       */
               "0208",  /* 15 - putAlHi       */
               "040A",  /* 16 - resetPeak     */
               "040B",  /* 17 - resetValley   */
               "0400",  /* 18 - setRemote     */
               "0401"   /* 19 - setLocal      */
            },
    },

    {
        NULL,   noSup, maxModel,  lov__preProcess, lov__ioCompletion,
            {
               NULL,    /*  0 */
               NULL,    /*  1 */
               NULL,    /*  2 */
               NULL,    /*  3 */
               NULL,    /*  4 */
               NULL,    /*  5 */
               NULL,    /*  6 */
               NULL,    /*  7 */
               NULL,    /*  8 */
               NULL,    /*  9 */
               NULL,    /* 10 */
               NULL,    /* 11 */
               NULL,    /* 12 */
               NULL,    /* 13 */
               NULL,    /* 14 */
               NULL,    /* 15 */
               NULL,    /* 16 */
               NULL,    /* 17 */
               NULL,    /* 18 */
               NULL     /* 19 */
            }
    }
};


/* Declare local forward references for device-processing methods */
static long lov__report( int );
static void lov__callback( asynUser* );
static long lov__deviceInit( int );
static void lov__getDecPts( rLOVREC* );
static void lov__executeCommand( rLOVREC* );
static asynStatus lov__validateParams( asynUser*, char*, rLOVREC* );


/* Declare local forward references for record-processing methods */
static asynStatus lov__queueIt( dbCommon* );
static asynStatus lov__recordInit( dbCommon*, DBLINK*, eFuncType, eRecType );

static long ai__init( struct aiRecord* );
static long ai__read( struct aiRecord* );

static long ao__init( struct aoRecord* );
static long ao__write( struct aoRecord* );

static long bi__init( struct biRecord* );
static long bi__read( struct biRecord* );

static long bo__init( struct boRecord* );
static long bo__write( struct boRecord* );

static long mbbi__init( struct mbbiRecord* );
static long mbbi__read( struct mbbiRecord* );


/* Define DSET structures */
static rLOV__DSET devAiAsynLove   =
{
    6,
    lov__report,
    lov__deviceInit,
    ai__init,
    NULL,
    ai__read,
    NULL
};
static rLOV__DSET devAoAsynLove   =
{
    6,
    NULL,
    NULL,
    ao__init,
    NULL,
    ao__write,
    NULL
};
static rLOV__DSET devBiAsynLove   =
{
    5,
    NULL,
    NULL,
    bi__init,
    NULL,
    bi__read,
    NULL
};
static rLOV__DSET devBoAsynLove   =
{
    5,
    NULL,
    NULL,
    bo__init,
    NULL,
    bo__write,
    NULL
};
static rLOV__DSET devMbbiAsynLove =
{
    5,
    NULL,
    NULL,
    mbbi__init,
    NULL,
    mbbi__read,
    NULL
};


/* Publish DSET structure references to EPICS */
epicsExportAddress(dset, devAiAsynLove);
epicsExportAddress(dset, devAoAsynLove);
epicsExportAddress(dset, devBiAsynLove);
epicsExportAddress(dset, devBoAsynLove);
epicsExportAddress(dset, devMbbiAsynLove);


/* Define global variants */

/* Define macros */
#define ASYN__IS_OK(s)      ( asynSuccess == (s) )
#define ASYN__IS_NOTOK(s)   ( !ASYN__IS_OK(s) )


/****************************************************************************
 * Define device-specific methods
 ****************************************************************************/


/*
 * lov__deviceInit()
 *
 * Description:
 *    This method is called twice during IOC initialization. Once before
 *    any database records are initialized (after = 0) and once after all
 *    records are initialized (after = 1) but before the scan tasks are
 *    started.
 *
 * Input Parameters:
 *    after - Indicates when called.
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
static long lov__deviceInit( int after )
{

    /* Before record initialization */
    if( after == 0 )
    {
    }

    /* After record initialization */
    if( after == 1 )
    {
    }

    /* Return completion status */
    return( 0 );

} /* end-method: lov__deviceInit() */


/*
 * lov__report()
 *
 * Description:
 *    This method is called from the dbior shell command. It outputs
 *    information about the configuration.
 *
 * Input Parameters:
 *    level - Indicates interest level of information.
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
static long lov__report( int level )
{
    int      i;
    rLOVREC* prLov;

    /* Output configuration information */
    printf( "\nLove Controller Configuration\n" );
    printf( "\tInterest level                  - %d\n", level );
    printf( "\tRecord read count               - %d\n", lov__recReadCount );
    printf( "\tRecord write count              - %d\n", lov__recWritCount );
    printf( "\tRecord instance count           - %d\n", lov__recInstCount );
    printf( "\tEPICS release version           - %s\n", epicsReleaseVersion );

    /* Output instance information */
    for( i = 1, prLov = lov__prInstances; prLov; ++i, prLov = prLov->prNext )
    {
        printf( "\t%02d ", i );
        printf( "0x%2.2X ", prLov->iAddr );

        switch( prLov->prModel->modelType )
        {
        case model16A:
            printf( "16A  " );
            break;
        case model32A:
            printf( "32A  " );
            break;
        case model1600:
            printf( "1600 " );
            break;
        case model2600:
            printf( "2600 " );
            break;
        case model8600:
            printf( "8600 " );
            break;
        case maxModel:
            printf( "INV  " );
            break;
        }

        switch( prLov->cmdType )
        {
        case getValue:
            printf( "getValue      " );
            break;
        case getSP1:
            printf( "getSP1        " );
            break;
        case getSP2:
            printf( "getSP2        " );
            break;
        case getAlLo:
            printf( "getAlLo       " );
            break;
        case getAlHi:
            printf( "getAlHi       " );
            break;
        case getPeak:
            printf( "getPeak       " );
            break;
        case getValley:
            printf( "getValley     " );
            break;
        case getAlStatus:
            printf( "getAlStatus   " );
            break;
        case getAlMode:
            printf( "getAlMode     " );
            break;
        case getInpType:
            printf( "getInptype    " );
            break;
        case getCommStatus:
            printf( "getCommStatus " );
            break;
        case getDecPt:
            printf( "getDecPt      " );
            break;
        case putSP1:
            printf( "putSP1        " );
            break;
        case putSP2:
            printf( "putSP2        " );
            break;
        case putAlLo:
            printf( "putAlLo       " );
            break;
        case putAlHi:
            printf( "putAlHi       " );
            break;
        case resetPeak:
            printf( "resetPeak     " );
            break;
        case resetValley:
            printf( "resetValley   " );
            break;
        case setRemote:
            printf( "setRemote     " );
            break;
        case setLocal:
            printf( "setLocal      " );
            break;
        case maxCmd:
            printf( "INV           " );
            break;
        }

        switch( prLov->recType )
        {
        case aiRec:
            printf( "ai   " );
            break;
        case aoRec:
            printf( "ao   " );
            break;
        case biRec:
            printf( "bi   " );
            break;
        case boRec:
            printf( "bo   " );
            break;
        case mbbiRec:
            printf( "mbbi " );
            break;
        case maxRec:
            printf( "INV  " );
            break;
        }

        switch( prLov->sts )
        {
        case asynSuccess:
            printf( "OK  " );
            break;
        case asynTimeout:
            printf( "TMO " );
            break;
        case asynOverflow:
            printf( "OVR " );
            break;
        case asynError:
            printf( "ERR " );
            break;
        default:
            printf( "??? " );
            break;
        }

        printf( "%s ",     ((prLov->prRec->pact) ? "Y"   : "N"  ) );
        printf( "%06d ",   prLov->procCount   );
        printf( "\"%s\" ", prLov->pportName   );
        printf( "\"%s\" ", prLov->prRec->name );

        printf( "\n"  );
    }

    /* Return completion status */
    return( 0 );

} /* end-method: lov__report() */


/*
 * lov__callback()
 *
 * Description:
 *    This method calls the 'interpose interface' layer to interact
 *    with the Love controllers.
 *
 * Input Parameters:
 *    pasynUser - Pointer to ASYN User structure.
 *
 * Output Parameters:
 *    None.
 *
 * Returns:
 *    None.
 *
 * Developer notes:
 *    - Calling the record process method results in the read/write
 *      methods being called again, but with the process active
 *      indicator equal to one (pact=1).
 *    - Return status is placed in the Love structure status.
 *
 */
static void lov__callback( asynUser* pasynUser )
{
    rset*     prRSET;
    rLOVREC*  prLov;
    dbCommon* prRec;


    /* Output message */
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "devAsynLove::lov__callback\n" );

    /* Initialize local variants */
    prRec  = (dbCommon*)pasynUser->userPvt;
    prLov  = (rLOVREC*)prRec->dpvt;
    prRSET = (rset*)prRec->rset;

    /* Call pre-processing method */
    prLov->prModel->preProcess( prLov );
    if( ASYN__IS_NOTOK(prLov->sts) )
    {
        return;
    }

    /* Execute command */
    lov__executeCommand( prLov );
    if( ASYN__IS_NOTOK(prLov->sts) )
    {
        return;
    }

    /* Process the record */
    dbScanLock( prRec );
    prRSET->process( prRec );
    dbScanUnlock( prRec );

} /* end-method: lov__callback() */


/*
 * lov__executeCommand()
 *
 * Description:
 *    This method sends a command to the Love controller then waits
 *    for a response.
 *
 * Input Parameters:
 *    prLov - Pointer to the Love structure.
 *
 * Output Parameters:
 *    None.
 *
 * Returns:
 *    None.
 *
 * Developer notes:
 *    - A communication delay is incorporated to allow the Love controller
 *      time between its response and the next command. This is to compensate
 *      for the hardware.
 *    - If the communication results in a timeout, a retry occurs until
 *      the maximum of LOV__K_COMMRETRY is reached.
 *    - Return status is placed in the Love structure status.
 *
 */
static void lov__executeCommand( rLOVREC* prLov )
{
    int         i;
    asynStatus  sts;
    size_t      eomReason;
    size_t      bytesRead;
    size_t      bytesToWrite;
    size_t      bytesWritten;


    /* Output message */
    asynPrint( prLov->pasynUser, ASYN_TRACE_FLOW, "devAsynLove::lov__executeCommand\n" );

    /* Perform message pre-processing */
    prLov->sts                = asynSuccess;
    bytesToWrite              = strlen( prLov->wrBuf );
    prLov->pasynUser->timeout = LOV__K_TIMEOUT;

    /* Perform communication given retry count */
    for( i = 0; i < LOV__K_COMMRETRY; ++i )
    {

        /* Incorporate communication delay */
        epicsThreadSleep( LOV__K_SLEEPDELAY );

        /* Execute write operation (to interpose interface) */
        sts = prLov->pasynOctet->write( prLov->pasynOctetPvt, prLov->pasynUser, prLov->wrBuf, bytesToWrite, &bytesWritten );
        asynPrintIO( prLov->pasynUser, ASYN_TRACEIO_DEVICE, prLov->wrBuf, bytesToWrite, "devAsynLove::lov__executeCommand::write %s\n", prLov->prRec->name );

        /* Evaluate completion status */
        if( ASYN__IS_NOTOK(sts) )
        {

            /* Evaluate completion status */
            if( sts == asynTimeout )
            {
                asynPrint( prLov->pasynUser, ASYN_TRACE_ERROR, "devAsynLove::lov__executeCommand::write timeout, retrying\n" );
                continue;
            }

            asynPrint( prLov->pasynUser, ASYN_TRACE_ERROR, "devAsynLove::lov__executeCommand::write failure\n" );
            prLov->sts = sts;
            return;
        }

        /* Execute read operation (to interpose interface) */
        sts = prLov->pasynOctet->read( prLov->pasynOctetPvt, prLov->pasynUser, prLov->rdBuf, LOV__S_BUFMAX, &bytesRead, &eomReason );
        asynPrintIO( prLov->pasynUser, ASYN_TRACEIO_DEVICE, prLov->rdBuf, bytesRead, "devAsynLove::lov__executeCommand::read %s\n", prLov->prRec->name );

        /* Evaluate completion status */
        if( ASYN__IS_NOTOK(sts) )
        {

            /* Evaluate completion status */
            if( sts == asynTimeout )
            {
                asynPrint( prLov->pasynUser, ASYN_TRACE_ERROR, "devAsynLove::lov__executeCommand::read timeout, retrying\n" );
                continue;
            }

            asynPrint( prLov->pasynUser, ASYN_TRACE_ERROR, "devAsynLove::lov__executeCommand::read failure\n" );
            prLov->sts = sts;
            return;
        }

        /* Perform message post-processing */
        prLov->sts              = asynSuccess;
        prLov->bytesRead        = bytesRead;
        prLov->bytesWritten     = bytesWritten;
        prLov->rdBuf[bytesRead] = '\0';

        /* Exit iteration */
        break;
    }

    /* Evaluate retry counter */
    if( i == LOV__K_COMMRETRY )
    {
        prLov->sts = asynTimeout;
        asynPrint( prLov->pasynUser, ASYN_TRACE_ERROR, "devAsynLove::lov__executeCommand::retries exceeded\n" );
    }

    /* Return to caller */
    return;

} /* end-method: lov__executeCommand() */


/*
 * lov__getDecPts()
 *
 * Description:
 *    This method requests the decimal points from the controller.
 *
 * Input Parameters:
 *    prLov - Pointer to the Love structure.
 *
 * Output Parameters:
 *    None.
 *
 * Returns:
 *    None.
 *
 * Developer notes:
 *    - Return status is placed in the Love structure status.
 *
 */
static void lov__getDecPts( rLOVREC* prLov )
{
    rLOVREC rLov;


    /* Duplicate Love structure data */
    memcpy( &rLov, prLov, sizeof(rLOVREC) );

    /* Output message */
    asynPrint( rLov.pasynUser, ASYN_TRACE_FLOW, "devAsynLove::lov__getDecPts\n" );

    /* Initialize Love structure with specific information */
    rLov.cmdType = getDecPt;
    sprintf( rLov.wrBuf, "%2.2X%s", rLov.iAddr, rLov.prModel->strCmds[getDecPt] );

    /* Execute command */
    lov__executeCommand( &rLov );

    /* Evaluate completion status */
    if( ASYN__IS_OK(rLov.sts) )
    {
        /* Execute IO completion */
        rLov.prModel->ioCompletion( &rLov );

        /* Evaluate completion status */
        if( ASYN__IS_OK(rLov.sts) )
        {
            prLov->decPts = rLov.decPts;
        }

    }
    else
    {
        asynPrint( rLov.pasynUser, ASYN_TRACE_ERROR, "devAsynLove::lov__getDecPts failure to acquire decimal points\n" );
    }

    /* Return completion status */
    prLov->sts = rLov.sts;

    /* Return to caller */
    return;

} /* end-method: lov__getDecPts() */


/****************************************************************************
 * Define record-specific methods
 ****************************************************************************/


/*
 * lov__recordInit()
 *
 * Description:
 *    This method serves for common record initialization. It it called
 *    by all record types.
 *
 * Input Parameters:
 *    prRec  - Pointer to record.
 *    prInp  - Pointer to INP field data.
 *    eFunc  - Indicates function type (i.e input, output, etc..).
 *    eRec   - Indicates record type (i.e. ai, bo, mbbi, etc ..).
 *
 * Output Parameters:
 *    None.
 *
 * Returns:
 *    asynStatus
 *
 * Developer notes:
 *  1) Structure linkage:
 *     pasynUser->userPvt = prRec->dpvt =
 *          prLov->pasynUser      = pasynUser
 *               -> pasynOctet    = pasynIface->pinterface
 *               -> pasynOctetPvt = pasynIface->drvPvt
 *
 */
static asynStatus lov__recordInit(
                                   dbCommon* prRec,
                                   DBLINK*   prInp,
                                   eFuncType eFunc,
                                   eRecType  eRec
                                 )
{
    rLOVREC*       prLov;
    asynStatus     sts;
    asynUser*      pasynUser;
    asynInterface* pasynIface;
    char*          puserParams;


    /* Allocate memory for record instance structure */
    prLov = callocMustSucceed( 1, sizeof(rLOVREC), "devAsynLove::lov__recordInit" );

    /* Create an asynUser */
    pasynUser = pasynManager->createAsynUser( lov__callback, NULL );
    if( pasynUser == NULL )
    {
        asynPrint( pasynUser, ASYN_TRACE_ERROR, "devAsynLove::lov__recordInit, failure to create asynUser - %s\n", prRec->name );
        return( asynError );
    }

    /* Output message */
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "devAsynLove::lov__recordInit - %s\n", prRec->name );

    /* Parse input ASYN-specific string */
    sts = pasynEpicsUtils->parseLink( pasynUser, prInp, &prLov->pportName, &prLov->iAddr, &puserParams );
    if( ASYN__IS_NOTOK(sts) )
    {
        asynPrint( pasynUser, ASYN_TRACE_ERROR, "devAsynLove::lov__recordInit, parse link failure - %s\n", prRec->name );

        pasynManager->freeAsynUser( pasynUser );
        free( prLov );

        return( sts );
    }

    /* Connect to device */
    sts = pasynManager->connectDevice( pasynUser, prLov->pportName, prLov->iAddr );
    if( ASYN__IS_NOTOK(sts) )
    {
        asynPrint( pasynUser, ASYN_TRACE_ERROR, "devAsynLove::lov__recordInit, connect device failure - %s\n", prRec->name );

        pasynManager->freeAsynUser( pasynUser );
        free( prLov );

        return( sts );
    }

    /* Find supported interface */
    pasynIface = pasynManager->findInterface( pasynUser, asynOctetType, 1 );
    if( pasynIface == NULL )
    {
        asynPrint( pasynUser, ASYN_TRACE_ERROR, "devAsynLove::lov__recordInit, find interface failure - %s\n", prRec->name );

        pasynManager->freeAsynUser( pasynUser );
        free( prLov );

        return( sts );
    }

    /* Parse and validate user parameters */
    sts = lov__validateParams( pasynUser, puserParams, prLov );
    if( ASYN__IS_NOTOK(sts) )
    {
        asynPrint( pasynUser, ASYN_TRACE_ERROR, "devAsynLove::lov__recordInit, parameter validation failure - %s\n", prRec->name );

        pasynManager->freeAsynUser( pasynUser );
        free( prLov );

        return( sts );
    }

    /* Output message */
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "devAsynLove::lov__recordInit, name=\"%s\", port=\"%s\", addr=%d, params=\"%s\"\n", prRec->name, prLov->pportName, prLov->iAddr, puserParams );

    /* Assign record private member */
    prRec->dpvt          = prLov;

    /* Assign asynUser structure members */
    pasynUser->userPvt   = prRec;

    /* Assign rLOVREC structure members */
    prLov->sts           = asynSuccess;
    prLov->decPts        = LOV__K_DECPTS;
    prLov->recType       = eRec;
    prLov->funcType      = eFunc;
    prLov->prRec         = prRec;
    prLov->procCount     = 0;

    prLov->pasynUser     = pasynUser;
    prLov->pasynOctet    = (asynOctet*)pasynIface->pinterface;
    prLov->pasynOctetPvt = pasynIface->drvPvt;

    /* Increment record instance counter */
    ++lov__recInstCount;

    /* Link into list */
    if( lov__prInstances )
    {
        prLov->prNext = lov__prInstances;
    }
    lov__prInstances = prLov;

    /* Return completion status */
    return( asynSuccess );

} /* end-method: lov__recordInit() */


/*
 * lov__queueIt()
 *
 * Description:
 *    This results in the callback method being called. It queues the request
 *    to the port thread dedicated to the ASYN port.
 *
 * Input Parameters:
 *    prRec  - Pointer to record.
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
static asynStatus lov__queueIt( dbCommon* prRec )
{
    asynStatus sts;
    rLOVREC*   prLov;
    asynUser*  pasynUser;


    /* Initialize local variants */
    prLov       = (rLOVREC*)prRec->dpvt;
    pasynUser   = prLov->pasynUser;

    /* Indicate processing active and status */
    prLov->sts  = asynSuccess;
    prRec->pact = LOV__K_ACTIVE;

    /* Queue request */
    asynPrint( pasynUser, ASYN_TRACE_FLOW, "devAsynLove::lov__queueIt\n" );
    sts = pasynManager->queueRequest( pasynUser, 0, 0 );

    /* Return completion status */
    return( sts );

} /* end-method: lov__queueIt() */


/*
 * lov__validateParams()
 *
 * Description:
 *    This method validates the parameters that are passed
 *    in during record initialization.
 *
 * Input Parameters:
 *    pasynUser   - Pointer to ASYN user.
 *    puserParams - Pointer to the user parameters.
 *    prLov       - Pointer to LOVREC structure.
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
static asynStatus lov__validateParams( asynUser* pasynUser, char* puserParams, rLOVREC* prLov )
{
    asynStatus sts;
    int        i;
    char       model[8];
    eCmdType   command;
    static int modelSize = (sizeof(lov__supModels) / sizeof(rMODEL));


    /* Extract user parameters (from INP field) */
    sscanf( puserParams, "%d %s", (int*)&command, model );

    /* Evaluate command */
    if( command >= maxCmd )
    {
        asynPrint( pasynUser, ASYN_TRACE_ERROR, "devAsynLove::lov__validateParams::invalid command %d\n", command );
        return( asynError );
    }

    /* Search for supported model */
    for( i = 0; i < modelSize; ++i )
    {
        /* Compare model strings */
        if( strcmp( model, lov__supModels[i].strID ) == 0 )
        {
            /* Verify support */
            if( lov__supModels[i].isSup == yaSup )
            {
                break;
            }
            else
            {
                asynPrint( pasynUser, ASYN_TRACE_ERROR, "devAsynLove::lov__validateParams::unsupport model type %s\n", lov__supModels[i].strID );
            }

        }

    }

    /* Evaluate model found */
    if( i < modelSize )
    {
        sts            = asynSuccess;
        prLov->cmdType = command;
        prLov->prModel = &lov__supModels[i];
    }
    else
    {
        sts            = asynError;
        asynPrint( pasynUser, ASYN_TRACE_ERROR, "devAsynLove::lov__validateParams::invalid model %s\n", model );
    }

    /* Return completion status */
    return( sts );

} /* end-method: lov__queueIt() */


/*
 * lov__preProcess()
 *
 * Description:
 *    This method performs any requires 'pre' processing prior to
 *    executing the command.
 *
 * Input Parameters:
 *    prLov     - Pointer to LOVREC structure.
 *
 * Output Parameters:
 *    None.
 *
 * Returns:
 *    None.
 *
 * Developer notes:
 *    - For models other than a 1600, the decimal points must
 *      be acquired prior to a write operation only.
 *    - Return status is placed in the Love structure status.
 *
 */
static void lov__preProcess( rLOVREC* prLov )
{

    /* Output message */
    asynPrint( prLov->pasynUser, ASYN_TRACE_FLOW, "devAsynLove::lov__preProcess\n" );

    /* Assume success (unless otherwise indicated) */
    prLov->sts = asynSuccess;

    /* Evaluate function type */
    if( prLov->funcType == outFunc )
    {
        /* Acquire decimal points */
        lov__getDecPts( prLov );
        if( ASYN__IS_NOTOK(prLov->sts) )
        {
            return;
        }

    }

    /* Insert controller address and command */
    sprintf( prLov->wrBuf, "%2.2X%s", prLov->iAddr, prLov->prModel->strCmds[prLov->cmdType] );

    /* Complete command string given command type */
    switch( prLov->cmdType )
    {
    case getValue:          /* AI-based commands */
    case getSP1:
    case getSP2:
    case getAlLo:
    case getAlHi:
    case getPeak:
    case getValley:
    case getDecPt:
        break;

    case getAlStatus:       /* BI-based commands */
    case getCommStatus:
        break;

    case getAlMode:         /* MBBI-based commands */
    case getInpType:
        break;

    case putSP1:            /* AO-based commands */
    case putSP2:
    case putAlLo:
    case putAlHi:
        {
            int    sign;
            int    idata;
            double ddata;
            char   strdata[8];


            /* Preserve write data */
            ddata = prLov->rawData.dData;

            /* Determine sign */
            if( prLov->rawData.dData < 0 )
            {
                sign  = 0xFF;
                ddata = ddata * -1.0;
            }
            else
            {
                sign  = 0;
            }

            /* Insert command data and sign */
            idata = (int)(ddata * lov__convFactor[prLov->funcType][prLov->decPts]);
            sprintf( strdata, "%4.4d%2.2X", idata, sign );

            /* Build final command string */
            strcat( prLov->wrBuf, strdata );

        }
        break;

    case resetPeak:         /* BO-based commands */
    case resetValley:
    case setRemote:
    case setLocal:
        break;

    default:
        prLov->sts = asynError;
        asynPrint( prLov->pasynUser, ASYN_TRACE_ERROR, "devAsynLove::lov__preProcess::invalid cmdType\n" );
        break;
    }

    /* Return to caller */
    return;

} /* end-method: lov__preProcess() */


/*
 * lov__preProcess1600()
 *
 * Description:
 *    This method performs any requires 'pre' processing prior to
 *    executing the command specifically for a model 1600 controller.
 *
 * Input Parameters:
 *    prLov     - Pointer to LOVREC structure.
 *
 * Output Parameters:
 *    None.
 *
 * Returns:
 *    None.
 *
 * Developer notes:
 *    - For model 1600, the decimal points must be acquired
 *      prior to any read or write operation.
 *    - Return status is placed in the Love structure status.
 *
 */
static void lov__preProcess1600( rLOVREC* prLov )
{

    /* Output message */
    asynPrint( prLov->pasynUser, ASYN_TRACE_FLOW, "devAsynLove::lov__preProcess1600\n" );

    /* Assume success (unless otherwise indicated) */
    prLov->sts = asynSuccess;

    /* Acquire decimal points (always) */
    lov__getDecPts( prLov );
    if( ASYN__IS_NOTOK(prLov->sts) )
    {
        return;
    }

    /* Insert controller address and command */
    sprintf( prLov->wrBuf, "%2.2X%s", prLov->iAddr, prLov->prModel->strCmds[prLov->cmdType] );

    /* Complete command string given command type */
    switch( prLov->cmdType )
    {
    case getValue:          /* AI-based commands */
    case getSP1:
    case getSP2:
    case getAlLo:
    case getAlHi:
    case getPeak:
    case getValley:
    case getDecPt:
        break;

    case getAlStatus:       /* BI-based commands */
    case getCommStatus:
        break;

    case getAlMode:         /* MBBI-based commands */
    case getInpType:
        break;

    case putSP1:            /* AO-based commands */
    case putSP2:
    case putAlLo:
    case putAlHi:
        {
            int    sign;
            int    idata;
            double ddata;
            char   strdata[8];


            /* Preserve write data */
            ddata = prLov->rawData.dData;

            /* Determine sign */
            if( prLov->rawData.dData < 0 )
            {
                sign  = 0xFF;
                ddata = ddata * -1.0;
            }
            else
            {
                sign  = 0;
            }

            /* Insert command data and sign */
            idata = (int)(ddata * lov__convFactor[prLov->funcType][prLov->decPts]);
            sprintf( strdata, "%4.4d%2.2X", idata, sign );

            /* Build final command string */
            strcat( prLov->wrBuf, strdata );

        }
        break;

    case resetPeak:         /* BO-based commands */
    case resetValley:
    case setRemote:
    case setLocal:
        /* No further processing required */
        break;

    default:
        prLov->sts = asynError;
        asynPrint( prLov->pasynUser, ASYN_TRACE_ERROR, "devAsynLove::lov__preProcess1600::invalid cmdType\n" );
        break;
    }

    /* Return to caller */
    return;

} /* end-method: lov__preProcess1600() */


/*
 * lov__ioCompletion()
 *
 * Description:
 *    This method performs any required IO completion during record
 *    processing specifically for a model non-1600 controller.
 *
 * Input Parameters:
 *    prLov     - Pointer to LOVREC structure.
 *
 * Output Parameters:
 *    None.
 *
 * Returns:
 *    None.
 *
 * Developer notes:
 *    - Return status is placed in the Love structure status.
 *
 */
static void lov__ioCompletion( rLOVREC* prLov )
{
    uREADBACK *puMsg;


    /* Output message */
    asynPrint( prLov->pasynUser, ASYN_TRACE_FLOW, "devAsynLove::lov__ioCompletion\n" );

    /* Initialize local variants */
    prLov->sts = asynSuccess;
    puMsg      = (uREADBACK*)&prLov->rdBuf[0];

    /* Post-process given command */
    switch( prLov->cmdType )
    {
    case getValue:
        {
            int sts;
            int dpts;
            int data;

            /* Convert message components */
            sscanf( puMsg->rValue.stat, "%4x", &sts  );
            sscanf( puMsg->rValue.data, "%4d", &data );

            /* Extract decimal points */
            dpts = (sts & LOV__M_STS_DPTS);
            dpts = (dpts >> LOV__V_STS_DPTS);

            /* Extract sign */
            if( (sts & LOV__M_STS_SIGN) )
            {
                data = data * -1;
            }

            /* Make final assigmment */
            prLov->decPts        = dpts;
            prLov->rawData.dData = (double)data;

        }
        break;

    case getAlStatus:
        {
            int sts;
            int data;

            /* Convert message components */
            sscanf( puMsg->rValue.stat, "%4x", &sts  );
            sscanf( puMsg->rValue.data, "%4d", &data );

            /* Indicate alarm status */
            prLov->rawData.ulData = (sts & LOV__M_STS_ALM) ? TRUE : FALSE;

        }
        break;

    case getSP1:
    case getSP2:
    case getAlLo:
    case getAlHi:
    case getPeak:
    case getValley:
        {
            int dpts;
            int sign;
            int info;
            int data;

            /* Convert message components */
            sscanf( puMsg->rSigned.info, "%2x", &info );
            sscanf( puMsg->rSigned.data, "%4d", &data );

            /* Extract decimal points */
            dpts = (info & LOV__M_DPTS);
            dpts = (dpts >> LOV__V_DPTS);

            /* Extract sign */
            sign = (info & LOV__M_SIGN);
            sign = (sign >> LOV__V_SIGN);

            /* Convert data (if necessary) */
            if( sign )
            {
                data *= -1;
            }

            /* Assign return sign-extended raw data */
            prLov->decPts        = dpts;
            prLov->rawData.dData = (double)data;

        }
        break;

    case getCommStatus:
        {
            int data;

            /* Extract configuration data */
            sscanf( puMsg->rUnsigned.data, "%2x", &data );

            /* Indicate communication status (TRUE=remote, FALSE=local) */
            prLov->rawData.ulData = ( (data) ? TRUE : FALSE );

        }
        break;

    case getInpType:
        {
            int inpType;

            /* Extract input type data */
            sscanf( puMsg->rUnsigned.data, "%2x", &inpType );

            /* Assign return input type raw data */
            prLov->rawData.lData = inpType;

        }
        break;

    case getDecPt:
        {
            int decPt;

            /* Extract decimal points data */
            sscanf( puMsg->rUnsigned.data, "%2d", &decPt );

            /* Assign return decimal points raw data */
            prLov->decPts        = decPt;
            prLov->rawData.lData = decPt;

        }
        break;

    case getAlMode:
        {
            int alMode;

            /* Extract alarm mode data */
            sscanf( puMsg->rUnsigned.data, "%2x", &alMode );

            /* Assign return alarm mode raw data */
            prLov->rawData.ulData = alMode;

        }
        break;

    case putSP1:
    case putSP2:
    case putAlLo:
    case putAlHi:
    case resetPeak:
    case resetValley:
    case setRemote:
    case setLocal:
        {
            int writeResp;

            /* Extract write response data */
            sscanf( puMsg->rWriteResp.resp, "%2d", &writeResp );

            /* Evaluate response */
            if( writeResp )
            {
                prLov->sts = asynError;
                asynPrint( prLov->pasynUser, ASYN_TRACE_ERROR, "devAsynLove::lov__ioCompletion::command not accepted\n" );
            }

        }
        break;

    default:
        break;
    }

    /* Return to caller */
    return;

} /* end-method: lov__ioCompletion() */


/*
 * lov__ioCompletion1600()
 *
 * Description:
 *    This method performs any required IO completion during record
 *    processing specifically for a model 1600 controller.
 *
 * Input Parameters:
 *    prLov     - Pointer to LOVREC structure.
 *
 * Output Parameters:
 *    None.
 *
 * Returns:
 *    None.
 *
 * Developer notes:
 *    - Return status is placed in the Love structure status.
 *
 */
static void lov__ioCompletion1600( rLOVREC* prLov )
{
    uREADBACK_1600 *puMsg;


    /* Output message */
    asynPrint( prLov->pasynUser, ASYN_TRACE_FLOW, "devAsynLove::lov__ioCompletion1600\n" );

    /* Initialize local variants */
    prLov->sts = asynSuccess;
    puMsg      = (uREADBACK_1600*)&prLov->rdBuf[0];

    /* Post-process given command */
    switch( prLov->cmdType )
    {
    case getValue:
        {
            int sts;
            int data;

            /* Convert message components */
            sscanf( puMsg->rValue.stat, "%4x", &sts  );
            sscanf( puMsg->rValue.data, "%4d", &data );

            /* Assign return sign-extended raw data */
            if( (sts & LOV__M_STS_SIGN_1600) )
            {
                data = data * -1;
            }

            /* Make final assigmment */
            prLov->rawData.dData = (double)data;

        }
        break;

    case getAlStatus:
        {
            int sts;
            int data;

            /* Convert message components */
            sscanf( puMsg->rValue.stat, "%4x", &sts  );
            sscanf( puMsg->rValue.data, "%4d", &data );

            /* Indicate alarm status */
            prLov->rawData.ulData = (sts & LOV__M_STS_ALM_1600) ? TRUE : FALSE;

        }
        break;

    case getSP1:
    case getSP2:
    case getAlLo:
    case getAlHi:
    case getPeak:
    case getValley:
        {
            int sign;
            int data;

            /* Convert message components */
            sscanf( puMsg->rSigned.sign, "%2d", &sign );
            sscanf( puMsg->rSigned.data, "%4d", &data );

            /* Convert data */
            if( sign )
            {
                data *= -1;
            }

            /* Assign return sign-extended raw data */
            prLov->rawData.dData = (double)data;

        }
        break;

    case getCommStatus:
        {
            int config;

            /* Extract configuration data */
            sscanf( puMsg->rConfig.config, "%2d", &config );

            /* Indicate communication status (TRUE=remote, FALSE=local) */
            prLov->rawData.ulData = ( (config) ? TRUE : FALSE );

        }
        break;

    case getInpType:
        {
            int inpType;

            /* Extract input type data */
            sscanf( puMsg->rInpTyp.type, "%2x", &inpType );

            /* Assign return input type raw data */
            prLov->rawData.lData = inpType;

        }
        break;

    case getDecPt:
        {
            int decPt;

            /* Extract decimal points data */
            sscanf( puMsg->rDecPt.decpt, "%2d", &decPt );

            /* Assign return decimal points raw data */
            prLov->decPts        = decPt;
            prLov->rawData.lData = decPt;

        }
        break;

    case getAlMode:
        {
            int alMode;

            /* Extract alarm mode data */
            sscanf( puMsg->rAlMode.mode, "%2x", &alMode );

            /* Assign return alarm mode raw data */
            prLov->rawData.ulData = alMode;

        }
        break;

    case putSP1:
    case putSP2:
    case putAlLo:
    case putAlHi:
    case resetPeak:
    case resetValley:
    case setRemote:
    case setLocal:
        {
            int writeResp;

            /* Extract write response data */
            sscanf( puMsg->rWriteResp.resp, "%2d", &writeResp );

            /* Evaluate response */
            if( writeResp )
            {
                prLov->sts = asynError;
                asynPrint( prLov->pasynUser, ASYN_TRACE_ERROR, "devAsynLove::lov__ioCompletion1600::command not accepted\n" );
            }

        }
        break;

    default:
        break;
    }

    /* Return to caller */
    return;

} /* end-method: lov__ioCompletion1600() */


/*
 * ai__init()
 *
 * Description:
 *    This method performs the initialization for an AI record.
 *
 * Input Parameters:
 *    pai   - Address of aiRecord.
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
static long ai__init( struct aiRecord* pai )
{
    long sts;


    /* Call common initialization method */
    sts = lov__recordInit( (dbCommon*)pai, &pai->inp, inpFunc, aiRec );

    /* Return completion status */
    return( sts );

} /* end-method: ai__init() */


/*
 * ai__read()
 *
 * Description:
 *    This method performs the read for an AI record.
 *
 * Input Parameters:
 *    pai   - Address of aiRecord.
 *
 * Output Parameters:
 *    None.
 *
 * Returns:
 *    LOV__STS_OKNOVAL
 *
 * Developer notes:
 *
 */
static long ai__read( struct aiRecord* pai )
{
    rLOVREC* prLov;


    /* Initialize local variants */
    prLov = (rLOVREC*)pai->dpvt;

    /* Validate device private pointer */
    if( prLov == NULL)
    {
       pai->pact = LOV__K_ACTIVE;
       asynPrint( prLov->pasynUser, ASYN_TRACE_ERROR, "devAsynLove::ai__read dpvt is NULL \"%s\"\n", pai->name );

       return( LOV__STS_OKNOVAL );
    }

    /* When process record in inactive (pact = 0) */
    if( pai->pact == LOV__K_INACTIVE )
    {
        /* Queue request */
        lov__queueIt( (dbCommon*)pai );

        /* Return completion status */
        return( LOV__STS_OKNOVAL );
    }

    /* When process record is active (pact = 1) */
    if( pai->pact == LOV__K_ACTIVE )
    {
        /* Call IO completion method */
        prLov->prModel->ioCompletion( prLov );

        /* Evaluate ASYN completion status */
        if( ASYN__IS_OK(prLov->sts) )
        {
            pai->val = (prLov->rawData.dData * lov__convFactor[prLov->funcType][prLov->decPts]);
            pai->udf = 0;
        }
        else
        {
            pai->val = 0.0;
            pai->udf = 1;

            recGblSetSevr( pai, READ_ALARM, INVALID_ALARM );
            asynPrint( prLov->pasynUser, ASYN_TRACE_ERROR, "devAsynLove::ai__read failure in \"%s\"\n", pai->name );
        }

        /* Increment record read counter */
        ++lov__recReadCount;
        ++prLov->procCount;

    }

    /* Return completion status (always OKNOVAL) */
    return( LOV__STS_OKNOVAL );

} /* end-method: ai__read() */


/*
 * ao__init()
 *
 * Description:
 *    This method performs the initialization for an AO record.
 *
 * Input Parameters:
 *    pao   - Address of aoRecord.
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
static long ao__init( struct aoRecord* pao )
{
    long sts;


    /* Call common initialization method */
    sts = lov__recordInit( (dbCommon*)pao, &pao->out, outFunc, aoRec );

    /* Return completion status */
    return( sts );

} /* end-method: ao__init() */


/*
 * ao__write()
 *
 * Description:
 *    This method performs the write for an AO record.
 *
 * Input Parameters:
 *    pao   - Address of aiRecord.
 *
 * Output Parameters:
 *    None.
 *
 * Returns:
 *    LOV__STS_OKNOVAL
 *
 * Developer notes:
 *
 */
static long ao__write( struct aoRecord* pao )
{
    rLOVREC* prLov;


    /* Initialize  variants */
    prLov = (rLOVREC*)pao->dpvt;

    /* Validate device private pointer */
    if( prLov == NULL)
    {
       pao->pact = LOV__K_ACTIVE;
       asynPrint( prLov->pasynUser, ASYN_TRACE_ERROR, "devAsynLove::ao__write dpvt is NULL \"%s\"\n", pao->name );

       return( LOV__STS_OKNOVAL );
    }

    /* When process record is inactive (pact = 0) */
    if( pao->pact == LOV__K_INACTIVE )
    {
        /* Acquire write value */
        prLov->rawData.dData = pao->val;

        /* Queue request */
        lov__queueIt( (dbCommon*)pao );

        /* Return completion status */
        return( LOV__STS_OKNOVAL );
    }

    /* When process record is active (pact = 1) */
    if( pao->pact == LOV__K_ACTIVE )
    {
        /* Call IO completion method */
        prLov->prModel->ioCompletion( prLov );

        /* Evaluate ASYN completion status */
        if( ASYN__IS_OK(prLov->sts) )
        {
            pao->rbv = (epicsInt32)(prLov->rawData.dData * prLov->decPts);
            pao->udf = 0;
        }
        else
        {
            pao->rbv = 0;
            pao->udf = 1;

            recGblSetSevr( pao, WRITE_ALARM, INVALID_ALARM );
            asynPrint( prLov->pasynUser, ASYN_TRACE_ERROR, "devAsynLove::ao__write failure in \"%s\"\n", pao->name );
        }

        /* Increment record write counter */
        ++lov__recWritCount;
        ++prLov->procCount;

    }

    /* Return completion status (always OKNOVAL) */
    return( LOV__STS_OKNOVAL );

} /* end-method: ao__write() */


/*
 * bi__init()
 *
 * Description:
 *    This method performs the initialization for a BI record.
 *
 * Input Parameters:
 *    pbi   - Address of the BI record.
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
static long bi__init( struct biRecord* pbi )
{
    long sts;


    /* Call common initialization method */
    sts = lov__recordInit( (dbCommon*)pbi, &pbi->inp, inpFunc, biRec );

    /* Return completion status */
    return( sts );

} /* end-method: bi__init() */


/*
 * bi__read()
 *
 * Description:
 *    This method performs the read for a BI record.
 *
 * Input Parameters:
 *    pbi   - Address of the BI record.
 *
 * Output Parameters:
 *    None.
 *
 * Returns:
 *    LOV__STS_OK
 *    LOV__STS_OKNOVAL
 *
 * Developer notes:
 *
 */
static long bi__read( struct biRecord* pbi )
{
    rLOVREC* prLov;


    /* Initialize local variants */
    prLov = (rLOVREC*)pbi->dpvt;

    /* Validate device private pointer */
    if( prLov == NULL)
    {
       pbi->pact = LOV__K_ACTIVE;
       asynPrint( prLov->pasynUser, ASYN_TRACE_ERROR, "devAsynLove::bi__read dpvt is NULL \"%s\"\n", pbi->name );

       return( LOV__STS_OKNOVAL );
    }

    /* When process record in inactive (pact = 0) */
    if( pbi->pact == LOV__K_INACTIVE )
    {
        /* Queue request */
        lov__queueIt( (dbCommon*)pbi );

        /* Return completion status */
        return( LOV__STS_OK );
    }

    /* When process record is active (pact = 1) */
    if( pbi->pact == LOV__K_ACTIVE )
    {
        /* Call IO completion method */
        prLov->prModel->ioCompletion( prLov );

        /* Evaluate ASYN completion status */
        if( ASYN__IS_OK(prLov->sts) )
        {
            pbi->rval = prLov->rawData.ulData;
            pbi->udf  = 0;
        }
        else
        {
            pbi->val  = 0;
            pbi->udf  = 1;

            recGblSetSevr( pbi, READ_ALARM, INVALID_ALARM );
            asynPrint( prLov->pasynUser, ASYN_TRACE_ERROR, "devAsynLove::bi__read failure in \"%s\"\n", pbi->name );
        }

        /* Increment record read counter */
        ++lov__recReadCount;
        ++prLov->procCount;

    }

    /* Return completion status (always OKVAL) */
    return( LOV__STS_OK );

} /* end-method: bi__read() */


/*
 * bo__init()
 *
 * Description:
 *    This method performs the initialization for a BO record.
 *
 * Input Parameters:
 *    pbo   - Address of BO record.
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
static long bo__init( struct boRecord* pbo )
{
    long sts;


    /* Call common initialization method */
    sts = lov__recordInit( (dbCommon*)pbo, &pbo->out, outFunc, boRec );

    /* Return completion status */
    return( sts );

} /* end-method: bo__init() */


/*
 * bo__write()
 *
 * Description:
 *    This method performs the write for a BO record.
 *
 * Input Parameters:
 *    pbo   - Address of BO record.
 *
 * Output Parameters:
 *    None.
 *
 * Returns:
 *    LOV__STS_OK
 *    LOV__STS_OKNOVAL
 *
 * Developer notes:
 *
 */
static long bo__write( struct boRecord* pbo )
{
    rLOVREC* prLov;


    /* Initialize local variants */
    prLov = (rLOVREC*)pbo->dpvt;

    /* Validate device private pointer */
    if( prLov == NULL)
    {
       pbo->pact = LOV__K_ACTIVE;
       asynPrint( prLov->pasynUser, ASYN_TRACE_ERROR, "devAsynLove::bo__write dpvt is NULL \"%s\"\n", pbo->name );

       return( LOV__STS_OKNOVAL );
    }

    /* When process record in inactive (pact = 0) */
    if( pbo->pact == LOV__K_INACTIVE )
    {
        /* Queue request */
        lov__queueIt( (dbCommon*)pbo );

        /* Return completion status */
        return( LOV__STS_OK );
    }

    /* When process record is active (pact = 1) */
    if( pbo->pact == LOV__K_ACTIVE )
    {
        /* Call IO completion method */
        prLov->prModel->ioCompletion( prLov );

        /* Evaluate ASYN completion status */
        if( ASYN__IS_OK(prLov->sts) )
        {
            pbo->rbv = prLov->rawData.ulData;
            pbo->udf  = 0;
        }
        else
        {
            pbo->rbv  = 0;
            pbo->udf  = 1;

            recGblSetSevr( pbo, WRITE_ALARM, INVALID_ALARM );
            asynPrint( prLov->pasynUser, ASYN_TRACE_ERROR, "devAsynLove::bo__write failure in \"%s\"\n", pbo->name );
        }

        /* Increment record read counter */
        ++lov__recWritCount;
        ++prLov->procCount;

    }

    /* Return completion status (always OKVAL) */
    return( LOV__STS_OK );

} /* end-method: bo__write() */


/*
 * mbbi__init()
 *
 * Description:
 *    This method performs the initialization for a MBBI record.
 *
 * Input Parameters:
 *    pmbbi - Address of MBBI record.
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
static long mbbi__init( struct mbbiRecord* pmbbi )
{
    long sts;


    /* Call common initialization method */
    sts = lov__recordInit( (dbCommon*)pmbbi, &pmbbi->inp, inpFunc, mbbiRec );

    /* Return completion status */
    return( sts );

} /* end-method: mbbi__init() */


/*
 * mbbi__read()
 *
 * Description:
 *    This method performs the read for a MBBI record.
 *
 * Input Parameters:
 *    pmbbi    - Address of MBBI record.
 *
 * Output Parameters:
 *    None.
 *
 * Returns:
 *    LOV__STS_OK
 *    LOV__STS_OKNOVAL
 *
 * Developer notes:
 *
 */
static long mbbi__read( struct mbbiRecord* pmbbi )
{
    rLOVREC* prLov;


    /* Initialize local variants */
    prLov = (rLOVREC*)pmbbi->dpvt;

    /* Validate device private pointer */
    if( prLov == NULL)
    {
       pmbbi->pact = LOV__K_ACTIVE;
       asynPrint( prLov->pasynUser, ASYN_TRACE_ERROR, "devAsynLove::mbbi__read dpvt is NULL \"%s\"\n", pmbbi->name );

       return( LOV__STS_OKNOVAL );
    }

    /* When process record in inactive (pact = 0) */
    if( pmbbi->pact == LOV__K_INACTIVE )
    {
        /* Queue request */
        lov__queueIt( (dbCommon*)pmbbi );

        /* Return completion status */
        return( LOV__STS_OK );
    }

    /* When process record is active (pact = 1) */
    if( pmbbi->pact == LOV__K_ACTIVE )
    {
        /* Call IO completion method */
        prLov->prModel->ioCompletion( prLov );

        /* Evaluate ASYN completion status */
        if( ASYN__IS_OK(prLov->sts) )
        {
            pmbbi->rval = prLov->rawData.ulData;
            pmbbi->udf  = 0;
        }
        else
        {
            pmbbi->val  = 0;
            pmbbi->udf  = 1;

            recGblSetSevr( pmbbi, READ_ALARM, INVALID_ALARM );
            asynPrint( prLov->pasynUser, ASYN_TRACE_ERROR, "devAsynLove::mbbi__read failure in \"%s\"\n", pmbbi->name );
        }

        /* Increment record read counter */
        ++lov__recReadCount;
        ++prLov->procCount;

    }

    /* Return completion status (always OKVAL) */
    return( LOV__STS_OK );

} /* end-method: mbbi__read() */

