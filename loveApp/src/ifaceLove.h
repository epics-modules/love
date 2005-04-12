#ifndef ifaceLoveH
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
    This module defines an interface for communication with the Love
    controllers. It resides between device support (devLove) and the
    drive (drvLove) to provide a method of hiding processing details
    specific to a model.

    The method ifaceLoveInit must first be called to initialize the
    interface. It is called from the startup script and has the following
    calling sequence.

        ifaceLoveInit(port)

        Where:  port - Love port driver name (i.e. "L0" )

    Next, the method ifaceLoveConfigure must be called from the startup
    script and has the following calling sequence:

        ifaceLoveConfig(port,addr,model,decpts)

        Where:  port   - Love port driver name (i.e. "L0" )
                addr   - Love controller address
                model  - Model type (supported types are "1600" and "16A")
                decpts - Number of decimal points used

    Device support should use the methods ifaceLoveRead or ifaceLoveWrite
    as examples of how a record instance communicates with a Love controller.
    Furthermore, the supported string commands are listed in the CmdTbl
    located in ifaceLove.c.

 Source control info:
    Modified by:    $Author: dkline $
                    $Date: 2005-04-12 17:40:31 $
                    $Revision: 1.1 $

 =============================================================================
 History:
 Author: David M. Kline
 ------------------.-----------------------------------------------------------
 2005-Apr-01  DMK  Begin.
 -----------------------------------------------------------------------------

*/


#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* Declare love interface */
#define ifaceLoveType "ifaceLove"

typedef struct ifaceLove
{
    asynStatus (*read)(void* drvPvt,asynUser* pasynUser,double* value);
    asynStatus (*write)(void* drvPvt,asynUser* pasynUser,double value);
    asynStatus (*setupUser)(void* drvPvt,asynUser* pasynUser,const char* cmd);
} ifaceLove;

extern int ifaceLoveInit(const char* port);
extern int ifaceLoveRead(const char* port,int addr,const char* cmd);
extern int ifaceLoveWrite(const char* port,int addr,const char* cmd,double value);
extern int ifaceLoveConfig(const char* port,int addr,const char* model,int decpts);

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* ifaceLoveH */
