---
layout: default
title: Release Notes
nav_order: 4
---


love Release Notes
------------------

This document describes the changes which have been made to the synApps / Love controller support software. The earliest version appears at the bottom, with more recent releases above it.

Release 3-2-9
-------------

*   Documentation converted to github pages

Release 3-2-8
-------------

*   iocsh files installed to top level folder from loveApp/iocsh

Release 3-2-7
-------------

*   Added iocsh startup files
*   req files installed to top level db folder

Release 3-2-5
-------------

*   Modified RELEASE; deleted RELEASE.arch
*   Added .opi files for CSS-BOY

Release 3-2-3
-------------

This major version has minor changes:

Modified:

*   Corrected a problem when communication would timeout.
    
*   Modified MEDM screen field colors.
    
*   Added .req file for save/restore.
    

Release 3-2-2
-------------

This major version has minor changes:

Modified:

*   Primary MEDM screen.
    
*   Modified startup scripts for Linux.
    

  

Changed:

*   Added devLove.dbd which is necessary for other components/applications to have Love Controller support.
    

Release 3-2-0
-------------

This major version has significant architecture changes and provides the following:

Modified:

*   Module drvLove implements the standard Asyn intefaces asynInt32, asynUInt32Digitial, and asynDrvUser,
    

  

Changed:

*   Enhanced MEDM screens for diagnostics,
    
*   Database for monitoring and setting controllers,
    
*   Removed all unused files (i.e. sources, databases, MEDM screens),
    

Release 3-1-0
-------------

This major version has significant architecture changes and provides the following:

Added:

*   Module drvLove, multidevice port driver,
    

*   Module ifaceLove, Love-specific interface,
    
*   Module devLove, device support,
    

  

Changed:

*   Supports the ai,ao,bi,bo, and mbbi record types, removed support for longin,
    
*   Enhanced MEDM screens for diagnostics,
    
*   Format of INP/OUT field has been modified, refer to database definition files,
    
*   Removed all unused files (i.e. sources, databases, MEDM screens),
    

Release 3-0-1
-------------

This minor version provides the following:

Added:

*   Support for the longin record type,
    

  

Changed:

*   Modified MEDM screens for accuracy,
    
*   The location of the MEDM screens has been changed for consistency with other synApps modules,
    
*   Corrected a problem where the loveApp would not build under Solaris and other operating systems,
    
*   Applied comments / suggestions from code inspections,
    

Release 3-0-0
-------------

This major version provides new support for the love controllers based on Asyn. It was tested using base 3.14.6 and 3.14.7, Asyn 4.2, and Ipac 2.7a and 2.8.

Added:

*   Asyn device support for the ao, ai, bo, bi, and mbbi record types,
    
*   Asyn Interpose interface for Lovelink,
    
*   An sample application and database is provided that works under vxWorks and Linux,
    
*   Startup scripts for vxWorks and Linux are provided to configure Ipac, Asyn, and the Interpose interface,
    
*   MEDM displays are provided to compliment the sample application,
    

  

Changed:

  

*   Supported for EPICS base 3.14.6 or greater,
    
*   MPF is no longer supported; however, the source code is preserved only for historical purposes,
    

Release 2-0-0
-------------

Converted to EPICS 3.14 (vxWorks only).

Release 1-3-0
-------------

Support for MPF R1-10's new "bind" call for DevMpf.

