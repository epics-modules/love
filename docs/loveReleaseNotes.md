---
layout: default
title: Release Notes
nav_order: 2
---

# Release Notes
{: .no_toc}

## Table of contents
{: .no_toc .text-delta}

- TOC
{:toc}

## Release 3-2-10

- **Automated build dependencies** -- Added cfg-based dependency
  declarations so downstream IOC applications can automatically
  discover the Love module's DBD files and libraries.

## Release 3-2-9

- **Documentation** -- Converted to GitHub Pages.

## Release 3-2-8

- **iocsh files** -- Installed iocsh files to top-level folder from
  `loveApp/iocsh`.

## Release 3-2-7

- **iocsh startup files** -- Added iocsh startup files.
- **req files** -- Installed req files to top-level `db` folder.

## Release 3-2-5

- **RELEASE cleanup** -- Modified RELEASE; deleted RELEASE.arch.
- **CSS-BOY screens** -- Added `.opi` files for CSS-BOY.

## Release 3-2-3

- **Communication timeout** -- Corrected a problem when communication
  would timeout.
- **MEDM screens** -- Modified MEDM screen field colors.
- **Save/restore** -- Added `.req` file for save/restore.

## Release 3-2-2

- **MEDM screen** -- Modified primary MEDM screen.
- **Linux startup** -- Modified startup scripts for Linux.
- **devLove.dbd** -- Added `devLove.dbd` which is necessary for other
  components/applications to have Love Controller support.

## Release 3-2-0

- **Asyn interfaces** -- Module `drvLove` implements the standard asyn
  interfaces `asynInt32`, `asynUInt32Digital`, and `asynDrvUser`.
- **MEDM screens** -- Enhanced MEDM screens for diagnostics.
- **Database** -- Updated database for monitoring and setting
  controllers.
- **Cleanup** -- Removed all unused files (sources, databases, MEDM
  screens).

## Release 3-1-0

- **Multi-device port driver** -- Added module `drvLove`.
- **Love-specific interface** -- Added module `ifaceLove`.
- **Device support** -- Added module `devLove`.
- **Record types** -- Supports ai, ao, bi, bo, and mbbi record types;
  removed support for longin.
- **MEDM screens** -- Enhanced MEDM screens for diagnostics.
- **INP/OUT format** -- Format of INP/OUT field has been modified.
- **Cleanup** -- Removed all unused files.

## Release 3-0-1

- **longin support** -- Added support for the longin record type.
- **MEDM screens** -- Modified MEDM screens for accuracy.
- **Screen location** -- Changed MEDM screen location for consistency
  with other synApps modules.
- **Portability** -- Corrected a problem where loveApp would not build
  under Solaris and other operating systems.
- **Code review** -- Applied comments and suggestions from code
  inspections.

## Release 3-0-0

New asyn-based support for Love controllers. Tested with Base
3.14.6/3.14.7, asyn 4.2, and ipac 2.7a/2.8.

- **Asyn device support** -- Added asyn device support for ao, ai, bo,
  bi, and mbbi record types.
- **Lovelink interpose** -- Added asyn interpose interface for
  Lovelink.
- **Example application** -- Provided sample application and database
  for vxWorks and Linux.
- **Startup scripts** -- Provided startup scripts for vxWorks and
  Linux to configure ipac, asyn, and the interpose interface.
- **MEDM displays** -- Provided MEDM displays for the sample
  application.
- **Base 3.14** -- Requires EPICS Base 3.14.6 or greater.
- **MPF removed** -- MPF is no longer supported; source code preserved
  for historical purposes only.

## Release 2-0-0

- **Base 3.14** -- Converted to EPICS 3.14 (vxWorks only).

## Release 1-3-0

- **MPF update** -- Support for MPF R1-10's new "bind" call for
  DevMpf.
