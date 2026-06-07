---
layout: default
title: Overview
nav_order: 1
---

# Love Controller Driver Support
{: .no_toc}

## Table of contents
{: .no_toc .text-delta}

- TOC
{:toc}

## Description

The love module is part of
[synApps](https://github.com/BCDA-APS/synApps) and provides EPICS
support for Love temperature and process controllers. It includes an
asyn-based multi-device port driver, EPICS databases, operator
interface screens, and an iocsh configuration snippet.

The Love Controller is an instrument that can monitor temperature and
pressure, serve as a thermocouple, or control pressure, flow,
humidity, motion, or pH given the proper hardware. Control functions
such as selecting the input type are programmed from the front panel.
Communication with the controller uses a half-duplex, multi-drop,
RS-485 serial bus.

## Supported Hardware

Two controller models are supported:

| Model | Documentation |
| - | - |
| 1600 | [User Manual](1600/1600_Documentation.pdf), [Communication Protocol](1600/1600_CommProtocol.pdf) |
| 16A | [User Manual](16A/16A_Documentation.pdf), [Communication Protocol](16A/16A_CommProtocol.pdf), [Data Sheet](16A/16A_DataSheet.pdf) |

An RS-232 to RS-485 converter such as the
[B&B Electronics 485LDRC](485LDRC/485LDRC_Datasheet.pdf) is needed to
connect the controllers to a serial port.

## Architecture

The driver (`drvLove`) is an asyn multi-device port driver that binds
standard EPICS device support with the serial bus through
`drvAsynSerialPort`. It implements the following asyn interfaces:

- **asynDrvUser** -- allows user-specific information to be
  communicated to and from the port driver.
- **asynCommon** -- driver reporting, connection, and disconnection.
- **asynInt32** -- integer-based communication with a device.
- **asynUInt32Digital** -- bit-level communication via an Int32
  register.

Each controller on the RS-485 bus is addressed individually. The
asynRecord can be used to select tracing masks for debugging
communication with individual controllers.

## Dependencies

| Module | Required | Notes |
| - | - | - |
| [asyn](https://github.com/epics-modules/asyn) | Yes | Serial port driver and asyn interfaces |
| [ipac](https://github.com/epics-modules/ipac) | vxWorks only | Industry Pack carrier and IP-Octal RS-485 module support |
