# Makefile
TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

DBD += devLove.dbd
DBD += devAsynLove.dbd

LIBRARY_IOC_vxWorks += love

love_SRCS += devLove.cc
love_SRCS += loveServer.cc
love_SRCS += devAsynLove.c
love_SRCS += interposeLovelink.c
love_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

