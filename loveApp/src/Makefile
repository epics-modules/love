# Makefile
TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

DBD += devLove.dbd

LIBRARY_IOC_DEFAULT += love
LIBRARY_IOC_solaris += -nil-

love_SRCS += devLove.cc loveServer.cc
love_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

