TOP=../..

include $(TOP)/configure/CONFIG
#-----------------------------------------------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================================================================


#=============================================================================
# Build an IOC support library
LIBRARY_IOC += love

CFG += LOVE_DEPS

#-----------------------------------------------------------------------------
# Install database definition files into <top>/dbd
DBD += devLove.dbd

#-----------------------------------------------------------------------------
# The following are compiled and added to the Support library
love_SRCS += drvLove.c

love_LIBS += asyn
love_LIBS += $(EPICS_BASE_IOC_LIBS)
#
#==============================================================================

include $(TOP)/configure/RULES
#------------------------------------------------------------------------------
#  ADD RULES AFTER THIS LINE

