TOP=../..

include $(TOP)/configure/CONFIG

PROD_NAME = loveExApp
PROD_IOC = $(PROD_NAME)

DBD += $(PROD_NAME).dbd

# Love support (via cfg-deps)
$(PROD_NAME)_DBD += $(LOVE_IOC_DBDS)

# Asyn serial port support
$(PROD_NAME)_DBD += drvAsynSerialPort.dbd

# vxWorks IPAC/tyGSOctal support
$(PROD_NAME)_DBD_vxWorks += drvIpac.dbd
$(PROD_NAME)_DBD_vxWorks += tyGSOctal.dbd

$(PROD_NAME)_SRCS += $(PROD_NAME)_registerRecordDeviceDriver.cpp
$(PROD_NAME)_SRCS_DEFAULT += $(PROD_NAME)Main.cpp
$(PROD_NAME)_SRCS_vxWorks += -nil-

$(PROD_NAME)_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

$(PROD_NAME)_LIBS += $(LOVE_IOC_LIBS)
$(PROD_NAME)_LIBS += asyn
$(PROD_NAME)_LIBS_vxWorks += TyGSOctal Ipac
$(PROD_NAME)_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
