TOP = ../..
include $(TOP)/configure/CONFIG

PROD_IOC = hexapodDemo
DBD += hexapodDemo.dbd
hexapodDemo_DBD += base.dbd
hexapodDemo_DBD += asyn.dbd
hexapodDemo_DBD += drvAsynPowerPMACPort.dbd
hexapodDemo_DBD += motorSupport.dbd
hexapodDemo_DBD += devSoftMotor.dbd
hexapodDemo_DBD += pmacAsynMotorPort.dbd
hexapodDemo_DBD += SYMETRIE_EPICS.dbd
hexapodDemo_DBD += busySupport.dbd
hexapodDemo_DBD += calcSupport.dbd
hexapodDemo_SRCS += hexapodDemo_registerRecordDeviceDriver.cpp
hexapodDemo_LIBS += calc
hexapodDemo_LIBS += busy
hexapodDemo_LIBS += pmacAsynMotorPort
hexapodDemo_LIBS += softMotor
hexapodDemo_LIBS += motor
hexapodDemo_LIBS += powerPmacAsynPort
hexapodDemo_LIBS += symetrie
hexapodDemo_LIBS += asyn
hexapodDemo_LIBS += $(EPICS_BASE_IOC_LIBS)
hexapodDemo_SRCS += hexapodDemoMain.cpp

include $(TOP)/configure/RULES

