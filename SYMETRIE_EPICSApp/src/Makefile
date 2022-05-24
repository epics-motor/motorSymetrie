TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC_Linux += symetrie

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install SYMETRIE_EPICS.dbd into <top>/dbd
DBD += SYMETRIE_EPICS.dbd

INC += SymetrieHexapod.h

# specify all source files to be compiled and added to the library
symetrie_SRCS += SymetrieHexapod.cpp
symetrie_LIBS += asyn
symetrie_LIBS += motor
symetrie_LIBS += pmacAsynMotorPort
symetrie_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

