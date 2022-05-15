< envPaths

dbLoadDatabase "$(TOP)/dbd/hexapodDemo.dbd"
hexapodDemo_registerRecordDeviceDriver(pdbbase)

# Create SSH Port (PortName, IPAddress, Username, Password, Priority, DisableAutoConnect, noProcessEos)
drvAsynPowerPMACPortConfigure("PPMAC_SSH", "10.54.160.42", "root", "deltatau", "0", "0", "0")

# Configure Symetrie Hexapod Controller Driver (ControllerPort, LowLevelDriverPort, Address, MovingPollPeriod, IdlePollPeriod)
symetrieHexapod("HEXAPOD", "PPMAC_SSH", 0, .05, 1)

dbLoadTemplate "$(MOTOR)/db/SymetriePmac.substitutions", "P=SYM:,R=HEX01:,PORT=HEXAPOD"
dbLoadTemplate "motor.substitutions"
iocInit
