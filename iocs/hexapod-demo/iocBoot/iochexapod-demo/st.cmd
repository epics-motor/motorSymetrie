< envPaths

cd "$(TOP)"

dbLoadDatabase "dbd/hexapodDemo.dbd"
hexapodDemo_registerRecordDeviceDriver(pdbbase)

# Create SSH Port (PortName, IPAddress, Username, Password, Priority, DisableAutoConnect, noProcessEos)
#drvAsynPowerPMACPortConfigure("PPMAC_SSH", "192.168.56.10", "root", "deltatau", "0", "0", "0")
drvAsynPowerPMACPortConfigure("PPMAC_SSH", "10.54.160.42", "root", "deltatau", "0", "0", "0")

# Configure Symetrie Hexapod Controller Driver (ControllerPort, LowLevelDriverPort, Address)
symetrieHexapod("HEXAPOD", "PPMAC_SSH", 0)

dbLoadRecords "$(MOTOR)/db/SymetriePmac.template", "P=SYM:,R=HEX01:,PORT=HEXAPOD"
iocInit
