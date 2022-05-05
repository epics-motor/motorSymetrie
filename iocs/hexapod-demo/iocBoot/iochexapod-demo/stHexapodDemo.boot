< envPaths

cd "$(TOP)"

epicsEnvSet "EPICS_TS_MIN_WEST", '0'


# Loading libraries
# -----------------

# Device initialisation
# ---------------------

cd "$(TOP)"

dbLoadDatabase "dbd/hexapodDemo.dbd"
hexapodDemo_registerRecordDeviceDriver(pdbbase)

# Create SSH Port (PortName, IPAddress, Username, Password, Priority, DisableAutoConnect, noProcessEos)
#drvAsynPowerPMACPortConfigure("PPMAC_SSH", "192.168.56.10", "root", "deltatau", "0", "0", "0")
drvAsynPowerPMACPortConfigure("PPMAC_SSH", "10.2.2.199", "root", "deltatau", "0", "0", "0")

# Configure Symetrie Hexapod Controller Driver (ControllerPort, LowLevelDriverPort, Address)
symetrieHexapod("HEXAPOD", "PPMAC_SSH", 0)

# Final ioc initialisation
# ------------------------
cd "$(TOP)"
dbLoadRecords 'db/hexapodDemo_expanded.db'
iocInit
