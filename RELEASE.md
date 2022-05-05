symetrie_hexapod Releases
=========================

The versions of EPICS base, asyn, and motor modules used for
each release can be obtained from configure/RELEASE.local. The file
configure/RELEASE.linux-x86_64.Common should be modified to reflect
site location of dependencies.


Release Notes
=========================

5.0.210830
======================
Update to offer compatibility with SYMETRIE positionning hexapod API version 5.0.210830
Add:
• API: Error codes: from 34 to 36
• API: Command returns: -92; -1017

5.0.201028
======================
Update to offer compatibility with SYMETRIE positionning hexapod API version 5.0.201028
Add:
• API: CFG_POWER status command
• API: Error codes: from 30 to 33.
• API: Command returns: from -83 to -90.
Update:
• API: POWER command: modification of the command arguments.
• API: CFG_CONTROL command: add of delay parameter on c_par(1).

5.0.191103
======================
Creation.
Compatible with SYMETRIE positionning hexapod API version 5.0.191103