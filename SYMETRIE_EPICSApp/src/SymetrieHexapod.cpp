/********************************************
 *  pmacController.cpp
 *
 *  PMAC Asyn motor based on the
 *  asynMotorController class.
 *
 *  Matthew Pearson
 *  23 May 2012
 *
 ********************************************/
#include <epicsString.h>
#include <epicsMath.h>
#include <iocsh.h>
#include <drvSup.h>
#include <registryFunction.h>
#include <sstream>
#include <epicsExport.h>
#include "SymetrieHexapod.h"

#define CHAR_ARRAY_SIZE 2048

const char *errors[37] = {
  "No error",
  "An emergency stop has been pressed.",
  "A safety input has been triggered. The status of the inputs is given in the DATA field.",
  "A temperature sensor has exceeded the limit threshold. The sensor number is given in Axis field. The temperature which triggered the error is indicated in DATA2.",
  "Controller system status error (Sys.Status).",
  "Controller \"abort all\" input has been triggered. (Sys.AbortAll).",
  "Controller watchdog error (Sys.WDTFault).",
  "Configuration load error. The error is defined as \"fatal\" to forbid any movement before actions is made on the configuration. To recover from this error a sequence of command is required: the CFG_DEFAULT and the CFG_SAVE commands. Using this sequence of commands, this error should not be present at the next controller start-up.",
  "Configuration failed: a wrong hexapod ID has been detected. Detected ID is given in DATA field.",
  "Home task has failed.",
  "Virtual home write task has failed.",
  "The motion program did not start in the defined timeout.",
  "The home task did not start in the defined timeout.",
  "A kinematic error has occurred. Kinematic error number is given in DATA field.",
  "Controller coordinate error status (Coord.ErrorStatus). Error number is given in DATA field.",
  "An error has been detected on an encoder. The DATA field is a bitfield. It indicates which encoder security processes have detected some encoder errors. (DATA: bit0:MotorEncLoss, bit1:CountError, bit2:SerialEncErrorBits, bit3:MotorAuxFault, bit4:PosIsAnError, bit5:NoChangeCount).",
  "Brake should have been engaged as the motor control was off.",
  "Controller motor status: Auxiliary fault (AuxFault).",
  "Controller motor status: Encoder loss (EncLoss).",
  "Controller motor status: Amplifier warning (AmpWarn).",
  "Controller motor status: Trigger not found (TriggerNotFound).",
  "Controller motor status: Integrated current \"I2T\" fault (I2tFault).",
  "Controller motor status: Software positive limit reach (SoftPlusLimit).",
  "Controller motor status: Software negative limit reach (SoftMinusLimit).",
  "Controller motor status: Amplifier fault (AmpFault).",
  "Controller motor status: Stopped on hardware limit (LimitStop).",
  "Controller motor status: Fatal following error (FeFatal).",
  "Controller motor status: Warning following error (FeWarn).",
  "Controller motor status: Hardware positive limit reach (PlusLimit).",
  "Controller motor status: Hardware negative limit reach (MinusLimit).",
  "An application error has occurred. Please refer to the hardware user manual to get more details on this error.",
  "The next sequence position is not valid. The validation code is given in the DATA field. The sequence position index is given in DATA2 field.",
  "The motor PTC thermistor input has been triggered.",
  "The channel assignment has been changed successfully. When controller is turn off, operator should change the machine wiring to match the new configuration. New assignment will be active on the next controller restart.",
  "A temperature sensor has exceeded the warning limit threshold. The sensor number is given in Axis field. The temperature which triggered the warning is indicated in DATA2.",
  "The reading of a temperature sensor value failed.",
  "An error occurred on the Modbus client interface."
};

//C function prototypes, for the functions that can be called on IOC shell.
extern "C"
{
asynStatus symHexCreateController(const char *portName,
                                  const char *lowLevelPortName,
                                  int lowLevelPortAddress,
                                  double movingPollPeriod,
                                  double idlePollPeriod);
}

/**
 * SymetrieHexapod constructor.
 * @param portName The Asyn port name to use (that the motor record connects to).
 * @param lowLevelPortName The name of the low level port that has already been created, to enable comms to the controller.
 * @param lowLevelPortAddress The asyn address for the low level port
 * @param movingPollPeriod The time (in milliseconds) between polling when axes are moving
 * @param movingPollPeriod The time (in milliseconds) between polling when axes are idle
 */
SymetrieHexapod::SymetrieHexapod(const char *portName,
                                 const char *lowLevelPortName,
                                 int lowLevelPortAddress,
                                 double movingPollPeriod, 
                                 double idlePollPeriod)
        : pmacController(portName,
                         lowLevelPortName,
                         lowLevelPortAddress,
                         20, // Not sure what this channels argument does
                         movingPollPeriod,
                         idlePollPeriod),
        movingPollPeriod_(movingPollPeriod),
        idlePollPeriod_(idlePollPeriod)
{
  static const char *functionName = "SymetrieHexapod::SymetrieHexapod";
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s Constructor.\n", functionName);

  current_error_ = 0;
  no_of_errors_ = 0;

  // Create Symetrie Hexapod specific parameters
  createParam(SYM_HEX_FirstParamString, asynParamInt32, &SYM_HEX_FirstParam_);

  // Create the status parameters
  createParam(SYM_HEX_STOPString,        asynParamInt32,   &SYM_HEX_STOP_);
  createParam(SYM_HEX_CONTROLONString,   asynParamInt32,   &SYM_HEX_CONTROLON_);
  createParam(SYM_HEX_CONTROLOFFString,  asynParamInt32,   &SYM_HEX_CONTROLOFF_);
  createParam(SYM_HEX_HOMEString,        asynParamInt32,   &SYM_HEX_HOME_);
  createParam(SYM_HEX_HOMEVIRTUALString, asynParamInt32,   &SYM_HEX_HOMEVIRTUAL_);
  createParam(SYM_HEX_CLEARERRORString,  asynParamInt32,   &SYM_HEX_CLEARERROR_);
  createParam(SYM_HEX_STATUS_READString, asynParamInt32,   &SYM_HEX_STATUS_READ_);
  createParam(SYM_HEX_S_HEXAString,      asynParamInt32,   &SYM_HEX_S_HEXA_);
  createParam(SYM_HEX_S_ACTIONString,    asynParamInt32,   &SYM_HEX_S_ACTION_);
  createParam(SYM_HEX_S_UTO_TXString,    asynParamFloat64, &SYM_HEX_S_UTO_TX_);
  createParam(SYM_HEX_S_UTO_TYString,    asynParamFloat64, &SYM_HEX_S_UTO_TY_);
  createParam(SYM_HEX_S_UTO_TZString,    asynParamFloat64, &SYM_HEX_S_UTO_TZ_);
  createParam(SYM_HEX_S_UTO_RXString,    asynParamFloat64, &SYM_HEX_S_UTO_RX_);
  createParam(SYM_HEX_S_UTO_RYString,    asynParamFloat64, &SYM_HEX_S_UTO_RY_);
  createParam(SYM_HEX_S_UTO_RZString,    asynParamFloat64, &SYM_HEX_S_UTO_RZ_);
  createParam(SYM_HEX_S_MTP_TXString,    asynParamFloat64, &SYM_HEX_S_MTP_TX_);
  createParam(SYM_HEX_S_MTP_TYString,    asynParamFloat64, &SYM_HEX_S_MTP_TY_);
  createParam(SYM_HEX_S_MTP_TZString,    asynParamFloat64, &SYM_HEX_S_MTP_TZ_);
  createParam(SYM_HEX_S_MTP_RXString,    asynParamFloat64, &SYM_HEX_S_MTP_RX_);
  createParam(SYM_HEX_S_MTP_RYString,    asynParamFloat64, &SYM_HEX_S_MTP_RY_);
  createParam(SYM_HEX_S_MTP_RZString,    asynParamFloat64, &SYM_HEX_S_MTP_RZ_);
  createParam(SYM_HEX_S_AX_1String,      asynParamInt32,   &SYM_HEX_S_AX_1_);
  createParam(SYM_HEX_S_AX_2String,      asynParamInt32,   &SYM_HEX_S_AX_2_);
  createParam(SYM_HEX_S_AX_3String,      asynParamInt32,   &SYM_HEX_S_AX_3_);
  createParam(SYM_HEX_S_AX_4String,      asynParamInt32,   &SYM_HEX_S_AX_4_);
  createParam(SYM_HEX_S_AX_5String,      asynParamInt32,   &SYM_HEX_S_AX_5_);
  createParam(SYM_HEX_S_AX_6String,      asynParamInt32,   &SYM_HEX_S_AX_6_);
  createParam(SYM_HEX_S_POS_AX_1String,  asynParamFloat64, &SYM_HEX_S_POS_AX_1_);
  createParam(SYM_HEX_S_POS_AX_2String,  asynParamFloat64, &SYM_HEX_S_POS_AX_2_);
  createParam(SYM_HEX_S_POS_AX_3String,  asynParamFloat64, &SYM_HEX_S_POS_AX_3_);
  createParam(SYM_HEX_S_POS_AX_4String,  asynParamFloat64, &SYM_HEX_S_POS_AX_4_);
  createParam(SYM_HEX_S_POS_AX_5String,  asynParamFloat64, &SYM_HEX_S_POS_AX_5_);
  createParam(SYM_HEX_S_POS_AX_6String,  asynParamFloat64, &SYM_HEX_S_POS_AX_6_);
  createParam(SYM_HEX_S_DIO_1String,     asynParamInt32,   &SYM_HEX_S_DIO_1_);
  createParam(SYM_HEX_S_DIO_2String,     asynParamInt32,   &SYM_HEX_S_DIO_2_);
  createParam(SYM_HEX_S_DIO_3String,     asynParamInt32,   &SYM_HEX_S_DIO_3_);
  createParam(SYM_HEX_S_DIO_4String,     asynParamInt32,   &SYM_HEX_S_DIO_4_);
  createParam(SYM_HEX_S_DIO_5String,     asynParamInt32,   &SYM_HEX_S_DIO_5_);
  createParam(SYM_HEX_S_DIO_6String,     asynParamInt32,   &SYM_HEX_S_DIO_6_);
  createParam(SYM_HEX_S_DIO_7String,     asynParamInt32,   &SYM_HEX_S_DIO_7_);
  createParam(SYM_HEX_S_DIO_8String,     asynParamInt32,   &SYM_HEX_S_DIO_8_);
  createParam(SYM_HEX_S_AI_1String,      asynParamFloat64, &SYM_HEX_S_AI_1_);
  createParam(SYM_HEX_S_AI_2String,      asynParamFloat64, &SYM_HEX_S_AI_2_);
  createParam(SYM_HEX_S_AI_3String,      asynParamFloat64, &SYM_HEX_S_AI_3_);
  createParam(SYM_HEX_S_AI_4String,      asynParamFloat64, &SYM_HEX_S_AI_4_);
  createParam(SYM_HEX_S_AI_5String,      asynParamFloat64, &SYM_HEX_S_AI_5_);
  createParam(SYM_HEX_S_AI_6String,      asynParamFloat64, &SYM_HEX_S_AI_6_);
  createParam(SYM_HEX_S_AI_7String,      asynParamFloat64, &SYM_HEX_S_AI_7_);
  createParam(SYM_HEX_S_AI_8String,      asynParamFloat64, &SYM_HEX_S_AI_8_);
  createParam(SYM_HEX_S_CYCLEString,     asynParamInt32,   &SYM_HEX_S_CYCLE_);
  createParam(SYM_HEX_S_INDEXString,     asynParamInt32,   &SYM_HEX_S_INDEX_);
  createParam(SYM_HEX_S_ERR_NRString,    asynParamInt32,   &SYM_HEX_S_ERR_NR_);

  // Configuration demand parameters
  createParam(SYM_HEX_VT_DMDString,              asynParamFloat64, &SYM_HEX_VT_DMD_);
  createParam(SYM_HEX_VR_DMDString,              asynParamFloat64, &SYM_HEX_VR_DMD_);
  createParam(SYM_HEX_TA_DMDString,              asynParamFloat64, &SYM_HEX_TA_DMD_);
  createParam(SYM_HEX_TXU_DMDString,             asynParamFloat64, &SYM_HEX_TXU_DMD_);
  createParam(SYM_HEX_TYU_DMDString,             asynParamFloat64, &SYM_HEX_TYU_DMD_);
  createParam(SYM_HEX_TZU_DMDString,             asynParamFloat64, &SYM_HEX_TZU_DMD_);
  createParam(SYM_HEX_RXU_DMDString,             asynParamFloat64, &SYM_HEX_RXU_DMD_);
  createParam(SYM_HEX_RYU_DMDString,             asynParamFloat64, &SYM_HEX_RYU_DMD_);
  createParam(SYM_HEX_RZU_DMDString,             asynParamFloat64, &SYM_HEX_RZU_DMD_);
  createParam(SYM_HEX_TXO_DMDString,             asynParamFloat64, &SYM_HEX_TXO_DMD_);
  createParam(SYM_HEX_TYO_DMDString,             asynParamFloat64, &SYM_HEX_TYO_DMD_);
  createParam(SYM_HEX_TZO_DMDString,             asynParamFloat64, &SYM_HEX_TZO_DMD_);
  createParam(SYM_HEX_RXO_DMDString,             asynParamFloat64, &SYM_HEX_RXO_DMD_);
  createParam(SYM_HEX_RYO_DMDString,             asynParamFloat64, &SYM_HEX_RYO_DMD_);
  createParam(SYM_HEX_RZO_DMDString,             asynParamFloat64, &SYM_HEX_RZO_DMD_);
  createParam(SYM_HEX_MLIM_ENABLE_DMDString,     asynParamInt32,   &SYM_HEX_MLIM_ENABLE_DMD_);
  createParam(SYM_HEX_MLIM_TX_NEG_DMDString,     asynParamFloat64, &SYM_HEX_MLIM_TX_NEG_DMD_);
  createParam(SYM_HEX_MLIM_TY_NEG_DMDString,     asynParamFloat64, &SYM_HEX_MLIM_TY_NEG_DMD_);
  createParam(SYM_HEX_MLIM_TZ_NEG_DMDString,     asynParamFloat64, &SYM_HEX_MLIM_TZ_NEG_DMD_);
  createParam(SYM_HEX_MLIM_RX_NEG_DMDString,     asynParamFloat64, &SYM_HEX_MLIM_RX_NEG_DMD_);
  createParam(SYM_HEX_MLIM_RY_NEG_DMDString,     asynParamFloat64, &SYM_HEX_MLIM_RY_NEG_DMD_);
  createParam(SYM_HEX_MLIM_RZ_NEG_DMDString,     asynParamFloat64, &SYM_HEX_MLIM_RZ_NEG_DMD_);
  createParam(SYM_HEX_MLIM_TX_POS_DMDString,     asynParamFloat64, &SYM_HEX_MLIM_TX_POS_DMD_);
  createParam(SYM_HEX_MLIM_TY_POS_DMDString,     asynParamFloat64, &SYM_HEX_MLIM_TY_POS_DMD_);
  createParam(SYM_HEX_MLIM_TZ_POS_DMDString,     asynParamFloat64, &SYM_HEX_MLIM_TZ_POS_DMD_);
  createParam(SYM_HEX_MLIM_RX_POS_DMDString,     asynParamFloat64, &SYM_HEX_MLIM_RX_POS_DMD_);
  createParam(SYM_HEX_MLIM_RY_POS_DMDString,     asynParamFloat64, &SYM_HEX_MLIM_RY_POS_DMD_);
  createParam(SYM_HEX_MLIM_RZ_POS_DMDString,     asynParamFloat64, &SYM_HEX_MLIM_RZ_POS_DMD_);
  createParam(SYM_HEX_ULIM_ENABLE_DMDString,     asynParamInt32,   &SYM_HEX_ULIM_ENABLE_DMD_);
  createParam(SYM_HEX_ULIM_TX_NEG_DMDString,     asynParamFloat64, &SYM_HEX_ULIM_TX_NEG_DMD_);
  createParam(SYM_HEX_ULIM_TY_NEG_DMDString,     asynParamFloat64, &SYM_HEX_ULIM_TY_NEG_DMD_);
  createParam(SYM_HEX_ULIM_TZ_NEG_DMDString,     asynParamFloat64, &SYM_HEX_ULIM_TZ_NEG_DMD_);
  createParam(SYM_HEX_ULIM_RX_NEG_DMDString,     asynParamFloat64, &SYM_HEX_ULIM_RX_NEG_DMD_);
  createParam(SYM_HEX_ULIM_RY_NEG_DMDString,     asynParamFloat64, &SYM_HEX_ULIM_RY_NEG_DMD_);
  createParam(SYM_HEX_ULIM_RZ_NEG_DMDString,     asynParamFloat64, &SYM_HEX_ULIM_RZ_NEG_DMD_);
  createParam(SYM_HEX_ULIM_TX_POS_DMDString,     asynParamFloat64, &SYM_HEX_ULIM_TX_POS_DMD_);
  createParam(SYM_HEX_ULIM_TY_POS_DMDString,     asynParamFloat64, &SYM_HEX_ULIM_TY_POS_DMD_);
  createParam(SYM_HEX_ULIM_TZ_POS_DMDString,     asynParamFloat64, &SYM_HEX_ULIM_TZ_POS_DMD_);
  createParam(SYM_HEX_ULIM_RX_POS_DMDString,     asynParamFloat64, &SYM_HEX_ULIM_RX_POS_DMD_);
  createParam(SYM_HEX_ULIM_RY_POS_DMDString,     asynParamFloat64, &SYM_HEX_ULIM_RY_POS_DMD_);
  createParam(SYM_HEX_ULIM_RZ_POS_DMDString,     asynParamFloat64, &SYM_HEX_ULIM_RZ_POS_DMD_);
  createParam(SYM_HEX_AUTO_ACTIVATE_DMDString,   asynParamInt32,   &SYM_HEX_AUTO_ACTIVATE_DMD_);
  createParam(SYM_HEX_AUTO_DEACTIVATE_DMDString, asynParamInt32,   &SYM_HEX_AUTO_DEACTIVATE_DMD_);
  createParam(SYM_HEX_AUTO_DELAY_DMDString,      asynParamFloat64, &SYM_HEX_AUTO_DELAY_DMD_);
  createParam(SYM_HEX_STALL_CURRENT_DMDString,   asynParamFloat64, &SYM_HEX_STALL_CURRENT_DMD_);
  createParam(SYM_HEX_BKL_AXIS_DMDString,        asynParamInt32,   &SYM_HEX_BKL_AXIS_DMD_);
  createParam(SYM_HEX_BKL_VALUE_DMDString,       asynParamFloat64, &SYM_HEX_BKL_VALUE_DMD_);
  createParam(SYM_HEX_SAFETY_INP_DMDString,      asynParamInt32,   &SYM_HEX_SAFETY_INPUT_DMD_);
  createParam(SYM_HEX_SAFETY_INPString,          asynParamInt32,   &SYM_HEX_SAFETY_INPUT_);
  createParam(SYM_HEX_CHANNEL_1_DMDString,       asynParamInt32,   &SYM_HEX_CHANNEL_1_DMD_);
  createParam(SYM_HEX_CHANNEL_1String,           asynParamInt32,   &SYM_HEX_CHANNEL_1_);
  createParam(SYM_HEX_CHANNEL_2_DMDString,       asynParamInt32,   &SYM_HEX_CHANNEL_2_DMD_);
  createParam(SYM_HEX_CHANNEL_2String,           asynParamInt32,   &SYM_HEX_CHANNEL_2_);
  createParam(SYM_HEX_CHANNEL_3_DMDString,       asynParamInt32,   &SYM_HEX_CHANNEL_3_DMD_);
  createParam(SYM_HEX_CHANNEL_3String,           asynParamInt32,   &SYM_HEX_CHANNEL_3_);
  createParam(SYM_HEX_CHANNEL_4_DMDString,       asynParamInt32,   &SYM_HEX_CHANNEL_4_DMD_);
  createParam(SYM_HEX_CHANNEL_4String,           asynParamInt32,   &SYM_HEX_CHANNEL_4_);
  createParam(SYM_HEX_CHANNEL_5_DMDString,       asynParamInt32,   &SYM_HEX_CHANNEL_5_DMD_);
  createParam(SYM_HEX_CHANNEL_5String,           asynParamInt32,   &SYM_HEX_CHANNEL_5_);
  createParam(SYM_HEX_CHANNEL_6_DMDString,       asynParamInt32,   &SYM_HEX_CHANNEL_6_DMD_);
  createParam(SYM_HEX_CHANNEL_6String,           asynParamInt32,   &SYM_HEX_CHANNEL_6_);
  createParam(SYM_HEX_HOME_AUTOString,           asynParamInt32,   &SYM_HEX_HOME_AUTO_);
  createParam(SYM_HEX_HOME_VIRTUALString,        asynParamInt32,   &SYM_HEX_HOME_VIRTUAL_);
  createParam(SYM_HEX_HOME_TYPEString,           asynParamInt32,   &SYM_HEX_HOME_TYPE_);
  createParam(SYM_HEX_KIN_MODE_DMDString,        asynParamInt32,   &SYM_HEX_KIN_MODE_DMD_);
  createParam(SYM_HEX_KIN_MODEString,            asynParamInt32,   &SYM_HEX_KIN_MODE_);
  createParam(SYM_HEX_KIN_AXESString,            asynParamInt32,   &SYM_HEX_KIN_AXES_);
  createParam(SYM_HEX_TUNING_IDX_DMDString,      asynParamInt32,   &SYM_HEX_TUNING_IDX_DMD_);
  createParam(SYM_HEX_TUNING_IDXString,          asynParamInt32,   &SYM_HEX_TUNING_IDX_);
  createParam(SYM_HEX_CFG_POWER_ENABLE_DMDString,asynParamInt32,   &SYM_HEX_CFG_POWER_ENABLE_DMD_);
  createParam(SYM_HEX_CFG_POWER_ENABLEString,    asynParamInt32,   &SYM_HEX_CFG_POWER_ENABLE_);
  createParam(SYM_HEX_CFG_POWER_AUTO_DMDString,  asynParamInt32,   &SYM_HEX_CFG_POWER_AUTO_DMD_);
  createParam(SYM_HEX_CFG_POWER_AUTOString,      asynParamInt32,   &SYM_HEX_CFG_POWER_AUTO_);
  createParam(SYM_HEX_SAFETY_SETString,          asynParamInt32,   &SYM_HEX_SAFETY_SET_);
  createParam(SYM_HEX_CHANNEL_SETString,         asynParamInt32,   &SYM_HEX_CHANNEL_SET_);
  createParam(SYM_HEX_SPEED_SETString,           asynParamInt32,   &SYM_HEX_SPEED_SET_);
  createParam(SYM_HEX_TA_SETString,              asynParamInt32,   &SYM_HEX_TA_SET_);
  createParam(SYM_HEX_CS_SETString,              asynParamInt32,   &SYM_HEX_CS_SET_);
  createParam(SYM_HEX_MLIM_SETString,            asynParamInt32,   &SYM_HEX_MLIM_SET_);
  createParam(SYM_HEX_MLIM_ENABLE_SETString,     asynParamInt32,   &SYM_HEX_MLIM_ENABLE_SET_);
  createParam(SYM_HEX_ULIM_SETString,            asynParamInt32,   &SYM_HEX_ULIM_SET_);
  createParam(SYM_HEX_ULIM_ENABLE_SETString,     asynParamInt32,   &SYM_HEX_ULIM_ENABLE_SET_);
  createParam(SYM_HEX_CONTROL_SETString,         asynParamInt32,   &SYM_HEX_CONTROL_SET_);
  createParam(SYM_HEX_STALL_SETString,           asynParamInt32,   &SYM_HEX_STALL_SET_);
  createParam(SYM_HEX_BKL_SETString,             asynParamInt32,   &SYM_HEX_BKL_SET_);
  createParam(SYM_HEX_KIN_SETString,             asynParamInt32,   &SYM_HEX_KIN_SET_);
  createParam(SYM_HEX_TUNING_SETString,          asynParamInt32,   &SYM_HEX_TUNING_SET_);
  createParam(SYM_HEX_CFG_POWER_SETString,       asynParamInt32,   &SYM_HEX_CFG_POWER_SET_);
  
  createParam(SYM_HEX_VTString, asynParamFloat64, &SYM_HEX_VT_);
  createParam(SYM_HEX_VT_MINString, asynParamFloat64, &SYM_HEX_VT_MIN_);
  createParam(SYM_HEX_VT_MAXString, asynParamFloat64, &SYM_HEX_VT_MAX_);
  createParam(SYM_HEX_VRString, asynParamFloat64, &SYM_HEX_VR_);
  createParam(SYM_HEX_VR_MINString, asynParamFloat64, &SYM_HEX_VR_MIN_);
  createParam(SYM_HEX_VR_MAXString, asynParamFloat64, &SYM_HEX_VR_MAX_);
  createParam(SYM_HEX_TAString, asynParamFloat64, &SYM_HEX_TA_);
  createParam(SYM_HEX_TA_MINString, asynParamFloat64, &SYM_HEX_TA_MIN_);
  createParam(SYM_HEX_TA_MAXString, asynParamFloat64, &SYM_HEX_TA_MAX_);
  createParam(SYM_HEX_LastParamString, asynParamInt32, &SYM_HEX_LastParam_);
  createParam(SYM_HEX_TXUString, asynParamFloat64, &SYM_HEX_TXU_);
  createParam(SYM_HEX_TYUString, asynParamFloat64, &SYM_HEX_TYU_);
  createParam(SYM_HEX_TZUString, asynParamFloat64, &SYM_HEX_TZU_);
  createParam(SYM_HEX_RXUString, asynParamFloat64, &SYM_HEX_RXU_);
  createParam(SYM_HEX_RYUString, asynParamFloat64, &SYM_HEX_RYU_);
  createParam(SYM_HEX_RZUString, asynParamFloat64, &SYM_HEX_RZU_);
  createParam(SYM_HEX_TXOString, asynParamFloat64, &SYM_HEX_TXO_);
  createParam(SYM_HEX_TYOString, asynParamFloat64, &SYM_HEX_TYO_);
  createParam(SYM_HEX_TZOString, asynParamFloat64, &SYM_HEX_TZO_);
  createParam(SYM_HEX_RXOString, asynParamFloat64, &SYM_HEX_RXO_);
  createParam(SYM_HEX_RYOString, asynParamFloat64, &SYM_HEX_RYO_);
  createParam(SYM_HEX_RZOString, asynParamFloat64, &SYM_HEX_RZO_);
  createParam(SYM_HEX_MLIM_ENABLEString, asynParamInt32, &SYM_HEX_MLIM_ENABLE_);
  createParam(SYM_HEX_MLIM_TX_NEGString, asynParamFloat64, &SYM_HEX_MLIM_TX_NEG_);
  createParam(SYM_HEX_MLIM_TY_NEGString, asynParamFloat64, &SYM_HEX_MLIM_TY_NEG_);
  createParam(SYM_HEX_MLIM_TZ_NEGString, asynParamFloat64, &SYM_HEX_MLIM_TZ_NEG_);
  createParam(SYM_HEX_MLIM_RX_NEGString, asynParamFloat64, &SYM_HEX_MLIM_RX_NEG_);
  createParam(SYM_HEX_MLIM_RY_NEGString, asynParamFloat64, &SYM_HEX_MLIM_RY_NEG_);
  createParam(SYM_HEX_MLIM_RZ_NEGString, asynParamFloat64, &SYM_HEX_MLIM_RZ_NEG_);
  createParam(SYM_HEX_MLIM_TX_POSString, asynParamFloat64, &SYM_HEX_MLIM_TX_POS_);
  createParam(SYM_HEX_MLIM_TY_POSString, asynParamFloat64, &SYM_HEX_MLIM_TY_POS_);
  createParam(SYM_HEX_MLIM_TZ_POSString, asynParamFloat64, &SYM_HEX_MLIM_TZ_POS_);
  createParam(SYM_HEX_MLIM_RX_POSString, asynParamFloat64, &SYM_HEX_MLIM_RX_POS_);
  createParam(SYM_HEX_MLIM_RY_POSString, asynParamFloat64, &SYM_HEX_MLIM_RY_POS_);
  createParam(SYM_HEX_MLIM_RZ_POSString, asynParamFloat64, &SYM_HEX_MLIM_RZ_POS_);
  createParam(SYM_HEX_ULIM_ENABLEString, asynParamInt32, &SYM_HEX_ULIM_ENABLE_);
  createParam(SYM_HEX_ULIM_TX_NEGString, asynParamFloat64, &SYM_HEX_ULIM_TX_NEG_);
  createParam(SYM_HEX_ULIM_TY_NEGString, asynParamFloat64, &SYM_HEX_ULIM_TY_NEG_);
  createParam(SYM_HEX_ULIM_TZ_NEGString, asynParamFloat64, &SYM_HEX_ULIM_TZ_NEG_);
  createParam(SYM_HEX_ULIM_RX_NEGString, asynParamFloat64, &SYM_HEX_ULIM_RX_NEG_);
  createParam(SYM_HEX_ULIM_RY_NEGString, asynParamFloat64, &SYM_HEX_ULIM_RY_NEG_);
  createParam(SYM_HEX_ULIM_RZ_NEGString, asynParamFloat64, &SYM_HEX_ULIM_RZ_NEG_);
  createParam(SYM_HEX_ULIM_TX_POSString, asynParamFloat64, &SYM_HEX_ULIM_TX_POS_);
  createParam(SYM_HEX_ULIM_TY_POSString, asynParamFloat64, &SYM_HEX_ULIM_TY_POS_);
  createParam(SYM_HEX_ULIM_TZ_POSString, asynParamFloat64, &SYM_HEX_ULIM_TZ_POS_);
  createParam(SYM_HEX_ULIM_RX_POSString, asynParamFloat64, &SYM_HEX_ULIM_RX_POS_);
  createParam(SYM_HEX_ULIM_RY_POSString, asynParamFloat64, &SYM_HEX_ULIM_RY_POS_);
  createParam(SYM_HEX_ULIM_RZ_POSString, asynParamFloat64, &SYM_HEX_ULIM_RZ_POS_);
  createParam(SYM_HEX_AUTO_ACTIVATEString,   asynParamInt32,   &SYM_HEX_AUTO_ACTIVATE_);
  createParam(SYM_HEX_AUTO_DEACTIVATEString, asynParamInt32,   &SYM_HEX_AUTO_DEACTIVATE_);
  createParam(SYM_HEX_AUTO_DELAYString,      asynParamFloat64, &SYM_HEX_AUTO_DELAY_);
  createParam(SYM_HEX_STALL_CURRENTString,   asynParamFloat64, &SYM_HEX_STALL_CURRENT_);
  createParam(SYM_HEX_ERROR_QTYString,       asynParamInt32,   &SYM_HEX_ERROR_QTY_);
  createParam(SYM_HEX_ERROR_CODEString,      asynParamInt32,   &SYM_HEX_ERROR_CODE_);
  createParam(SYM_HEX_ERROR_DESCString,      asynParamOctet,   &SYM_HEX_ERROR_DESC_);
  createParam(SYM_HEX_ERROR_TIMEString,      asynParamFloat64, &SYM_HEX_ERROR_TIME_);
  createParam(SYM_HEX_ERROR_GROUPString,     asynParamInt32,   &SYM_HEX_ERROR_GROUP_);
  createParam(SYM_HEX_ERROR_AXISString,      asynParamInt32,   &SYM_HEX_ERROR_AXIS_);
  createParam(SYM_HEX_ERROR_RETURNString,    asynParamInt32,   &SYM_HEX_ERROR_RETURN_);
  createParam(SYM_HEX_ERROR_DATA1String,     asynParamInt32,   &SYM_HEX_ERROR_DATA1_);
  createParam(SYM_HEX_ERROR_DATA2String,     asynParamInt32,   &SYM_HEX_ERROR_DATA2_);
  createParam(SYM_HEX_SPC_MOVE_DMDString,    asynParamInt32,   &SYM_HEX_SPC_MOVE_DMD_);
  createParam(SYM_HEX_SPC_MOVEString,        asynParamInt32,   &SYM_HEX_SPC_MOVE_);
  createParam(SYM_HEX_MOVE_TX_DMDString,     asynParamFloat64, &SYM_HEX_MOVE_TX_DMD_);
  createParam(SYM_HEX_MOVE_TY_DMDString,     asynParamFloat64, &SYM_HEX_MOVE_TY_DMD_);
  createParam(SYM_HEX_MOVE_TZ_DMDString,     asynParamFloat64, &SYM_HEX_MOVE_TZ_DMD_);
  createParam(SYM_HEX_MOVE_RX_DMDString,     asynParamFloat64, &SYM_HEX_MOVE_RX_DMD_);
  createParam(SYM_HEX_MOVE_RY_DMDString,     asynParamFloat64, &SYM_HEX_MOVE_RY_DMD_);
  createParam(SYM_HEX_MOVE_RZ_DMDString,     asynParamFloat64, &SYM_HEX_MOVE_RZ_DMD_);
  createParam(SYM_HEX_MOVE_TYPEString,       asynParamInt32,   &SYM_HEX_MOVE_TYPE_);
  createParam(SYM_HEX_MOVEString,            asynParamInt32,   &SYM_HEX_MOVE_);

  createParam(SYM_HEX_MOVE_SEQString,        asynParamInt32,   &SYM_HEX_MOVE_SEQ_);
  createParam(SYM_HEX_VALID_PTP_MODEString,  asynParamInt32,   &SYM_HEX_VALID_PTP_MODE_);
  createParam(SYM_HEX_VALID_PTP_TYPEString,  asynParamInt32,   &SYM_HEX_VALID_PTP_TYPE_);
  createParam(SYM_HEX_VALID_PTP_TXString,    asynParamFloat64, &SYM_HEX_VALID_PTP_TX_);
  createParam(SYM_HEX_VALID_PTP_TYString,    asynParamFloat64, &SYM_HEX_VALID_PTP_TY_);
  createParam(SYM_HEX_VALID_PTP_TZString,    asynParamFloat64, &SYM_HEX_VALID_PTP_TZ_);
  createParam(SYM_HEX_VALID_PTP_RXString,    asynParamFloat64, &SYM_HEX_VALID_PTP_RX_);
  createParam(SYM_HEX_VALID_PTP_RYString,    asynParamFloat64, &SYM_HEX_VALID_PTP_RY_);
  createParam(SYM_HEX_VALID_PTP_RZString,    asynParamFloat64, &SYM_HEX_VALID_PTP_RZ_);
  createParam(SYM_HEX_VALID_PTPString,       asynParamInt32,   &SYM_HEX_VALID_PTP_);
  createParam(SYM_HEX_VALID_PTP_RESULTString,asynParamInt32,   &SYM_HEX_VALID_PTP_RESULT_);
  createParam(SYM_HEX_POWER_POWERString,     asynParamInt32,   &SYM_HEX_POWER_POWER_);
  createParam(SYM_HEX_POWERString,           asynParamInt32,   &SYM_HEX_POWER_);
  createParam(SYM_HEX_MNTN_MODEString,       asynParamInt32,   &SYM_HEX_MAINTENANCE_MODE_);
  createParam(SYM_HEX_MNTN_AXISString,       asynParamInt32,   &SYM_HEX_MAINTENANCE_AXIS_);
  createParam(SYM_HEX_MNTNString,            asynParamInt32,   &SYM_HEX_MAINTENANCE_);
  createParam(SYM_HEX_JOG_AXISString,        asynParamInt32,   &SYM_HEX_JOG_AXIS_);
  createParam(SYM_HEX_JOG_INCRString,        asynParamFloat64, &SYM_HEX_JOG_INCR_);
  createParam(SYM_HEX_JOGString,             asynParamInt32,   &SYM_HEX_JOG_);
  createParam(SYM_HEX_REBOOTString,          asynParamInt32,   &SYM_HEX_REBOOT_);
  createParam(SYM_HEX_TERMINAL_SENDString,   asynParamOctet,   &SYM_HEX_TERMINAL_SEND_);
  createParam(SYM_HEX_TERMINAL_RECVString,   asynParamOctet,   &SYM_HEX_TERMINAL_RECV_);

  createParam(SYM_HEX_USR_TO_OBJString,      asynParamInt32,   &SYM_HEX_USR_TO_OBJ_);
  createParam(SYM_HEX_OBJ_TO_USRString,      asynParamInt32,   &SYM_HEX_OBJ_TO_USR_);
  createParam(SYM_HEX_BKL_AXISString,        asynParamInt32,   &SYM_HEX_BKL_AXIS_);
  createParam(SYM_HEX_BKL_VALUEString,       asynParamFloat64, &SYM_HEX_BKL_VALUE_);

  createParam(SYM_HEX_CFG_GETString,         asynParamInt32,   &SYM_HEX_CFG_GET_);
  createParam(SYM_HEX_CFG_SETString,         asynParamInt32,   &SYM_HEX_CFG_SET_);
  createParam(SYM_HEX_CFG_DFLTString,        asynParamInt32,   &SYM_HEX_CFG_DFLT_);
  createParam(SYM_HEX_CFG_SAVEString,        asynParamInt32,   &SYM_HEX_CFG_SAVE_);

  createParam(SYM_HEX_CMD_CODEString,        asynParamInt32,   &SYM_HEX_CMD_CODE_);
  createParam(SYM_HEX_CMD_DESCString,        asynParamOctet,   &SYM_HEX_CMD_DESC_);

  createParam(SYM_HEX_VERS_CTRL_IDString,    asynParamInt32,   &SYM_HEX_VERS_CTRL_ID_);
  createParam(SYM_HEX_VERS_CTRL_VERString,   asynParamOctet,   &SYM_HEX_VERS_CTRL_VER_);
  createParam(SYM_HEX_VERS_API_VERString,    asynParamOctet,   &SYM_HEX_VERS_API_VER_);
  createParam(SYM_HEX_VERS_SYS_IDString,     asynParamInt32,   &SYM_HEX_VERS_SYS_ID_);
  createParam(SYM_HEX_VERS_SYS_NUMString,    asynParamInt32,   &SYM_HEX_VERS_SYS_NUM_);
  createParam(SYM_HEX_VERS_SYS_CFGString,    asynParamInt32,   &SYM_HEX_VERS_SYS_CFG_);

  getVersion();
  readAllConfig();
  readStatus();
  setIntegerParam(SYM_HEX_SPC_MOVE_DMD_, 1);
  setIntegerParam(SYM_HEX_MOVE_TYPE_, 0);
  
  // Set the demand position for each axis equal to the current readback and create SymetrieAxis objects
  for (int axis=0; axis<NUM_MOTOR_AXES; axis++) {
    double position;
    getDoubleParam(SYM_HEX_S_UTO_TX_ + axis, &position);
    setDoubleParam(SYM_HEX_MOVE_TX_DMD_ + axis, position);
    new SymetrieAxis(this, axis);
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);

}

void SymetrieHexapod::readStatus()
{
  char response[CHAR_ARRAY_SIZE];
  std::vector<std::string> vals;
  // First read all of the status
  immediateWriteRead("s_hexa,50,1", response);
  std::string sresp(response);
  std::size_t current, previous = 0;
  current = sresp.find('\r');
  while (current != std::string::npos){
    vals.push_back(sresp.substr(previous, current-previous));
    previous = current + 1;
    current = sresp.find('\r', previous);
  }

  if (vals.size() < 45){
    debug(DEBUG_TRACE, "SymetrieHexapod::readStatus", "Did not receive full status message");
  } else {
    int ivalue = 0;
    double dvalue = 0.0;

    // Parse s_hexa
    sscanf(vals[0].c_str(), "%d", &ivalue);
    setIntegerParam(SYM_HEX_S_HEXA_, ivalue);

    // Parse s_action
    sscanf(vals[1].c_str(), "%d", &ivalue);
    setIntegerParam(SYM_HEX_S_ACTION_, ivalue);

    // Parse s_uto
    sscanf(vals[2].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_UTO_TX_, dvalue);
    sscanf(vals[3].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_UTO_TY_, dvalue);
    sscanf(vals[4].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_UTO_TZ_, dvalue);
    sscanf(vals[5].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_UTO_RX_, dvalue);
    sscanf(vals[6].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_UTO_RY_, dvalue);
    sscanf(vals[7].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_UTO_RZ_, dvalue);

    // Parse s_mtp
    sscanf(vals[8].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_MTP_TX_, dvalue);
    sscanf(vals[9].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_MTP_TY_, dvalue);
    sscanf(vals[10].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_MTP_TZ_, dvalue);
    sscanf(vals[11].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_MTP_RX_, dvalue);
    sscanf(vals[12].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_MTP_RY_, dvalue);
    sscanf(vals[13].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_MTP_RZ_, dvalue);

    // Parse s_ax
    sscanf(vals[14].c_str(), "%d", &ivalue);
    setIntegerParam(SYM_HEX_S_AX_1_, ivalue);
    sscanf(vals[15].c_str(), "%d", &ivalue);
    setIntegerParam(SYM_HEX_S_AX_2_, ivalue);
    sscanf(vals[16].c_str(), "%d", &ivalue);
    setIntegerParam(SYM_HEX_S_AX_3_, ivalue);
    sscanf(vals[17].c_str(), "%d", &ivalue);
    setIntegerParam(SYM_HEX_S_AX_4_, ivalue);
    sscanf(vals[18].c_str(), "%d", &ivalue);
    setIntegerParam(SYM_HEX_S_AX_5_, ivalue);
    sscanf(vals[19].c_str(), "%d", &ivalue);
    setIntegerParam(SYM_HEX_S_AX_6_, ivalue);

    // Parse s_pos_ax
    sscanf(vals[20].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_POS_AX_1_, dvalue);
    sscanf(vals[21].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_POS_AX_2_, dvalue);
    sscanf(vals[22].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_POS_AX_3_, dvalue);
    sscanf(vals[23].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_POS_AX_4_, dvalue);
    sscanf(vals[24].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_POS_AX_5_, dvalue);
    sscanf(vals[25].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_POS_AX_6_, dvalue);

    // Parse s_dio
    sscanf(vals[26].c_str(), "%d", &ivalue);
    setIntegerParam(SYM_HEX_S_DIO_1_, ivalue);
    sscanf(vals[27].c_str(), "%d", &ivalue);
    setIntegerParam(SYM_HEX_S_DIO_2_, ivalue);
    sscanf(vals[28].c_str(), "%d", &ivalue);
    setIntegerParam(SYM_HEX_S_DIO_3_, ivalue);
    sscanf(vals[29].c_str(), "%d", &ivalue);
    setIntegerParam(SYM_HEX_S_DIO_4_, ivalue);
    sscanf(vals[30].c_str(), "%d", &ivalue);
    setIntegerParam(SYM_HEX_S_DIO_5_, ivalue);
    sscanf(vals[31].c_str(), "%d", &ivalue);
    setIntegerParam(SYM_HEX_S_DIO_6_, ivalue);
    sscanf(vals[32].c_str(), "%d", &ivalue);
    setIntegerParam(SYM_HEX_S_DIO_7_, ivalue);
    sscanf(vals[33].c_str(), "%d", &ivalue);
    setIntegerParam(SYM_HEX_S_DIO_8_, ivalue);

    // Parse s_ai
    sscanf(vals[34].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_AI_1_, dvalue);
    sscanf(vals[35].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_AI_2_, dvalue);
    sscanf(vals[36].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_AI_3_, dvalue);
    sscanf(vals[37].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_AI_4_, dvalue);
    sscanf(vals[38].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_AI_5_, dvalue);
    sscanf(vals[39].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_AI_6_, dvalue);
    sscanf(vals[40].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_AI_7_, dvalue);
    sscanf(vals[41].c_str(), "%lf", &dvalue);
    setDoubleParam(SYM_HEX_S_AI_8_, dvalue);

    // Parse s_cycle
    sscanf(vals[42].c_str(), "%d", &ivalue);
    setIntegerParam(SYM_HEX_S_CYCLE_, ivalue);

    // Parse s_index
    sscanf(vals[43].c_str(), "%d", &ivalue);
    setIntegerParam(SYM_HEX_S_INDEX_, ivalue);

    // Parse s_err_nr
    sscanf(vals[44].c_str(), "%d", &ivalue);
    setIntegerParam(SYM_HEX_S_ERR_NR_, ivalue);
    if (ivalue != no_of_errors_){
      no_of_errors_ = ivalue;
      readErrors();
    }

    callParamCallbacks();
  }
}

void SymetrieHexapod::readAllConfig()
{
  double values[13];
  int ivalues[13];

  getConfigItem("C_CFG_SAVE", 1, ivalues);
  setIntegerParam(SYM_HEX_CFG_SAVE_, ivalues[0]);

  getConfigItem("C_CFG_SAFETYINPUT", 1, ivalues);
  setIntegerParam(SYM_HEX_SAFETY_INPUT_, ivalues[0]);

  getConfigItem("C_CFG_CHANNEL", 12, ivalues);
  setIntegerParam(SYM_HEX_CHANNEL_1_, ivalues[0]);
  setIntegerParam(SYM_HEX_CHANNEL_2_, ivalues[1]);
  setIntegerParam(SYM_HEX_CHANNEL_3_, ivalues[2]);
  setIntegerParam(SYM_HEX_CHANNEL_4_, ivalues[3]);
  setIntegerParam(SYM_HEX_CHANNEL_5_, ivalues[4]);
  setIntegerParam(SYM_HEX_CHANNEL_6_, ivalues[5]);

  getConfigItem("C_CFG_SPEED", 6, values);
  setDoubleParam(SYM_HEX_VT_, values[0]);
  setDoubleParam(SYM_HEX_VR_, values[1]);
  setDoubleParam(SYM_HEX_VT_MIN_, values[2]);
  setDoubleParam(SYM_HEX_VR_MIN_, values[3]);
  setDoubleParam(SYM_HEX_VT_MAX_, values[4]);
  setDoubleParam(SYM_HEX_VR_MAX_, values[5]);

  getConfigItem("C_CFG_TA", 3, values);
  setDoubleParam(SYM_HEX_TA_, values[0]);
  setDoubleParam(SYM_HEX_TA_MIN_, values[1]);
  setDoubleParam(SYM_HEX_TA_MAX_, values[2]);

  getConfigItem("C_CFG_CS", 12, values);
  setDoubleParam(SYM_HEX_TXU_, values[0]);
  setDoubleParam(SYM_HEX_TYU_, values[1]);
  setDoubleParam(SYM_HEX_TZU_, values[2]);
  setDoubleParam(SYM_HEX_RXU_, values[3]);
  setDoubleParam(SYM_HEX_RYU_, values[4]);
  setDoubleParam(SYM_HEX_RZU_, values[5]);
  setDoubleParam(SYM_HEX_TXO_, values[6]);
  setDoubleParam(SYM_HEX_TYO_, values[7]);
  setDoubleParam(SYM_HEX_TZO_, values[8]);
  setDoubleParam(SYM_HEX_RXO_, values[9]);
  setDoubleParam(SYM_HEX_RYO_, values[10]);
  setDoubleParam(SYM_HEX_RZO_, values[11]);
  getConfigItem("C_CFG_LIMITENABLE", 3, ivalues);
  setIntegerParam(SYM_HEX_MLIM_ENABLE_, ivalues[1]);

  getConfigItem("C_CFG_LIMIT", 13, values, "c_par(0)=1");
  setDoubleParam(SYM_HEX_MLIM_TX_NEG_, values[1]);
  setDoubleParam(SYM_HEX_MLIM_TY_NEG_, values[2]);
  setDoubleParam(SYM_HEX_MLIM_TZ_NEG_, values[3]);
  setDoubleParam(SYM_HEX_MLIM_RX_NEG_, values[4]);
  setDoubleParam(SYM_HEX_MLIM_RY_NEG_, values[5]);
  setDoubleParam(SYM_HEX_MLIM_RZ_NEG_, values[6]);
  setDoubleParam(SYM_HEX_MLIM_TX_POS_, values[7]);
  setDoubleParam(SYM_HEX_MLIM_TY_POS_, values[8]);
  setDoubleParam(SYM_HEX_MLIM_TZ_POS_, values[9]);
  setDoubleParam(SYM_HEX_MLIM_RX_POS_, values[10]);
  setDoubleParam(SYM_HEX_MLIM_RY_POS_, values[11]);
  setDoubleParam(SYM_HEX_MLIM_RZ_POS_, values[12]);

  getConfigItem("C_CFG_LIMITENABLE", 3, ivalues);
  setIntegerParam(SYM_HEX_ULIM_ENABLE_, ivalues[2]);

  getConfigItem("C_CFG_LIMIT", 13, values, "c_par(0)=2");
  setDoubleParam(SYM_HEX_ULIM_TX_NEG_, values[1]);
  setDoubleParam(SYM_HEX_ULIM_TY_NEG_, values[2]);
  setDoubleParam(SYM_HEX_ULIM_TZ_NEG_, values[3]);
  setDoubleParam(SYM_HEX_ULIM_RX_NEG_, values[4]);
  setDoubleParam(SYM_HEX_ULIM_RY_NEG_, values[5]);
  setDoubleParam(SYM_HEX_ULIM_RZ_NEG_, values[6]);
  setDoubleParam(SYM_HEX_ULIM_TX_POS_, values[7]);
  setDoubleParam(SYM_HEX_ULIM_TY_POS_, values[8]);
  setDoubleParam(SYM_HEX_ULIM_TZ_POS_, values[9]);
  setDoubleParam(SYM_HEX_ULIM_RX_POS_, values[10]);
  setDoubleParam(SYM_HEX_ULIM_RY_POS_, values[11]);
  setDoubleParam(SYM_HEX_ULIM_RZ_POS_, values[12]);

  getConfigItem("C_CFG_CONTROL", 2, values);
  int auto_on = ((int)values[0])&1;
  int auto_off = (((int)values[0])&2)>>1;
  setIntegerParam(SYM_HEX_AUTO_ACTIVATE_, auto_on);
  setIntegerParam(SYM_HEX_AUTO_DEACTIVATE_, auto_off);
  setDoubleParam(SYM_HEX_AUTO_DELAY_, values[1]);
  
  getConfigItem("C_CFG_STALLCURRENT", 1, values);
  setDoubleParam(SYM_HEX_STALL_CURRENT_, values[0]);

  getConfigItem("C_CFG_BACKLASH", 2, values);
  setIntegerParam(SYM_HEX_BKL_AXIS_, (int)values[0]);
  setDoubleParam(SYM_HEX_BKL_VALUE_, values[1]);

  getConfigItem("C_CFG_HOME", 3, ivalues);
  setIntegerParam(SYM_HEX_HOME_AUTO_, ivalues[0]);
  setIntegerParam(SYM_HEX_HOME_VIRTUAL_, ivalues[1]);
  setIntegerParam(SYM_HEX_HOME_TYPE_, ivalues[2]);

  getConfigItem("C_CFG_KIN", 2, ivalues);
  setIntegerParam(SYM_HEX_KIN_MODE_, ivalues[0]);
  setIntegerParam(SYM_HEX_KIN_AXES_, ivalues[1]);

  getConfigItem("C_CFG_TUNING", 1, ivalues);
  setIntegerParam(SYM_HEX_TUNING_IDX_, ivalues[0]);
  
  getConfigItem("C_CFG_POWER", 2, ivalues);
  setIntegerParam(SYM_HEX_CFG_POWER_ENABLE_, ivalues[0]);
  setIntegerParam(SYM_HEX_CFG_POWER_AUTO_, ivalues[1]);
  
  callParamCallbacks();
}


int SymetrieHexapod::applyConfigSafety()
{
  int ivalues[13];
  int status = 0;

  // Read the safety input parameter and apply
  getIntegerParam(SYM_HEX_SAFETY_INPUT_DMD_, &ivalues[0]);
  status = setConfigItem("C_CFG_SAFETYINPUT", 1, ivalues);
  if (status < 0){
    debug(DEBUG_ERROR, "SymetrieHexapod::applyConfig", "C_CFG_SAFETYINPUT failed");
  }

  return status;
}

int SymetrieHexapod::applyConfigChannel()
{
  int ivalues[13];
  int status = 0;

  // Read the channel parameters and apply them
  getIntegerParam(SYM_HEX_CHANNEL_1_DMD_, &ivalues[0]);
  getIntegerParam(SYM_HEX_CHANNEL_2_DMD_, &ivalues[1]);
  getIntegerParam(SYM_HEX_CHANNEL_3_DMD_, &ivalues[2]);
  getIntegerParam(SYM_HEX_CHANNEL_4_DMD_, &ivalues[3]);
  getIntegerParam(SYM_HEX_CHANNEL_5_DMD_, &ivalues[4]);
  getIntegerParam(SYM_HEX_CHANNEL_6_DMD_, &ivalues[5]);
  status = setConfigItem("C_CFG_CHANNEL", 6, ivalues);
  if (status < 0){
    debug(DEBUG_ERROR, "SymetrieHexapod::applyConfig", "C_CFG_CHANNEL failed");
  }

  return status;
}

int SymetrieHexapod::applyConfigSpeed()
{
  double values[13];
  int status = 0;

  // Read the velocity parameters and apply them
  getDoubleParam(SYM_HEX_VT_DMD_, &values[0]);
  getDoubleParam(SYM_HEX_VR_DMD_, &values[1]);
  status = setConfigItem("C_CFG_SPEED", 2, values);
  if (status < 0){
    debug(DEBUG_ERROR, "SymetrieHexapod::applyConfig", "C_CFG_SPEED failed");
  }

  return status;
}

int SymetrieHexapod::applyConfigTa()
{
  double values[13];
  int status = 0;

  // Read the acceleration parameters and apply them
  getDoubleParam(SYM_HEX_TA_DMD_, &values[0]);
  status = setConfigItem("C_CFG_TA", 1, values);
  if (status < 0){
    debug(DEBUG_ERROR, "SymetrieHexapod::applyConfig", "C_CFG_TA failed");
  }

  return status;
}

int SymetrieHexapod::applyConfigCS()
{
  double values[13];
  int status = 0;

  // Read the CS parameters and apply them
  getDoubleParam(SYM_HEX_TXU_DMD_, &values[0]);
  getDoubleParam(SYM_HEX_TYU_DMD_, &values[1]);
  getDoubleParam(SYM_HEX_TZU_DMD_, &values[2]);
  getDoubleParam(SYM_HEX_RXU_DMD_, &values[3]);
  getDoubleParam(SYM_HEX_RYU_DMD_, &values[4]);
  getDoubleParam(SYM_HEX_RZU_DMD_, &values[5]);
  getDoubleParam(SYM_HEX_TXO_DMD_, &values[6]);
  getDoubleParam(SYM_HEX_TYO_DMD_, &values[7]);
  getDoubleParam(SYM_HEX_TZO_DMD_, &values[8]);
  getDoubleParam(SYM_HEX_RXO_DMD_, &values[9]);
  getDoubleParam(SYM_HEX_RYO_DMD_, &values[10]);
  getDoubleParam(SYM_HEX_RZO_DMD_, &values[11]);
  status = setConfigItem("C_CFG_CS", 12, values);
  if (status < 0){
    debug(DEBUG_ERROR, "SymetrieHexapod::applyConfig", "C_CFG_CS failed");
  }

  return status;
}

int SymetrieHexapod::applyConfigMachineLimit()
{
  double values[13];
  int status = 0;

  // Read the machine limit parameters and apply them
  values[0] = 1;
  getDoubleParam(SYM_HEX_MLIM_TX_NEG_DMD_, &values[1]);
  getDoubleParam(SYM_HEX_MLIM_TY_NEG_DMD_, &values[2]);
  getDoubleParam(SYM_HEX_MLIM_TZ_NEG_DMD_, &values[3]);
  getDoubleParam(SYM_HEX_MLIM_RX_NEG_DMD_, &values[4]);
  getDoubleParam(SYM_HEX_MLIM_RY_NEG_DMD_, &values[5]);
  getDoubleParam(SYM_HEX_MLIM_RZ_NEG_DMD_, &values[6]);
  getDoubleParam(SYM_HEX_MLIM_TX_POS_DMD_, &values[7]);
  getDoubleParam(SYM_HEX_MLIM_TY_POS_DMD_, &values[8]);
  getDoubleParam(SYM_HEX_MLIM_TZ_POS_DMD_, &values[9]);
  getDoubleParam(SYM_HEX_MLIM_RX_POS_DMD_, &values[10]);
  getDoubleParam(SYM_HEX_MLIM_RY_POS_DMD_, &values[11]);
  getDoubleParam(SYM_HEX_MLIM_RZ_POS_DMD_, &values[12]);
  status = setConfigItem("C_CFG_LIMIT", 13, values);
  if (status < 0){
    debug(DEBUG_ERROR, "SymetrieHexapod::applyConfig", "C_CFG_LIMIT failed");
  }

  return status;
}

int SymetrieHexapod::applyConfigMachineLimitEnable()
{
  int ivalues[2];
  int status = 0;

  // Read the machine limit parameters and apply them
  ivalues[0] = 1;
  getIntegerParam(SYM_HEX_MLIM_ENABLE_DMD_, &ivalues[1]);
  status = setConfigItem("C_CFG_LIMITENABLE", 2, ivalues);
  if (status < 0){
    debug(DEBUG_ERROR, "SymetrieHexapod::applyConfig", "C_CFG_LIMITENABLE failed");
  }

  return status;
}

int SymetrieHexapod::applyConfigUserLimit()
{
  double values[13];
  int status = 0;

  // Read the user limit parameters and apply them
  values[0] = 2;
  getDoubleParam(SYM_HEX_ULIM_TX_NEG_DMD_, &values[1]);
  getDoubleParam(SYM_HEX_ULIM_TY_NEG_DMD_, &values[2]);
  getDoubleParam(SYM_HEX_ULIM_TZ_NEG_DMD_, &values[3]);
  getDoubleParam(SYM_HEX_ULIM_RX_NEG_DMD_, &values[4]);
  getDoubleParam(SYM_HEX_ULIM_RY_NEG_DMD_, &values[5]);
  getDoubleParam(SYM_HEX_ULIM_RZ_NEG_DMD_, &values[6]);
  getDoubleParam(SYM_HEX_ULIM_TX_POS_DMD_, &values[7]);
  getDoubleParam(SYM_HEX_ULIM_TY_POS_DMD_, &values[8]);
  getDoubleParam(SYM_HEX_ULIM_TZ_POS_DMD_, &values[9]);
  getDoubleParam(SYM_HEX_ULIM_RX_POS_DMD_, &values[10]);
  getDoubleParam(SYM_HEX_ULIM_RY_POS_DMD_, &values[11]);
  getDoubleParam(SYM_HEX_ULIM_RZ_POS_DMD_, &values[12]);
  status = setConfigItem("C_CFG_LIMIT", 13, values);
  if (status < 0){
    debug(DEBUG_ERROR, "SymetrieHexapod::applyConfig", "C_CFG_LIMIT failed");
  }

  return status;
}

int SymetrieHexapod::applyConfigUserLimitEnable()
{
  int ivalues[2];
  int status = 0;

  // Read the machine limit parameters and apply them
  ivalues[0] = 2;
  getIntegerParam(SYM_HEX_ULIM_ENABLE_DMD_, &ivalues[1]);
  status = setConfigItem("C_CFG_LIMITENABLE", 2, ivalues);
  if (status < 0){
    debug(DEBUG_ERROR, "SymetrieHexapod::applyConfig", "C_CFG_LIMITENABLE failed");
  }

  return status;
}

int SymetrieHexapod::applyConfigControl()
{
  double values[13]={epicsNAN};//NAN
  int status = 0;

  // Read the auto activate parameters and apply them
  int activate = 0;
  int deactivate = 0;
  getIntegerParam(SYM_HEX_AUTO_ACTIVATE_DMD_, &activate);
  getIntegerParam(SYM_HEX_AUTO_DEACTIVATE_DMD_, &deactivate);
  values[0] = (double)(activate + (deactivate<<1));
  getDoubleParam(SYM_HEX_AUTO_DELAY_DMD_, &values[1]);
  
  status = setConfigItem("C_CFG_CONTROL", 2, values);
  if (status < 0){
    debug(DEBUG_ERROR, "SymetrieHexapod::applyConfig", "C_CFG_CONTROL failed");
  }

  return status;
}

int SymetrieHexapod::applyConfigStallCurrent()
{
  double values[13];
  int status = 0;
 
  // Read the stall current parameter and apply it
  getDoubleParam(SYM_HEX_STALL_CURRENT_DMD_, &values[0]);
  status = setConfigItem("C_CFG_STALLCURRENT", 1, values);
  if (status < 0){
    debug(DEBUG_ERROR, "SymetrieHexapod::applyConfig", "C_CFG_STALLCURRENT failed");
  }

  return status;
}

int SymetrieHexapod::applyConfigBacklash()
{
  double values[13];
  int status = 0;

  // Read the backlash strategy parameters and apply them
  int axis = 0;
  getIntegerParam(SYM_HEX_BKL_AXIS_DMD_, &axis);
  values[0] = (double)axis;
  getDoubleParam(SYM_HEX_BKL_VALUE_DMD_, &values[1]);
  status = setConfigItem("C_CFG_BACKLASH", 2, values);
  if (status < 0){
    debug(DEBUG_ERROR, "SymetrieHexapod::applyConfig", "C_CFG_BACKLASH failed");
  }

  return status;
}

int SymetrieHexapod::applyConfigKinematic()
{
  int ivalues[13];
  int status = 0;

  // Read the kinematic mode parameter and apply
  getIntegerParam(SYM_HEX_KIN_MODE_DMD_, &ivalues[0]);
  status = setConfigItem("C_CFG_KIN", 1, ivalues);
  if (status < 0){
    debug(DEBUG_ERROR, "SymetrieHexapod::applyConfig", "C_CFG_KIN failed");
  }

  return status;
}

int SymetrieHexapod::applyConfigTuning()
{
  int ivalues[13];
  int status = 0;

  // Read the tuning index parameter and apply
  getIntegerParam(SYM_HEX_TUNING_IDX_DMD_, &ivalues[0]);
  status = setConfigItem("C_CFG_TUNING", 1, ivalues);
  if (status < 0){
    debug(DEBUG_ERROR, "SymetrieHexapod::applyConfig", "C_CFG_TUNING failed");
  }

  return status;
}

int SymetrieHexapod::applyConfigPower()
{
  int ivalues[13];
  int status = 0;

  // Read the tuning index parameter and apply
  getIntegerParam(SYM_HEX_CFG_POWER_ENABLE_DMD_, &ivalues[0]);
  getIntegerParam(SYM_HEX_CFG_POWER_AUTO_DMD_, &ivalues[1]);
  status = setConfigItem("C_CFG_POWER", 2, ivalues);
  if (status < 0){
    debug(DEBUG_ERROR, "SymetrieHexapod::applyConfig", "C_CFG_POWER failed");
  }

  return status;
}


void SymetrieHexapod::applyConfig()
{
//  double values[13];
//  int ivalues[13];
  int status = 0;
  int reply = 0;


  reply = applyConfigSafety();
  if (reply < 0){
    status = reply;
  }
  reply = applyConfigChannel();
  if (reply < 0){
    status = reply;
  }
  reply = applyConfigSpeed();
  if (reply < 0){
    status = reply;
  }
  reply = applyConfigTa();
  if (reply < 0){
    status = reply;
  }
  reply = applyConfigCS();
  if (reply < 0){
    status = reply;
  }
  reply = applyConfigMachineLimit();
  if (reply < 0){
    status = reply;
  }
  reply = applyConfigMachineLimitEnable();
  if (reply < 0){
    status = reply;
  }
  reply = applyConfigUserLimit();
  if (reply < 0){
    status = reply;
  }
  reply = applyConfigUserLimitEnable();
  if (reply < 0){
    status = reply;
  }
  reply = applyConfigControl();
  if (reply < 0){
    status = reply;
  }
  reply = applyConfigStallCurrent();
  if (reply < 0){
    status = reply;
  }
  reply = applyConfigBacklash();
  if (reply < 0){
    status = reply;
  }
  reply = applyConfigKinematic();
  if (reply < 0){
    status = reply;
  }
  reply = applyConfigTuning();
  if (reply < 0){
    status = reply;
  }
  reply = applyConfigPower();
  if (reply < 0){
    status = reply;
  }
  
  recordCmdResult(status);
}

void SymetrieHexapod::readErrors()
{
  char command[CHAR_ARRAY_SIZE];
  char response[CHAR_ARRAY_SIZE];
  int ival = 0;
  double dval = 0;

  // Iterate over all errors
  std::vector<std::string> vals;
  setIntegerParam(SYM_HEX_ERROR_QTY_, no_of_errors_);
  for (int index = 0; index < 20; index++){
    std::stringstream err_msg;
    vals.clear();
    if (index < no_of_errors_){
      sprintf(command, "c_par(0)=%d c_cmd=C_ERR_INFO", index);
      immediateWriteRead(command, response);
      waitForCommandComplete();

      immediateWriteRead("c_par(0),8,1", response);
      std::string sresp(response);
      std::size_t current, previous = 0;
      current = sresp.find('\r');
      while (current != std::string::npos){
        vals.push_back(sresp.substr(previous, current-previous));
        previous = current + 1;
        current = sresp.find('\r', previous);
      }

      sscanf(vals[1].c_str(), "%d", &ival);
      setIntegerParam(index, SYM_HEX_ERROR_CODE_, ival);

      err_msg << std::string(errors[ival]);

      sscanf(vals[2].c_str(), "%lf", &dval);
      setDoubleParam(index, SYM_HEX_ERROR_TIME_, dval);

      sscanf(vals[3].c_str(), "%d", &ival);
      setIntegerParam(index, SYM_HEX_ERROR_GROUP_, ival);

      sscanf(vals[4].c_str(), "%d", &ival);
      setIntegerParam(index, SYM_HEX_ERROR_AXIS_, ival);

      if (ival > 0){
        err_msg << " AXIS: " << ival;
      }

      sscanf(vals[5].c_str(), "%d", &ival);
      setIntegerParam(index, SYM_HEX_ERROR_RETURN_, ival);

      if (ival > 0){
        err_msg << " RETURN: " << ival;
      }

      sscanf(vals[6].c_str(), "%d", &ival);
      setIntegerParam(index, SYM_HEX_ERROR_DATA1_, ival);

      if (ival > 0){
        err_msg << " DATA1: " << ival;
      }

      sscanf(vals[7].c_str(), "%d", &ival);
      setIntegerParam(index, SYM_HEX_ERROR_DATA2_, ival);

      if (ival > 0){
        err_msg << " DATA2: " << ival;
      }
      
      setStringParam(index, SYM_HEX_ERROR_DESC_, err_msg.str().c_str());

    } else {
      setIntegerParam(index, SYM_HEX_ERROR_QTY_, 0);
      setIntegerParam(index, SYM_HEX_ERROR_CODE_, 0);
      setStringParam(index, SYM_HEX_ERROR_DESC_, "");
      setDoubleParam(index, SYM_HEX_ERROR_TIME_, 0.0);
      setIntegerParam(index, SYM_HEX_ERROR_GROUP_, 0);
      setIntegerParam(index, SYM_HEX_ERROR_AXIS_, 0);
      setIntegerParam(index, SYM_HEX_ERROR_RETURN_, 0);
      setIntegerParam(index, SYM_HEX_ERROR_DATA1_, 0);
      setIntegerParam(index, SYM_HEX_ERROR_DATA2_, 0);
    }
    callParamCallbacks(index);
  }
  callParamCallbacks();
}

int SymetrieHexapod::getConfigItem(const std::string& cmd, int num_params, double *params)
{
  return getConfigItem(cmd, num_params, params, "");
}

int SymetrieHexapod::getConfigItem(const std::string& cmd, int num_params, double *params, const std::string& extra)
{
  char command[CHAR_ARRAY_SIZE];
  char response[CHAR_ARRAY_SIZE];
  int c_cmd = 1;
  if (extra == ""){
    sprintf(command, "c_cfg=0 c_cmd=%s", cmd.c_str());
  } else {
    sprintf(command, "c_cfg=0 %s c_cmd=%s", extra.c_str(), cmd.c_str());
  }
  debug(DEBUG_TRACE, "SymetrieHexapod::getConfigItem", "Writing message", command);
  immediateWriteRead(command, response);
  c_cmd = waitForCommandComplete();
  for (int index = 0; index < num_params; index++){
    sprintf(command, "c_par(%d)", index);
    debug(DEBUG_TRACE, "SymetrieHexapod::getConfigItem", "Writing message", command);
    immediateWriteRead(command, response);
    debug(DEBUG_TRACE, "SymetrieHexapod::getConfigItem", "Reading response", response);
    sscanf(response, "%lf", &params[index]);
  }
  return c_cmd;
}

int SymetrieHexapod::getConfigItem(const std::string& cmd, int num_params, int *params)
{
  return getConfigItem(cmd, num_params, params, "");
}

int SymetrieHexapod::getConfigItem(const std::string& cmd, int num_params, int *params, const std::string& extra)
{
  char command[CHAR_ARRAY_SIZE];
  char response[CHAR_ARRAY_SIZE];
  int c_cmd = 1;
  if (extra == ""){
    sprintf(command, "c_cfg=0 c_cmd=%s", cmd.c_str());
  } else {
    sprintf(command, "c_cfg=0 %s c_cmd=%s", extra.c_str(), cmd.c_str());
  }
  debug(DEBUG_TRACE, "SymetrieHexapod::getConfigItem", "Writing message", command);
  immediateWriteRead(command, response);
  c_cmd = waitForCommandComplete();
  for (int index = 0; index < num_params; index++){
    sprintf(command, "c_par(%d)", index);
    debug(DEBUG_TRACE, "SymetrieHexapod::getConfigItem", "Writing message", command);
    immediateWriteRead(command, response);
    debug(DEBUG_TRACE, "SymetrieHexapod::getConfigItem", "Reading response", response);
    sscanf(response, "%d", &params[index]);
  }
  return c_cmd;
}

int SymetrieHexapod::setConfigItem(const std::string& cmd, int num_params, double *params)
{
  char response[CHAR_ARRAY_SIZE];
  int c_cmd = 1;
  std::stringstream ss;
  ss << "c_cfg=1";
  for (int index = 0; index < num_params; index++){
    ss << " c_par(" << index << ")=" << params[index];
  }
  ss << " c_cmd=" << cmd;
  debug(DEBUG_TRACE, "SymetrieHexapod::setConfigItem", ss.str());
  immediateWriteRead(ss.str().c_str(), response);
  c_cmd = waitForCommandComplete();
  return c_cmd;
}

int SymetrieHexapod::setConfigItem(const std::string& cmd, int num_params, int *params)
{
  char response[CHAR_ARRAY_SIZE];
  int c_cmd = 1;
  std::stringstream ss;
  ss << "c_cfg=1";
  for (int index = 0; index < num_params; index++){
    ss << " c_par(" << index << ")=" << params[index];
  }
  ss << " c_cmd=" << cmd;
  debug(DEBUG_TRACE, "SymetrieHexapod::setConfigItem", ss.str());
  immediateWriteRead(ss.str().c_str(), response);
  c_cmd = waitForCommandComplete();
  return c_cmd;
}

void SymetrieHexapod::getVersion()
{
  char command[CHAR_ARRAY_SIZE];
  char response[CHAR_ARRAY_SIZE];
  int params[12] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};
  std::stringstream ctrl_ver;
  std::stringstream api_ver;
  debug(DEBUG_TRACE, "SymetrieHexapod::getVersion");

  immediateWriteRead("c_cmd=C_VERSION", response);
  waitForCommandComplete();
  for (int index = 0; index < 12; index++){
    sprintf(command, "c_par(%d)", index);
    immediateWriteRead(command, response);
    sscanf(response, "%d", &params[index]);
  }
  // Now place the retrived values into the correct asyn parameters
  setIntegerParam(SYM_HEX_VERS_CTRL_ID_, params[0]);
  ctrl_ver << params[1] << "." << params[2] << "." << params[3] << "." << params[4];
  setStringParam(SYM_HEX_VERS_CTRL_VER_, ctrl_ver.str().c_str());
  api_ver << params[5] << "." << params[6] << "." << params[7] << "." << params[8];
  setStringParam(SYM_HEX_VERS_API_VER_, api_ver.str().c_str());
  setIntegerParam(SYM_HEX_VERS_SYS_ID_, params[9]);
  setIntegerParam(SYM_HEX_VERS_SYS_NUM_, params[10]);
  setIntegerParam(SYM_HEX_VERS_SYS_CFG_, params[11]);
  callParamCallbacks();
}

int SymetrieHexapod::waitForCommandComplete()
{
  char response[CHAR_ARRAY_SIZE];
  int c_cmd = 1;
  int counter = 0;
  while (c_cmd > 0 && counter < 250){
    immediateWriteRead("c_cmd", response);
    sscanf(response, "%d", &c_cmd);
    if (c_cmd > 0){
      counter++;
      epicsThreadSleep(0.01);
    }
  }
  if (counter >= 250){
    debug(DEBUG_ERROR, "SymetrieHexapod::waitForCommandComplete", "Timed out waiting for c_cmd to equal 0", c_cmd);
    c_cmd = -1;
  }
  if (c_cmd < 0){
    debug(DEBUG_TRACE, "SymetrieHexapod::waitForCommandComplete", "Command failure", c_cmd);
  }
  return c_cmd;
}

void SymetrieHexapod::recordCmdResult(int code)
{
  std::string failure = "";
  switch (code)
  {
    case -1:
      failure = "Undefined error.";
      break;
    case -10:
      failure = "Wrong value for parameter at index 0.";
      break;
    case -11:
      failure = "Wrong value for parameter at index 1.";
      break;
    case -12:
      failure = "Wrong value for parameter at index 2.";
      break;
    case -13:
      failure = "Wrong value for parameter at index 3.";
      break;
    case -14:
      failure = "Wrong value for parameter at index 4.";
      break;
    case -15:
      failure = "Wrong value for parameter at index 5.";
      break;
    case -16:
      failure = "Wrong value for parameter at index 6.";
      break;
    case -17:
      failure = "Wrong value for parameter at index 7.";
      break;
    case -18:
      failure = "Wrong value for parameter at index 8.";
      break;
    case -19:
      failure = "Wrong value for parameter at index 9.";
      break;
    case -20:
      failure = "Wrong value for parameter at index 10.";
      break;
    case -21:
      failure = "Wrong value for parameter at index 11.";
      break;
    case -22:
      failure = "Wrong value for parameter at index 12.";
      break;
    case -23:
      failure = "Wrong value for parameter at index 13.";
      break;
    case -24:
      failure = "Wrong value for parameter at index 14.";
      break;
    case -25:
      failure = "Wrong value for parameter at index 15.";
      break;
    case -26:
      failure = "Wrong value for parameter at index 16.";
      break;
    case -27:
      failure = "Wrong value for parameter at index 17.";
      break;
    case -28:
      failure = "Wrong value for parameter at index 18.";
      break;
    case -29:
      failure = "Wrong value for parameter at index 19.";
      break;
    case -30:
      failure = "Unknown command number.";
      break;
    case -31:
      failure = "This configuration command is a 'get' only type.";
      break;
    case -32:
      failure = "This configuration command is a 'set' only type.";
      break;
    case -33:
      failure = "The axis number does not correspond to an axis defined on the controller.";
      break;
    case -34:
      failure = "A stop task is running.";
      break;
    case -35:
      failure = "All motors need to be control on.";
      break;
    case -36:
      failure = "All motors need to be control off.";
      break;
    case -37:
      failure = "Emergency stop is pressed.";
      break;
    case -38:
      failure = "A motion task is running.";
      break;
    case -39:
      failure = "A home task is running.";
      break;
    case -40:
      failure = "Requested move is not feasible.";
      break;
    case -41:
      failure = "Power supply of limit switches is off.";
      break;
    case -42:
      failure = "Power supply of encoders is off.";
      break;
    case -43:
      failure = "A fatal error is present.  This type of error needs a controller restart to be removed.";
      break;
    case -44:
      failure = "An error is present, error reset is required.";
      break;
    case -45:
      failure = "Home is not completed.";
      break;
    case -46:
      failure = "Software option not available (can be linked to hardware configuration).";
      break;
    case -47:
      failure = "Virtual home: file was created on another controller (different MAC address).";
      break;
    case -48:
      failure = "Virtual home: some positions read in file are out of software limits.";
      break;
    case -49:
      failure = "Virtual home: file data were stored while hexapod was moving.";
      break;
    case -50:
      failure = "Virtual home: no data available.";
      break;
    case -51:
      failure = "Command has been rejected.";
      break;
    case -52:
      failure = "Timeout waiting for home complete status.";
      break;
    case -53:
      failure = "Timeout waiting for control on status.";
      break;
    case -54:
      failure = "Timeout on motion program start.";
      break;
    case -55:
      failure = "Timeout on home task start.";
      break;
    case -56:
      failure = "Timeout on virtual home write file task.";
      break;
    case -57:
      failure = "Timeout on virtual home delete file task.";
      break;
    case -58:
      failure = "Timeout on virtual home read file task.";
      break;
    case -59:
      failure = "Timeout on disk access verification task.";
      break;
    case -60:
      failure = "Configuration file: save process failed.";
      break;
    case -61:
      failure = "Configuration file: loaded file is empty.";
      break;
    case -62:
      failure = "Configuration file: loaded data are corrupted.";
      break;
    case -63:
      failure = "No access to the memory disk.";
      break;
    case -64:
      failure = "File does not exist.";
      break;
    case -65:
      failure = "Folder access failed.";
      break;
    case -66:
      failure = "Creation of folder tree on the memory disk failed.";
      break;
    case -67:
      failure = "Generation of write of the checksum failed.";
      break;
    case -68:
      failure = "File read: no data or wrong data size.";
      break;
    case -69:
      failure = "File read: no checksum.";
      break;
    case -70:
      failure = "File read: incorrect checksum.";
      break;
    case -71:
      failure = "File write: failed.";
      break;
    case -72:
      failure = "File open: failed.";
      break;
    case -73:
      failure = "File delete: failed.";
      break;
    case -74:
      failure = "Get MAC address failed.";
      break;
    case -75:
      failure = "NaN (Not a Number) or infinite value found.";
      break;
    case -76:
      failure = "The coordinate system transformations are not initialized.";
      break;
    case -77:
      failure = "A kinematic error is present.";
      break;
    case -78:
      failure = "The motor phase process failed (phase search or phase set from position offset).";
      break;
    case -79:
      failure = "The motor phase is not found.";
      break;
    case -80:
      failure = "Timeout waiting for control off status.";
      break;
    case -81:
      failure = "The requested kinematic mode (number) is not defined for the machine.";
      break;
    case -82:
      failure = "Timeout waiting for phase found status.";
      break;
    case -83: failure = "An error has been generated. Command abort."; break;
    case -84: failure = "The absolute encoder reading returned an error."; break;
    case -85: failure = "The power dissipation reduction functionality is disabled. The command \"CFG_POWER\" can be used to enable the functionality."; break;
    case -86: failure = "The absolute encoder reading returned an error. Serial error bits are true."; break;
    case -87: failure = "The absolute encoder reading returned an error. Read position is considered as an error (On some encoders, a zero position can be considered as an error)."; break;
    case -88: failure = "When the configuration file failed to load, a recovery sequence is requested before the command \"CFG_SAVE\" is accepted, this to force the execution of some configuration commands. Here, the \"CFG_DEFAULT\" command is requested."; break;
    case -89: failure = "When the configuration file failed to load, a recovery sequence is requested before the command \"CFG_SAVE\" is accepted, this to force the execution of some configuration commands. Here, the \"CFG_CHANEL\" command is requested."; break;
    case -90: failure = "When the configuration file failed to load, a recovery sequence is requested. Here, the \"CFG_SAVE\" is requested."; break;
    
    case -92: failure = "Not available with the current kinematic mode."; break;
    
    case -1000: failure = "Internal error: \"RET_Dev_CfS_NaNReturned\"."; break;
    case -1001: failure = "Internal error: \"RET_Dev_CfS_FctNotAvailableInKernel\"."; break;
    case -1002: failure = "Internal error: \"RET_Dev_CfS_UndefinedCfSType\"."; break;
    case -1003: failure = "Internal error: \"RET_Dev_CfS_FIO_UndefinedFioType\"."; break;
    case -1004: failure = "Internal error: \"RET_Dev_CfS_FIO_HomeFile_UndefinedAction\"."; break;
    case -1005: failure = "Internal error: \"RET_Dev_UndefinedEnumValue\"."; break;
    case -1006: failure = "Internal error: \"RET_Dev_LdataCmdStatusIsNegative\"."; break;
    case -1007: failure = "Internal error: \"RET_Dev_NumMotorsInCoord_Sup_DEF_aGrQ_SIZE\"."; break;
    case -1008: failure = "Internal error: \"RET_Dev_NumMotorsInCoord_WrongNumber\"."; break;
    case -1009: failure = "Internal error: \"RET_String_StrCat_DestSizeReached\"."; break;
    case -1010: failure = "Internal error: \"RET_String_LengthOverStringSize\"."; break;
    case -1011: failure = "Internal error: \"RET_String_AllCharShouldIntBetween_0_255\"."; break;
    case -1012: failure = "Internal error: \"RET_String_StrCpy_DestSizeReached\"."; break;
    case -1013: failure = "Internal error: \"RET_ErrAction_HomeReset\"."; break;
    case -1014: failure = "Internal error: \"RET_Home_StopReceivedWhileRunning\"."; break;
    case -1015: failure = "Internal error: \"RET_UndefinedKinAssembly\"."; break;
    case -1016: failure = "Internal error: \"RET_WrongPmcConfig\"."; break;
    case -1017: failure = "Internal error: \"RET_WrongScrewingCalculation\"."; break;
    
    default:
      failure = "";
  }
  if (code < 0){
    debug(DEBUG_ERROR, "SymetrieHexapod::recordCmdResult", "Command Code", code);
    debug(DEBUG_ERROR, "SymetrieHexapod::recordCmdResult", "Command messsage", failure);
  }
  setIntegerParam(SYM_HEX_CMD_CODE_, code);
  setStringParam(SYM_HEX_CMD_DESC_, failure.c_str());
}

void SymetrieHexapod::copyUserToObject()
{
  double cs[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  getDoubleParam(SYM_HEX_TXU_, &cs[0]);
  getDoubleParam(SYM_HEX_TYU_, &cs[1]);
  getDoubleParam(SYM_HEX_TZU_, &cs[2]);
  getDoubleParam(SYM_HEX_RXU_, &cs[3]);
  getDoubleParam(SYM_HEX_RYU_, &cs[4]);
  getDoubleParam(SYM_HEX_RZU_, &cs[5]);
  cs[6] = cs[0];
  cs[7] = cs[1];
  cs[8] = cs[2];
  cs[9] = cs[3];
  cs[10] = cs[4];
  cs[11] = cs[5];
  setConfigItem("C_CFG_CS", 12, cs);
  readAllConfig();
}

void SymetrieHexapod::copyObjectToUser()
{
  double cs[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  getDoubleParam(SYM_HEX_TXO_, &cs[6]);
  getDoubleParam(SYM_HEX_TYO_, &cs[7]);
  getDoubleParam(SYM_HEX_TZO_, &cs[8]);
  getDoubleParam(SYM_HEX_RXO_, &cs[9]);
  getDoubleParam(SYM_HEX_RYO_, &cs[10]);
  getDoubleParam(SYM_HEX_RZO_, &cs[11]);
  cs[0] = cs[6];
  cs[1] = cs[7];
  cs[2] = cs[8];
  cs[3] = cs[9];
  cs[4] = cs[10];
  cs[5] = cs[11];
  setConfigItem("C_CFG_CS", 12, cs);
  readAllConfig();
}

/**
 * Deal with controller specific epicsInt32 params.
 * @param pasynUser
 * @param value
 * @param asynStatus
 */
asynStatus SymetrieHexapod::writeInt32(asynUser *pasynUser, epicsInt32 value) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  int currentValue = 0.0;
  getIntegerParam(function, &currentValue);
  const std::string functionName = "SymetrieHexapod::writeInt32";

  debug(DEBUG_TRACE, functionName);
  if (function == SYM_HEX_SAFETY_SET_){
    int result = applyConfigSafety();
    recordCmdResult(result);
    readAllConfig();
  }
  if (function == SYM_HEX_CHANNEL_SET_){
    int result = applyConfigChannel();
    recordCmdResult(result);
    readAllConfig();
  }
  if (function == SYM_HEX_SPEED_SET_){
    int result = applyConfigSpeed();
    recordCmdResult(result);
    readAllConfig();
  }
  if (function == SYM_HEX_TA_SET_){
    int result = applyConfigTa();
    recordCmdResult(result);
    readAllConfig();
  }
  if (function == SYM_HEX_CS_SET_){
    int result = applyConfigCS();
    recordCmdResult(result);
    readAllConfig();
  }
  if (function == SYM_HEX_MLIM_SET_){
    int result = applyConfigMachineLimit();
    recordCmdResult(result);
    readAllConfig();
  }
  if (function == SYM_HEX_MLIM_ENABLE_SET_){
    int result = applyConfigMachineLimitEnable();
    recordCmdResult(result);
    readAllConfig();
  }
  if (function == SYM_HEX_ULIM_SET_){
    int result = applyConfigUserLimit();
    recordCmdResult(result);
    readAllConfig();
  }
  if (function == SYM_HEX_ULIM_ENABLE_SET_){
    int result = applyConfigUserLimitEnable();
    recordCmdResult(result);
    readAllConfig();
  }
  if (function == SYM_HEX_CONTROL_SET_){
    int result = applyConfigControl();
    recordCmdResult(result);
    readAllConfig();
  }
  if (function == SYM_HEX_STALL_SET_){
    int result = applyConfigStallCurrent();
    recordCmdResult(result);
    readAllConfig();
  }
  if (function == SYM_HEX_BKL_SET_){
    int result = applyConfigBacklash();
    recordCmdResult(result);
    readAllConfig();
  }
  if (function == SYM_HEX_KIN_SET_){
    int result = applyConfigKinematic();
    recordCmdResult(result);
    readAllConfig();
  }
  if (function == SYM_HEX_TUNING_SET_){
    int result = applyConfigTuning();
    recordCmdResult(result);
    readAllConfig();
  }
  if (function == SYM_HEX_CFG_POWER_SET_){
    int result = applyConfigPower();
    recordCmdResult(result);
    readAllConfig();
  }
  if (function == SYM_HEX_STOP_){
    char response[CHAR_ARRAY_SIZE];
    immediateWriteRead("c_cmd=C_STOP", response);
    int result = waitForCommandComplete();
    recordCmdResult(result);
  }
  if (function == SYM_HEX_CONTROLON_){
    char response[CHAR_ARRAY_SIZE];
    immediateWriteRead("c_cmd=C_CONTROLON", response);
    int result = waitForCommandComplete();
    recordCmdResult(result);
  }
  if (function == SYM_HEX_CONTROLOFF_){
    char response[CHAR_ARRAY_SIZE];
    immediateWriteRead("c_cmd=C_CONTROLOFF", response);
    int result = waitForCommandComplete();
    recordCmdResult(result);
  }
  if (function == SYM_HEX_HOME_){
    char response[CHAR_ARRAY_SIZE];
    immediateWriteRead("c_cmd=C_HOME", response);
    int result = waitForCommandComplete();
    recordCmdResult(result);
  }
  if (function == SYM_HEX_HOMEVIRTUAL_){
    char response[CHAR_ARRAY_SIZE];
    immediateWriteRead("c_cmd=C_HOMEVIRTUAL", response);
    int result = waitForCommandComplete();
    recordCmdResult(result);
  }
  if (function == SYM_HEX_CLEARERROR_){
    char response[CHAR_ARRAY_SIZE];
    immediateWriteRead("c_cmd=C_CLEARERROR", response);
    int result = waitForCommandComplete();
    recordCmdResult(result);
  }
  if (function == SYM_HEX_STATUS_READ_){
    readStatus();
  }
  if (function == SYM_HEX_SPC_MOVE_) {
    int demand = 0;
    char command[CHAR_ARRAY_SIZE];
    char response[CHAR_ARRAY_SIZE];
    getIntegerParam(SYM_HEX_SPC_MOVE_DMD_, &demand);
    sprintf(command, "c_par(0)=%d c_cmd=C_MOVE_SPECIFICPOS", demand);
    immediateWriteRead(command, response);
    int result = waitForCommandComplete();
    recordCmdResult(result);
  }
  if (function == SYM_HEX_MOVE_) {
    int type = 0;
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;
    std::stringstream cmd;
    char response[CHAR_ARRAY_SIZE];
    getIntegerParam(SYM_HEX_MOVE_TYPE_, &type);
    getDoubleParam(SYM_HEX_MOVE_TX_DMD_, &tx);
    getDoubleParam(SYM_HEX_MOVE_TY_DMD_, &ty);
    getDoubleParam(SYM_HEX_MOVE_TZ_DMD_, &tz);
    getDoubleParam(SYM_HEX_MOVE_RX_DMD_, &rx);
    getDoubleParam(SYM_HEX_MOVE_RY_DMD_, &ry);
    getDoubleParam(SYM_HEX_MOVE_RZ_DMD_, &rz);
    cmd << "c_par(0)=" << type << " ";
    cmd << "c_par(1)=" << tx << " ";
    cmd << "c_par(2)=" << ty << " ";
    cmd << "c_par(3)=" << tz << " ";
    cmd << "c_par(4)=" << rx << " ";
    cmd << "c_par(5)=" << ry << " ";
    cmd << "c_par(6)=" << rz << " ";
    cmd << "c_cmd=C_MOVE_PTP";
    immediateWriteRead(cmd.str().c_str(), response);
    int result = waitForCommandComplete();
    recordCmdResult(result);
  }
  if (function == SYM_HEX_MOVE_SEQ_){
    char response[CHAR_ARRAY_SIZE];
    immediateWriteRead("c_cmd=C_MOVE_SEQ", response);
    int result = waitForCommandComplete();
    recordCmdResult(result);
  }
  if (function == SYM_HEX_VALID_PTP_){
    int mode = 0;
    int type = 0;
    int bit_reply = 0;
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;
    std::stringstream cmd;
    char response[CHAR_ARRAY_SIZE];
    getIntegerParam(SYM_HEX_VALID_PTP_MODE_, &mode);
    getIntegerParam(SYM_HEX_VALID_PTP_TYPE_, &type);
    getDoubleParam(SYM_HEX_VALID_PTP_TX_, &tx);
    getDoubleParam(SYM_HEX_VALID_PTP_TY_, &ty);
    getDoubleParam(SYM_HEX_VALID_PTP_TZ_, &tz);
    getDoubleParam(SYM_HEX_VALID_PTP_RX_, &rx);
    getDoubleParam(SYM_HEX_VALID_PTP_RY_, &ry);
    getDoubleParam(SYM_HEX_VALID_PTP_RZ_, &rz);
    cmd << "c_par(0)=" << mode << " ";
    cmd << "c_par(1)=" << type << " ";
    cmd << "c_par(2)=" << tx << " ";
    cmd << "c_par(3)=" << ty << " ";
    cmd << "c_par(4)=" << tz << " ";
    cmd << "c_par(5)=" << rx << " ";
    cmd << "c_par(6)=" << ry << " ";
    cmd << "c_par(7)=" << rz << " ";
    cmd << "c_cmd=C_VALID_PTP";
    debug(DEBUG_TRACE, "SymetrieHexapod::writeInt32", "Validating", cmd.str());
    immediateWriteRead(cmd.str().c_str(), response);
    int result = waitForCommandComplete();
    recordCmdResult(result);
    if (result == 0){
      immediateWriteRead("c_par(0)", response);
      sscanf(response, "%d", &bit_reply);
      setIntegerParam(SYM_HEX_VALID_PTP_RESULT_, bit_reply);
    }
  }
  if (function == SYM_HEX_POWER_){
    int power = 0;
    std::stringstream cmd;
    char response[CHAR_ARRAY_SIZE];
    getIntegerParam(SYM_HEX_POWER_POWER_, &power);
    cmd << "c_par(0)=" << power << " ";
    cmd << "c_cmd=C_POWER";
    immediateWriteRead(cmd.str().c_str(), response);
    int result = waitForCommandComplete();
    recordCmdResult(result);
  }
  
  if (function == SYM_HEX_MAINTENANCE_){
    int mode = 0;
    int axis = 0;
    std::stringstream cmd;
    char response[CHAR_ARRAY_SIZE];
    getIntegerParam(SYM_HEX_MAINTENANCE_MODE_, &mode);
    getIntegerParam(SYM_HEX_MAINTENANCE_AXIS_, &axis);
    cmd << "c_par(0)=" << mode << " ";
    if (mode == 2){
       cmd << "c_par(1)=" <<axis << " ";
    }
    cmd << "c_cmd=C_MAINENANCE";
    immediateWriteRead(cmd.str().c_str(), response);
    int result = waitForCommandComplete();
    recordCmdResult(result);
  }
  if (function == SYM_HEX_JOG_){
    int axis = 0;
    double inc = 0.0;
    std::stringstream cmd;
    char response[CHAR_ARRAY_SIZE];
    getIntegerParam(SYM_HEX_JOG_AXIS_, &axis);
    getDoubleParam(SYM_HEX_JOG_INCR_, &inc);
    cmd << "c_ax=" << axis << " ";
    cmd << "c_par(0)=" << inc << " ";
    cmd << "c_cmd=C_AXIS_JOG";
    immediateWriteRead(cmd.str().c_str(), response);
    int result = waitForCommandComplete();
    recordCmdResult(result);
  }
  if (function == SYM_HEX_REBOOT_){
    char response[CHAR_ARRAY_SIZE];
    immediateWriteRead("system reboot", response);
  }
  if (function == SYM_HEX_USR_TO_OBJ_){
    copyUserToObject();
  }
  if (function == SYM_HEX_OBJ_TO_USR_){
    copyObjectToUser();
  }
  if (function == SYM_HEX_CFG_GET_){
    readAllConfig();
  }
  if (function == SYM_HEX_CFG_SET_){
    applyConfig();
    // Once applied read back all of the values
    readAllConfig();
  }
  if (function == SYM_HEX_CFG_DFLT_){
    int vals[] = {0};
    int reply = setConfigItem("C_CFG_DEFAULT", 0, vals);
    if (reply < 0){
      debug(DEBUG_ERROR, "SymetrieHexapod::writeInt32", "C_CFG_DEFAULT failed");
      recordCmdResult(reply);
    }
    // Now read back all config items
    readAllConfig();
  }
  if (function == SYM_HEX_CFG_SAVE_){
    // Issue a CFG_SAVE command
    int vals[] = {0};
    int reply = setConfigItem("C_CFG_SAVE", 0, vals);
    if (reply < 0){
      debug(DEBUG_ERROR, "SymetrieHexapod::writeInt32", "C_CFG_SAVE failed");
      recordCmdResult(reply);
    }
  }
  callParamCallbacks();

  //Call base class method. This will handle callCallbacks even if the function was handled here.
  status = pmacController::writeInt32(pasynUser, value);
  return status;
}


/**
 * Deal with controller specific epicsInt32 params.
 * @param pasynUser
 * @param value
 * @param asynStatus
 */
asynStatus SymetrieHexapod::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  double currentValue = 0.0;
  getDoubleParam(function, &currentValue);

  debug(DEBUG_TRACE, "SymetrieHexapod::writeFloat64");

  //Call base class method. This will handle callCallbacks even if the function was handled here.
  status = pmacController::writeFloat64(pasynUser, value);
  return status;
}

/** Called when asyn clients call pasynOctet->write().
  * This function performs actions for some parameters, including AttributesFile.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Address of the string to write.
  * \param[in] nChars Number of characters to write.
  * \param[out] nActual Number of characters actually written. */
asynStatus SymetrieHexapod::writeOctet(asynUser *pasynUser, const char *value, size_t nChars, size_t *nActual) {
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  const char *functionName = "SymetrieHexapod::writeOctet";

  // Set the parameter in the parameter library.
  status = (asynStatus) setStringParam(function, (char *)value);
  if (status != asynSuccess) return (status);

  if (function == SYM_HEX_TERMINAL_SEND_){
    char response[CHAR_ARRAY_SIZE];
    debug(DEBUG_ERROR, functionName, "Sending terminal command", value);
    immediateWriteRead(value, response);
    debug(DEBUG_ERROR, functionName, "Response from terminal command", response);
    setStringParam(SYM_HEX_TERMINAL_RECV_, response);
  }

  // Do callbacks so higher layers see any changes
  callParamCallbacks();

  //Call base class method. This will handle callCallbacks even if the function was handled here.
  status = pmacController::writeOctet(pasynUser, value, nChars, nActual);
  return status;
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void SymetrieHexapod::report(FILE *fp, int level)
{
    fprintf(fp, "SymetrieHexapod motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
      this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

    // Call the base class method
    pmacController::report(fp, level);
}

// These are the methods that override those for asynMotorController and asynMotorAxis

/** Returns a pointer to an SymetrieAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
SymetrieAxis* SymetrieHexapod::getAxis(asynUser *pasynUser)
{
  return static_cast<SymetrieAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an SymetrieAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
SymetrieAxis* SymetrieHexapod::getAxis(int axisNo)
{
  return static_cast<SymetrieAxis*>(asynMotorController::getAxis(axisNo));
}

// We implement this method because we can read the status for all axes in a single call
asynStatus SymetrieHexapod::poll()
{
  readStatus();
  return asynSuccess;
}

// These are the SymetrieAxis methods

/** Creates a new SymetrieAxis object.
  * \param[in] pC Pointer to the SymetrieHexapod to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
SymetrieAxis::SymetrieAxis(SymetrieHexapod *pC, int axisNo)
  : pmacAxis(pC, axisNo),
    pC_(pC)
{  
}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void SymetrieAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n",
            axisNo_);
  }

  // Call the base class method
  pmacAxis::report(fp, level);
}

asynStatus SymetrieAxis::setVelocityAndAcceleration(double velocity, double acceleration)
{
  double velos[2];
  int status;
  static const char *functionName = "SymetrieAxis::setVelocityAndAcceleration";

  // Convert from motor record units to device units
  velocity /= MOTOR_SCALE_FACTOR;
  acceleration /= MOTOR_SCALE_FACTOR;
  // Acceleration is in units/s^2 but device needs time
  acceleration = velocity/acceleration;
  if (axisNo_ < 3)
    pC_->setDoubleParam(pC_->SYM_HEX_VT_DMD_, velocity);
  else
    pC_->setDoubleParam(pC_->SYM_HEX_VR_DMD_, velocity);
  // Read the velocity parameters and apply them
  pC_->getDoubleParam(pC_->SYM_HEX_VT_DMD_, &velos[0]);
  pC_->getDoubleParam(pC_->SYM_HEX_VR_DMD_, &velos[1]);
  status = pC_->setConfigItem("C_CFG_SPEED", 2, velos);
  if (status < 0){
    debug(DEBUG_ERROR, "%s: C_CFG_SPEED failed", functionName);
  }
  pC_->setDoubleParam(pC_->SYM_HEX_TA_DMD_, acceleration);
  status = pC_->setConfigItem("C_CFG_TA", 1, &acceleration);
  if (status < 0) {
    debug(DEBUG_ERROR, "%s: C_CFG_TA failed", functionName);
  }
  return status ? asynError : asynSuccess;
}

asynStatus SymetrieAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  //static const char *functionName = "SymetrieAxis::move";

  //printf("%s, position=%f, relative=%d, minVelocity=%f, maxVelocity=%f, acceleration=%f\n",
  //       functionName, position, relative, minVelocity, maxVelocity, acceleration);
  setVelocityAndAcceleration(maxVelocity, acceleration);
  // If relative is true we use USER_RELATIVE coordinate system.  May want to change to OBJECT_RELATIVE (1)
  int moveType = relative ? 2 : 0;
  // Target positions of all axes
  double positions[NUM_MOTOR_AXES];
  // Set target position for this axis;
  pC_->setDoubleParam(pC_->SYM_HEX_MOVE_TX_DMD_ + axisNo_, position/MOTOR_SCALE_FACTOR);

  std::stringstream cmd;
  char response[CHAR_ARRAY_SIZE];
  cmd << "c_par(0)=" << moveType << " ";
  for (int axis=0; axis<NUM_MOTOR_AXES; axis++) {
    pC_->getDoubleParam(pC_->SYM_HEX_MOVE_TX_DMD_ + axis, &positions[axis]);
    cmd << "c_par(" << axis+1 << ")=" << positions[axis] << " ";
  }
  cmd << "c_cmd=C_MOVE_PTP";
  pC_->immediateWriteRead(cmd.str().c_str(), response);
  int result = pC_->waitForCommandComplete();
  pC_->recordCmdResult(result);
  return result ? asynError : asynSuccess;
}

asynStatus SymetrieAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  char response[CHAR_ARRAY_SIZE];
  //static const char *functionName = "SymetrieAxis::home";

  //printf("%s, minVelocity=%f, maxVelocity=%f, acceleration=%f, forwards=%d\n", 
  //       functionName, minVelocity, maxVelocity, acceleration, forwards);
  setVelocityAndAcceleration(maxVelocity, acceleration);
  pC_->immediateWriteRead("c_cmd=C_HOME", response);
  int result = pC_->waitForCommandComplete();
  pC_->recordCmdResult(result);
  return result ? asynError : asynSuccess;
}

asynStatus SymetrieAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  static const char *functionName = "SymetrieAxis::moveVelocity";

  asynPrint(pasynUser_, ASYN_TRACE_ERROR,
            "%s: moveVelocity not supported, minVelocity=%f, maxVelocity=%f, acceleration=%f\n", 
            functionName, minVelocity, maxVelocity, acceleration);
  return asynError;
}

asynStatus SymetrieAxis::stop(double acceleration )
{
  char response[CHAR_ARRAY_SIZE];
  static const char *functionName = "SymetrieAxis::stop";

  printf("%s, acceleration=%f\n", functionName, acceleration);
  pC_->immediateWriteRead("c_cmd=C_STOP", response);
  int result = pC_->waitForCommandComplete();
  pC_->recordCmdResult(result);
  return result ? asynError : asynSuccess;
}

asynStatus SymetrieAxis::setPosition(double position)
{
  static const char *functionName = "SymetrieAxis::setPosition";

  printf("%s: position=%f\n", functionName, position);
  // Set target position for this axis;
  setDoubleParam(pC_->SYM_HEX_MOVE_TX_DMD_ + axisNo_, position/MOTOR_SCALE_FACTOR);
  return asynSuccess;
}

asynStatus SymetrieAxis::setClosedLoop(bool closedLoop)
{
  //static const char *functionName = "SymetrieAxis::setClosedLoop";
  char response[CHAR_ARRAY_SIZE];

  //asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s, closedLoop=%s\n", functionName, closedLoop ? "true" : "false");
  if (closedLoop)
    pC_->immediateWriteRead("c_cmd=C_CONTROLON", response);
  else
    pC_->immediateWriteRead("c_cmd=C_CONTROLOFF", response);
  int result = pC_->waitForCommandComplete();
  pC_->recordCmdResult(result);
  //asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s, result=%d returning=%d\n", functionName, result, result ? asynError : asynSuccess);
  return result ? asynError : asynSuccess;
}

/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status, 
  * and the drive power-on status. 
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus SymetrieAxis::poll(bool *moving)
{ 
  int hexapodStatus;
  int motionDone;
  int driveOn;
  double position;
  //static const char *functionName = "SymetrieAxis::poll";

  // All of the values have already been updated by the call to readStatus in the SymetrieHexapos::poll() method

  // Current motor position
  pC_->getDoubleParam(pC_->SYM_HEX_S_UTO_TX_ + axisNo_, &position);
  setDoubleParam(pC_->motorPosition_, position*MOTOR_SCALE_FACTOR);

  // Moving status.  Use status of motion task.
  pC_->getIntegerParam(pC_->SYM_HEX_S_HEXA_, &hexapodStatus);
  motionDone = (hexapodStatus & 16) == 0;
  setIntegerParam(pC_->motorStatusDone_, motionDone);
  *moving = motionDone ? false : true;

  // Limit status.  
  // Virtual axes don't have hardware limits
  // We may want to use software limit status from checking move validity
  setIntegerParam(pC_->motorStatusHighLimit_, 0);
  setIntegerParam(pC_->motorStatusLowLimit_, 0);
  setIntegerParam(pC_->motorStatusAtHome_, 0);

  // Gain support.  This allows turning power on and off
  setIntegerParam(pC_->motorStatusGainSupport_, 1);
  
  // Drive power on status
  driveOn = (hexapodStatus & 4) != 0;
  setIntegerParam(pC_->motorStatusPowerOn_, driveOn);
 
  // No problem status for now.
  setIntegerParam(pC_->motorStatusProblem_, 0);

  callParamCallbacks();
  return asynSuccess;
}


SymetrieHexapod::~SymetrieHexapod()
{
}

/*************************************************************************************/
/** The following functions have C linkage, and can be called directly or from iocsh */

extern "C" {

/**
 * C wrapper for the SymetrieHexapod constructor.
 */
asynStatus
symHexCreateController(const char *portName,
                       const char *lowLevelPortName, 
                       int lowLevelPortAddress,
                       double movingPollPeriod,
                       double idlePollPeriod) 
{
  if (movingPollPeriod == 0) movingPollPeriod = 5.;
  if (idlePollPeriod == 0) idlePollPeriod = 5.;
  new SymetrieHexapod(portName,
                      lowLevelPortName,
                      lowLevelPortAddress,
                      movingPollPeriod,
                      idlePollPeriod);
  return asynSuccess;
}

/* Code for iocsh registration */

/* pmacCreateController */
static const iocshArg symHexCreateControllerArg0 = {"Controller port name", iocshArgString};
static const iocshArg symHexCreateControllerArg1 = {"Low level port name", iocshArgString};
static const iocshArg symHexCreateControllerArg2 = {"Low level port address", iocshArgInt};
static const iocshArg symHexCreateControllerArg3 = {"Moving poll period", iocshArgDouble};
static const iocshArg symHexCreateControllerArg4 = {"Idle poll period", iocshArgDouble};
static const iocshArg *const symHexCreateControllerArgs[] = {&symHexCreateControllerArg0,
                                                             &symHexCreateControllerArg1,
                                                             &symHexCreateControllerArg2,
                                                             &symHexCreateControllerArg3,
                                                             &symHexCreateControllerArg4};
static const iocshFuncDef configsymHexCreateController = {"symetrieHexapod", 5,
                                                          symHexCreateControllerArgs};
static void configsymHexCreateControllerCallFunc(const iocshArgBuf *args) {
  symHexCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].dval, args[4].dval);
}


static void symHexControllerRegister(void) {
  iocshRegister(&configsymHexCreateController, configsymHexCreateControllerCallFunc);
}
epicsExportRegistrar(symHexControllerRegister);

#ifdef vxWorks
//VxWorks register functions
epicsRegisterFunction(symHexCreateController);
#endif
} // extern "C"

