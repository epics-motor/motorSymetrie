/********************************************
 *  pmacController.h 
 * 
 *  PMAC Asyn motor based on the 
 *  asynMotorController class.
 * 
 *  Matthew Pearson
 *  23 May 2012
 * 
 ********************************************/

#ifndef SymetrieHexapod_H
#define SymetrieHexapod_H

#include <math.h>
#include "pmacController.h"

#define NUM_MOTOR_AXES 6
// This converts motor record integer "steps" into hexapod user units
#define MOTOR_SCALE_FACTOR 1.e4

#define SYM_HEX_FirstParamString           "SYM_HEX_FIRSTPARAM"
#define SYM_HEX_LastParamString            "SYM_HEX_LASTPARAM"

#define SYM_HEX_STOPString                 "SYM_HEX_STOP"
#define SYM_HEX_CONTROLONString            "SYM_HEX_CONTROLON"
#define SYM_HEX_CONTROLOFFString           "SYM_HEX_CONTROLOFF"
#define SYM_HEX_HOMEString                 "SYM_HEX_HOME"
#define SYM_HEX_HOMEVIRTUALString          "SYM_HEX_HOMEVIRTUAL"
#define SYM_HEX_CLEARERRORString           "SYM_HEX_CLEARERROR"

#define SYM_HEX_STATUS_READString          "SYM_HEX_STATUS_READ"
#define SYM_HEX_S_HEXAString               "SYM_HEX_S_HEXA"
#define SYM_HEX_S_ACTIONString             "SYM_HEX_S_ACTION"
#define SYM_HEX_S_UTO_TXString             "SYM_HEX_S_UTO_TX"
#define SYM_HEX_S_UTO_TYString             "SYM_HEX_S_UTO_TY"
#define SYM_HEX_S_UTO_TZString             "SYM_HEX_S_UTO_TZ"
#define SYM_HEX_S_UTO_RXString             "SYM_HEX_S_UTO_RX"
#define SYM_HEX_S_UTO_RYString             "SYM_HEX_S_UTO_RY"
#define SYM_HEX_S_UTO_RZString             "SYM_HEX_S_UTO_RZ"
#define SYM_HEX_S_MTP_TXString             "SYM_HEX_S_MTP_TX"
#define SYM_HEX_S_MTP_TYString             "SYM_HEX_S_MTP_TY"
#define SYM_HEX_S_MTP_TZString             "SYM_HEX_S_MTP_TZ"
#define SYM_HEX_S_MTP_RXString             "SYM_HEX_S_MTP_RX"
#define SYM_HEX_S_MTP_RYString             "SYM_HEX_S_MTP_RY"
#define SYM_HEX_S_MTP_RZString             "SYM_HEX_S_MTP_RZ"
#define SYM_HEX_S_AX_1String               "SYM_HEX_S_AX_1"
#define SYM_HEX_S_AX_2String               "SYM_HEX_S_AX_2"
#define SYM_HEX_S_AX_3String               "SYM_HEX_S_AX_3"
#define SYM_HEX_S_AX_4String               "SYM_HEX_S_AX_4"
#define SYM_HEX_S_AX_5String               "SYM_HEX_S_AX_5"
#define SYM_HEX_S_AX_6String               "SYM_HEX_S_AX_6"
#define SYM_HEX_S_POS_AX_1String           "SYM_HEX_S_POS_AX_1"
#define SYM_HEX_S_POS_AX_2String           "SYM_HEX_S_POS_AX_2"
#define SYM_HEX_S_POS_AX_3String           "SYM_HEX_S_POS_AX_3"
#define SYM_HEX_S_POS_AX_4String           "SYM_HEX_S_POS_AX_4"
#define SYM_HEX_S_POS_AX_5String           "SYM_HEX_S_POS_AX_5"
#define SYM_HEX_S_POS_AX_6String           "SYM_HEX_S_POS_AX_6"
#define SYM_HEX_S_DIO_1String              "SYM_HEX_S_DIO_1"
#define SYM_HEX_S_DIO_2String              "SYM_HEX_S_DIO_2"
#define SYM_HEX_S_DIO_3String              "SYM_HEX_S_DIO_3"
#define SYM_HEX_S_DIO_4String              "SYM_HEX_S_DIO_4"
#define SYM_HEX_S_DIO_5String              "SYM_HEX_S_DIO_5"
#define SYM_HEX_S_DIO_6String              "SYM_HEX_S_DIO_6"
#define SYM_HEX_S_DIO_7String              "SYM_HEX_S_DIO_7"
#define SYM_HEX_S_DIO_8String              "SYM_HEX_S_DIO_8"
#define SYM_HEX_S_AI_1String               "SYM_HEX_S_AI_1"
#define SYM_HEX_S_AI_2String               "SYM_HEX_S_AI_2"
#define SYM_HEX_S_AI_3String               "SYM_HEX_S_AI_3"
#define SYM_HEX_S_AI_4String               "SYM_HEX_S_AI_4"
#define SYM_HEX_S_AI_5String               "SYM_HEX_S_AI_5"
#define SYM_HEX_S_AI_6String               "SYM_HEX_S_AI_6"
#define SYM_HEX_S_AI_7String               "SYM_HEX_S_AI_7"
#define SYM_HEX_S_AI_8String               "SYM_HEX_S_AI_8"
#define SYM_HEX_S_CYCLEString              "SYM_HEX_S_CYCLE"
#define SYM_HEX_S_INDEXString              "SYM_HEX_S_INDEX"
#define SYM_HEX_S_ERR_NRString             "SYM_HEX_S_ERR_NR"

#define SYM_HEX_VT_DMDString               "SYM_HEX_VT_DMD"
#define SYM_HEX_VR_DMDString               "SYM_HEX_VR_DMD"
#define SYM_HEX_TA_DMDString               "SYM_HEX_TA_DMD"
#define SYM_HEX_TXU_DMDString              "SYM_HEX_TXU_DMD"
#define SYM_HEX_TYU_DMDString              "SYM_HEX_TYU_DMD"
#define SYM_HEX_TZU_DMDString              "SYM_HEX_TZU_DMD"
#define SYM_HEX_RXU_DMDString              "SYM_HEX_RXU_DMD"
#define SYM_HEX_RYU_DMDString              "SYM_HEX_RYU_DMD"
#define SYM_HEX_RZU_DMDString              "SYM_HEX_RZU_DMD"
#define SYM_HEX_TXO_DMDString              "SYM_HEX_TXO_DMD"
#define SYM_HEX_TYO_DMDString              "SYM_HEX_TYO_DMD"
#define SYM_HEX_TZO_DMDString              "SYM_HEX_TZO_DMD"
#define SYM_HEX_RXO_DMDString              "SYM_HEX_RXO_DMD"
#define SYM_HEX_RYO_DMDString              "SYM_HEX_RYO_DMD"
#define SYM_HEX_RZO_DMDString              "SYM_HEX_RZO_DMD"
#define SYM_HEX_MLIM_ENABLE_DMDString      "SYM_HEX_MLIM_ENABLE_DMD"
#define SYM_HEX_MLIM_TX_NEG_DMDString      "SYM_HEX_MLIM_TX_NEG_DMD"
#define SYM_HEX_MLIM_TY_NEG_DMDString      "SYM_HEX_MLIM_TY_NEG_DMD"
#define SYM_HEX_MLIM_TZ_NEG_DMDString      "SYM_HEX_MLIM_TZ_NEG_DMD"
#define SYM_HEX_MLIM_RX_NEG_DMDString      "SYM_HEX_MLIM_RX_NEG_DMD"
#define SYM_HEX_MLIM_RY_NEG_DMDString      "SYM_HEX_MLIM_RY_NEG_DMD"
#define SYM_HEX_MLIM_RZ_NEG_DMDString      "SYM_HEX_MLIM_RZ_NEG_DMD"
#define SYM_HEX_MLIM_TX_POS_DMDString      "SYM_HEX_MLIM_TX_POS_DMD"
#define SYM_HEX_MLIM_TY_POS_DMDString      "SYM_HEX_MLIM_TY_POS_DMD"
#define SYM_HEX_MLIM_TZ_POS_DMDString      "SYM_HEX_MLIM_TZ_POS_DMD"
#define SYM_HEX_MLIM_RX_POS_DMDString      "SYM_HEX_MLIM_RX_POS_DMD"
#define SYM_HEX_MLIM_RY_POS_DMDString      "SYM_HEX_MLIM_RY_POS_DMD"
#define SYM_HEX_MLIM_RZ_POS_DMDString      "SYM_HEX_MLIM_RZ_POS_DMD"
#define SYM_HEX_ULIM_ENABLE_DMDString      "SYM_HEX_ULIM_ENABLE_DMD"
#define SYM_HEX_ULIM_TX_NEG_DMDString      "SYM_HEX_ULIM_TX_NEG_DMD"
#define SYM_HEX_ULIM_TY_NEG_DMDString      "SYM_HEX_ULIM_TY_NEG_DMD"
#define SYM_HEX_ULIM_TZ_NEG_DMDString      "SYM_HEX_ULIM_TZ_NEG_DMD"
#define SYM_HEX_ULIM_RX_NEG_DMDString      "SYM_HEX_ULIM_RX_NEG_DMD"
#define SYM_HEX_ULIM_RY_NEG_DMDString      "SYM_HEX_ULIM_RY_NEG_DMD"
#define SYM_HEX_ULIM_RZ_NEG_DMDString      "SYM_HEX_ULIM_RZ_NEG_DMD"
#define SYM_HEX_ULIM_TX_POS_DMDString      "SYM_HEX_ULIM_TX_POS_DMD"
#define SYM_HEX_ULIM_TY_POS_DMDString      "SYM_HEX_ULIM_TY_POS_DMD"
#define SYM_HEX_ULIM_TZ_POS_DMDString      "SYM_HEX_ULIM_TZ_POS_DMD"
#define SYM_HEX_ULIM_RX_POS_DMDString      "SYM_HEX_ULIM_RX_POS_DMD"
#define SYM_HEX_ULIM_RY_POS_DMDString      "SYM_HEX_ULIM_RY_POS_DMD"
#define SYM_HEX_ULIM_RZ_POS_DMDString      "SYM_HEX_ULIM_RZ_POS_DMD"
#define SYM_HEX_AUTO_ACTIVATE_DMDString    "SYM_HEX_AUTO_ACTIVATE_DMD"
#define SYM_HEX_AUTO_DEACTIVATE_DMDString  "SYM_HEX_AUTO_DEACTIVATE_DMD"
#define SYM_HEX_AUTO_DELAY_DMDString       "SYM_HEX_AUTO_DELAY_DMD"
#define SYM_HEX_STALL_CURRENT_DMDString    "SYM_HEX_STALL_CURRENT_DMD"
#define SYM_HEX_BKL_AXIS_DMDString         "SYM_HEX_BKL_AXIS_DMD"
#define SYM_HEX_BKL_VALUE_DMDString        "SYM_HEX_BKL_VALUE_DMD"

#define SYM_HEX_SAFETY_SETString           "SYM_HEX_SAFETY_SET"
#define SYM_HEX_CHANNEL_SETString          "SYM_HEX_CHANNEL_SET"
#define SYM_HEX_SPEED_SETString            "SYM_HEX_SPEED_SET"
#define SYM_HEX_TA_SETString               "SYM_HEX_TA_SET"
#define SYM_HEX_CS_SETString               "SYM_HEX_CS_SET"
#define SYM_HEX_MLIM_SETString             "SYM_HEX_MLIM_SET"
#define SYM_HEX_MLIM_ENABLE_SETString      "SYM_HEX_MLIM_ENABLE_SET"
#define SYM_HEX_ULIM_SETString             "SYM_HEX_ULIM_SET"
#define SYM_HEX_ULIM_ENABLE_SETString      "SYM_HEX_ULIM_ENABLE_SET"
#define SYM_HEX_CONTROL_SETString          "SYM_HEX_CONTROL_SET"
#define SYM_HEX_STALL_SETString            "SYM_HEX_STALL_SET"
#define SYM_HEX_BKL_SETString              "SYM_HEX_BKL_SET"
#define SYM_HEX_KIN_SETString              "SYM_HEX_KIN_SET"
#define SYM_HEX_TUNING_SETString           "SYM_HEX_TUNING_SET"
#define SYM_HEX_CFG_POWER_SETString        "SYM_HEX_CFG_POWER_SET"

#define SYM_HEX_VTString                   "SYM_HEX_VT"
#define SYM_HEX_VT_MINString               "SYM_HEX_VT_MIN"
#define SYM_HEX_VT_MAXString               "SYM_HEX_VT_MAX"
#define SYM_HEX_VRString                   "SYM_HEX_VR"
#define SYM_HEX_VR_MINString               "SYM_HEX_VR_MIN"
#define SYM_HEX_VR_MAXString               "SYM_HEX_VR_MAX"
#define SYM_HEX_TAString                   "SYM_HEX_TA"
#define SYM_HEX_TA_MINString               "SYM_HEX_TA_MIN"
#define SYM_HEX_TA_MAXString               "SYM_HEX_TA_MAX"
#define SYM_HEX_TXUString                  "SYM_HEX_TXU"
#define SYM_HEX_TYUString                  "SYM_HEX_TYU"
#define SYM_HEX_TZUString                  "SYM_HEX_TZU"
#define SYM_HEX_RXUString                  "SYM_HEX_RXU"
#define SYM_HEX_RYUString                  "SYM_HEX_RYU"
#define SYM_HEX_RZUString                  "SYM_HEX_RZU"
#define SYM_HEX_TXOString                  "SYM_HEX_TXO"
#define SYM_HEX_TYOString                  "SYM_HEX_TYO"
#define SYM_HEX_TZOString                  "SYM_HEX_TZO"
#define SYM_HEX_RXOString                  "SYM_HEX_RXO"
#define SYM_HEX_RYOString                  "SYM_HEX_RYO"
#define SYM_HEX_RZOString                  "SYM_HEX_RZO"

#define SYM_HEX_CFG_GETString              "SYM_HEX_CFG_GET"
#define SYM_HEX_CFG_SETString              "SYM_HEX_CFG_SET"
#define SYM_HEX_CFG_DFLTString             "SYM_HEX_CFG_DEFAULT"
#define SYM_HEX_CFG_SAVEString             "SYM_HEX_CFG_SAVE"

#define SYM_HEX_MLIM_ENABLEString          "SYM_HEX_MLIM_ENABLE"
#define SYM_HEX_MLIM_TX_NEGString          "SYM_HEX_MLIM_TX_NEG"
#define SYM_HEX_MLIM_TY_NEGString          "SYM_HEX_MLIM_TY_NEG"
#define SYM_HEX_MLIM_TZ_NEGString          "SYM_HEX_MLIM_TZ_NEG"
#define SYM_HEX_MLIM_RX_NEGString          "SYM_HEX_MLIM_RX_NEG"
#define SYM_HEX_MLIM_RY_NEGString          "SYM_HEX_MLIM_RY_NEG"
#define SYM_HEX_MLIM_RZ_NEGString          "SYM_HEX_MLIM_RZ_NEG"
#define SYM_HEX_MLIM_TX_POSString          "SYM_HEX_MLIM_TX_POS"
#define SYM_HEX_MLIM_TY_POSString          "SYM_HEX_MLIM_TY_POS"
#define SYM_HEX_MLIM_TZ_POSString          "SYM_HEX_MLIM_TZ_POS"
#define SYM_HEX_MLIM_RX_POSString          "SYM_HEX_MLIM_RX_POS"
#define SYM_HEX_MLIM_RY_POSString          "SYM_HEX_MLIM_RY_POS"
#define SYM_HEX_MLIM_RZ_POSString          "SYM_HEX_MLIM_RZ_POS"

#define SYM_HEX_ULIM_ENABLEString          "SYM_HEX_ULIM_ENABLE"
#define SYM_HEX_ULIM_TX_NEGString          "SYM_HEX_ULIM_TX_NEG"
#define SYM_HEX_ULIM_TY_NEGString          "SYM_HEX_ULIM_TY_NEG"
#define SYM_HEX_ULIM_TZ_NEGString          "SYM_HEX_ULIM_TZ_NEG"
#define SYM_HEX_ULIM_RX_NEGString          "SYM_HEX_ULIM_RX_NEG"
#define SYM_HEX_ULIM_RY_NEGString          "SYM_HEX_ULIM_RY_NEG"
#define SYM_HEX_ULIM_RZ_NEGString          "SYM_HEX_ULIM_RZ_NEG"
#define SYM_HEX_ULIM_TX_POSString          "SYM_HEX_ULIM_TX_POS"
#define SYM_HEX_ULIM_TY_POSString          "SYM_HEX_ULIM_TY_POS"
#define SYM_HEX_ULIM_TZ_POSString          "SYM_HEX_ULIM_TZ_POS"
#define SYM_HEX_ULIM_RX_POSString          "SYM_HEX_ULIM_RX_POS"
#define SYM_HEX_ULIM_RY_POSString          "SYM_HEX_ULIM_RY_POS"
#define SYM_HEX_ULIM_RZ_POSString          "SYM_HEX_ULIM_RZ_POS"

#define SYM_HEX_AUTO_ACTIVATEString        "SYM_HEX_AUTO_ACTIVATE"
#define SYM_HEX_AUTO_DEACTIVATEString      "SYM_HEX_AUTO_DEACTIVATE"
#define SYM_HEX_AUTO_DELAYString           "SYM_HEX_AUTO_DELAY"

#define SYM_HEX_STALL_CURRENTString        "SYM_HEX_STALL_CURRENT"

#define SYM_HEX_SAFETY_INP_DMDString       "SYM_HEX_SAFETY_INPUT_DMD"
#define SYM_HEX_SAFETY_INPString           "SYM_HEX_SAFETY_INPUT"
#define SYM_HEX_CHANNEL_1_DMDString        "SYM_HEX_CHANNEL_1_DMD"
#define SYM_HEX_CHANNEL_1String            "SYM_HEX_CHANNEL_1"
#define SYM_HEX_CHANNEL_2_DMDString        "SYM_HEX_CHANNEL_2_DMD"
#define SYM_HEX_CHANNEL_2String            "SYM_HEX_CHANNEL_2"
#define SYM_HEX_CHANNEL_3_DMDString        "SYM_HEX_CHANNEL_3_DMD"
#define SYM_HEX_CHANNEL_3String            "SYM_HEX_CHANNEL_3"
#define SYM_HEX_CHANNEL_4_DMDString        "SYM_HEX_CHANNEL_4_DMD"
#define SYM_HEX_CHANNEL_4String            "SYM_HEX_CHANNEL_4"
#define SYM_HEX_CHANNEL_5_DMDString        "SYM_HEX_CHANNEL_5_DMD"
#define SYM_HEX_CHANNEL_5String            "SYM_HEX_CHANNEL_5"
#define SYM_HEX_CHANNEL_6_DMDString        "SYM_HEX_CHANNEL_6_DMD"
#define SYM_HEX_CHANNEL_6String            "SYM_HEX_CHANNEL_6"

#define SYM_HEX_HOME_AUTOString            "SYM_HEX_HOME_AUTO"
#define SYM_HEX_HOME_VIRTUALString         "SYM_HEX_HOME_VIRTUAL"
#define SYM_HEX_HOME_TYPEString            "SYM_HEX_HOME_TYPE"
#define SYM_HEX_KIN_MODE_DMDString         "SYM_HEX_KIN_MODE_DMD"
#define SYM_HEX_KIN_MODEString             "SYM_HEX_KIN_MODE"
#define SYM_HEX_KIN_AXESString             "SYM_HEX_KIN_AXES"
#define SYM_HEX_TUNING_IDX_DMDString       "SYM_HEX_TUNING_IDX_DMD"
#define SYM_HEX_TUNING_IDXString           "SYM_HEX_TUNING_IDX"
#define SYM_HEX_CFG_POWER_ENABLE_DMDString "SYM_HEX_CFG_POWER_ENABLE_DMD"
#define SYM_HEX_CFG_POWER_ENABLEString     "SYM_HEX_CFG_POWER_ENABLE"
#define SYM_HEX_CFG_POWER_AUTO_DMDString   "SYM_HEX_CFG_POWER_AUTO_DMD"
#define SYM_HEX_CFG_POWER_AUTOString       "SYM_HEX_CFG_POWER_AUTO"

#define SYM_HEX_ERROR_QTYString            "SYM_HEX_ERROR_QTY"
#define SYM_HEX_ERROR_CODEString           "SYM_HEX_ERROR_CODE"
#define SYM_HEX_ERROR_DESCString           "SYM_HEX_ERROR_DESC"
#define SYM_HEX_ERROR_TIMEString           "SYM_HEX_ERROR_TIME"
#define SYM_HEX_ERROR_GROUPString          "SYM_HEX_ERROR_GROUP"
#define SYM_HEX_ERROR_AXISString           "SYM_HEX_ERROR_AXIS"
#define SYM_HEX_ERROR_RETURNString         "SYM_HEX_ERROR_RETURN"
#define SYM_HEX_ERROR_DATA1String          "SYM_HEX_ERROR_DATA1"
#define SYM_HEX_ERROR_DATA2String          "SYM_HEX_ERROR_DATA2"

#define SYM_HEX_SPC_MOVE_DMDString         "SYM_HEX_SPC_MOVE_DMD"
#define SYM_HEX_SPC_MOVEString             "SYM_HEX_SPC_MOVE"

#define SYM_HEX_MOVE_TX_DMDString          "SYM_HEX_MOVE_TX_DMD"
#define SYM_HEX_MOVE_TY_DMDString          "SYM_HEX_MOVE_TY_DMD"
#define SYM_HEX_MOVE_TZ_DMDString          "SYM_HEX_MOVE_TZ_DMD"
#define SYM_HEX_MOVE_RX_DMDString          "SYM_HEX_MOVE_RX_DMD"
#define SYM_HEX_MOVE_RY_DMDString          "SYM_HEX_MOVE_RY_DMD"
#define SYM_HEX_MOVE_RZ_DMDString          "SYM_HEX_MOVE_RZ_DMD"
#define SYM_HEX_MOVE_TYPEString            "SYM_HEX_MOVE_TYPE"
#define SYM_HEX_MOVEString                 "SYM_HEX_MOVE"

#define SYM_HEX_MOVE_SEQString             "SYM_HEX_MOVE_SEQ"
#define SYM_HEX_VALID_PTP_MODEString       "SYM_HEX_VALID_PTP_MODE"
#define SYM_HEX_VALID_PTP_TYPEString       "SYM_HEX_VALID_PTP_TYPE"
#define SYM_HEX_VALID_PTP_TXString         "SYM_HEX_VALID_PTP_TX"
#define SYM_HEX_VALID_PTP_TYString         "SYM_HEX_VALID_PTP_TY"
#define SYM_HEX_VALID_PTP_TZString         "SYM_HEX_VALID_PTP_TZ"
#define SYM_HEX_VALID_PTP_RXString         "SYM_HEX_VALID_PTP_RX"
#define SYM_HEX_VALID_PTP_RYString         "SYM_HEX_VALID_PTP_RY"
#define SYM_HEX_VALID_PTP_RZString         "SYM_HEX_VALID_PTP_RZ"
#define SYM_HEX_VALID_PTPString            "SYM_HEX_VALID_PTP"
#define SYM_HEX_VALID_PTP_RESULTString     "SYM_HEX_VALID_PTP_RESULT"

#define SYM_HEX_POWER_POWERString          "SYM_HEX_POWER_POWER"
#define SYM_HEX_POWERString                "SYM_HEX_POWER"
#define SYM_HEX_MNTN_MODEString            "SYM_HEX_MAINTENANCE_MODE"
#define SYM_HEX_MNTN_AXISString            "SYM_HEX_MAINTENANCE_AXIS"
#define SYM_HEX_MNTNString                 "SYM_HEX_MAINTENANCE"
#define SYM_HEX_JOG_AXISString             "SYM_HEX_JOG_AXIS"
#define SYM_HEX_JOG_INCRString             "SYM_HEX_JOG_INCR"
#define SYM_HEX_JOGString                  "SYM_HEX_JOG"
#define SYM_HEX_REBOOTString               "SYM_HEX_REBOOT"
#define SYM_HEX_TERMINAL_SENDString        "SYM_HEX_TERMINAL_SEND"
#define SYM_HEX_TERMINAL_RECVString        "SYM_HEX_TERMINAL_RECV"

#define SYM_HEX_BKL_AXISString             "SYM_HEX_BKL_AXIS"
#define SYM_HEX_BKL_VALUEString            "SYM_HEX_BKL_VALUE"

#define SYM_HEX_USR_TO_OBJString           "SYM_HEX_USR_TO_OBJ"
#define SYM_HEX_OBJ_TO_USRString           "SYM_HEX_OBJ_TO_USR"

#define SYM_HEX_CMD_CODEString             "SYM_HEX_CMD_CODE"
#define SYM_HEX_CMD_DESCString             "SYM_HEX_CMD_DESC"

#define SYM_HEX_VERS_CTRL_IDString         "SYM_HEX_VERS_CTRL_ID"
#define SYM_HEX_VERS_CTRL_VERString        "SYM_HEX_VERS_CTRL_VER"
#define SYM_HEX_VERS_API_VERString         "SYM_HEX_VERS_API_VER"
#define SYM_HEX_VERS_SYS_IDString          "SYM_HEX_VERS_SYS_ID"
#define SYM_HEX_VERS_SYS_NUMString         "SYM_HEX_VERS_SYS_NUM"
#define SYM_HEX_VERS_SYS_CFGString         "SYM_HEX_VERS_SYS_CFG"


class epicsShareClass SymetrieAxis : public pmacAxis
{
public:
    /* These are the methods we override from the base class */
    SymetrieAxis(class SymetrieHexapod *pC, int axis);
    void report(FILE *fp, int level);
    asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
    asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
    asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
    asynStatus stop(double acceleration);
    asynStatus poll(bool *moving);
    asynStatus setPosition(double position);
    asynStatus setClosedLoop(bool closedLoop);

private:
    asynStatus setVelocityAndAcceleration(double velocity, double acceleration);
    SymetrieHexapod *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                     *   Abbreviated because it is used very frequently */
friend class SymetrieHexapod;
};



class epicsShareClass SymetrieHexapod
        : public pmacController {

public:
    SymetrieHexapod(const char *portName, const char *lowLevelPortName, int lowLevelPortAddress,
                    double movingPollPeriod, double idlePollPeriod);
    virtual ~SymetrieHexapod();

    void configTask();
    void readStatus();
    void readAllConfig();

    int applyConfigSafety();
    int applyConfigChannel();
    int applyConfigSpeed();
    int applyConfigTa();
    int applyConfigCS();
    int applyConfigMachineLimit();
    int applyConfigMachineLimitEnable();
    int applyConfigUserLimit();
    int applyConfigUserLimitEnable();
    int applyConfigControl();
    int applyConfigStallCurrent();
    int applyConfigBacklash();
    int applyConfigKinematic();
    int applyConfigTuning();
    int applyConfigPower();

    void applyConfig();
    void readErrors();
    int getConfigItem(const std::string& cmd, int num_params, double *params);
    int getConfigItem(const std::string& cmd, int num_params, double *params, const std::string& extra);
    int getConfigItem(const std::string& cmd, int num_params, int *params);
    int getConfigItem(const std::string& cmd, int num_params, int *params, const std::string& extra);
    int setConfigItem(const std::string& cmd, int num_params, double *params);
    int setConfigItem(const std::string& cmd, int num_params, int *params);
    void getVersion();
    int waitForCommandComplete();
    void recordCmdResult(int code);
    void copyUserToObject();
    void copyObjectToUser();

    // Asyn driver methods that we override
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t nChars, size_t *nActual);
    
    // asynMotorController methods we override
    void report(FILE *fp, int level);
    asynStatus poll();
    SymetrieAxis* getAxis(asynUser *pasynUser);
    SymetrieAxis* getAxis(int axisNo);

    friend class SymetrieAxis;


protected:
    int SYM_HEX_FirstParam_;
#define FIRST_SYM_HEX_PARAM SYM_HEX_FirstParam_
    int SYM_HEX_STOP_;
    int SYM_HEX_CONTROLON_;
    int SYM_HEX_CONTROLOFF_;
    int SYM_HEX_HOME_;
    int SYM_HEX_HOMEVIRTUAL_;
    int SYM_HEX_CLEARERROR_;
    int SYM_HEX_STATUS_READ_;
    int SYM_HEX_S_HEXA_;
    int SYM_HEX_S_ACTION_;
    int SYM_HEX_S_UTO_TX_;
    int SYM_HEX_S_UTO_TY_;
    int SYM_HEX_S_UTO_TZ_;
    int SYM_HEX_S_UTO_RX_;
    int SYM_HEX_S_UTO_RY_;
    int SYM_HEX_S_UTO_RZ_;
    int SYM_HEX_S_MTP_TX_;
    int SYM_HEX_S_MTP_TY_;
    int SYM_HEX_S_MTP_TZ_;
    int SYM_HEX_S_MTP_RX_;
    int SYM_HEX_S_MTP_RY_;
    int SYM_HEX_S_MTP_RZ_;
    int SYM_HEX_S_AX_1_;
    int SYM_HEX_S_AX_2_;
    int SYM_HEX_S_AX_3_;
    int SYM_HEX_S_AX_4_;
    int SYM_HEX_S_AX_5_;
    int SYM_HEX_S_AX_6_;
    int SYM_HEX_S_POS_AX_1_;
    int SYM_HEX_S_POS_AX_2_;
    int SYM_HEX_S_POS_AX_3_;
    int SYM_HEX_S_POS_AX_4_;
    int SYM_HEX_S_POS_AX_5_;
    int SYM_HEX_S_POS_AX_6_;
    int SYM_HEX_S_DIO_1_;
    int SYM_HEX_S_DIO_2_;
    int SYM_HEX_S_DIO_3_;
    int SYM_HEX_S_DIO_4_;
    int SYM_HEX_S_DIO_5_;
    int SYM_HEX_S_DIO_6_;
    int SYM_HEX_S_DIO_7_;
    int SYM_HEX_S_DIO_8_;
    int SYM_HEX_S_AI_1_;
    int SYM_HEX_S_AI_2_;
    int SYM_HEX_S_AI_3_;
    int SYM_HEX_S_AI_4_;
    int SYM_HEX_S_AI_5_;
    int SYM_HEX_S_AI_6_;
    int SYM_HEX_S_AI_7_;
    int SYM_HEX_S_AI_8_;
    int SYM_HEX_S_CYCLE_;
    int SYM_HEX_S_INDEX_;
    int SYM_HEX_S_ERR_NR_;
    int SYM_HEX_VT_DMD_;
    int SYM_HEX_VR_DMD_;
    int SYM_HEX_TA_DMD_;
    int SYM_HEX_TXU_DMD_;
    int SYM_HEX_TYU_DMD_;
    int SYM_HEX_TZU_DMD_;
    int SYM_HEX_RXU_DMD_;
    int SYM_HEX_RYU_DMD_;
    int SYM_HEX_RZU_DMD_;
    int SYM_HEX_TXO_DMD_;
    int SYM_HEX_TYO_DMD_;
    int SYM_HEX_TZO_DMD_;
    int SYM_HEX_RXO_DMD_;
    int SYM_HEX_RYO_DMD_;
    int SYM_HEX_RZO_DMD_;
    int SYM_HEX_MLIM_ENABLE_DMD_;
    int SYM_HEX_MLIM_TX_NEG_DMD_;
    int SYM_HEX_MLIM_TY_NEG_DMD_;
    int SYM_HEX_MLIM_TZ_NEG_DMD_;
    int SYM_HEX_MLIM_RX_NEG_DMD_;
    int SYM_HEX_MLIM_RY_NEG_DMD_;
    int SYM_HEX_MLIM_RZ_NEG_DMD_;
    int SYM_HEX_MLIM_TX_POS_DMD_;
    int SYM_HEX_MLIM_TY_POS_DMD_;
    int SYM_HEX_MLIM_TZ_POS_DMD_;
    int SYM_HEX_MLIM_RX_POS_DMD_;
    int SYM_HEX_MLIM_RY_POS_DMD_;
    int SYM_HEX_MLIM_RZ_POS_DMD_;
    int SYM_HEX_ULIM_ENABLE_DMD_;
    int SYM_HEX_ULIM_TX_NEG_DMD_;
    int SYM_HEX_ULIM_TY_NEG_DMD_;
    int SYM_HEX_ULIM_TZ_NEG_DMD_;
    int SYM_HEX_ULIM_RX_NEG_DMD_;
    int SYM_HEX_ULIM_RY_NEG_DMD_;
    int SYM_HEX_ULIM_RZ_NEG_DMD_;
    int SYM_HEX_ULIM_TX_POS_DMD_;
    int SYM_HEX_ULIM_TY_POS_DMD_;
    int SYM_HEX_ULIM_TZ_POS_DMD_;
    int SYM_HEX_ULIM_RX_POS_DMD_;
    int SYM_HEX_ULIM_RY_POS_DMD_;
    int SYM_HEX_ULIM_RZ_POS_DMD_;
    int SYM_HEX_AUTO_ACTIVATE_DMD_;
    int SYM_HEX_AUTO_DEACTIVATE_DMD_;
    int SYM_HEX_AUTO_DELAY_DMD_;
    int SYM_HEX_STALL_CURRENT_DMD_;
    int SYM_HEX_BKL_AXIS_DMD_;
    int SYM_HEX_BKL_VALUE_DMD_;
    int SYM_HEX_SAFETY_SET_;
    int SYM_HEX_CHANNEL_SET_;
    int SYM_HEX_SPEED_SET_;
    int SYM_HEX_TA_SET_;
    int SYM_HEX_CS_SET_;
    int SYM_HEX_MLIM_SET_;
    int SYM_HEX_MLIM_ENABLE_SET_;
    int SYM_HEX_ULIM_SET_;
    int SYM_HEX_ULIM_ENABLE_SET_;
    int SYM_HEX_CONTROL_SET_;
    int SYM_HEX_STALL_SET_;
    int SYM_HEX_BKL_SET_;
    int SYM_HEX_KIN_SET_;
    int SYM_HEX_TUNING_SET_;
    int SYM_HEX_CFG_POWER_SET_;
    int SYM_HEX_VT_;
    int SYM_HEX_VT_MIN_;
    int SYM_HEX_VT_MAX_;
    int SYM_HEX_VR_;
    int SYM_HEX_VR_MIN_;
    int SYM_HEX_VR_MAX_;
    int SYM_HEX_TA_;
    int SYM_HEX_TA_MIN_;
    int SYM_HEX_TA_MAX_;
    int SYM_HEX_TXU_;
    int SYM_HEX_TYU_;
    int SYM_HEX_TZU_;
    int SYM_HEX_RXU_;
    int SYM_HEX_RYU_;
    int SYM_HEX_RZU_;
    int SYM_HEX_TXO_;
    int SYM_HEX_TYO_;
    int SYM_HEX_TZO_;
    int SYM_HEX_RXO_;
    int SYM_HEX_RYO_;
    int SYM_HEX_RZO_;
    int SYM_HEX_MLIM_ENABLE_;
    int SYM_HEX_MLIM_TX_NEG_;
    int SYM_HEX_MLIM_TY_NEG_;
    int SYM_HEX_MLIM_TZ_NEG_;
    int SYM_HEX_MLIM_RX_NEG_;
    int SYM_HEX_MLIM_RY_NEG_;
    int SYM_HEX_MLIM_RZ_NEG_;
    int SYM_HEX_MLIM_TX_POS_;
    int SYM_HEX_MLIM_TY_POS_;
    int SYM_HEX_MLIM_TZ_POS_;
    int SYM_HEX_MLIM_RX_POS_;
    int SYM_HEX_MLIM_RY_POS_;
    int SYM_HEX_MLIM_RZ_POS_;
    int SYM_HEX_ULIM_ENABLE_;
    int SYM_HEX_ULIM_TX_NEG_;
    int SYM_HEX_ULIM_TY_NEG_;
    int SYM_HEX_ULIM_TZ_NEG_;
    int SYM_HEX_ULIM_RX_NEG_;
    int SYM_HEX_ULIM_RY_NEG_;
    int SYM_HEX_ULIM_RZ_NEG_;
    int SYM_HEX_ULIM_TX_POS_;
    int SYM_HEX_ULIM_TY_POS_;
    int SYM_HEX_ULIM_TZ_POS_;
    int SYM_HEX_ULIM_RX_POS_;
    int SYM_HEX_ULIM_RY_POS_;
    int SYM_HEX_ULIM_RZ_POS_;
    int SYM_HEX_AUTO_ACTIVATE_;
    int SYM_HEX_AUTO_DEACTIVATE_;
    int SYM_HEX_AUTO_DELAY_;
    int SYM_HEX_STALL_CURRENT_;
    int SYM_HEX_SAFETY_INPUT_DMD_;
    int SYM_HEX_SAFETY_INPUT_;
    int SYM_HEX_CHANNEL_1_DMD_;
    int SYM_HEX_CHANNEL_1_;
    int SYM_HEX_CHANNEL_2_DMD_;
    int SYM_HEX_CHANNEL_2_;
    int SYM_HEX_CHANNEL_3_DMD_;
    int SYM_HEX_CHANNEL_3_;
    int SYM_HEX_CHANNEL_4_DMD_;
    int SYM_HEX_CHANNEL_4_;
    int SYM_HEX_CHANNEL_5_DMD_;
    int SYM_HEX_CHANNEL_5_;
    int SYM_HEX_CHANNEL_6_DMD_;
    int SYM_HEX_CHANNEL_6_;
    int SYM_HEX_HOME_AUTO_;
    int SYM_HEX_HOME_VIRTUAL_;
    int SYM_HEX_HOME_TYPE_;
    int SYM_HEX_KIN_MODE_DMD_;
    int SYM_HEX_KIN_MODE_;
    int SYM_HEX_KIN_AXES_;
    int SYM_HEX_TUNING_IDX_DMD_;
    int SYM_HEX_TUNING_IDX_;
    int SYM_HEX_CFG_POWER_ENABLE_DMD_;
    int SYM_HEX_CFG_POWER_ENABLE_;
    int SYM_HEX_CFG_POWER_AUTO_DMD_;
    int SYM_HEX_CFG_POWER_AUTO_;
    int SYM_HEX_ERROR_QTY_;
    int SYM_HEX_ERROR_CODE_;
    int SYM_HEX_ERROR_DESC_;
    int SYM_HEX_ERROR_TIME_;
    int SYM_HEX_ERROR_GROUP_;
    int SYM_HEX_ERROR_AXIS_;
    int SYM_HEX_ERROR_RETURN_;
    int SYM_HEX_ERROR_DATA1_;
    int SYM_HEX_ERROR_DATA2_;
    int SYM_HEX_SPC_MOVE_DMD_;
    int SYM_HEX_SPC_MOVE_;
    int SYM_HEX_MOVE_TX_DMD_;
    int SYM_HEX_MOVE_TY_DMD_;
    int SYM_HEX_MOVE_TZ_DMD_;
    int SYM_HEX_MOVE_RX_DMD_;
    int SYM_HEX_MOVE_RY_DMD_;
    int SYM_HEX_MOVE_RZ_DMD_;
    int SYM_HEX_MOVE_TYPE_;
    int SYM_HEX_MOVE_;
    int SYM_HEX_MOVE_SEQ_;
    int SYM_HEX_VALID_PTP_MODE_;
    int SYM_HEX_VALID_PTP_TYPE_;
    int SYM_HEX_VALID_PTP_TX_;
    int SYM_HEX_VALID_PTP_TY_;
    int SYM_HEX_VALID_PTP_TZ_;
    int SYM_HEX_VALID_PTP_RX_;
    int SYM_HEX_VALID_PTP_RY_;
    int SYM_HEX_VALID_PTP_RZ_;
    int SYM_HEX_VALID_PTP_;
    int SYM_HEX_VALID_PTP_RESULT_;
    int SYM_HEX_POWER_POWER_;
    int SYM_HEX_POWER_;
    int SYM_HEX_MAINTENANCE_MODE_;
    int SYM_HEX_MAINTENANCE_AXIS_;
    int SYM_HEX_MAINTENANCE_;
    int SYM_HEX_JOG_AXIS_;
    int SYM_HEX_JOG_INCR_;
    int SYM_HEX_JOG_;
    int SYM_HEX_REBOOT_;
    int SYM_HEX_TERMINAL_SEND_;
    int SYM_HEX_TERMINAL_RECV_;
    int SYM_HEX_BKL_AXIS_;
    int SYM_HEX_BKL_VALUE_;
    int SYM_HEX_USR_TO_OBJ_;
    int SYM_HEX_OBJ_TO_USR_;
    int SYM_HEX_CFG_GET_;
    int SYM_HEX_CFG_SET_;
    int SYM_HEX_CFG_DFLT_;
    int SYM_HEX_CFG_SAVE_;
    int SYM_HEX_CMD_CODE_;
    int SYM_HEX_CMD_DESC_;
    int SYM_HEX_VERS_CTRL_ID_;
    int SYM_HEX_VERS_CTRL_VER_;
    int SYM_HEX_VERS_API_VER_;
    int SYM_HEX_VERS_SYS_ID_;
    int SYM_HEX_VERS_SYS_NUM_;
    int SYM_HEX_VERS_SYS_CFG_;
    int SYM_HEX_LastParam_;
#define LAST_SYM_HEX_PARAM SYM_HEX_LastParam_

    int current_error_;
    int no_of_errors_;
    double movingPollPeriod_;
    double idlePollPeriod_;
};

#endif /* SymetrieHexapod_H */

