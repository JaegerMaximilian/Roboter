/*
 * File: WMRctrlVelPosOdo0_types.h
 *
 * Real-Time Workshop code generated for Simulink model WMRctrlVelPosOdo0.
 *
 * Model version                        : 1.344
 * Real-Time Workshop file version      : 7.5  (R2010a)  25-Jan-2010
 * Real-Time Workshop file generated on : Tue Feb 01 22:58:37 2011
 * TLC version                          : 7.5 (Jan 19 2010)
 * C/C++ source code generated on       : Tue Feb 01 22:58:38 2011
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Atmel->AVR
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_WMRctrlVelPosOdo0_types_h_
#define RTW_HEADER_WMRctrlVelPosOdo0_types_h_
#include "rtwtypes.h"
#ifndef _DEFINED_TYPEDEF_FOR_BUSinWMRctrl_
#define _DEFINED_TYPEDEF_FOR_BUSinWMRctrl_

typedef struct {
  real_T P0_XYdXdYThdTh_desired[6];
  int32_T encLR[2];
  boolean_T swReset;
  real_T x0y0theta0[3];
} BUSinWMRctrl;

#endif

#ifndef _DEFINED_TYPEDEF_FOR_BUSouWMRctrl_
#define _DEFINED_TYPEDEF_FOR_BUSouWMRctrl_

typedef struct {
  boolean_T errorFlag;
  uint8_T errorNr;
  real_T uldurd[2];
  real_T XYtheta[3];
  real_T vomega[2];
  real_T XRYRtheta[3];
  real_T ou_ctrlVelBUS[4];
  real_T ou_ctrlPosBUS[7];
} BUSouWMRctrl;

#endif

/* Type definition for custom storage class: Struct */
typedef struct PCtrlOdo_tag {
  real_T PCtrl_DynKmDRa;
  real_T PCtrl_DynL;
  real_T PCtrl_DynN;
  real_T PCtrl_DynR;
  real_T PCtrl_PosKAntiWind;
  real_T PCtrl_PosKI;
  real_T PCtrl_PosKp;
  real_T PCtrl_PosLr;
  real_T PCtrl_PosUsat[2];
  real_T PCtrl_Ts1;
  real_T PCtrl_VelKAntiWind;
  real_T PCtrl_VelKI[2];
  real_T PCtrl_VelKp[2];
  real_T PCtrl_VelUsat;
  real_T PCtrl_VelnormPlantGains[2];
  real_T PEncWhl_IperRev;
  real_T PEncWhl_l;
  real_T PEncWhl_rl[2];
  int32_T PEncWhl_maxCntr;
} PCtrlOdo_type;

#endif                                 /* RTW_HEADER_WMRctrlVelPosOdo0_types_h_ */

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
