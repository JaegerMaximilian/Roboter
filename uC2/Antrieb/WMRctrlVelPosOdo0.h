/*
 * File: WMRctrlVelPosOdo0.h
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

#ifndef RTW_HEADER_WMRctrlVelPosOdo0_h_
#define RTW_HEADER_WMRctrlVelPosOdo0_h_
#ifndef WMRctrlVelPosOdo0_COMMON_INCLUDES_
# define WMRctrlVelPosOdo0_COMMON_INCLUDES_
#include <float.h>
#include <math.h>
#include <string.h>
#include "rtwtypes.h"
#include "rt_defines.h"
#include "rt_SATURATE.h"
#include "rt_SGN.h"
#include "rt_atan2.h"
#include "rt_matrixlib.h"
#include "rt_mod.h"
#endif                                 /* WMRctrlVelPosOdo0_COMMON_INCLUDES_ */

#include "WMRctrlVelPosOdo0_types.h"

/* Child system includes */
#include "WMRctrlVelPosOdo0_ctrlPos.h"
#include "WMRctrlVelPosOdo0_ctrlVel.h"
#include "WMRctrlVelPosOdo0_encOdometry.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((void*) 0)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((void) 0)
#endif

/* Block signals (auto storage) */
typedef struct {
  real_T P0_XYdXdYThdTh_desired[6];    /* '<S1>/Bus Selector2' */
  real_T x0y0theta0[3];                /* '<S1>/Bus Selector1' */
  real_T TmpSignalConversionAtBusCreator[3];
  real_T TmpSignalConversionAtctrlPosInp[3];
  real_T TmpSignalConversionAtBusCreat_h[2];
  real_T TmpSignalConversionAtctrlVelInp[2];
  real_T TmpSignalConversionAtBusCreat_p[3];
  real_T TmpSignalConversionAtBusCreat_e[4];
  real_T TmpSignalConversionAtBusCreat_a[7];
  int32_T encLR[2];                    /* '<S1>/Bus Selector1' */
  boolean_T swReset;                   /* '<S1>/Bus Selector1' */
  rtB_encOdometry_WMRctrlVelPosOd encOdometry;/* '<S1>/encOdometry' */
  rtB_ctrlVel_WMRctrlVelPosOdo0 ctrlVel;/* '<S1>/ctrlVel' */
  rtB_ctrlPos_WMRctrlVelPosOdo0 ctrlPos;/* '<S1>/ctrlPos' */
} BlockIO_WMRctrlVelPosOdo0;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  rtDW_encOdometry_WMRctrlVelPosO encOdometry;/* '<S1>/encOdometry' */
  rtDW_ctrlVel_WMRctrlVelPosOdo0 ctrlVel;/* '<S1>/ctrlVel' */
  rtDW_ctrlPos_WMRctrlVelPosOdo0 ctrlPos;/* '<S1>/ctrlPos' */
} D_Work_WMRctrlVelPosOdo0;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  BUSinWMRctrl in_WMRctrlBUS;          /* '<Root>/in_WMRctrlBUS' */
} ExternalInputs_WMRctrlVelPosOdo;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  BUSouWMRctrl ou_WMRctrlBUS;          /* '<Root>/ou_WMRctrlBUS' */
} ExternalOutputs_WMRctrlVelPosOd;

/* External data declarations for dependent source files */
extern BUSinWMRctrl WMRctrlVelPosOdo0_rtZBUSinWMRctrl;/* BUSinWMRctrl ground */
extern BUSouWMRctrl WMRctrlVelPosOdo0_rtZBUSouWMRctrl;/* BUSouWMRctrl ground */
extern BUSinWMRctrl WMRctrlVelPosOdo0_rtZBUSinWMRctrl;/* BUSinWMRctrl ground */
extern BUSouWMRctrl WMRctrlVelPosOdo0_rtZBUSouWMRctrl;/* BUSouWMRctrl ground */

/* Model entry point functions */
extern void WMRctrlVelPosOdo0_initialize(BlockIO_WMRctrlVelPosOdo0
  *WMRctrlVelPosOdo0_B, D_Work_WMRctrlVelPosOdo0 *WMRctrlVelPosOdo0_DWork,
  ExternalInputs_WMRctrlVelPosOdo *WMRctrlVelPosOdo0_U,
  ExternalOutputs_WMRctrlVelPosOd *WMRctrlVelPosOdo0_Y);
extern void WMRctrlVelPosOdo0_step(BlockIO_WMRctrlVelPosOdo0
  *WMRctrlVelPosOdo0_B, D_Work_WMRctrlVelPosOdo0 *WMRctrlVelPosOdo0_DWork,
  ExternalInputs_WMRctrlVelPosOdo *WMRctrlVelPosOdo0_U,
  ExternalOutputs_WMRctrlVelPosOd *WMRctrlVelPosOdo0_Y);
extern void WMRctrlVelPosOdo0_terminate(void);

/* Declaration for custom storage class: Struct */
extern PCtrlOdo_type PCtrlOdo;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('srcWMRctrlVelPosOdo/WMRctrlVelPosOdo')    - opens subsystem srcWMRctrlVelPosOdo/WMRctrlVelPosOdo
 * hilite_system('srcWMRctrlVelPosOdo/WMRctrlVelPosOdo/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : srcWMRctrlVelPosOdo
 * '<S1>'   : srcWMRctrlVelPosOdo/WMRctrlVelPosOdo
 * '<S2>'   : srcWMRctrlVelPosOdo/WMRctrlVelPosOdo/ctrlPos
 * '<S3>'   : srcWMRctrlVelPosOdo/WMRctrlVelPosOdo/ctrlVel
 * '<S4>'   : srcWMRctrlVelPosOdo/WMRctrlVelPosOdo/encOdometry
 * '<S5>'   : srcWMRctrlVelPosOdo/WMRctrlVelPosOdo/ctrlPos/P0toPR
 * '<S6>'   : srcWMRctrlVelPosOdo/WMRctrlVelPosOdo/ctrlPos/PI_antiWind
 * '<S7>'   : srcWMRctrlVelPosOdo/WMRctrlVelPosOdo/ctrlPos/createPRtraj
 * '<S8>'   : srcWMRctrlVelPosOdo/WMRctrlVelPosOdo/ctrlPos/createPRtraj/P0toPR
 * '<S9>'   : srcWMRctrlVelPosOdo/WMRctrlVelPosOdo/ctrlPos/createPRtraj/map2+-pi
 * '<S10>'  : srcWMRctrlVelPosOdo/WMRctrlVelPosOdo/ctrlVel/PDynB
 * '<S11>'  : srcWMRctrlVelPosOdo/WMRctrlVelPosOdo/encOdometry/deltaIncrement
 * '<S12>'  : srcWMRctrlVelPosOdo/WMRctrlVelPosOdo/encOdometry/odometry
 * '<S13>'  : srcWMRctrlVelPosOdo/WMRctrlVelPosOdo/encOdometry/deltaIncrement/Difference
 */
#endif                                 /* RTW_HEADER_WMRctrlVelPosOdo0_h_ */

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
