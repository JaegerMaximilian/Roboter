/*
 * File: WMRctrlVelPosOdo0_ctrlVel.h
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

#ifndef RTW_HEADER_WMRctrlVelPosOdo0_ctrlVel_h_
#define RTW_HEADER_WMRctrlVelPosOdo0_ctrlVel_h_
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

/* Block signals for system '<S1>/ctrlVel' */
typedef struct {
  real_T gain2;                        /* '<S10>/gain2' */
  real_T gain1;                        /* '<S10>/gain1' */
  real_T Reshape[4];                   /* '<S10>/Reshape' */
  real_T Product6;                     /* '<S10>/Product6' */
  real_T Product5[4];                  /* '<S10>/Product5' */
  real_T Product[4];                   /* '<S3>/Product' */
  real_T ZeroOrderHold[2];             /* '<S3>/Zero-Order Hold' */
  real_T ZeroOrderHold1[2];            /* '<S3>/Zero-Order Hold1' */
  real_T e[2];                         /* '<S3>/Sum' */
  real_T P_kP[2];                      /* '<S3>/P_kP' */
  real_T ctrlVel_INT[2];               /* '<S3>/ctrlVel_INT' */
  real_T gain3[2];                     /* '<S3>/gain3' */
  real_T Sum1[2];                      /* '<S3>/Sum1' */
  real_T uvuw[2];                      /* '<S3>/Product1' */
  real_T Product2[2];                  /* '<S3>/Product2' */
  real_T ulur[2];                      /* '<S3>/Saturation' */
  real_T e_saturation[2];              /* '<S3>/Add' */
  real_T Product3[2];                  /* '<S3>/Product3' */
  real_T Product4[2];                  /* '<S3>/Product4' */
  real_T P_kAntiWind[2];               /* '<S3>/P_kAntiWind' */
  real_T Sum2[2];                      /* '<S3>/Sum2' */
  real_T P_kI[2];                      /* '<S3>/P_kI' */
} rtB_ctrlVel_WMRctrlVelPosOdo0;

/* Block states (auto storage) for system '<S1>/ctrlVel' */
typedef struct {
  real_T ctrlVel_INT_DSTATE[2];        /* '<S3>/ctrlVel_INT' */
  real_T Product_DWORK1[4];            /* '<S3>/Product' */
  real_T Product_DWORK3[4];            /* '<S3>/Product' */
  real_T Product_DWORK4[4];            /* '<S3>/Product' */
  int32_T Product_DWORK2[2];           /* '<S3>/Product' */
} rtDW_ctrlVel_WMRctrlVelPosOdo0;

extern void WMRctrlVelPosOdo0_ctrlVel_Start(rtDW_ctrlVel_WMRctrlVelPosOdo0
  *localDW);
extern void WMRctrlVelPosOdo0_ctrlVel(const real_T rtu_vdwd[2], const real_T
  rtu_vw[2], rtB_ctrlVel_WMRctrlVelPosOdo0 *localB,
  rtDW_ctrlVel_WMRctrlVelPosOdo0 *localDW);

#endif                                 /* RTW_HEADER_WMRctrlVelPosOdo0_ctrlVel_h_ */

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
