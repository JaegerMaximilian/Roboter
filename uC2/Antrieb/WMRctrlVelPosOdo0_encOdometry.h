/*
 * File: WMRctrlVelPosOdo0_encOdometry.h
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

#ifndef RTW_HEADER_WMRctrlVelPosOdo0_encOdometry_h_
#define RTW_HEADER_WMRctrlVelPosOdo0_encOdometry_h_
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

/* Block signals for system '<S1>/encOdometry' */
typedef struct {
  real_T DataTypeConversion[2];        /* '<S12>/Data Type Conversion' */
  real_T Gain1[2];                     /* '<S4>/Gain1' */
  real_T inc2m[2];                     /* '<S4>/Divide' */
  real_T inc2m_j[2];                   /* '<S12>/inc2m' */
  real_T Fcn1;                         /* '<S12>/Fcn1' */
  real_T vTs;                          /* '<S12>/gain' */
  real_T v;                            /* '<S12>/Divide1' */
  real_T Fcn;                          /* '<S12>/Fcn' */
  real_T omegaTs;                      /* '<S12>/Divide' */
  real_T theta;                        /* '<S12>/int_theta' */
  real_T Fcn2;                         /* '<S12>/Fcn2' */
  real_T Divide3;                      /* '<S12>/Divide3' */
  real_T int_X;                        /* '<S12>/int_X' */
  real_T x;                            /* '<S12>/gain3' */
  real_T Fcn3;                         /* '<S12>/Fcn3' */
  real_T Divide4;                      /* '<S12>/Divide4' */
  real_T int_Y;                        /* '<S12>/int_Y' */
  real_T y;                            /* '<S12>/gain4' */
  real_T TrigonometricFunction1;       /* '<S4>/Trigonometric Function1' */
  real_T TrigonometricFunction;        /* '<S4>/Trigonometric Function' */
  real_T Gain3[2];                     /* '<S4>/Gain3' */
  real_T Add2[2];                      /* '<S4>/Add2' */
  real_T Divide2;                      /* '<S12>/Divide2' */
  int32_T Uk1[2];                      /* '<S13>/UD' */
  int32_T Diff[2];                     /* '<S13>/Diff' */
  int32_T Constant;                    /* '<S4>/Constant' */
  int32_T Sign[2];                     /* '<S11>/Sign' */
  int32_T Abs[2];                      /* '<S11>/Abs' */
  int32_T Gain;                        /* '<S11>/Gain' */
  int32_T Product1[2];                 /* '<S11>/Product1' */
  int32_T Switch[2];                   /* '<S11>/Switch' */
  int32_T Gain1_p[2];                  /* '<S11>/Gain1' */
  int32_T Switch1[2];                  /* '<S11>/Switch1' */
  int32_T Add1[2];                     /* '<S11>/Add1' */
  boolean_T RelationalOperator[2];     /* '<S11>/Relational Operator' */
} rtB_encOdometry_WMRctrlVelPosOd;

/* Block states (auto storage) for system '<S1>/encOdometry' */
typedef struct {
  real_T theta;                        /* '<S12>/int_theta' */
  real_T x;                            /* '<S12>/int_X' */
  real_T y;                            /* '<S12>/int_Y' */
  int32_T UD_DSTATE[2];                /* '<S13>/UD' */
  int8_T int_theta_PrevResetState;     /* '<S12>/int_theta' */
  int8_T int_X_PrevResetState;         /* '<S12>/int_X' */
  int8_T int_Y_PrevResetState;         /* '<S12>/int_Y' */
  uint8_T int_theta_IC_LOADING;        /* '<S12>/int_theta' */
  uint8_T int_X_IC_LOADING;            /* '<S12>/int_X' */
  uint8_T int_Y_IC_LOADING;            /* '<S12>/int_Y' */
} rtDW_encOdometry_WMRctrlVelPosO;

extern void WMRctrlVelPosO_encOdometry_Init(rtDW_encOdometry_WMRctrlVelPosO
  *localDW);
extern void WMRctrlVelPosOdo0_encOdometry(const int32_T rtu_in_encLR[2],
  boolean_T rtu_in_swReset, real_T rtu_in_x0y0theta0, real_T rtu_in_x0y0theta0_c,
  real_T rtu_in_x0y0theta0_p, rtB_encOdometry_WMRctrlVelPosOd *localB,
  rtDW_encOdometry_WMRctrlVelPosO *localDW);

#endif                                 /* RTW_HEADER_WMRctrlVelPosOdo0_encOdometry_h_ */

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
