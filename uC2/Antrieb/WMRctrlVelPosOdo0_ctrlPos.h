/*
 * File: WMRctrlVelPosOdo0_ctrlPos.h
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

#ifndef RTW_HEADER_WMRctrlVelPosOdo0_ctrlPos_h_
#define RTW_HEADER_WMRctrlVelPosOdo0_ctrlPos_h_
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

/* Block signals for system '<S1>/ctrlPos' */
typedef struct {
  real_T ZeroOrderHold[6];             /* '<S2>/Zero-Order Hold' */
  real_T TrigonometricFunction3;       /* '<S8>/Trigonometric Function3' */
  real_T TrigonometricFunction2;       /* '<S8>/Trigonometric Function2' */
  real_T Abs1[2];                      /* '<S7>/Abs1' */
  real_T SumofElements1;               /* '<S7>/Sum of Elements1' */
  real_T Switch1;                      /* '<S7>/Switch1' */
  real_T kDIR;                         /* '<S7>/Switch' */
  real_T lr_signed;                    /* '<S7>/Product' */
  real_T mul[2];                       /* '<S8>/mul' */
  real_T Add3[2];                      /* '<S8>/Add3' */
  real_T ZeroOrderHold1[3];            /* '<S2>/Zero-Order Hold1' */
  real_T TrigonometricFunction3_d;     /* '<S5>/Trigonometric Function3' */
  real_T TrigonometricFunction2_h;     /* '<S5>/Trigonometric Function2' */
  real_T mul_g[2];                     /* '<S5>/mul' */
  real_T Add3_d[2];                    /* '<S5>/Add3' */
  real_T e[2];                         /* '<S2>/Add' */
  real_T TrigonometricFunction1;       /* '<S7>/Trigonometric Function1' */
  real_T TrigonometricFunction;        /* '<S7>/Trigonometric Function' */
  real_T Gain;                         /* '<S7>/Gain' */
  real_T Product1;                     /* '<S7>/Product1' */
  real_T Product3;                     /* '<S7>/Product3' */
  real_T Reshape[4];                   /* '<S7>/Reshape' */
  real_T MathFunction[2];              /* '<S7>/Math Function' */
  real_T SumofElements;                /* '<S7>/Sum of Elements' */
  real_T MathFunction1;                /* '<S7>/Math Function1' */
  real_T Product4;                     /* '<S7>/Product4' */
  real_T TmpSignalConversionAtProduct2In[2];
  real_T Product2[2];                  /* '<S7>/Product2' */
  real_T Kp[2];                        /* '<S6>/Kp' */
  real_T ctrlVel_INT[2];               /* '<S6>/ctrlVel_INT' */
  real_T gain3[2];                     /* '<S6>/gain3' */
  real_T Sum1[2];                      /* '<S6>/Sum1' */
  real_T Saturation[2];                /* '<S6>/Saturation' */
  real_T u1u2[2];                      /* '<S2>/Add1' */
  real_T TrigonometricFunction_i;      /* '<S2>/Trigonometric Function' */
  real_T Product3_n;                   /* '<S2>/Product3' */
  real_T Product1_c;                   /* '<S2>/Product1' */
  real_T Gain1;                        /* '<S2>/Gain1' */
  real_T Add[2];                       /* '<S6>/Add' */
  real_T P_kAntiWind[2];               /* '<S6>/P_kAntiWind' */
  real_T Sum2[2];                      /* '<S6>/Sum2' */
  real_T P_kI[2];                      /* '<S6>/P_kI' */
  real_T TrigonometricFunction1_h;     /* '<S2>/Trigonometric Function1' */
  real_T Product;                      /* '<S2>/Product' */
  real_T Reshape_p[4];                 /* '<S2>/Reshape' */
  real_T vdwd[2];                      /* '<S2>/feebackLin' */
  real_T Bias1;                        /* '<S9>/Bias1' */
  real_T MathFunction2;                /* '<S9>/Math Function2' */
  real_T Bias;                         /* '<S9>/Bias' */
  real_T TrigonometricFunction3_j;     /* '<S7>/Trigonometric Function3' */
  real_T Add_f;                        /* '<S7>/Add' */
  real_T Abs;                          /* '<S7>/Abs' */
} rtB_ctrlPos_WMRctrlVelPosOdo0;

/* Block states (auto storage) for system '<S1>/ctrlPos' */
typedef struct {
  real_T ctrlVel_INT_DSTATE[2];        /* '<S6>/ctrlVel_INT' */
} rtDW_ctrlPos_WMRctrlVelPosOdo0;

extern void WMRctrlVelPosOdo0_ctrlPos(const real_T rtu_P0_XYdXdYThdTh_desired[6],
  const real_T rtu_P0_XYtheta_meas[3], rtB_ctrlPos_WMRctrlVelPosOdo0 *localB,
  rtDW_ctrlPos_WMRctrlVelPosOdo0 *localDW);

#endif                                 /* RTW_HEADER_WMRctrlVelPosOdo0_ctrlPos_h_ */

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
