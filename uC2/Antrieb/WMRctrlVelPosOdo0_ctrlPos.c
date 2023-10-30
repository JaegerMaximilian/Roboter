/*
 * File: WMRctrlVelPosOdo0_ctrlPos.c
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

#include "WMRctrlVelPosOdo0_ctrlPos.h"

/* Include model header file for global data */
#include "WMRctrlVelPosOdo0.h"
#include "WMRctrlVelPosOdo0_private.h"
#include <stdlib.h> 


/*
 * Output and update for atomic system: '<S1>/ctrlPos'
 * Block description for: '<S1>/ctrlPos'
 *   TEST_______________
 */
void WMRctrlVelPosOdo0_ctrlPos(const real_T rtu_P0_XYdXdYThdTh_desired[6], const
  real_T rtu_P0_XYtheta_meas[3], rtB_ctrlPos_WMRctrlVelPosOdo0 *localB,
  rtDW_ctrlPos_WMRctrlVelPosOdo0 *localDW)
{
  int16_T i;
  real_T unnamed_idx;
  real_T y_idx;

  /* ZeroOrderHold: '<S2>/Zero-Order Hold' */
  for (i = 0; i < 6; i++) {
    localB->ZeroOrderHold[i] = rtu_P0_XYdXdYThdTh_desired[i];
  }

  /* Trigonometry: '<S8>/Trigonometric Function3' */
  localB->TrigonometricFunction3 = cos(localB->ZeroOrderHold[4]);

  /* Trigonometry: '<S8>/Trigonometric Function2' */
  localB->TrigonometricFunction2 = sin(localB->ZeroOrderHold[4]);

  /* Abs: '<S7>/Abs1' */
  localB->Abs1[0] = fabs(localB->ZeroOrderHold[2]);
  localB->Abs1[1] = fabs(localB->ZeroOrderHold[3]);

  /* Sum: '<S7>/Sum of Elements1' */
  unnamed_idx = localB->Abs1[0];
  unnamed_idx += localB->Abs1[1];
  localB->SumofElements1 = unnamed_idx;

  /* Switch: '<S7>/Switch1' incorporates:
   *  Constant: '<S7>/const4'
   */
  if (localB->SumofElements1 > rtcP_pooled3) {
    /* Bias: '<S9>/Bias1' */
    localB->Bias1 = localB->ZeroOrderHold[4] + rtcP_Bias1_Bias;

    /* Math: '<S9>/Math Function2' incorporates:
     *  Constant: '<S9>/const3'
     */
    localB->MathFunction2 = rt_mod(localB->Bias1, rtcP_pooled2);

    /* Bias: '<S9>/Bias' */
    localB->Bias = localB->MathFunction2 + rtcP_Bias_Bias;

    /* Trigonometry: '<S7>/Trigonometric Function3' */
    localB->TrigonometricFunction3_j = rt_atan2(localB->ZeroOrderHold[3],
      localB->ZeroOrderHold[2]);

    /* Sum: '<S7>/Add' */
    localB->Add_f = localB->TrigonometricFunction3_j - localB->Bias;

    /* Abs: '<S7>/Abs' */
    localB->Abs = fabs(localB->Add_f);
    localB->Switch1 = localB->Abs;
  } else {
    localB->Switch1 = rtcP_pooled3;
  }

  /* Switch: '<S7>/Switch' incorporates:
   *  Constant: '<S7>/const1'
   *  Constant: '<S7>/const2'
   */
  if (localB->Switch1 >= rtcP_Switch_Threshold) {
    localB->kDIR = rtcP_pooled1;
  } else {
    localB->kDIR = rtcP_pooled4;
  }

  /* Product: '<S7>/Product' incorporates:
   *  Constant: '<S7>/Constant'
   */
  localB->lr_signed = PCtrlOdo.PCtrl_PosLr * localB->kDIR;

  /* Product: '<S8>/mul' */
  localB->mul[0] = localB->TrigonometricFunction3 * localB->lr_signed;
  localB->mul[1] = localB->TrigonometricFunction2 * localB->lr_signed;

  /* Sum: '<S8>/Add3' */
  localB->Add3[0] = localB->ZeroOrderHold[0] + localB->mul[0];
  localB->Add3[1] = localB->ZeroOrderHold[1] + localB->mul[1];

  /* ZeroOrderHold: '<S2>/Zero-Order Hold1' */
  localB->ZeroOrderHold1[0] = rtu_P0_XYtheta_meas[0];
  localB->ZeroOrderHold1[1] = rtu_P0_XYtheta_meas[1];
  localB->ZeroOrderHold1[2] = rtu_P0_XYtheta_meas[2];

  /* Trigonometry: '<S5>/Trigonometric Function3' */
  localB->TrigonometricFunction3_d = cos(localB->ZeroOrderHold1[2]);

  /* Trigonometry: '<S5>/Trigonometric Function2' */
  localB->TrigonometricFunction2_h = sin(localB->ZeroOrderHold1[2]);

  /* Product: '<S5>/mul' */
  localB->mul_g[0] = localB->TrigonometricFunction3_d * localB->lr_signed;
  localB->mul_g[1] = localB->TrigonometricFunction2_h * localB->lr_signed;

  /* Trigonometry: '<S7>/Trigonometric Function1' */
  localB->TrigonometricFunction1 = cos(localB->ZeroOrderHold[4]);

  /* Trigonometry: '<S7>/Trigonometric Function' */
  localB->TrigonometricFunction = sin(localB->ZeroOrderHold[4]);

  /* Gain: '<S7>/Gain' */
  localB->Gain = rtcP_pooled1 * localB->lr_signed;

  /* Product: '<S7>/Product1' */
  localB->Product1 = localB->TrigonometricFunction * localB->Gain;

  /* Product: '<S7>/Product3' */
  localB->Product3 = localB->TrigonometricFunction1 * localB->lr_signed;

  /* Reshape: '<S7>/Reshape' */
  localB->Reshape[0] = localB->TrigonometricFunction1;
  localB->Reshape[1] = localB->TrigonometricFunction;
  localB->Reshape[2] = localB->Product1;
  localB->Reshape[3] = localB->Product3;

  /* Sum: '<S5>/Add3' */
  localB->Add3_d[0] = localB->ZeroOrderHold1[0] + localB->mul_g[0];

  /* Sum: '<S2>/Add' */
  localB->e[0] = localB->Add3[0] - localB->Add3_d[0];

  /* Math: '<S7>/Math Function' */
  localB->MathFunction[0] = localB->ZeroOrderHold[2] * localB->ZeroOrderHold[2];

  /* Sum: '<S5>/Add3' */
  localB->Add3_d[1] = localB->ZeroOrderHold1[1] + localB->mul_g[1];

  /* Sum: '<S2>/Add' */
  localB->e[1] = localB->Add3[1] - localB->Add3_d[1];

  /* Math: '<S7>/Math Function' */
  localB->MathFunction[1] = localB->ZeroOrderHold[3] * localB->ZeroOrderHold[3];

  /* Sum: '<S7>/Sum of Elements' */
  unnamed_idx = localB->MathFunction[0];
  unnamed_idx += localB->MathFunction[1];
  localB->SumofElements = unnamed_idx;

  /* Math: '<S7>/Math Function1'
   *
   * About '<S7>/Math Function1':
   *  Operator: sqrt
   */
  localB->MathFunction1 = localB->SumofElements < 0.0 ? -sqrt(fabs
    (localB->SumofElements)) : sqrt(localB->SumofElements);

  /* Product: '<S7>/Product4' */
  localB->Product4 = localB->kDIR * localB->MathFunction1;

  /* SignalConversion: '<S7>/TmpSignal ConversionAtProduct2Inport2' */
  localB->TmpSignalConversionAtProduct2In[0] = localB->Product4;
  localB->TmpSignalConversionAtProduct2In[1] = localB->ZeroOrderHold[5];

  /* Trigonometry: '<S2>/Trigonometric Function' */
  localB->TrigonometricFunction_i = sin(localB->ZeroOrderHold1[2]);

  /* Product: '<S2>/Product3' */
  localB->Product3_n = 1.0 / localB->lr_signed;

  /* Product: '<S2>/Product1' */
  localB->Product1_c = localB->TrigonometricFunction_i * localB->Product3_n;

  /* Gain: '<S2>/Gain1' */
  localB->Gain1 = rtcP_pooled1 * localB->Product1_c;

  /* Product: '<S7>/Product2' */
  localB->Product2[0] = 0.0;
  localB->Product2[0] = localB->Reshape[0] *
    localB->TmpSignalConversionAtProduct2In[0] + localB->Product2[0];
  localB->Product2[0] = localB->Reshape[2] *
    localB->TmpSignalConversionAtProduct2In[1] + localB->Product2[0];

  /* Gain: '<S6>/Kp' */
  localB->Kp[0] = PCtrlOdo.PCtrl_PosKp * localB->e[0];

  /* DiscreteIntegrator: '<S6>/ctrlVel_INT' */
  localB->ctrlVel_INT[0] = localDW->ctrlVel_INT_DSTATE[0];

  /* Gain: '<S6>/gain3' */
  localB->gain3[0] = PCtrlOdo.PCtrl_Ts1 * localB->ctrlVel_INT[0];

  /* Sum: '<S6>/Sum1' */
  localB->Sum1[0] = localB->Kp[0] + localB->gain3[0];
  unnamed_idx = localB->Sum1[0];
  y_idx = -PCtrlOdo.PCtrl_PosUsat[0];
  localB->Saturation[0] = rt_SATURATE(unnamed_idx, y_idx,
    PCtrlOdo.PCtrl_PosUsat[0]);

  /* Sum: '<S2>/Add1' */
  localB->u1u2[0] = localB->Product2[0] + localB->Saturation[0];

  /* Sum: '<S6>/Add' */
  localB->Add[0] = localB->Saturation[0] - localB->Sum1[0];

  /* Gain: '<S6>/P_kAntiWind' */
  localB->P_kAntiWind[0] = PCtrlOdo.PCtrl_PosKAntiWind * localB->Add[0];

  /* Sum: '<S6>/Sum2' */
  localB->Sum2[0] = localB->e[0] + localB->P_kAntiWind[0];

  /* Gain: '<S6>/P_kI' */
  localB->P_kI[0] = PCtrlOdo.PCtrl_PosKI * localB->Sum2[0];

  /* Product: '<S7>/Product2' */
  localB->Product2[1] = 0.0;
  localB->Product2[1] = localB->Reshape[1] *
    localB->TmpSignalConversionAtProduct2In[0] + localB->Product2[1];
  localB->Product2[1] = localB->Reshape[3] *
    localB->TmpSignalConversionAtProduct2In[1] + localB->Product2[1];

  /* Gain: '<S6>/Kp' */
  localB->Kp[1] = PCtrlOdo.PCtrl_PosKp * localB->e[1];

  /* DiscreteIntegrator: '<S6>/ctrlVel_INT' */
  localB->ctrlVel_INT[1] = localDW->ctrlVel_INT_DSTATE[1];

  /* Gain: '<S6>/gain3' */
  localB->gain3[1] = PCtrlOdo.PCtrl_Ts1 * localB->ctrlVel_INT[1];

  /* Sum: '<S6>/Sum1' */
  localB->Sum1[1] = localB->Kp[1] + localB->gain3[1];
  unnamed_idx = localB->Sum1[1];
  y_idx = -PCtrlOdo.PCtrl_PosUsat[1];
  localB->Saturation[1] = rt_SATURATE(unnamed_idx, y_idx,
    PCtrlOdo.PCtrl_PosUsat[1]);

  /* Sum: '<S2>/Add1' */
  localB->u1u2[1] = localB->Product2[1] + localB->Saturation[1];

  /* Sum: '<S6>/Add' */
  localB->Add[1] = localB->Saturation[1] - localB->Sum1[1];

  /* Gain: '<S6>/P_kAntiWind' */
  localB->P_kAntiWind[1] = PCtrlOdo.PCtrl_PosKAntiWind * localB->Add[1];

  /* Sum: '<S6>/Sum2' */
  localB->Sum2[1] = localB->e[1] + localB->P_kAntiWind[1];

  /* Gain: '<S6>/P_kI' */
  localB->P_kI[1] = PCtrlOdo.PCtrl_PosKI * localB->Sum2[1];

  /* Trigonometry: '<S2>/Trigonometric Function1' */
  localB->TrigonometricFunction1_h = cos(localB->ZeroOrderHold1[2]);

  /* Product: '<S2>/Product' */
  localB->Product = localB->TrigonometricFunction1_h * localB->Product3_n;

  /* Reshape: '<S2>/Reshape' */
  localB->Reshape_p[0] = localB->TrigonometricFunction1_h;
  localB->Reshape_p[1] = localB->Gain1;
  localB->Reshape_p[2] = localB->TrigonometricFunction_i;
  localB->Reshape_p[3] = localB->Product;

  /* Product: '<S2>/feebackLin' */
  localB->vdwd[0] = 0.0;
  localB->vdwd[0] = localB->Reshape_p[0] * localB->u1u2[0] + localB->vdwd[0];
  localB->vdwd[0] = localB->Reshape_p[2] * localB->u1u2[1] + localB->vdwd[0];

  /* Update for DiscreteIntegrator: '<S6>/ctrlVel_INT' */
  localDW->ctrlVel_INT_DSTATE[0] = rtcP_pooled5 * localB->P_kI[0] +
    localDW->ctrlVel_INT_DSTATE[0];

  /* Product: '<S2>/feebackLin' */
  localB->vdwd[1] = 0.0;
  localB->vdwd[1] = localB->Reshape_p[1] * localB->u1u2[0] + localB->vdwd[1];
  localB->vdwd[1] = localB->Reshape_p[3] * localB->u1u2[1] + localB->vdwd[1];

  /* Update for DiscreteIntegrator: '<S6>/ctrlVel_INT' */
  localDW->ctrlVel_INT_DSTATE[1] = rtcP_pooled5 * localB->P_kI[1] +
    localDW->ctrlVel_INT_DSTATE[1];
}

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
