/*
 * File: WMRctrlVelPosOdo0_encOdometry.c
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

#include "WMRctrlVelPosOdo0_encOdometry.h"

/* Include model header file for global data */
#include "WMRctrlVelPosOdo0.h"
#include "WMRctrlVelPosOdo0_private.h"

/* Initial conditions for atomic system: '<S1>/encOdometry' */
void WMRctrlVelPosO_encOdometry_Init(rtDW_encOdometry_WMRctrlVelPosO *localDW)
{
  /* InitializeConditions for DiscreteIntegrator: '<S12>/int_theta' */
  localDW->int_theta_IC_LOADING = 1U;
  localDW->int_theta_PrevResetState = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S12>/int_X' */
  localDW->int_X_IC_LOADING = 1U;
  localDW->int_X_PrevResetState = 0;

  /* InitializeConditions for DiscreteIntegrator: '<S12>/int_Y' */
  localDW->int_Y_IC_LOADING = 1U;
  localDW->int_Y_PrevResetState = 0;
}

/* Output and update for atomic system: '<S1>/encOdometry' */
void WMRctrlVelPosOdo0_encOdometry(const int32_T rtu_in_encLR[2], boolean_T
  rtu_in_swReset, real_T rtu_in_x0y0theta0, real_T rtu_in_x0y0theta0_c, real_T
  rtu_in_x0y0theta0_p, rtB_encOdometry_WMRctrlVelPosOd *localB,
  rtDW_encOdometry_WMRctrlVelPosO *localDW)
{
  /* Constant: '<S4>/Constant' */
  localB->Constant = PCtrlOdo.PEncWhl_maxCntr;

  /* Gain: '<S11>/Gain' */
  localB->Gain = mul_s32_s32_s32_sr31(rtcP_Gain_Gain, localB->Constant);

  /* UnitDelay: '<S13>/UD' */
  localB->Uk1[0] = localDW->UD_DSTATE[0];

  /* Sum: '<S13>/Diff' */
  localB->Diff[0] = rtu_in_encLR[0] - localB->Uk1[0];

  /* Signum: '<S11>/Sign' */
  localB->Sign[0] = rt_SGN(localB->Diff[0]);

  /* Abs: '<S11>/Abs' */
  if (localB->Diff[0] < 0L) {
    localB->Abs[0] = (int32_T)(uint32_T)(-localB->Diff[0]);
  } else {
    localB->Abs[0] = localB->Diff[0];
  }

  /* RelationalOperator: '<S11>/Relational Operator' */
  localB->RelationalOperator[0] = (localB->Abs[0] >= localB->Gain);

  /* Product: '<S11>/Product1' */
  localB->Product1[0] = localB->RelationalOperator[0] ? localB->Sign[0] : 0L;

  /* Switch: '<S11>/Switch' */
  if (localB->Product1[0] > rtcP_pooled8) {
    localB->Switch[0] = localB->Constant;
  } else {
    localB->Switch[0] = ((int32_T)0L);
  }

  /* Gain: '<S11>/Gain1' */
  localB->Gain1_p[0] = mul_s32_s32_s32_sr31(rtcP_Gain1_Gain, localB->Product1[0]);

  /* Switch: '<S11>/Switch1' */
  if (localB->Gain1_p[0] > rtcP_pooled8) {
    localB->Switch1[0] = localB->Constant;
  } else {
    localB->Switch1[0] = ((int32_T)0L);
  }

  /* Sum: '<S11>/Add1' */
  localB->Add1[0] = (localB->Diff[0] - localB->Switch[0]) + localB->Switch1[0];

  /* DataTypeConversion: '<S12>/Data Type Conversion' */
  localB->DataTypeConversion[0] = (real_T)localB->Add1[0];

  /* Gain: '<S4>/Gain1' incorporates:
   *  Constant: '<S4>/Constant1'
   */
  localB->Gain1[0] = rtcP_pooled2 * PCtrlOdo.PEncWhl_rl[0];

  /* Product: '<S4>/Divide' incorporates:
   *  Constant: '<S4>/Constant3'
   */
  localB->inc2m[0] = localB->Gain1[0] / PCtrlOdo.PEncWhl_IperRev;

  /* Product: '<S12>/inc2m' */
  localB->inc2m_j[0] = localB->DataTypeConversion[0] * localB->inc2m[0];

  /* UnitDelay: '<S13>/UD' */
  localB->Uk1[1] = localDW->UD_DSTATE[1];

  /* Sum: '<S13>/Diff' */
  localB->Diff[1] = rtu_in_encLR[1] - localB->Uk1[1];

  /* Signum: '<S11>/Sign' */
  localB->Sign[1] = rt_SGN(localB->Diff[1]);

  /* Abs: '<S11>/Abs' */
  if (localB->Diff[1] < 0L) {
    localB->Abs[1] = (int32_T)(uint32_T)(-localB->Diff[1]);
  } else {
    localB->Abs[1] = localB->Diff[1];
  }

  /* RelationalOperator: '<S11>/Relational Operator' */
  localB->RelationalOperator[1] = (localB->Abs[1] >= localB->Gain);

  /* Product: '<S11>/Product1' */
  localB->Product1[1] = localB->RelationalOperator[1] ? localB->Sign[1] : 0L;

  /* Switch: '<S11>/Switch' */
  if (localB->Product1[1] > rtcP_pooled8) {
    localB->Switch[1] = localB->Constant;
  } else {
    localB->Switch[1] = ((int32_T)0L);
  }

  /* Gain: '<S11>/Gain1' */
  localB->Gain1_p[1] = mul_s32_s32_s32_sr31(rtcP_Gain1_Gain, localB->Product1[1]);

  /* Switch: '<S11>/Switch1' */
  if (localB->Gain1_p[1] > rtcP_pooled8) {
    localB->Switch1[1] = localB->Constant;
  } else {
    localB->Switch1[1] = ((int32_T)0L);
  }

  /* Sum: '<S11>/Add1' */
  localB->Add1[1] = (localB->Diff[1] - localB->Switch[1]) + localB->Switch1[1];

  /* DataTypeConversion: '<S12>/Data Type Conversion' */
  localB->DataTypeConversion[1] = (real_T)localB->Add1[1];

  /* Gain: '<S4>/Gain1' incorporates:
   *  Constant: '<S4>/Constant1'
   */
  localB->Gain1[1] = rtcP_pooled2 * PCtrlOdo.PEncWhl_rl[1];

  /* Product: '<S4>/Divide' incorporates:
   *  Constant: '<S4>/Constant3'
   */
  localB->inc2m[1] = localB->Gain1[1] / PCtrlOdo.PEncWhl_IperRev;

  /* Product: '<S12>/inc2m' */
  localB->inc2m_j[1] = localB->DataTypeConversion[1] * localB->inc2m[1];

  /* Fcn: '<S12>/Fcn1' */
  localB->Fcn1 = localB->inc2m_j[0] + localB->inc2m_j[1];

  /* Gain: '<S12>/gain' */
  localB->vTs = rtcP_pooled6 * localB->Fcn1;

  /* Product: '<S12>/Divide1' incorporates:
   *  Constant: '<S12>/Constant'
   */
  localB->v = localB->vTs / PCtrlOdo.PCtrl_Ts1;

  /* Fcn: '<S12>/Fcn' */
  localB->Fcn = (-localB->inc2m_j[0]) + localB->inc2m_j[1];

  /* Product: '<S12>/Divide' incorporates:
   *  Constant: '<S4>/Constant2'
   */
  localB->omegaTs = localB->Fcn / PCtrlOdo.PEncWhl_l;

  /* DiscreteIntegrator: '<S12>/int_theta' */
  if (localDW->int_theta_IC_LOADING) {
    localDW->theta = rtu_in_x0y0theta0;
    localB->theta = localDW->theta;
  } else if (rtu_in_swReset || (localDW->int_theta_PrevResetState != 0)) {
    localDW->theta = rtu_in_x0y0theta0;
    localB->theta = localDW->theta;
  } else {
    localB->theta = rtcP_pooled7 * localB->omegaTs + localDW->theta;
  }

  /* Fcn: '<S12>/Fcn2' */
  localB->Fcn2 = localB->v * cos(localB->theta);

  /* Product: '<S12>/Divide3' incorporates:
   *  Constant: '<S12>/Constant'
   */
  localB->Divide3 = rtu_in_x0y0theta0_c / PCtrlOdo.PCtrl_Ts1;

  /* DiscreteIntegrator: '<S12>/int_X' */
  if (localDW->int_X_IC_LOADING) {
    localDW->x = localB->Divide3;
    localB->int_X = localDW->x;
  } else if (rtu_in_swReset || (localDW->int_X_PrevResetState != 0)) {
    localDW->x = localB->Divide3;
    localB->int_X = localDW->x;
  } else {
    localB->int_X = rtcP_pooled7 * localB->Fcn2 + localDW->x;
  }

  /* Gain: '<S12>/gain3' */
  localB->x = PCtrlOdo.PCtrl_Ts1 * localB->int_X;

  /* Fcn: '<S12>/Fcn3' */
  localB->Fcn3 = localB->v * sin(localB->theta);

  /* Product: '<S12>/Divide4' incorporates:
   *  Constant: '<S12>/Constant'
   */
  localB->Divide4 = rtu_in_x0y0theta0_p / PCtrlOdo.PCtrl_Ts1;

  /* DiscreteIntegrator: '<S12>/int_Y' */
  if (localDW->int_Y_IC_LOADING) {
    localDW->y = localB->Divide4;
    localB->int_Y = localDW->y;
  } else if (rtu_in_swReset || (localDW->int_Y_PrevResetState != 0)) {
    localDW->y = localB->Divide4;
    localB->int_Y = localDW->y;
  } else {
    localB->int_Y = rtcP_pooled7 * localB->Fcn3 + localDW->y;
  }

  /* Gain: '<S12>/gain4' */
  localB->y = PCtrlOdo.PCtrl_Ts1 * localB->int_Y;

  /* Trigonometry: '<S4>/Trigonometric Function1' */
  localB->TrigonometricFunction1 = cos(localB->theta);

  /* Trigonometry: '<S4>/Trigonometric Function' */
  localB->TrigonometricFunction = sin(localB->theta);

  /* Gain: '<S4>/Gain3' */
  localB->Gain3[0] = PCtrlOdo.PCtrl_PosLr * localB->TrigonometricFunction1;
  localB->Gain3[1] = PCtrlOdo.PCtrl_PosLr * localB->TrigonometricFunction;

  /* Sum: '<S4>/Add2' */
  localB->Add2[0] = localB->x + localB->Gain3[0];
  localB->Add2[1] = localB->y + localB->Gain3[1];

  /* Product: '<S12>/Divide2' incorporates:
   *  Constant: '<S12>/Constant'
   */
  localB->Divide2 = localB->omegaTs / PCtrlOdo.PCtrl_Ts1;

  /* Update for UnitDelay: '<S13>/UD' */
  localDW->UD_DSTATE[0] = rtu_in_encLR[0];
  localDW->UD_DSTATE[1] = rtu_in_encLR[1];

  /* Update for DiscreteIntegrator: '<S12>/int_theta' */
  localDW->int_theta_IC_LOADING = 0U;
  if (!rtu_in_swReset) {
    localDW->theta = rtcP_pooled7 * localB->omegaTs + localB->theta;
  }

  if (rtu_in_swReset) {
    localDW->int_theta_PrevResetState = 1;
  } else {
    localDW->int_theta_PrevResetState = 0;
  }

  /* Update for DiscreteIntegrator: '<S12>/int_X' */
  localDW->int_X_IC_LOADING = 0U;
  if (!rtu_in_swReset) {
    localDW->x = rtcP_pooled7 * localB->Fcn2 + localB->int_X;
  }

  if (rtu_in_swReset) {
    localDW->int_X_PrevResetState = 1;
  } else {
    localDW->int_X_PrevResetState = 0;
  }

  /* Update for DiscreteIntegrator: '<S12>/int_Y' */
  localDW->int_Y_IC_LOADING = 0U;
  if (!rtu_in_swReset) {
    localDW->y = rtcP_pooled7 * localB->Fcn3 + localB->int_Y;
  }

  if (rtu_in_swReset) {
    localDW->int_Y_PrevResetState = 1;
  } else {
    localDW->int_Y_PrevResetState = 0;
  }
}

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
