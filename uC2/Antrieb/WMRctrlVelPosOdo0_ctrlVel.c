/*
 * File: WMRctrlVelPosOdo0_ctrlVel.c
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

#include "WMRctrlVelPosOdo0_ctrlVel.h"

/* Include model header file for global data */
#include "WMRctrlVelPosOdo0.h"
#include "WMRctrlVelPosOdo0_private.h"

/* Start for atomic system: '<S1>/ctrlVel' */
void WMRctrlVelPosOdo0_ctrlVel_Start(rtDW_ctrlVel_WMRctrlVelPosOdo0 *localDW)
{
  /* Create Identity Matrix for Block: '<S3>/Product' */
  {
    int_T i;
    real_T *dWork = &localDW->Product_DWORK4[0];
    for (i = 0; i < 4; i++) {
      *dWork++ = 0.0;
    }

    dWork = &localDW->Product_DWORK4[0];
    while (dWork < &localDW->Product_DWORK4[0]+4) {
      *dWork = 1;
      dWork += 3;
    }
  }
}

/* Output and update for atomic system: '<S1>/ctrlVel' */
void WMRctrlVelPosOdo0_ctrlVel(const real_T rtu_vdwd[2], const real_T rtu_vw[2],
  rtB_ctrlVel_WMRctrlVelPosOdo0 *localB, rtDW_ctrlVel_WMRctrlVelPosOdo0 *localDW)
{
  real_T y;
  real_T unnamed_idx;

  /* Gain: '<S10>/gain2' incorporates:
   *  Constant: '<S10>/Constant1'
   */
  localB->gain2 = rtcP_pooled6 * PCtrlOdo.PCtrl_DynL;

  /* Gain: '<S10>/gain1' */
  localB->gain1 = rtcP_pooled1 * localB->gain2;

  /* Reshape: '<S10>/Reshape' incorporates:
   *  Constant: '<S10>/Constant'
   */
  localB->Reshape[0] = rtcP_pooled4;
  localB->Reshape[1] = localB->gain1;
  localB->Reshape[2] = rtcP_pooled4;
  localB->Reshape[3] = localB->gain2;

  /* Product: '<S10>/Product6' incorporates:
   *  Constant: '<S10>/Constant2'
   *  Constant: '<S10>/Constant3'
   *  Constant: '<S10>/Constant4'
   */
  localB->Product6 = PCtrlOdo.PCtrl_DynKmDRa * PCtrlOdo.PCtrl_DynN /
    PCtrlOdo.PCtrl_DynR;

  /* Product: '<S10>/Product5' */
  localB->Product5[0] = localB->Reshape[0] * localB->Product6;
  localB->Product5[1] = localB->Reshape[1] * localB->Product6;
  localB->Product5[2] = localB->Reshape[2] * localB->Product6;
  localB->Product5[3] = localB->Reshape[3] * localB->Product6;

  /* Product: '<S3>/Product' */
  {
    static const int_T dims[3] = { 2, 2, 2 };

    rt_MatDivRR_Dbl(localB->Product, localB->Product5,
                    &localDW->Product_DWORK4[0], (real_T*)
                    &localDW->Product_DWORK1[0],
                    &localDW->Product_DWORK2[0], (real_T*)
                    &localDW->Product_DWORK3[0],
                    &dims[0]);
  }

  /* ZeroOrderHold: '<S3>/Zero-Order Hold' */
  localB->ZeroOrderHold[0] = rtu_vdwd[0];

  /* ZeroOrderHold: '<S3>/Zero-Order Hold1' */
  localB->ZeroOrderHold1[0] = rtu_vw[0];

  /* Sum: '<S3>/Sum' */
  localB->e[0] = localB->ZeroOrderHold[0] - localB->ZeroOrderHold1[0];

  /* Gain: '<S3>/P_kP' */
  localB->P_kP[0] = PCtrlOdo.PCtrl_VelKp[0] * localB->e[0];

  /* DiscreteIntegrator: '<S3>/ctrlVel_INT' */
  localB->ctrlVel_INT[0] = localDW->ctrlVel_INT_DSTATE[0];

  /* Gain: '<S3>/gain3' */
  localB->gain3[0] = PCtrlOdo.PCtrl_Ts1 * localB->ctrlVel_INT[0];

  /* Sum: '<S3>/Sum1' */
  localB->Sum1[0] = localB->P_kP[0] + localB->gain3[0];

  /* Product: '<S3>/Product1' incorporates:
   *  Constant: '<S3>/normalize'
   */
  localB->uvuw[0] = PCtrlOdo.PCtrl_VelnormPlantGains[0] * localB->Sum1[0];

  /* ZeroOrderHold: '<S3>/Zero-Order Hold' */
  localB->ZeroOrderHold[1] = rtu_vdwd[1];

  /* ZeroOrderHold: '<S3>/Zero-Order Hold1' */
  localB->ZeroOrderHold1[1] = rtu_vw[1];

  /* Sum: '<S3>/Sum' */
  localB->e[1] = localB->ZeroOrderHold[1] - localB->ZeroOrderHold1[1];

  /* Gain: '<S3>/P_kP' */
  localB->P_kP[1] = PCtrlOdo.PCtrl_VelKp[1] * localB->e[1];

  /* DiscreteIntegrator: '<S3>/ctrlVel_INT' */
  localB->ctrlVel_INT[1] = localDW->ctrlVel_INT_DSTATE[1];

  /* Gain: '<S3>/gain3' */
  localB->gain3[1] = PCtrlOdo.PCtrl_Ts1 * localB->ctrlVel_INT[1];

  /* Sum: '<S3>/Sum1' */
  localB->Sum1[1] = localB->P_kP[1] + localB->gain3[1];

  /* Product: '<S3>/Product1' incorporates:
   *  Constant: '<S3>/normalize'
   */
  localB->uvuw[1] = PCtrlOdo.PCtrl_VelnormPlantGains[1] * localB->Sum1[1];

  /* Saturate: '<S3>/Saturation' */
  y = -PCtrlOdo.PCtrl_VelUsat;

  /* Product: '<S3>/Product2' */
  localB->Product2[0] = 0.0;
  localB->Product2[0] = localB->Product[0] * localB->uvuw[0] + localB->Product2
    [0];
  localB->Product2[0] = localB->Product[2] * localB->uvuw[1] + localB->Product2
    [0];
  unnamed_idx = localB->Product2[0];
  localB->ulur[0] = rt_SATURATE(unnamed_idx, y, PCtrlOdo.PCtrl_VelUsat);

  /* Sum: '<S3>/Add' */
  localB->e_saturation[0] = localB->ulur[0] - localB->Product2[0];

  /* Product: '<S3>/Product2' */
  localB->Product2[1] = 0.0;
  localB->Product2[1] = localB->Product[1] * localB->uvuw[0] + localB->Product2
    [1];
  localB->Product2[1] = localB->Product[3] * localB->uvuw[1] + localB->Product2
    [1];
  unnamed_idx = localB->Product2[1];
  localB->ulur[1] = rt_SATURATE(unnamed_idx, y, PCtrlOdo.PCtrl_VelUsat);

  /* Sum: '<S3>/Add' */
  localB->e_saturation[1] = localB->ulur[1] - localB->Product2[1];

  /* Product: '<S3>/Product3' */
  localB->Product3[0] = 0.0;
  localB->Product3[0] = localB->Product5[0] * localB->e_saturation[0] +
    localB->Product3[0];
  localB->Product3[0] = localB->Product5[2] * localB->e_saturation[1] +
    localB->Product3[0];

  /* Product: '<S3>/Product4' incorporates:
   *  Constant: '<S3>/normalize'
   */
  localB->Product4[0] = 1.0 / PCtrlOdo.PCtrl_VelnormPlantGains[0] *
    localB->Product3[0];

  /* Gain: '<S3>/P_kAntiWind' */
  localB->P_kAntiWind[0] = PCtrlOdo.PCtrl_VelKAntiWind * localB->Product4[0];

  /* Sum: '<S3>/Sum2' */
  localB->Sum2[0] = localB->e[0] + localB->P_kAntiWind[0];

  /* Gain: '<S3>/P_kI' */
  localB->P_kI[0] = PCtrlOdo.PCtrl_VelKI[0] * localB->Sum2[0];

  /* Update for DiscreteIntegrator: '<S3>/ctrlVel_INT' */
  localDW->ctrlVel_INT_DSTATE[0] = rtcP_pooled5 * localB->P_kI[0] +
    localDW->ctrlVel_INT_DSTATE[0];

  /* Product: '<S3>/Product3' */
  localB->Product3[1] = 0.0;
  localB->Product3[1] = localB->Product5[1] * localB->e_saturation[0] +
    localB->Product3[1];
  localB->Product3[1] = localB->Product5[3] * localB->e_saturation[1] +
    localB->Product3[1];

  /* Product: '<S3>/Product4' incorporates:
   *  Constant: '<S3>/normalize'
   */
  localB->Product4[1] = 1.0 / PCtrlOdo.PCtrl_VelnormPlantGains[1] *
    localB->Product3[1];

  /* Gain: '<S3>/P_kAntiWind' */
  localB->P_kAntiWind[1] = PCtrlOdo.PCtrl_VelKAntiWind * localB->Product4[1];

  /* Sum: '<S3>/Sum2' */
  localB->Sum2[1] = localB->e[1] + localB->P_kAntiWind[1];

  /* Gain: '<S3>/P_kI' */
  localB->P_kI[1] = PCtrlOdo.PCtrl_VelKI[1] * localB->Sum2[1];

  /* Update for DiscreteIntegrator: '<S3>/ctrlVel_INT' */
  localDW->ctrlVel_INT_DSTATE[1] = rtcP_pooled5 * localB->P_kI[1] +
    localDW->ctrlVel_INT_DSTATE[1];
}

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
