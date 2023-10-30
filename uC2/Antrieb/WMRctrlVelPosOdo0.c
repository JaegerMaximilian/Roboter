/*
 * File: WMRctrlVelPosOdo0.c
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

#include "WMRctrlVelPosOdo0.h"
#include "WMRctrlVelPosOdo0_private.h"

BUSinWMRctrl WMRctrlVelPosOdo0_rtZBUSinWMRctrl = {
  {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
  ,                                    /* P0_XYdXdYThdTh_desired */

  {
    0, 0 }
  ,                                    /* encLR */
  FALSE,                               /* swReset */

  {
    0.0, 0.0, 0.0 }
  /* x0y0theta0 */
} ;                                    /* BUSinWMRctrl ground */

BUSouWMRctrl WMRctrlVelPosOdo0_rtZBUSouWMRctrl = {
  FALSE,                               /* errorFlag */
  0U,                                  /* errorNr */

  {
    0.0, 0.0 }
  ,                                    /* uldurd */

  {
    0.0, 0.0, 0.0 }
  ,                                    /* XYtheta */

  {
    0.0, 0.0 }
  ,                                    /* vomega */

  {
    0.0, 0.0, 0.0 }
  ,                                    /* XRYRtheta */

  {
    0.0, 0.0, 0.0, 0.0 }
  ,                                    /* ou_ctrlVelBUS */

  {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
  /* ou_ctrlPosBUS */
} ;                                    /* BUSouWMRctrl ground */

/* Definition for custom storage class: Struct */
PCtrlOdo_type PCtrlOdo = {
  /* PCtrl_DynKmDRa */
  2.2461538461538460E-002,

  /* PCtrl_DynL */
  0.24,

  /* PCtrl_DynN */
  14.0,

  /* PCtrl_DynR */
  0.03965,

  /* PCtrl_PosKAntiWind */
  0.0,

  /* PCtrl_PosKI */
  0.0,

  /* PCtrl_PosKp */
  10.0,

  /* PCtrl_PosLr */
  0.05,

  /* PCtrl_PosUsat */
  { 1.0E+005, 1.0E+005 },

  /* PCtrl_Ts1 */
  0.006,

  /* PCtrl_VelKAntiWind */
  2.0,

  /* PCtrl_VelKI */
  { 0.0, 0.0 },

  /* PCtrl_VelKp */
  { 5.0, 4.4 },

  /* PCtrl_VelUsat */
  24.0,

  /* PCtrl_VelnormPlantGains */
  { 1.643808E+002, 2.3642 },

  /* PEncWhl_IperRev */
  20000.0,

  /* PEncWhl_l */
  0.1804,

  /* PEncWhl_rl */
  { 0.02095, 0.02095 },

  /* PEncWhl_maxCntr */
  2147483647
};

void mul_wide_s32(int32_T in0, int32_T in1, uint32_T *ptrOutBitsHi, uint32_T
                  *ptrOutBitsLo)
{
  uint32_T absIn0;
  uint32_T absIn1;
  uint32_T in0Lo;
  uint32_T in0Hi;
  uint32_T in1Hi;
  uint32_T productHiLo;
  uint32_T productLoHi;
  absIn0 = (uint32_T)(in0 < (int32_T)0 ? -in0 : in0);
  absIn1 = (uint32_T)(in1 < (int32_T)0 ? -in1 : in1);
  in0Hi = absIn0 >> 16;
  in0Lo = absIn0 & 65535UL;
  in1Hi = absIn1 >> 16;
  absIn0 = absIn1 & 65535UL;
  productHiLo = in0Hi * absIn0;
  productLoHi = in0Lo * in1Hi;
  absIn0 = in0Lo * absIn0;
  absIn1 = (uint32_T)0;
  in0Lo = (productLoHi << 16) + absIn0;
  if (in0Lo < absIn0) {
    absIn1 = (uint32_T)1;
  }

  absIn0 = in0Lo;
  in0Lo = (productHiLo << 16) + in0Lo;
  if (in0Lo < absIn0) {
    absIn1 = absIn1 + (uint32_T)1;
  }

  absIn0 = ((in0Hi * in1Hi + absIn1) + (productLoHi >> 16)) + (productHiLo >> 16);
  if (!((in0 == (int32_T)0) || (in1 == (int32_T)0) || ((in0 > (int32_T)0) ==
        (in1 > (int32_T)0)))) {
    absIn0 = ~absIn0;
    in0Lo = ~in0Lo;
    in0Lo = in0Lo + (uint32_T)1;
    if (in0Lo == (uint32_T)0) {
      absIn0 = absIn0 + (uint32_T)1;
    }
  }

  *ptrOutBitsHi = absIn0;
  *ptrOutBitsLo = in0Lo;
}

int32_T mul_s32_s32_s32_sr31(int32_T a, int32_T b)
{
  int32_T result;
  uint32_T u32_chi;
  uint32_T u32_clo;
  mul_wide_s32(a, b, &u32_chi, &u32_clo);
  u32_clo = (uint32_T)(u32_chi << 1 | u32_clo >> 31);
  result = (int32_T)u32_clo;
  return result;
}

/* Model step function */
void WMRctrlVelPosOdo0_step(BlockIO_WMRctrlVelPosOdo0 *WMRctrlVelPosOdo0_B,
  D_Work_WMRctrlVelPosOdo0 *WMRctrlVelPosOdo0_DWork,
  ExternalInputs_WMRctrlVelPosOdo *WMRctrlVelPosOdo0_U,
  ExternalOutputs_WMRctrlVelPosOd *WMRctrlVelPosOdo0_Y)
{
  int16_T i;

  /* BusSelector: '<S1>/Bus Selector2' incorporates:
   *  Inport: '<Root>/in_WMRctrlBUS'
   */
  for (i = 0; i < 6; i++) {
    WMRctrlVelPosOdo0_B->P0_XYdXdYThdTh_desired[i] =
      WMRctrlVelPosOdo0_U->in_WMRctrlBUS.P0_XYdXdYThdTh_desired[i];
  }

  /* BusSelector: '<S1>/Bus Selector1' incorporates:
   *  Inport: '<Root>/in_WMRctrlBUS'
   */
  WMRctrlVelPosOdo0_B->encLR[0] = WMRctrlVelPosOdo0_U->in_WMRctrlBUS.encLR[0];
  WMRctrlVelPosOdo0_B->encLR[1] = WMRctrlVelPosOdo0_U->in_WMRctrlBUS.encLR[1];
  WMRctrlVelPosOdo0_B->swReset = WMRctrlVelPosOdo0_U->in_WMRctrlBUS.swReset;
  WMRctrlVelPosOdo0_B->x0y0theta0[0] =
    WMRctrlVelPosOdo0_U->in_WMRctrlBUS.x0y0theta0[0];
  WMRctrlVelPosOdo0_B->x0y0theta0[1] =
    WMRctrlVelPosOdo0_U->in_WMRctrlBUS.x0y0theta0[1];
  WMRctrlVelPosOdo0_B->x0y0theta0[2] =
    WMRctrlVelPosOdo0_U->in_WMRctrlBUS.x0y0theta0[2];

  /* Outputs for atomic SubSystem: '<S1>/encOdometry' */
  WMRctrlVelPosOdo0_encOdometry(WMRctrlVelPosOdo0_B->encLR,
    WMRctrlVelPosOdo0_B->swReset, WMRctrlVelPosOdo0_B->x0y0theta0[2],
    WMRctrlVelPosOdo0_B->x0y0theta0[0], WMRctrlVelPosOdo0_B->x0y0theta0[1],
    &WMRctrlVelPosOdo0_B->encOdometry, &WMRctrlVelPosOdo0_DWork->encOdometry);

  /* end of Outputs for SubSystem: '<S1>/encOdometry' */

  /* SignalConversion: '<S1>/TmpSignal ConversionAtBus CreatorInport4' */
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreator[0] =
    WMRctrlVelPosOdo0_B->encOdometry.x;
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreator[1] =
    WMRctrlVelPosOdo0_B->encOdometry.y;
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreator[2] =
    WMRctrlVelPosOdo0_B->encOdometry.theta;

  /* SignalConversion: '<S1>/TmpSignal ConversionAtctrlPosInport2' */
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtctrlPosInp[0] =
    WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreator[0];
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtctrlPosInp[1] =
    WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreator[1];
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtctrlPosInp[2] =
    WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreator[2];

  /* Outputs for atomic SubSystem: '<S1>/ctrlPos' */
  WMRctrlVelPosOdo0_ctrlPos(WMRctrlVelPosOdo0_B->P0_XYdXdYThdTh_desired,
    WMRctrlVelPosOdo0_B->TmpSignalConversionAtctrlPosInp,
    &WMRctrlVelPosOdo0_B->ctrlPos, &WMRctrlVelPosOdo0_DWork->ctrlPos);

  /* end of Outputs for SubSystem: '<S1>/ctrlPos' */

  /* SignalConversion: '<S1>/TmpSignal ConversionAtBus CreatorInport5' */
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_h[0] =
    WMRctrlVelPosOdo0_B->encOdometry.v;
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_h[1] =
    WMRctrlVelPosOdo0_B->encOdometry.Divide2;

  /* SignalConversion: '<S1>/TmpSignal ConversionAtctrlVelInport2' */
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtctrlVelInp[0] =
    WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_h[0];
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtctrlVelInp[1] =
    WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_h[1];

  /* Outputs for atomic SubSystem: '<S1>/ctrlVel' */
  WMRctrlVelPosOdo0_ctrlVel(WMRctrlVelPosOdo0_B->ctrlPos.vdwd,
    WMRctrlVelPosOdo0_B->TmpSignalConversionAtctrlVelInp,
    &WMRctrlVelPosOdo0_B->ctrlVel, &WMRctrlVelPosOdo0_DWork->ctrlVel);

  /* end of Outputs for SubSystem: '<S1>/ctrlVel' */

  /* SignalConversion: '<S1>/TmpSignal ConversionAtBus CreatorInport6' incorporates:
   *  SignalConversion: '<S1>/TmpSignal ConversionAtBus CreatorInport7'
   */
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_p[0] =
    WMRctrlVelPosOdo0_B->encOdometry.Add2[0];
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_e[0] =
    WMRctrlVelPosOdo0_B->ctrlVel.ulur[0];
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_p[1] =
    WMRctrlVelPosOdo0_B->encOdometry.Add2[1];
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_e[1] =
    WMRctrlVelPosOdo0_B->ctrlVel.ulur[1];
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_p[2] =
    WMRctrlVelPosOdo0_B->encOdometry.theta;

  /* SignalConversion: '<S1>/TmpSignal ConversionAtBus CreatorInport8' incorporates:
   *  SignalConversion: '<S1>/TmpSignal ConversionAtBus CreatorInport7'
   */
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_e[2] =
    WMRctrlVelPosOdo0_B->ctrlVel.ulur[0];
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_a[0] =
    WMRctrlVelPosOdo0_B->ctrlPos.vdwd[0];
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_e[3] =
    WMRctrlVelPosOdo0_B->ctrlVel.ulur[1];
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_a[1] =
    WMRctrlVelPosOdo0_B->ctrlPos.vdwd[1];
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_a[2] =
    WMRctrlVelPosOdo0_B->ctrlPos.e[0];
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_a[3] =
    WMRctrlVelPosOdo0_B->ctrlPos.e[1];
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_a[4] =
    WMRctrlVelPosOdo0_B->ctrlPos.u1u2[0];
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_a[5] =
    WMRctrlVelPosOdo0_B->ctrlPos.u1u2[1];
  WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_a[6] =
    WMRctrlVelPosOdo0_B->ctrlPos.lr_signed;

  /* BusCreator: '<S1>/Bus Creator' incorporates:
   *  Constant: '<S1>/Constant'
   *  Constant: '<S1>/Constant1'
   */
  WMRctrlVelPosOdo0_Y->ou_WMRctrlBUS.errorFlag = rtcP_Constant_Value;
  WMRctrlVelPosOdo0_Y->ou_WMRctrlBUS.errorNr = rtcP_Constant1_Value;
  WMRctrlVelPosOdo0_Y->ou_WMRctrlBUS.uldurd[0] =
    WMRctrlVelPosOdo0_B->ctrlVel.ulur[0];
  WMRctrlVelPosOdo0_Y->ou_WMRctrlBUS.uldurd[1] =
    WMRctrlVelPosOdo0_B->ctrlVel.ulur[1];
  WMRctrlVelPosOdo0_Y->ou_WMRctrlBUS.XYtheta[0] =
    WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreator[0];
  WMRctrlVelPosOdo0_Y->ou_WMRctrlBUS.XYtheta[1] =
    WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreator[1];
  WMRctrlVelPosOdo0_Y->ou_WMRctrlBUS.XYtheta[2] =
    WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreator[2];
  WMRctrlVelPosOdo0_Y->ou_WMRctrlBUS.vomega[0] =
    WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_h[0];
  WMRctrlVelPosOdo0_Y->ou_WMRctrlBUS.vomega[1] =
    WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_h[1];
  WMRctrlVelPosOdo0_Y->ou_WMRctrlBUS.XRYRtheta[0] =
    WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_p[0];
  WMRctrlVelPosOdo0_Y->ou_WMRctrlBUS.XRYRtheta[1] =
    WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_p[1];
  WMRctrlVelPosOdo0_Y->ou_WMRctrlBUS.XRYRtheta[2] =
    WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_p[2];
  WMRctrlVelPosOdo0_Y->ou_WMRctrlBUS.ou_ctrlVelBUS[0] =
    WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_e[0];
  WMRctrlVelPosOdo0_Y->ou_WMRctrlBUS.ou_ctrlVelBUS[1] =
    WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_e[1];
  WMRctrlVelPosOdo0_Y->ou_WMRctrlBUS.ou_ctrlVelBUS[2] =
    WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_e[2];
  WMRctrlVelPosOdo0_Y->ou_WMRctrlBUS.ou_ctrlVelBUS[3] =
    WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_e[3];
  for (i = 0; i < 7; i++) {
    WMRctrlVelPosOdo0_Y->ou_WMRctrlBUS.ou_ctrlPosBUS[i] =
      WMRctrlVelPosOdo0_B->TmpSignalConversionAtBusCreat_a[i];
  }
}

/* Model initialize function */
void WMRctrlVelPosOdo0_initialize(BlockIO_WMRctrlVelPosOdo0 *WMRctrlVelPosOdo0_B,
  D_Work_WMRctrlVelPosOdo0 *WMRctrlVelPosOdo0_DWork,
  ExternalInputs_WMRctrlVelPosOdo *WMRctrlVelPosOdo0_U,
  ExternalOutputs_WMRctrlVelPosOd *WMRctrlVelPosOdo0_Y)
{
  /* Registration code */

  /* block I/O */
  (void) memset(((void *) WMRctrlVelPosOdo0_B), 0,
                sizeof(BlockIO_WMRctrlVelPosOdo0));

  /* states (dwork) */
  (void) memset((void *)WMRctrlVelPosOdo0_DWork, 0,
                sizeof(D_Work_WMRctrlVelPosOdo0));

  /* external inputs */
  WMRctrlVelPosOdo0_U->in_WMRctrlBUS = WMRctrlVelPosOdo0_rtZBUSinWMRctrl;

  /* external outputs */
  WMRctrlVelPosOdo0_Y->ou_WMRctrlBUS = WMRctrlVelPosOdo0_rtZBUSouWMRctrl;

  /* Start for atomic SubSystem: '<S1>/ctrlVel' */
  WMRctrlVelPosOdo0_ctrlVel_Start(&WMRctrlVelPosOdo0_DWork->ctrlVel);

  /* end of Start for SubSystem: '<S1>/ctrlVel' */

  /* InitializeConditions for atomic SubSystem: '<S1>/encOdometry' */
  WMRctrlVelPosO_encOdometry_Init(&WMRctrlVelPosOdo0_DWork->encOdometry);

  /* end of InitializeConditions for SubSystem: '<S1>/encOdometry' */
}

/* Model terminate function */
void WMRctrlVelPosOdo0_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
