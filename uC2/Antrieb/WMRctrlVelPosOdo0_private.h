/*
 * File: WMRctrlVelPosOdo0_private.h
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

#ifndef RTW_HEADER_WMRctrlVelPosOdo0_private_h_
#define RTW_HEADER_WMRctrlVelPosOdo0_private_h_
#include "rtwtypes.h"
#ifndef UCHAR_MAX
#include <limits.h>
#endif

#if ( UCHAR_MAX != (0xFFU) ) || ( SCHAR_MAX != (0x7F) )
#error "Code was generated for compiler with different sized uchar/char. Consider adjusting Emulation Hardware word size settings on the Hardware Implementation pane to match your compiler word sizes as defined in the compiler's limits.h header file. Alternatively, you can select 'None' for Emulation Hardware and select the 'Enable portable word sizes' option for ERT based targets, which will disable the preprocessor word size checks."
#endif

#if ( USHRT_MAX != (0xFFFFU) ) || ( SHRT_MAX != (0x7FFF) )
#error "Code was generated for compiler with different sized ushort/short. Consider adjusting Emulation Hardware word size settings on the Hardware Implementation pane to match your compiler word sizes as defined in the compilers limits.h header file. Alternatively, you can select 'None' for Emulation Hardware and select the 'Enable portable word sizes' option for ERT based targets, this will disable the preprocessor word size checks."
#endif

#if ( UINT_MAX != (0xFFFFU) ) || ( INT_MAX != (0x7FFF) )
#error "Code was generated for compiler with different sized uint/int. Consider adjusting Emulation Hardware word size settings on the Hardware Implementation pane to match your compiler word sizes as defined in the compilers limits.h header file. Alternatively, you can select 'None' for Emulation Hardware and select the 'Enable portable word sizes' option for ERT based targets, this will disable the preprocessor word size checks."
#endif

#if ( ULONG_MAX != (0xFFFFFFFFUL) ) || ( LONG_MAX != (0x7FFFFFFFL) )
#error "Code was generated for compiler with different sized ulong/long. Consider adjusting Emulation Hardware word size settings on the Hardware Implementation pane to match your compiler word sizes as defined in the compilers limits.h header file. Alternatively, you can select 'None' for Emulation Hardware and select the 'Enable portable word sizes' option for ERT based targets, this will disable the preprocessor word size checks."
#endif

#ifndef __RTWTYPES_H__
#error This file requires rtwtypes.h to be included
#else
#ifdef TMWTYPES_PREVIOUSLY_INCLUDED
#error This file requires rtwtypes.h to be included before tmwtypes.h
#else

/* Check for inclusion of an incorrect version of rtwtypes.h */
#ifndef RTWTYPES_ID_C08S16I16L32N08F1
#error This code was generated with a different "rtwtypes.h" than the file included
#endif                                 /* RTWTYPES_ID_C08S16I16L32N08F1 */
#endif                                 /* TMWTYPES_PREVIOUSLY_INCLUDED */
#endif                                 /* __RTWTYPES_H__ */

/* Expression: pi
 * Referenced by: '<S9>/Bias1'
 */
#define rtcP_Bias1_Bias                (3.1415926535897931E+000)

/* Expression: -pi
 * Referenced by: '<S9>/Bias'
 */
#define rtcP_Bias_Bias                 (-3.1415926535897931E+000)

/* Pooled Parameter (Expression: -1)
 * Referenced by:
 *   '<S2>/Gain1'
 *   '<S7>/const2'
 *   '<S7>/Gain'
 *   '<S10>/gain1'
 */
#define rtcP_pooled1                   (-1.0)

/* Pooled Parameter (Expression: 2*pi)
 * Referenced by:
 *   '<S4>/Gain1'
 *   '<S9>/const3'
 */
#define rtcP_pooled2                   (6.2831853071795862E+000)

/* Pooled Parameter (Expression: 0)
 * Referenced by:
 *   '<S3>/ctrlVel_INT'
 *   '<S6>/ctrlVel_INT'
 *   '<S7>/const4'
 *   '<S7>/Switch1'
 */
#define rtcP_pooled3                   (0.0)

/* Pooled Parameter (Expression: 1)
 * Referenced by:
 *   '<S7>/const1'
 *   '<S10>/Constant'
 */
#define rtcP_pooled4                   (1.0)

/* Expression: pi/10
 * Referenced by: '<S7>/Switch'
 */
#define rtcP_Switch_Threshold          (3.1415926535897931E-001)

/* Pooled Parameter (Expression: )
 * Referenced by:
 *   '<S3>/ctrlVel_INT'
 *   '<S6>/ctrlVel_INT'
 */
#define rtcP_pooled5                   (1.0)

/* Pooled Parameter (Mixed Expressions)
 * Referenced by:
 *   '<S10>/gain2'
 *   '<S12>/gain'
 */
#define rtcP_pooled6                   (0.5)

/* Pooled Parameter (Expression: )
 * Referenced by:
 *   '<S12>/int_X'
 *   '<S12>/int_Y'
 *   '<S12>/int_theta'
 */
#define rtcP_pooled7                   (0.5)

/* Pooled Parameter (Expression: )
 * Referenced by:
 *   '<S11>/Switch'
 *   '<S11>/Switch1'
 *   '<S13>/UD'
 */
#define rtcP_pooled8                   (((int32_T)0L))

/* Computed Parameter: Gain_Gain
 * Referenced by: '<S11>/Gain'
 */
#define rtcP_Gain_Gain                 (((int32_T)1073741824L))

/* Computed Parameter: Gain1_Gain
 * Referenced by: '<S11>/Gain1'
 */
#define rtcP_Gain1_Gain                (MIN_int32_T)

/* Computed Parameter: Constant1_Value
 * Referenced by: '<S1>/Constant1'
 */
#define rtcP_Constant1_Value           (((uint8_T)0U))

/* Computed Parameter: Constant_Value
 * Referenced by: '<S1>/Constant'
 */
#define rtcP_Constant_Value            (FALSE)

extern void mul_wide_s32(int32_T in0, int32_T in1, uint32_T *ptrOutBitsHi,
  uint32_T *ptrOutBitsLo);
extern int32_T mul_s32_s32_s32_sr31(int32_T a, int32_T b);

#endif                                 /* RTW_HEADER_WMRctrlVelPosOdo0_private_h_ */

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
