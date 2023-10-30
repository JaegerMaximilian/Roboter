/*
 * File: rt_atan2.c
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

#include "rt_atan2.h"
#include "rt_defines.h"
#include <math.h>

/* Calls ATAN2 directly, with no guards against domain error or non-finites */
real_T rt_atan2(real_T a, real_T b)
{
  return (atan2(a, b));
}                                      /* end rt_atan2 */

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
