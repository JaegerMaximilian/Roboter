/*
 * File: rt_mod.c
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

#include "rt_mod.h"
#include <float.h>
#include <math.h>
#include <stdlib.h> 

/*
 * Calls double-precision version of MOD directly, with no guards against
 * domain error or non-finites
 */
real_T rt_mod(const real_T xr, const real_T yr)
{
  real_T zr, yrf, tr, trf, wr;
  if (yr == 0.0) {
    zr = xr;
  } else {
    yrf = floor(yr);
    tr = xr/yr;
    if (yr == yrf) {
      /* Integer denominator.  Use conventional formula.*/
      trf = floor(tr);
      zr = xr - trf*yr;
    } else {
      /* Noninteger denominator. Check for nearly integer quotient.*/
      if (fabs(tr) < 4.5035996273704960E+015) {
        wr = ((tr < 0.0) ? -floor((fabs(tr) + 0.5)) : floor((fabs(tr) + 0.5)));
      } else {
        wr = tr;
      }

      if (fabs(tr - wr) <= ((DBL_EPSILON)*fabs(tr))) {
        zr = 0.0;
      } else {
        trf = floor(tr);
        zr = (tr - trf)*yr;
      }
    }
  }

  return zr;
}                                      /* end rt_mod */

/*
 * File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
