/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mod.c
 *
 * Code generation for function 'mod'
 *
 */

/* Include files */
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "ExtractDistanceTrajectory.h"
#include "mod.h"

/* Function Definitions */
real_T b_mod(real_T x, real_T y)
{
  real_T r;
  r = x;
  if ((!muDoubleScalarIsInf(x)) && (!muDoubleScalarIsNaN(x)) &&
      ((!muDoubleScalarIsInf(y)) && (!muDoubleScalarIsNaN(y)))) {
    if (x == 0.0) {
      r = 0.0;
    } else {
      if (y != 0.0) {
        r = muDoubleScalarRem(x, y);
        if (r == 0.0) {
          r = 0.0;
        } else {
          if (x < 0.0) {
            r += y;
          }
        }
      }
    }
  } else {
    if (y != 0.0) {
      r = rtNaN;
    }
  }

  return r;
}

/* End of code generation (mod.c) */
