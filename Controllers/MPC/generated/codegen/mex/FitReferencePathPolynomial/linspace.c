/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * linspace.c
 *
 * Code generation for function 'linspace'
 *
 */

/* Include files */
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "FitReferencePathPolynomial.h"
#include "linspace.h"

/* Function Definitions */
void linspace(real_T d2, real_T y[100])
{
  real_T delta1;
  int32_T k;
  y[99] = d2;
  y[0] = 0.0;
  if ((d2 < 0.0) && (muDoubleScalarAbs(d2) > 8.9884656743115785E+307)) {
    delta1 = d2 / 99.0;
    for (k = 0; k < 98; k++) {
      y[k + 1] = delta1 * (1.0 + (real_T)k);
    }
  } else {
    delta1 = d2 / 99.0;
    for (k = 0; k < 98; k++) {
      y[k + 1] = (1.0 + (real_T)k) * delta1;
    }
  }
}

/* End of code generation (linspace.c) */
