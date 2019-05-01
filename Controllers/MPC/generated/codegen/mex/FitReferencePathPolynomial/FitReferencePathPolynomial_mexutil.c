/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * FitReferencePathPolynomial_mexutil.c
 *
 * Code generation for function 'FitReferencePathPolynomial_mexutil'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "FitReferencePathPolynomial.h"
#include "FitReferencePathPolynomial_mexutil.h"

/* Function Definitions */
void l_error(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = b;
  emlrtCallMATLABR2012b(sp, 0, NULL, 1, &pArray, "error", true, location);
}

/* End of code generation (FitReferencePathPolynomial_mexutil.c) */
