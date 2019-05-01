/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * FitReferencePathPolynomial_initialize.c
 *
 * Code generation for function 'FitReferencePathPolynomial_initialize'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "FitReferencePathPolynomial.h"
#include "FitReferencePathPolynomial_initialize.h"
#include "_coder_FitReferencePathPolynomial_mex.h"
#include "FitReferencePathPolynomial_data.h"

/* Function Definitions */
void FitReferencePathPolynomial_initialize(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (FitReferencePathPolynomial_initialize.c) */
