/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * FitReferencePathPolynomial_terminate.c
 *
 * Code generation for function 'FitReferencePathPolynomial_terminate'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "FitReferencePathPolynomial.h"
#include "FitReferencePathPolynomial_terminate.h"
#include "_coder_FitReferencePathPolynomial_mex.h"
#include "FitReferencePathPolynomial_data.h"

/* Function Definitions */
void FitReferencePathPolynomial_atexit(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

void FitReferencePathPolynomial_terminate(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (FitReferencePathPolynomial_terminate.c) */
