/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ExtractWindowTrajectory_terminate.c
 *
 * Code generation for function 'ExtractWindowTrajectory_terminate'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "ExtractWindowTrajectory.h"
#include "ExtractWindowTrajectory_terminate.h"
#include "_coder_ExtractWindowTrajectory_mex.h"
#include "ExtractWindowTrajectory_data.h"

/* Function Definitions */
void ExtractWindowTrajectory_atexit(void)
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

void ExtractWindowTrajectory_terminate(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (ExtractWindowTrajectory_terminate.c) */
