/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ExtractDistanceTrajectory_terminate.c
 *
 * Code generation for function 'ExtractDistanceTrajectory_terminate'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "ExtractDistanceTrajectory.h"
#include "ExtractDistanceTrajectory_terminate.h"
#include "_coder_ExtractDistanceTrajectory_mex.h"
#include "ExtractDistanceTrajectory_data.h"

/* Function Definitions */
void ExtractDistanceTrajectory_atexit(void)
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

void ExtractDistanceTrajectory_terminate(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (ExtractDistanceTrajectory_terminate.c) */
