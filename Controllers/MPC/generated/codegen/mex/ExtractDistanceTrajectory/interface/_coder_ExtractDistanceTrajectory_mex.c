/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_ExtractDistanceTrajectory_mex.c
 *
 * Code generation for function '_coder_ExtractDistanceTrajectory_mex'
 *
 */

/* Include files */
#include "ExtractDistanceTrajectory.h"
#include "_coder_ExtractDistanceTrajectory_mex.h"
#include "ExtractDistanceTrajectory_terminate.h"
#include "_coder_ExtractDistanceTrajectory_api.h"
#include "ExtractDistanceTrajectory_initialize.h"
#include "ExtractDistanceTrajectory_data.h"

/* Function Declarations */
static void c_ExtractDistanceTrajectory_mex(int32_T nlhs, mxArray *plhs[4],
  int32_T nrhs, const mxArray *prhs[7]);

/* Function Definitions */
static void c_ExtractDistanceTrajectory_mex(int32_T nlhs, mxArray *plhs[4],
  int32_T nrhs, const mxArray *prhs[7])
{
  const mxArray *outputs[4];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 7) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 7, 4,
                        25, "ExtractDistanceTrajectory");
  }

  if (nlhs > 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 25,
                        "ExtractDistanceTrajectory");
  }

  /* Call the function. */
  ExtractDistanceTrajectory_api(prhs, nlhs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  ExtractDistanceTrajectory_terminate();
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  mexAtExit(ExtractDistanceTrajectory_atexit);

  /* Initialize the memory manager. */
  /* Module initialization. */
  ExtractDistanceTrajectory_initialize();

  /* Dispatch the entry-point. */
  c_ExtractDistanceTrajectory_mex(nlhs, plhs, nrhs, prhs);
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_ExtractDistanceTrajectory_mex.c) */
