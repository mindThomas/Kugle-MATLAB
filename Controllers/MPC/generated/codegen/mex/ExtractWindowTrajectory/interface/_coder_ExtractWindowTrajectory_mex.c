/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_ExtractWindowTrajectory_mex.c
 *
 * Code generation for function '_coder_ExtractWindowTrajectory_mex'
 *
 */

/* Include files */
#include "ExtractWindowTrajectory.h"
#include "_coder_ExtractWindowTrajectory_mex.h"
#include "ExtractWindowTrajectory_terminate.h"
#include "_coder_ExtractWindowTrajectory_api.h"
#include "ExtractWindowTrajectory_initialize.h"
#include "ExtractWindowTrajectory_data.h"

/* Function Declarations */
static void c_ExtractWindowTrajectory_mexFu(int32_T nlhs, mxArray *plhs[3],
  int32_T nrhs, const mxArray *prhs[9]);

/* Function Definitions */
static void c_ExtractWindowTrajectory_mexFu(int32_T nlhs, mxArray *plhs[3],
  int32_T nrhs, const mxArray *prhs[9])
{
  const mxArray *outputs[3];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 9) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 9, 4,
                        23, "ExtractWindowTrajectory");
  }

  if (nlhs > 3) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 23,
                        "ExtractWindowTrajectory");
  }

  /* Call the function. */
  ExtractWindowTrajectory_api(prhs, nlhs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  ExtractWindowTrajectory_terminate();
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  mexAtExit(ExtractWindowTrajectory_atexit);

  /* Initialize the memory manager. */
  /* Module initialization. */
  ExtractWindowTrajectory_initialize();

  /* Dispatch the entry-point. */
  c_ExtractWindowTrajectory_mexFu(nlhs, plhs, nrhs, prhs);
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_ExtractWindowTrajectory_mex.c) */
