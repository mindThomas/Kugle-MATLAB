/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_FitReferencePathPolynomial_mex.c
 *
 * Code generation for function '_coder_FitReferencePathPolynomial_mex'
 *
 */

/* Include files */
#include "FitReferencePathPolynomial.h"
#include "_coder_FitReferencePathPolynomial_mex.h"
#include "FitReferencePathPolynomial_terminate.h"
#include "_coder_FitReferencePathPolynomial_api.h"
#include "FitReferencePathPolynomial_initialize.h"
#include "FitReferencePathPolynomial_data.h"

/* Function Declarations */
static void c_FitReferencePathPolynomial_me(int32_T nlhs, mxArray *plhs[5],
  int32_T nrhs, const mxArray *prhs[5]);

/* Function Definitions */
static void c_FitReferencePathPolynomial_me(int32_T nlhs, mxArray *plhs[5],
  int32_T nrhs, const mxArray *prhs[5])
{
  const mxArray *outputs[5];
  int32_T b_nlhs;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 5) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 5, 4,
                        26, "FitReferencePathPolynomial");
  }

  if (nlhs > 5) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 26,
                        "FitReferencePathPolynomial");
  }

  /* Call the function. */
  FitReferencePathPolynomial_api(prhs, nlhs, outputs);

  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    b_nlhs = 1;
  } else {
    b_nlhs = nlhs;
  }

  emlrtReturnArrays(b_nlhs, plhs, outputs);

  /* Module termination. */
  FitReferencePathPolynomial_terminate();
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  mexAtExit(FitReferencePathPolynomial_atexit);

  /* Initialize the memory manager. */
  /* Module initialization. */
  FitReferencePathPolynomial_initialize();

  /* Dispatch the entry-point. */
  c_FitReferencePathPolynomial_me(nlhs, plhs, nrhs, prhs);
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_FitReferencePathPolynomial_mex.c) */
