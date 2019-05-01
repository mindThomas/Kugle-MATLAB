/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * assertValidSizeArg.c
 *
 * Code generation for function 'assertValidSizeArg'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "ExtractWindowTrajectory.h"
#include "assertValidSizeArg.h"

/* Variable Definitions */
static emlrtRTEInfo fb_emlrtRTEI = { 61,/* lineNo */
  15,                                  /* colNo */
  "assertValidSizeArg",                /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\assertValidSizeArg.m"/* pName */
};

static emlrtRTEInfo gb_emlrtRTEI = { 46,/* lineNo */
  19,                                  /* colNo */
  "assertValidSizeArg",                /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\assertValidSizeArg.m"/* pName */
};

/* Function Definitions */
void assertValidSizeArg(const emlrtStack *sp, const real_T varargin_1[2])
{
  int32_T k;
  boolean_T exitg1;
  real_T n;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (varargin_1[k] != varargin_1[k]) {
      emlrtErrorWithMessageIdR2018a(sp, &gb_emlrtRTEI,
        "Coder:toolbox:eml_assert_valid_size_arg_invalidSizeVector",
        "Coder:toolbox:eml_assert_valid_size_arg_invalidSizeVector", 4, 12,
        MIN_int32_T, 12, MAX_int32_T);
      exitg1 = true;
    } else {
      k++;
    }
  }

  n = 1.0;
  for (k = 0; k < 2; k++) {
    if (varargin_1[k] <= 0.0) {
      n = 0.0;
    } else {
      n *= varargin_1[k];
    }
  }

  if (!(n <= 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(sp, &fb_emlrtRTEI, "Coder:MATLAB:pmaxsize",
      "Coder:MATLAB:pmaxsize", 0);
  }
}

/* End of code generation (assertValidSizeArg.c) */
