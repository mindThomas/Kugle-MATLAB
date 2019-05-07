/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * error.c
 *
 * Code generation for function 'error'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "ExtractDistanceTrajectory.h"
#include "error.h"

/* Variable Definitions */
static emlrtRTEInfo cb_emlrtRTEI = { 19,/* lineNo */
  5,                                   /* colNo */
  "error",                             /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\shared\\coder\\coder\\+coder\\+internal\\error.m"/* pName */
};

/* Function Definitions */
void error(const emlrtStack *sp)
{
  static const char_T varargin_1[4] = { 's', 'q', 'r', 't' };

  emlrtErrorWithMessageIdR2018a(sp, &cb_emlrtRTEI,
    "Coder:toolbox:ElFunDomainError", "Coder:toolbox:ElFunDomainError", 3, 4, 4,
    varargin_1);
}

/* End of code generation (error.c) */
