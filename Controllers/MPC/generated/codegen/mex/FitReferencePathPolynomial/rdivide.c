/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * rdivide.c
 *
 * Code generation for function 'rdivide'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "FitReferencePathPolynomial.h"
#include "rdivide.h"
#include "FitReferencePathPolynomial_emxutil.h"

/* Variable Definitions */
static emlrtRTEInfo gb_emlrtRTEI = { 1,/* lineNo */
  14,                                  /* colNo */
  "rdivide",                           /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\ops\\rdivide.m"/* pName */
};

/* Function Definitions */
void rdivide(const emlrtStack *sp, const emxArray_real_T *y, emxArray_real_T *z)
{
  int32_T i9;
  int32_T loop_ub;
  i9 = z->size[0];
  z->size[0] = y->size[0];
  emxEnsureCapacity_real_T(sp, z, i9, &gb_emlrtRTEI);
  loop_ub = y->size[0];
  for (i9 = 0; i9 < loop_ub; i9++) {
    z->data[i9] = 1.0 / y->data[i9];
  }
}

/* End of code generation (rdivide.c) */
