/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ConstrainedLeastSquares.h
 *
 * Code generation for function 'ConstrainedLeastSquares'
 *
 */

#ifndef CONSTRAINEDLEASTSQUARES_H
#define CONSTRAINEDLEASTSQUARES_H

/* Include files */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include "rtwtypes.h"
#include "FitReferencePathPolynomial_types.h"

/* Function Declarations */
extern void ConstrainedLeastSquares(const emlrtStack *sp, const emxArray_real_T *
  A, const emxArray_real_T *b, const emxArray_real_T *Aeq, const real_T
  beq_data[], const int32_T beq_size[1], real_T lambda, emxArray_real_T *x);
extern void b_ConstrainedLeastSquares(const emlrtStack *sp, const
  emxArray_real_T *A, const emxArray_real_T *b, const emxArray_real_T *Aeq,
  const real_T beq[2], real_T lambda, emxArray_real_T *x);
extern void c_ConstrainedLeastSquares(const emlrtStack *sp, const
  emxArray_real_T *A, const real_T b[100], const emxArray_real_T *Aeq, const
  real_T beq_data[], const int32_T beq_size[1], emxArray_real_T *x);

#endif

/* End of code generation (ConstrainedLeastSquares.h) */
