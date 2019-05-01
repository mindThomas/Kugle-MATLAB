/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * EvaluatePolynomial.h
 *
 * Code generation for function 'EvaluatePolynomial'
 *
 */

#ifndef EVALUATEPOLYNOMIAL_H
#define EVALUATEPOLYNOMIAL_H

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
extern real_T EvaluatePolynomial(const emlrtStack *sp, const emxArray_real_T
  *coeff, real_T x);
extern void b_EvaluatePolynomial(const emlrtStack *sp, const emxArray_real_T
  *coeff, const real_T x[100], real_T y[100]);
extern void c_EvaluatePolynomial(const emlrtStack *sp, const emxArray_real_T
  *coeff, const emxArray_real_T *x, emxArray_real_T *y);

#endif

/* End of code generation (EvaluatePolynomial.h) */
