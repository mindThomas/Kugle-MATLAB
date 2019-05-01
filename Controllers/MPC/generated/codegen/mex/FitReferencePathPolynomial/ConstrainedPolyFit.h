/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ConstrainedPolyFit.h
 *
 * Code generation for function 'ConstrainedPolyFit'
 *
 */

#ifndef CONSTRAINEDPOLYFIT_H
#define CONSTRAINEDPOLYFIT_H

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
extern void ConstrainedPolyFit(const emlrtStack *sp, const emxArray_real_T *t,
  const emxArray_real_T *data, real_T order, emxArray_real_T *polyCoeffs);
extern void b_ConstrainedPolyFit(const emlrtStack *sp, const emxArray_real_T *t,
  const emxArray_real_T *data, real_T order, emxArray_real_T *polyCoeffs);
extern void c_ConstrainedPolyFit(const emlrtStack *sp, const real_T t[100],
  const real_T data[100], real_T order, emxArray_real_T *polyCoeffs);

#endif

/* End of code generation (ConstrainedPolyFit.h) */
