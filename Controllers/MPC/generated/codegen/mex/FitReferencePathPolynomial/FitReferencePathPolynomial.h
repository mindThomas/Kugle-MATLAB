/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * FitReferencePathPolynomial.h
 *
 * Code generation for function 'FitReferencePathPolynomial'
 *
 */

#ifndef FITREFERENCEPATHPOLYNOMIAL_H
#define FITREFERENCEPATHPOLYNOMIAL_H

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
extern void FitReferencePathPolynomial(const emlrtStack *sp, const
  emxArray_real_T *WindowTrajectoryPoints, real_T approximation_order, real_T
  velocity, real_T ts, real_T N, emxArray_real_T *TrajectoryPoints,
  emxArray_real_T *coeff_xs, emxArray_real_T *coeff_ys, real_T
  *windowTrajectoryLength, real_T *minDistancePoint);

#endif

/* End of code generation (FitReferencePathPolynomial.h) */
