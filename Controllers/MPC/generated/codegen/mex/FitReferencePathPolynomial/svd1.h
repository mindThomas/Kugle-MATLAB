/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * svd1.h
 *
 * Code generation for function 'svd1'
 *
 */

#ifndef SVD1_H
#define SVD1_H

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

/* Type Definitions */

/* Function Declarations */
extern void b_svd(const emlrtStack *sp, const emxArray_real_T *A,
                  emxArray_real_T *U, emxArray_real_T *s, emxArray_real_T *V);

#endif

/* End of code generation (svd1.h) */
