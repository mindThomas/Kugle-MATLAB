/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * power.h
 *
 * Code generation for function 'power'
 *
 */

#ifndef POWER_H
#define POWER_H

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
extern void b_power(const emlrtStack *sp, const emxArray_real_T *a, real_T b,
                    emxArray_real_T *y);
extern void c_power(const emlrtStack *sp, real_T a, const emxArray_real_T *b,
                    emxArray_real_T *y);
extern void d_power(const emlrtStack *sp, const real_T a[100], real_T b, real_T
                    y[100]);
extern void power(const emlrtStack *sp, const emxArray_real_T *a,
                  emxArray_real_T *y);

#endif

/* End of code generation (power.h) */
