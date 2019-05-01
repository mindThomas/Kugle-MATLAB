/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * diag.h
 *
 * Code generation for function 'diag'
 *
 */

#ifndef DIAG_H
#define DIAG_H

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
extern void b_diag(const emlrtStack *sp, const emxArray_real_T *v,
                   emxArray_real_T *d);
extern void diag(const emlrtStack *sp, const emxArray_real_T *v, emxArray_real_T
                 *d);

#endif

/* End of code generation (diag.h) */
