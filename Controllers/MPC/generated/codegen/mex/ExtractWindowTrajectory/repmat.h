/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * repmat.h
 *
 * Code generation for function 'repmat'
 *
 */

#ifndef REPMAT_H
#define REPMAT_H

/* Include files */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include "rtwtypes.h"
#include "ExtractWindowTrajectory_types.h"

/* Function Declarations */
extern void b_repmat(const emlrtStack *sp, const emxArray_real_T *a, const
                     real_T varargin_1[2], emxArray_real_T *b);
extern void repmat(const emlrtStack *sp, const real_T a[2], const real_T
                   varargin_1[2], emxArray_real_T *b);

#endif

/* End of code generation (repmat.h) */
