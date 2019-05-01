/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * FitReferencePathPolynomial_emxutil.h
 *
 * Code generation for function 'FitReferencePathPolynomial_emxutil'
 *
 */

#ifndef FITREFERENCEPATHPOLYNOMIAL_EMXUTIL_H
#define FITREFERENCEPATHPOLYNOMIAL_EMXUTIL_H

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
extern void emxEnsureCapacity_int32_T(const emlrtStack *sp, emxArray_int32_T
  *emxArray, int32_T oldNumel, const emlrtRTEInfo *srcLocation);
extern void emxEnsureCapacity_real_T(const emlrtStack *sp, emxArray_real_T
  *emxArray, int32_T oldNumel, const emlrtRTEInfo *srcLocation);
extern void emxEnsureCapacity_real_T1(const emlrtStack *sp, emxArray_real_T
  *emxArray, int32_T oldNumel, const emlrtRTEInfo *srcLocation);
extern void emxFreeMatrix_cell_wrap_1(const emlrtStack *sp, cell_wrap_1 pMatrix
  [2]);
extern void emxFree_int32_T(const emlrtStack *sp, emxArray_int32_T **pEmxArray);
extern void emxFree_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray);
extern void emxInitMatrix_cell_wrap_1(const emlrtStack *sp, cell_wrap_1 pMatrix
  [2], const emlrtRTEInfo *srcLocation, boolean_T doPush);
extern void emxInit_int32_T(const emlrtStack *sp, emxArray_int32_T **pEmxArray,
  int32_T numDimensions, const emlrtRTEInfo *srcLocation, boolean_T doPush);
extern void emxInit_real_T(const emlrtStack *sp, emxArray_real_T **pEmxArray,
  int32_T numDimensions, const emlrtRTEInfo *srcLocation, boolean_T doPush);
extern void emxInit_real_T1(const emlrtStack *sp, emxArray_real_T **pEmxArray,
  int32_T numDimensions, const emlrtRTEInfo *srcLocation, boolean_T doPush);

#endif

/* End of code generation (FitReferencePathPolynomial_emxutil.h) */
