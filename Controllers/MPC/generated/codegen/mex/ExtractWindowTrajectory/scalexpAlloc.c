/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * scalexpAlloc.c
 *
 * Code generation for function 'scalexpAlloc'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "ExtractWindowTrajectory.h"
#include "scalexpAlloc.h"

/* Function Definitions */
boolean_T dimagree(const emxArray_real_T *z, const emxArray_real_T *varargin_1)
{
  boolean_T p;
  boolean_T b_p;
  int32_T k;
  boolean_T exitg1;
  int32_T i6;
  int32_T i7;
  p = true;
  b_p = true;
  k = 1;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    if (k <= 1) {
      i6 = z->size[0];
      i7 = varargin_1->size[0];
    } else {
      i6 = 1;
      i7 = 1;
    }

    if (i6 != i7) {
      b_p = false;
      exitg1 = true;
    } else {
      k++;
    }
  }

  if (!b_p) {
    p = false;
  }

  return p;
}

/* End of code generation (scalexpAlloc.c) */
