/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sort1.c
 *
 * Code generation for function 'sort1'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "ExtractDistanceTrajectory.h"
#include "sort1.h"
#include "ExtractDistanceTrajectory_emxutil.h"
#include "sortIdx.h"
#include "eml_int_forloop_overflow_check.h"
#include "ExtractDistanceTrajectory_data.h"

/* Variable Definitions */
static emlrtRSInfo fb_emlrtRSI = { 76, /* lineNo */
  "sort",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m"/* pathName */
};

static emlrtRSInfo gb_emlrtRSI = { 79, /* lineNo */
  "sort",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m"/* pathName */
};

static emlrtRSInfo hb_emlrtRSI = { 81, /* lineNo */
  "sort",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m"/* pathName */
};

static emlrtRSInfo ib_emlrtRSI = { 84, /* lineNo */
  "sort",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m"/* pathName */
};

static emlrtRSInfo jb_emlrtRSI = { 87, /* lineNo */
  "sort",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m"/* pathName */
};

static emlrtRSInfo kb_emlrtRSI = { 90, /* lineNo */
  "sort",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m"/* pathName */
};

static emlrtRTEInfo o_emlrtRTEI = { 1, /* lineNo */
  20,                                  /* colNo */
  "sort",                              /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m"/* pName */
};

static emlrtRTEInfo r_emlrtRTEI = { 56,/* lineNo */
  1,                                   /* colNo */
  "sort",                              /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m"/* pName */
};

/* Function Definitions */
void sort(const emlrtStack *sp, emxArray_real_T *x, emxArray_int32_T *idx)
{
  int32_T dim;
  int32_T i5;
  emxArray_real_T *vwork;
  int32_T j;
  int32_T vstride;
  int32_T k;
  emxArray_int32_T *iidx;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  dim = 2;
  if (x->size[0] != 1) {
    dim = 1;
  }

  if (dim <= 1) {
    i5 = x->size[0];
  } else {
    i5 = 1;
  }

  emxInit_real_T1(sp, &vwork, 1, &r_emlrtRTEI, true);
  j = vwork->size[0];
  vwork->size[0] = i5;
  emxEnsureCapacity_real_T1(sp, vwork, j, &o_emlrtRTEI);
  vstride = x->size[0];
  j = idx->size[0];
  idx->size[0] = vstride;
  emxEnsureCapacity_int32_T(sp, idx, j, &o_emlrtRTEI);
  st.site = &fb_emlrtRSI;
  vstride = 1;
  k = 1;
  while (k <= dim - 1) {
    vstride *= x->size[0];
    k = 2;
  }

  st.site = &gb_emlrtRSI;
  st.site = &hb_emlrtRSI;
  if ((!(1 > vstride)) && (vstride > 2147483646)) {
    b_st.site = &l_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  j = 0;
  emxInit_int32_T(sp, &iidx, 1, &o_emlrtRTEI, true);
  while (j + 1 <= vstride) {
    st.site = &ib_emlrtRSI;
    if ((!(1 > i5)) && (i5 > 2147483646)) {
      b_st.site = &l_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (k = 0; k < i5; k++) {
      vwork->data[k] = x->data[j + k * vstride];
    }

    st.site = &jb_emlrtRSI;
    sortIdx(&st, vwork, iidx);
    st.site = &kb_emlrtRSI;
    for (k = 0; k < i5; k++) {
      x->data[j + k * vstride] = vwork->data[k];
      idx->data[j + k * vstride] = iidx->data[k];
    }

    j++;
  }

  emxFree_int32_T(sp, &iidx);
  emxFree_real_T(sp, &vwork);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (sort1.c) */
