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
#include "ExtractWindowTrajectory.h"
#include "sort1.h"
#include "ExtractWindowTrajectory_emxutil.h"
#include "sortIdx.h"
#include "eml_int_forloop_overflow_check.h"
#include "ExtractWindowTrajectory_data.h"

/* Variable Definitions */
static emlrtRSInfo cc_emlrtRSI = { 76, /* lineNo */
  "sort",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m"/* pathName */
};

static emlrtRSInfo dc_emlrtRSI = { 79, /* lineNo */
  "sort",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m"/* pathName */
};

static emlrtRSInfo ec_emlrtRSI = { 81, /* lineNo */
  "sort",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m"/* pathName */
};

static emlrtRSInfo fc_emlrtRSI = { 84, /* lineNo */
  "sort",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m"/* pathName */
};

static emlrtRSInfo gc_emlrtRSI = { 87, /* lineNo */
  "sort",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m"/* pathName */
};

static emlrtRSInfo hc_emlrtRSI = { 90, /* lineNo */
  "sort",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m"/* pathName */
};

static emlrtRTEInfo v_emlrtRTEI = { 1, /* lineNo */
  20,                                  /* colNo */
  "sort",                              /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m"/* pName */
};

static emlrtRTEInfo y_emlrtRTEI = { 56,/* lineNo */
  1,                                   /* colNo */
  "sort",                              /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sort.m"/* pName */
};

/* Function Definitions */
void sort(const emlrtStack *sp, emxArray_real_T *x)
{
  int32_T dim;
  int32_T i9;
  emxArray_real_T *vwork;
  int32_T j;
  int32_T vstride;
  int32_T k;
  emxArray_int32_T *od_emlrtRSI;
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
    i9 = x->size[0];
  } else {
    i9 = 1;
  }

  emxInit_real_T1(sp, &vwork, 1, &y_emlrtRTEI, true);
  j = vwork->size[0];
  vwork->size[0] = i9;
  emxEnsureCapacity_real_T1(sp, vwork, j, &v_emlrtRTEI);
  st.site = &cc_emlrtRSI;
  vstride = 1;
  k = 1;
  while (k <= dim - 1) {
    vstride *= x->size[0];
    k = 2;
  }

  st.site = &dc_emlrtRSI;
  st.site = &ec_emlrtRSI;
  if ((!(1 > vstride)) && (vstride > 2147483646)) {
    b_st.site = &x_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  j = 0;
  emxInit_int32_T(sp, &od_emlrtRSI, 1, &v_emlrtRTEI, true);
  while (j + 1 <= vstride) {
    st.site = &fc_emlrtRSI;
    if ((!(1 > i9)) && (i9 > 2147483646)) {
      b_st.site = &x_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (k = 0; k < i9; k++) {
      vwork->data[k] = x->data[j + k * vstride];
    }

    st.site = &gc_emlrtRSI;
    sortIdx(&st, vwork, od_emlrtRSI);
    st.site = &hc_emlrtRSI;
    for (k = 0; k < i9; k++) {
      x->data[j + k * vstride] = vwork->data[k];
    }

    j++;
  }

  emxFree_int32_T(sp, &od_emlrtRSI);
  emxFree_real_T(sp, &vwork);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (sort1.c) */
