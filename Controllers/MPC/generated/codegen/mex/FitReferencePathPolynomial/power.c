/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * power.c
 *
 * Code generation for function 'power'
 *
 */

/* Include files */
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "FitReferencePathPolynomial.h"
#include "power.h"
#include "FitReferencePathPolynomial_emxutil.h"
#include "error.h"
#include "eml_int_forloop_overflow_check.h"
#include "scalexpAlloc.h"
#include "FitReferencePathPolynomial_data.h"

/* Variable Definitions */
static emlrtRSInfo u_emlrtRSI = { 49,  /* lineNo */
  "power",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\ops\\power.m"/* pathName */
};

static emlrtRSInfo v_emlrtRSI = { 58,  /* lineNo */
  "power",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\ops\\power.m"/* pathName */
};

static emlrtRSInfo w_emlrtRSI = { 61,  /* lineNo */
  "power",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\ops\\power.m"/* pathName */
};

static emlrtRSInfo x_emlrtRSI = { 45,  /* lineNo */
  "applyBinaryScalarFunction",         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\applyBinaryScalarFunction.m"/* pathName */
};

static emlrtRSInfo y_emlrtRSI = { 65,  /* lineNo */
  "applyBinaryScalarFunction",         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\applyBinaryScalarFunction.m"/* pathName */
};

static emlrtRSInfo ab_emlrtRSI = { 189,/* lineNo */
  "applyBinaryScalarFunction",         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\applyBinaryScalarFunction.m"/* pathName */
};

static emlrtRSInfo xb_emlrtRSI = { 60, /* lineNo */
  "power",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\ops\\power.m"/* pathName */
};

static emlrtRSInfo yb_emlrtRSI = { 80, /* lineNo */
  "power",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\ops\\power.m"/* pathName */
};

static emlrtRSInfo fc_emlrtRSI = { 175,/* lineNo */
  "applyBinaryScalarFunction",         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\applyBinaryScalarFunction.m"/* pathName */
};

static emlrtRSInfo gc_emlrtRSI = { 73, /* lineNo */
  "power",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\ops\\power.m"/* pathName */
};

static emlrtRTEInfo k_emlrtRTEI = { 19,/* lineNo */
  24,                                  /* colNo */
  "scalexpAllocNoCheck",               /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\scalexpAllocNoCheck.m"/* pName */
};

static emlrtRTEInfo l_emlrtRTEI = { 45,/* lineNo */
  6,                                   /* colNo */
  "applyBinaryScalarFunction",         /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\applyBinaryScalarFunction.m"/* pName */
};

static emlrtRTEInfo m_emlrtRTEI = { 58,/* lineNo */
  5,                                   /* colNo */
  "power",                             /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\ops\\power.m"/* pName */
};

static emlrtRTEInfo n_emlrtRTEI = { 1, /* lineNo */
  14,                                  /* colNo */
  "power",                             /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\ops\\power.m"/* pName */
};

static emlrtRTEInfo pb_emlrtRTEI = { 17,/* lineNo */
  19,                                  /* colNo */
  "scalexpAlloc",                      /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\scalexpAlloc.m"/* pName */
};

/* Function Definitions */
void b_power(const emlrtStack *sp, const emxArray_real_T *a, real_T b,
             emxArray_real_T *y)
{
  emxArray_real_T *z;
  emxArray_real_T *b_z;
  uint32_T a_idx_0;
  int32_T k;
  uint32_T b_a_idx_0;
  boolean_T overflow;
  boolean_T p;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  emxInit_real_T1(sp, &z, 1, &n_emlrtRTEI, true);
  emxInit_real_T1(sp, &b_z, 1, &n_emlrtRTEI, true);
  st.site = &u_emlrtRSI;
  b_st.site = &v_emlrtRSI;
  c_st.site = &x_emlrtRSI;
  a_idx_0 = (uint32_T)a->size[0];
  k = b_z->size[0];
  b_z->size[0] = (int32_T)a_idx_0;
  emxEnsureCapacity_real_T(&c_st, b_z, k, &k_emlrtRTEI);
  a_idx_0 = (uint32_T)a->size[0];
  b_a_idx_0 = (uint32_T)a->size[0];
  k = z->size[0];
  z->size[0] = (int32_T)b_a_idx_0;
  emxEnsureCapacity_real_T(&c_st, z, k, &l_emlrtRTEI);
  if (!dimagree(z, a)) {
    emlrtErrorWithMessageIdR2018a(&c_st, &pb_emlrtRTEI, "MATLAB:dimagree",
      "MATLAB:dimagree", 0);
  }

  emxFree_real_T(&c_st, &z);
  b_a_idx_0 = (uint32_T)a->size[0];
  k = y->size[0];
  y->size[0] = (int32_T)b_a_idx_0;
  emxEnsureCapacity_real_T(&b_st, y, k, &m_emlrtRTEI);
  c_st.site = &y_emlrtRSI;
  d_st.site = &ab_emlrtRSI;
  overflow = ((!(1 > b_z->size[0])) && (b_z->size[0] > 2147483646));
  emxFree_real_T(&d_st, &b_z);
  if (overflow) {
    e_st.site = &t_emlrtRSI;
    check_forloop_overflow_error(&e_st);
  }

  for (k = 0; k < (int32_T)a_idx_0; k++) {
    y->data[k] = muDoubleScalarPower(a->data[k], b);
  }

  b_st.site = &xb_emlrtRSI;
  if (a->size[0] == 1) {
    if ((a->data[0] < 0.0) && (!muDoubleScalarIsNaN(b)) && (muDoubleScalarFloor
         (b) != b)) {
      p = true;
    } else {
      p = false;
    }
  } else if ((!muDoubleScalarIsNaN(b)) && (muDoubleScalarFloor(b) != b)) {
    c_st.site = &yb_emlrtRSI;
    d_st.site = &ac_emlrtRSI;
    p = false;
    e_st.site = &bc_emlrtRSI;
    overflow = ((!(1 > a->size[0])) && (a->size[0] > 2147483646));
    if (overflow) {
      f_st.site = &t_emlrtRSI;
      check_forloop_overflow_error(&f_st);
    }

    for (k = 1; k <= a->size[0]; k++) {
      if (p || (a->data[k - 1] < 0.0)) {
        p = true;
      } else {
        p = false;
      }
    }
  } else {
    p = false;
  }

  if (p) {
    b_st.site = &w_emlrtRSI;
    error(&b_st);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void c_power(const emlrtStack *sp, real_T a, const emxArray_real_T *b,
             emxArray_real_T *y)
{
  emxArray_real_T *z;
  emxArray_real_T *b_z;
  int32_T k;
  int32_T unnamed_idx_1;
  boolean_T overflow;
  boolean_T p;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  emxInit_real_T(sp, &z, 2, &n_emlrtRTEI, true);
  emxInit_real_T(sp, &b_z, 2, &n_emlrtRTEI, true);
  st.site = &u_emlrtRSI;
  b_st.site = &v_emlrtRSI;
  c_st.site = &x_emlrtRSI;
  k = b_z->size[0] * b_z->size[1];
  b_z->size[1] = b->size[1];
  emxEnsureCapacity_real_T1(&c_st, b_z, k, &k_emlrtRTEI);
  unnamed_idx_1 = b->size[1];
  k = z->size[0] * z->size[1];
  z->size[0] = 1;
  z->size[1] = b->size[1];
  emxEnsureCapacity_real_T1(&c_st, z, k, &l_emlrtRTEI);
  if (!b_dimagree(z, b)) {
    emlrtErrorWithMessageIdR2018a(&c_st, &pb_emlrtRTEI, "MATLAB:dimagree",
      "MATLAB:dimagree", 0);
  }

  emxFree_real_T(&c_st, &z);
  k = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = b->size[1];
  emxEnsureCapacity_real_T1(&b_st, y, k, &m_emlrtRTEI);
  c_st.site = &y_emlrtRSI;
  d_st.site = &fc_emlrtRSI;
  overflow = ((!(1 > b_z->size[1])) && (b_z->size[1] > 2147483646));
  emxFree_real_T(&d_st, &b_z);
  if (overflow) {
    e_st.site = &t_emlrtRSI;
    check_forloop_overflow_error(&e_st);
  }

  for (k = 0; k < unnamed_idx_1; k++) {
    y->data[k] = muDoubleScalarPower(a, b->data[k]);
  }

  b_st.site = &xb_emlrtRSI;
  if (a < 0.0) {
    c_st.site = &gc_emlrtRSI;
    d_st.site = &ac_emlrtRSI;
    p = false;
    e_st.site = &bc_emlrtRSI;
    overflow = ((!(1 > b->size[1])) && (b->size[1] > 2147483646));
    if (overflow) {
      f_st.site = &t_emlrtRSI;
      check_forloop_overflow_error(&f_st);
    }

    for (k = 0; k < b->size[1]; k++) {
      if (p || ((!muDoubleScalarIsNaN(b->data[k])) && (muDoubleScalarFloor
            (b->data[k]) != b->data[k]))) {
        p = true;
      } else {
        p = false;
      }
    }
  } else {
    p = false;
  }

  if (p) {
    b_st.site = &w_emlrtRSI;
    error(&b_st);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void d_power(const emlrtStack *sp, const real_T a[100], real_T b, real_T y[100])
{
  int32_T k;
  boolean_T p;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &u_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  for (k = 0; k < 100; k++) {
    y[k] = muDoubleScalarPower(a[k], b);
  }

  if ((!muDoubleScalarIsNaN(b)) && (muDoubleScalarFloor(b) != b)) {
    p = false;
    for (k = 0; k < 100; k++) {
      if (p || (a[k] < 0.0)) {
        p = true;
      } else {
        p = false;
      }
    }
  } else {
    p = false;
  }

  if (p) {
    b_st.site = &w_emlrtRSI;
    error(&b_st);
  }
}

void power(const emlrtStack *sp, const emxArray_real_T *a, emxArray_real_T *y)
{
  emxArray_real_T *z;
  emxArray_real_T *b_z;
  uint32_T a_idx_0;
  int32_T k;
  uint32_T b_a_idx_0;
  boolean_T overflow;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  emxInit_real_T1(sp, &z, 1, &n_emlrtRTEI, true);
  emxInit_real_T1(sp, &b_z, 1, &n_emlrtRTEI, true);
  st.site = &u_emlrtRSI;
  b_st.site = &v_emlrtRSI;
  c_st.site = &x_emlrtRSI;
  a_idx_0 = (uint32_T)a->size[0];
  k = b_z->size[0];
  b_z->size[0] = (int32_T)a_idx_0;
  emxEnsureCapacity_real_T(&c_st, b_z, k, &k_emlrtRTEI);
  a_idx_0 = (uint32_T)a->size[0];
  b_a_idx_0 = (uint32_T)a->size[0];
  k = z->size[0];
  z->size[0] = (int32_T)b_a_idx_0;
  emxEnsureCapacity_real_T(&c_st, z, k, &l_emlrtRTEI);
  if (!dimagree(z, a)) {
    emlrtErrorWithMessageIdR2018a(&c_st, &pb_emlrtRTEI, "MATLAB:dimagree",
      "MATLAB:dimagree", 0);
  }

  emxFree_real_T(&c_st, &z);
  b_a_idx_0 = (uint32_T)a->size[0];
  k = y->size[0];
  y->size[0] = (int32_T)b_a_idx_0;
  emxEnsureCapacity_real_T(&b_st, y, k, &m_emlrtRTEI);
  c_st.site = &y_emlrtRSI;
  d_st.site = &ab_emlrtRSI;
  overflow = ((!(1 > b_z->size[0])) && (b_z->size[0] > 2147483646));
  emxFree_real_T(&d_st, &b_z);
  if (overflow) {
    e_st.site = &t_emlrtRSI;
    check_forloop_overflow_error(&e_st);
  }

  for (k = 0; k < (int32_T)a_idx_0; k++) {
    y->data[k] = a->data[k] * a->data[k];
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (power.c) */
