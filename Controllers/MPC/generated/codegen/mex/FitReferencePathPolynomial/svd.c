/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * svd.c
 *
 * Code generation for function 'svd'
 *
 */

/* Include files */
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "FitReferencePathPolynomial.h"
#include "svd.h"
#include "FitReferencePathPolynomial_emxutil.h"
#include "svd1.h"
#include "eml_int_forloop_overflow_check.h"
#include "FitReferencePathPolynomial_data.h"

/* Variable Definitions */
static emlrtRSInfo pc_emlrtRSI = { 12, /* lineNo */
  "svd",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\matfun\\svd.m"/* pathName */
};

static emlrtRSInfo qc_emlrtRSI = { 25, /* lineNo */
  "svd",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\matfun\\svd.m"/* pathName */
};

static emlrtRSInfo rc_emlrtRSI = { 33, /* lineNo */
  "svd",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\matfun\\svd.m"/* pathName */
};

static emlrtRSInfo sc_emlrtRSI = { 28, /* lineNo */
  "anyNonFinite",                      /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\anyNonFinite.m"/* pathName */
};

static emlrtRTEInfo y_emlrtRTEI = { 1, /* lineNo */
  20,                                  /* colNo */
  "svd",                               /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\matfun\\svd.m"/* pName */
};

static emlrtRTEInfo ab_emlrtRTEI = { 25,/* lineNo */
  12,                                  /* colNo */
  "svd",                               /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\matfun\\svd.m"/* pName */
};

/* Function Definitions */
void svd(const emlrtStack *sp, const emxArray_real_T *A, emxArray_real_T *U,
         emxArray_real_T *S, emxArray_real_T *V)
{
  int32_T nx;
  boolean_T p;
  int32_T k;
  emxArray_real_T *s;
  emxArray_real_T *r8;
  uint32_T uv0[2];
  emxArray_real_T *U1;
  emxArray_real_T *V1;
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
  st.site = &pc_emlrtRSI;
  b_st.site = &sc_emlrtRSI;
  c_st.site = &ac_emlrtRSI;
  nx = A->size[0] * A->size[1];
  p = true;
  d_st.site = &bc_emlrtRSI;
  if ((!(1 > nx)) && (nx > 2147483646)) {
    e_st.site = &t_emlrtRSI;
    check_forloop_overflow_error(&e_st);
  }

  for (k = 0; k < nx; k++) {
    if (p && ((!muDoubleScalarIsInf(A->data[k])) && (!muDoubleScalarIsNaN
          (A->data[k])))) {
      p = true;
    } else {
      p = false;
    }
  }

  emxInit_real_T1(sp, &s, 1, &ab_emlrtRTEI, true);
  if (p) {
    st.site = &qc_emlrtRSI;
    b_svd(&st, A, U, s, V);
  } else {
    for (nx = 0; nx < 2; nx++) {
      uv0[nx] = (uint32_T)A->size[nx];
    }

    emxInit_real_T(sp, &r8, 2, &y_emlrtRTEI, true);
    nx = r8->size[0] * r8->size[1];
    r8->size[0] = (int32_T)uv0[0];
    r8->size[1] = (int32_T)uv0[1];
    emxEnsureCapacity_real_T1(sp, r8, nx, &y_emlrtRTEI);
    k = (int32_T)uv0[0] * (int32_T)uv0[1];
    for (nx = 0; nx < k; nx++) {
      r8->data[nx] = 0.0;
    }

    emxInit_real_T(sp, &U1, 2, &y_emlrtRTEI, true);
    emxInit_real_T(sp, &V1, 2, &y_emlrtRTEI, true);
    st.site = &rc_emlrtRSI;
    b_svd(&st, r8, U1, s, V1);
    nx = U->size[0] * U->size[1];
    U->size[0] = U1->size[0];
    U->size[1] = U1->size[1];
    emxEnsureCapacity_real_T1(sp, U, nx, &y_emlrtRTEI);
    k = U1->size[0] * U1->size[1];
    emxFree_real_T(sp, &r8);
    emxFree_real_T(sp, &U1);
    for (nx = 0; nx < k; nx++) {
      U->data[nx] = rtNaN;
    }

    k = s->size[0];
    nx = s->size[0];
    s->size[0] = k;
    emxEnsureCapacity_real_T(sp, s, nx, &y_emlrtRTEI);
    for (nx = 0; nx < k; nx++) {
      s->data[nx] = rtNaN;
    }

    nx = V->size[0] * V->size[1];
    V->size[0] = V1->size[0];
    V->size[1] = V1->size[1];
    emxEnsureCapacity_real_T1(sp, V, nx, &y_emlrtRTEI);
    k = V1->size[0] * V1->size[1];
    emxFree_real_T(sp, &V1);
    for (nx = 0; nx < k; nx++) {
      V->data[nx] = rtNaN;
    }
  }

  nx = S->size[0] * S->size[1];
  S->size[0] = U->size[1];
  S->size[1] = V->size[1];
  emxEnsureCapacity_real_T1(sp, S, nx, &y_emlrtRTEI);
  k = U->size[1] * V->size[1];
  for (nx = 0; nx < k; nx++) {
    S->data[nx] = 0.0;
  }

  for (k = 0; k < s->size[0]; k++) {
    S->data[k + S->size[0] * k] = s->data[k];
  }

  emxFree_real_T(sp, &s);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (svd.c) */
