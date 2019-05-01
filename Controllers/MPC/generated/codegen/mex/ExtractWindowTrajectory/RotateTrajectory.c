/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * RotateTrajectory.c
 *
 * Code generation for function 'RotateTrajectory'
 *
 */

/* Include files */
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "ExtractWindowTrajectory.h"
#include "RotateTrajectory.h"
#include "ExtractWindowTrajectory_emxutil.h"
#include "blas.h"

/* Variable Definitions */
static emlrtRSInfo y_emlrtRSI = { 5,   /* lineNo */
  "RotateTrajectory",                  /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\RotateTrajectory.m"/* pathName */
};

static emlrtRSInfo ab_emlrtRSI = { 52, /* lineNo */
  "eml_mtimes_helper",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_helper.m"/* pathName */
};

static emlrtRSInfo cb_emlrtRSI = { 118,/* lineNo */
  "mtimes",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\mtimes.m"/* pathName */
};

static emlrtRTEInfo n_emlrtRTEI = { 1, /* lineNo */
  26,                                  /* colNo */
  "RotateTrajectory",                  /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\RotateTrajectory.m"/* pName */
};

static emlrtRTEInfo o_emlrtRTEI = { 118,/* lineNo */
  13,                                  /* colNo */
  "mtimes",                            /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\mtimes.m"/* pName */
};

/* Function Definitions */
void RotateTrajectory(const emlrtStack *sp, const emxArray_real_T *trajectory,
                      real_T rotation, emxArray_real_T *rotatedPoints)
{
  emxArray_real_T *y;
  emxArray_real_T *b;
  real_T R[4];
  int32_T i3;
  int32_T loop_ub;
  int32_T i4;
  char_T TRANSA;
  char_T TRANSB;
  real_T alpha1;
  real_T beta1;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  emxInit_real_T(sp, &y, 2, &n_emlrtRTEI, true);
  emxInit_real_T(sp, &b, 2, &n_emlrtRTEI, true);
  R[0] = muDoubleScalarCos(rotation);
  R[2] = muDoubleScalarSin(rotation);
  R[1] = -muDoubleScalarSin(rotation);
  R[3] = muDoubleScalarCos(rotation);
  st.site = &y_emlrtRSI;
  i3 = b->size[0] * b->size[1];
  b->size[0] = 2;
  b->size[1] = trajectory->size[0];
  emxEnsureCapacity_real_T(&st, b, i3, &n_emlrtRTEI);
  loop_ub = trajectory->size[0];
  for (i3 = 0; i3 < loop_ub; i3++) {
    for (i4 = 0; i4 < 2; i4++) {
      b->data[i4 + b->size[0] * i3] = trajectory->data[i3 + trajectory->size[0] *
        i4];
    }
  }

  b_st.site = &ab_emlrtRSI;
  if (b->size[1] == 0) {
    i3 = y->size[0] * y->size[1];
    y->size[0] = 2;
    y->size[1] = 0;
    emxEnsureCapacity_real_T(&b_st, y, i3, &n_emlrtRTEI);
  } else {
    c_st.site = &cb_emlrtRSI;
    TRANSA = 'N';
    TRANSB = 'N';
    alpha1 = 1.0;
    beta1 = 0.0;
    m_t = (ptrdiff_t)2;
    n_t = (ptrdiff_t)b->size[1];
    k_t = (ptrdiff_t)2;
    lda_t = (ptrdiff_t)2;
    ldb_t = (ptrdiff_t)2;
    ldc_t = (ptrdiff_t)2;
    i3 = y->size[0] * y->size[1];
    y->size[0] = 2;
    y->size[1] = b->size[1];
    emxEnsureCapacity_real_T(&c_st, y, i3, &o_emlrtRTEI);
    dgemm(&TRANSA, &TRANSB, &m_t, &n_t, &k_t, &alpha1, &R[0], &lda_t, &b->data[0],
          &ldb_t, &beta1, &y->data[0], &ldc_t);
  }

  emxFree_real_T(&b_st, &b);
  i3 = rotatedPoints->size[0] * rotatedPoints->size[1];
  rotatedPoints->size[0] = y->size[1];
  rotatedPoints->size[1] = 2;
  emxEnsureCapacity_real_T(sp, rotatedPoints, i3, &n_emlrtRTEI);
  for (i3 = 0; i3 < 2; i3++) {
    loop_ub = y->size[1];
    for (i4 = 0; i4 < loop_ub; i4++) {
      rotatedPoints->data[i4 + rotatedPoints->size[0] * i3] = y->data[i3 +
        y->size[0] * i4];
    }
  }

  emxFree_real_T(sp, &y);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (RotateTrajectory.c) */
