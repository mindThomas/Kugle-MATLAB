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
#include "ExtractDistanceTrajectory.h"
#include "RotateTrajectory.h"
#include "ExtractDistanceTrajectory_emxutil.h"
#include "blas.h"

/* Variable Definitions */
static emlrtRSInfo m_emlrtRSI = { 5,   /* lineNo */
  "RotateTrajectory",                  /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\RotateTrajectory.m"/* pathName */
};

static emlrtRSInfo n_emlrtRSI = { 52,  /* lineNo */
  "eml_mtimes_helper",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_helper.m"/* pathName */
};

static emlrtRSInfo p_emlrtRSI = { 118, /* lineNo */
  "mtimes",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\mtimes.m"/* pathName */
};

static emlrtRTEInfo i_emlrtRTEI = { 1, /* lineNo */
  26,                                  /* colNo */
  "RotateTrajectory",                  /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\RotateTrajectory.m"/* pName */
};

static emlrtRTEInfo j_emlrtRTEI = { 118,/* lineNo */
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
  int32_T i1;
  int32_T loop_ub;
  int32_T i2;
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
  emxInit_real_T(sp, &y, 2, &i_emlrtRTEI, true);
  emxInit_real_T(sp, &b, 2, &i_emlrtRTEI, true);
  R[0] = muDoubleScalarCos(rotation);
  R[2] = muDoubleScalarSin(rotation);
  R[1] = -muDoubleScalarSin(rotation);
  R[3] = muDoubleScalarCos(rotation);
  st.site = &m_emlrtRSI;
  i1 = b->size[0] * b->size[1];
  b->size[0] = 2;
  b->size[1] = trajectory->size[0];
  emxEnsureCapacity_real_T(&st, b, i1, &i_emlrtRTEI);
  loop_ub = trajectory->size[0];
  for (i1 = 0; i1 < loop_ub; i1++) {
    for (i2 = 0; i2 < 2; i2++) {
      b->data[i2 + b->size[0] * i1] = trajectory->data[i1 + trajectory->size[0] *
        i2];
    }
  }

  b_st.site = &n_emlrtRSI;
  if (b->size[1] == 0) {
    i1 = y->size[0] * y->size[1];
    y->size[0] = 2;
    y->size[1] = 0;
    emxEnsureCapacity_real_T(&b_st, y, i1, &i_emlrtRTEI);
  } else {
    c_st.site = &p_emlrtRSI;
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
    i1 = y->size[0] * y->size[1];
    y->size[0] = 2;
    y->size[1] = b->size[1];
    emxEnsureCapacity_real_T(&c_st, y, i1, &j_emlrtRTEI);
    dgemm(&TRANSA, &TRANSB, &m_t, &n_t, &k_t, &alpha1, &R[0], &lda_t, &b->data[0],
          &ldb_t, &beta1, &y->data[0], &ldc_t);
  }

  emxFree_real_T(&b_st, &b);
  i1 = rotatedPoints->size[0] * rotatedPoints->size[1];
  rotatedPoints->size[0] = y->size[1];
  rotatedPoints->size[1] = 2;
  emxEnsureCapacity_real_T(sp, rotatedPoints, i1, &i_emlrtRTEI);
  for (i1 = 0; i1 < 2; i1++) {
    loop_ub = y->size[1];
    for (i2 = 0; i2 < loop_ub; i2++) {
      rotatedPoints->data[i2 + rotatedPoints->size[0] * i1] = y->data[i1 +
        y->size[0] * i2];
    }
  }

  emxFree_real_T(sp, &y);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (RotateTrajectory.c) */
