/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * EvaluatePolynomial.c
 *
 * Code generation for function 'EvaluatePolynomial'
 *
 */

/* Include files */
#include "mwmathutil.h"
#include <string.h>
#include "rt_nonfinite.h"
#include "FitReferencePathPolynomial.h"
#include "EvaluatePolynomial.h"
#include "FitReferencePathPolynomial_emxutil.h"
#include "power.h"
#include "FitReferencePathPolynomial_data.h"
#include "blas.h"

/* Variable Definitions */
static emlrtRSInfo ce_emlrtRSI = { 8,  /* lineNo */
  "EvaluatePolynomial",                /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\EvaluatePolynomial.m"/* pathName */
};

static emlrtRSInfo de_emlrtRSI = { 10, /* lineNo */
  "EvaluatePolynomial",                /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\EvaluatePolynomial.m"/* pathName */
};

static emlrtRTEInfo jb_emlrtRTEI = { 2,/* lineNo */
  14,                                  /* colNo */
  "EvaluatePolynomial",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\EvaluatePolynomial.m"/* pName */
};

static emlrtRTEInfo kb_emlrtRTEI = { 6,/* lineNo */
  5,                                   /* colNo */
  "EvaluatePolynomial",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\EvaluatePolynomial.m"/* pName */
};

static emlrtRTEInfo yb_emlrtRTEI = { 7,/* lineNo */
  14,                                  /* colNo */
  "EvaluatePolynomial",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\EvaluatePolynomial.m"/* pName */
};

static emlrtBCInfo kb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  8,                                   /* lineNo */
  13,                                  /* colNo */
  "M",                                 /* aName */
  "EvaluatePolynomial",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\EvaluatePolynomial.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo h_emlrtECI = { -1,  /* nDims */
  8,                                   /* lineNo */
  9,                                   /* colNo */
  "EvaluatePolynomial",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\EvaluatePolynomial.m"/* pName */
};

/* Function Definitions */
real_T EvaluatePolynomial(const emlrtStack *sp, const emxArray_real_T *coeff,
  real_T x)
{
  real_T y;
  emxArray_real_T *M;
  int32_T n;
  int32_T i18;
  int32_T loop_ub;
  int32_T i;
  int32_T i19;
  int32_T i20;
  ptrdiff_t n_t;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  emxInit_real_T(sp, &M, 2, &kb_emlrtRTEI, true);

  /*  Evaluate a polynomial   ( similar to Y = polyval(P,X) ) */
  n = coeff->size[0];

  /* y = (coeff' * (x.^(order:-1:0))')'; */
  i18 = M->size[0] * M->size[1];
  M->size[0] = 1;
  M->size[1] = coeff->size[0];
  emxEnsureCapacity_real_T1(sp, M, i18, &jb_emlrtRTEI);
  loop_ub = coeff->size[0];
  for (i18 = 0; i18 < loop_ub; i18++) {
    M->data[i18] = 0.0;
  }

  i18 = (int32_T)((-1.0 - ((real_T)coeff->size[0] - 1.0)) / -1.0);
  emlrtForLoopVectorCheckR2012b((real_T)coeff->size[0] - 1.0, -1.0, 0.0,
    mxDOUBLE_CLASS, i18, &yb_emlrtRTEI, sp);
  loop_ub = 0;
  while (loop_ub <= i18 - 1) {
    i = (n - loop_ub) - 1;
    st.site = &ce_emlrtRSI;
    i19 = M->size[1];
    i20 = (int32_T)((((real_T)n - 1.0) - (real_T)i) + 1.0);
    if (!((i20 >= 1) && (i20 <= i19))) {
      emlrtDynamicBoundsCheckR2012b(i20, 1, i19, &kb_emlrtBCI, sp);
    }

    M->data[M->size[0] * (i20 - 1)] = muDoubleScalarPower(x, i);
    loop_ub++;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  st.site = &de_emlrtRSI;
  b_st.site = &rd_emlrtRSI;
  if (!(M->size[1] == coeff->size[0])) {
    if ((M->size[1] == 1) || (coeff->size[0] == 1)) {
      emlrtErrorWithMessageIdR2018a(&b_st, &tb_emlrtRTEI,
        "Coder:toolbox:mtimes_noDynamicScalarExpansion",
        "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(&b_st, &ub_emlrtRTEI,
        "Coder:MATLAB:innerdim", "Coder:MATLAB:innerdim", 0);
    }
  }

  if ((M->size[1] == 1) || (coeff->size[0] == 1)) {
    y = 0.0;
    for (i18 = 0; i18 < M->size[1]; i18++) {
      y += M->data[M->size[0] * i18] * coeff->data[i18];
    }
  } else if (M->size[1] < 1) {
    y = 0.0;
  } else {
    n_t = (ptrdiff_t)M->size[1];
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    y = ddot(&n_t, &M->data[0], &incx_t, &coeff->data[0], &incy_t);
  }

  emxFree_real_T(&st, &M);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
  return y;
}

void b_EvaluatePolynomial(const emlrtStack *sp, const emxArray_real_T *coeff,
  const real_T x[100], real_T y[100])
{
  emxArray_real_T *M;
  int32_T n;
  int32_T i24;
  int32_T loop_ub;
  int32_T i;
  int32_T b_M;
  int32_T i25;
  char_T TRANSA;
  real_T alpha1;
  char_T TRANSB;
  real_T beta1;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  emxInit_real_T(sp, &M, 2, &kb_emlrtRTEI, true);

  /*  Evaluate a polynomial   ( similar to Y = polyval(P,X) ) */
  n = coeff->size[0];

  /* y = (coeff' * (x.^(order:-1:0))')'; */
  i24 = M->size[0] * M->size[1];
  M->size[0] = 100;
  M->size[1] = coeff->size[0];
  emxEnsureCapacity_real_T1(sp, M, i24, &jb_emlrtRTEI);
  loop_ub = 100 * coeff->size[0];
  for (i24 = 0; i24 < loop_ub; i24++) {
    M->data[i24] = 0.0;
  }

  i24 = (int32_T)((-1.0 - ((real_T)coeff->size[0] - 1.0)) / -1.0);
  emlrtForLoopVectorCheckR2012b((real_T)coeff->size[0] - 1.0, -1.0, 0.0,
    mxDOUBLE_CLASS, i24, &yb_emlrtRTEI, sp);
  loop_ub = 0;
  while (loop_ub <= i24 - 1) {
    i = (n - loop_ub) - 1;
    b_M = M->size[1];
    i25 = (int32_T)((((real_T)n - 1.0) - (real_T)i) + 1.0);
    if (!((i25 >= 1) && (i25 <= b_M))) {
      emlrtDynamicBoundsCheckR2012b(i25, 1, b_M, &kb_emlrtBCI, sp);
    }

    st.site = &ce_emlrtRSI;
    d_power(&st, x, i, *(real_T (*)[100])&M->data[M->size[0] * (i25 - 1)]);
    loop_ub++;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  st.site = &de_emlrtRSI;
  b_st.site = &rd_emlrtRSI;
  if (!(M->size[1] == coeff->size[0])) {
    if (coeff->size[0] == 1) {
      emlrtErrorWithMessageIdR2018a(&b_st, &tb_emlrtRTEI,
        "Coder:toolbox:mtimes_noDynamicScalarExpansion",
        "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(&b_st, &ub_emlrtRTEI,
        "Coder:MATLAB:innerdim", "Coder:MATLAB:innerdim", 0);
    }
  }

  if ((M->size[1] == 1) || (coeff->size[0] == 1)) {
    loop_ub = M->size[1];
    for (i24 = 0; i24 < 100; i24++) {
      y[i24] = 0.0;
      for (i25 = 0; i25 < loop_ub; i25++) {
        alpha1 = y[i24] + M->data[i24 + M->size[0] * i25] * coeff->data[i25];
        y[i24] = alpha1;
      }
    }
  } else if ((M->size[1] == 0) || (coeff->size[0] == 0)) {
    memset(&y[0], 0, 100U * sizeof(real_T));
  } else {
    TRANSA = 'N';
    TRANSB = 'N';
    alpha1 = 1.0;
    beta1 = 0.0;
    m_t = (ptrdiff_t)100;
    n_t = (ptrdiff_t)1;
    k_t = (ptrdiff_t)M->size[1];
    lda_t = (ptrdiff_t)100;
    ldb_t = (ptrdiff_t)M->size[1];
    ldc_t = (ptrdiff_t)100;
    dgemm(&TRANSA, &TRANSB, &m_t, &n_t, &k_t, &alpha1, &M->data[0], &lda_t,
          &coeff->data[0], &ldb_t, &beta1, &y[0], &ldc_t);
  }

  emxFree_real_T(&st, &M);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void c_EvaluatePolynomial(const emlrtStack *sp, const emxArray_real_T *coeff,
  const emxArray_real_T *x, emxArray_real_T *y)
{
  emxArray_real_T *M;
  int32_T n;
  int32_T i29;
  int32_T loop_ub;
  int32_T i;
  emxArray_int32_T *r20;
  emxArray_real_T *r21;
  int32_T b_i;
  int32_T i30;
  int32_T i31;
  int32_T iv15[1];
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
  emxInit_real_T(sp, &M, 2, &kb_emlrtRTEI, true);

  /*  Evaluate a polynomial   ( similar to Y = polyval(P,X) ) */
  n = coeff->size[0];

  /* y = (coeff' * (x.^(order:-1:0))')'; */
  i29 = M->size[0] * M->size[1];
  M->size[0] = x->size[0];
  M->size[1] = coeff->size[0];
  emxEnsureCapacity_real_T1(sp, M, i29, &jb_emlrtRTEI);
  loop_ub = x->size[0] * coeff->size[0];
  for (i29 = 0; i29 < loop_ub; i29++) {
    M->data[i29] = 0.0;
  }

  i29 = (int32_T)((-1.0 - ((real_T)coeff->size[0] - 1.0)) / -1.0);
  emlrtForLoopVectorCheckR2012b((real_T)coeff->size[0] - 1.0, -1.0, 0.0,
    mxDOUBLE_CLASS, i29, &yb_emlrtRTEI, sp);
  i = 0;
  emxInit_int32_T(sp, &r20, 1, &jb_emlrtRTEI, true);
  emxInit_real_T1(sp, &r21, 1, &jb_emlrtRTEI, true);
  while (i <= i29 - 1) {
    b_i = (n - i) - 1;
    loop_ub = M->size[0];
    i30 = r20->size[0];
    r20->size[0] = loop_ub;
    emxEnsureCapacity_int32_T(sp, r20, i30, &jb_emlrtRTEI);
    for (i30 = 0; i30 < loop_ub; i30++) {
      r20->data[i30] = i30;
    }

    i30 = M->size[1];
    i31 = (int32_T)((((real_T)n - 1.0) - (real_T)b_i) + 1.0);
    if (!((i31 >= 1) && (i31 <= i30))) {
      emlrtDynamicBoundsCheckR2012b(i31, 1, i30, &kb_emlrtBCI, sp);
    }

    i30 = i31 - 1;
    st.site = &ce_emlrtRSI;
    b_power(&st, x, b_i, r21);
    iv15[0] = r20->size[0];
    emlrtSubAssignSizeCheckR2012b(&iv15[0], 1, &(*(int32_T (*)[1])r21->size)[0],
      1, &h_emlrtECI, sp);
    loop_ub = r21->size[0];
    for (i31 = 0; i31 < loop_ub; i31++) {
      M->data[r20->data[i31] + M->size[0] * i30] = r21->data[i31];
    }

    i++;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  emxFree_real_T(sp, &r21);
  emxFree_int32_T(sp, &r20);
  st.site = &de_emlrtRSI;
  b_st.site = &rd_emlrtRSI;
  if (!(M->size[1] == coeff->size[0])) {
    if (((M->size[0] == 1) && (M->size[1] == 1)) || (coeff->size[0] == 1)) {
      emlrtErrorWithMessageIdR2018a(&b_st, &tb_emlrtRTEI,
        "Coder:toolbox:mtimes_noDynamicScalarExpansion",
        "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(&b_st, &ub_emlrtRTEI,
        "Coder:MATLAB:innerdim", "Coder:MATLAB:innerdim", 0);
    }
  }

  if ((M->size[1] == 1) || (coeff->size[0] == 1)) {
    i29 = y->size[0];
    y->size[0] = M->size[0];
    emxEnsureCapacity_real_T(&st, y, i29, &jb_emlrtRTEI);
    loop_ub = M->size[0];
    for (i29 = 0; i29 < loop_ub; i29++) {
      y->data[i29] = 0.0;
      n = M->size[1];
      for (i30 = 0; i30 < n; i30++) {
        y->data[i29] += M->data[i29 + M->size[0] * i30] * coeff->data[i30];
      }
    }
  } else {
    b_st.site = &qd_emlrtRSI;
    if ((M->size[0] == 0) || (M->size[1] == 0) || (coeff->size[0] == 0)) {
      i29 = y->size[0];
      y->size[0] = M->size[0];
      emxEnsureCapacity_real_T(&b_st, y, i29, &jb_emlrtRTEI);
      loop_ub = M->size[0];
      for (i29 = 0; i29 < loop_ub; i29++) {
        y->data[i29] = 0.0;
      }
    } else {
      c_st.site = &td_emlrtRSI;
      TRANSA = 'N';
      TRANSB = 'N';
      alpha1 = 1.0;
      beta1 = 0.0;
      m_t = (ptrdiff_t)M->size[0];
      n_t = (ptrdiff_t)1;
      k_t = (ptrdiff_t)M->size[1];
      lda_t = (ptrdiff_t)M->size[0];
      ldb_t = (ptrdiff_t)M->size[1];
      ldc_t = (ptrdiff_t)M->size[0];
      i29 = y->size[0];
      y->size[0] = M->size[0];
      emxEnsureCapacity_real_T(&c_st, y, i29, &t_emlrtRTEI);
      dgemm(&TRANSA, &TRANSB, &m_t, &n_t, &k_t, &alpha1, &M->data[0], &lda_t,
            &coeff->data[0], &ldb_t, &beta1, &y->data[0], &ldc_t);
    }
  }

  emxFree_real_T(&st, &M);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (EvaluatePolynomial.c) */
