/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ConstrainedLeastSquares.c
 *
 * Code generation for function 'ConstrainedLeastSquares'
 *
 */

/* Include files */
#include "mwmathutil.h"
#include <string.h>
#include "rt_nonfinite.h"
#include "FitReferencePathPolynomial.h"
#include "ConstrainedLeastSquares.h"
#include "FitReferencePathPolynomial_emxutil.h"
#include "rdivide.h"
#include "diag.h"
#include "svd.h"
#include "FitReferencePathPolynomial_data.h"
#include "blas.h"

/* Variable Definitions */
static emlrtRSInfo jc_emlrtRSI = { 13, /* lineNo */
  "ConstrainedLeastSquares",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\ConstrainedLeastSquares.m"/* pathName */
};

static emlrtRSInfo kc_emlrtRSI = { 25, /* lineNo */
  "ConstrainedLeastSquares",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\ConstrainedLeastSquares.m"/* pathName */
};

static emlrtRSInfo lc_emlrtRSI = { 26, /* lineNo */
  "ConstrainedLeastSquares",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\ConstrainedLeastSquares.m"/* pathName */
};

static emlrtRSInfo mc_emlrtRSI = { 27, /* lineNo */
  "ConstrainedLeastSquares",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\ConstrainedLeastSquares.m"/* pathName */
};

static emlrtRSInfo nc_emlrtRSI = { 29, /* lineNo */
  "ConstrainedLeastSquares",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\ConstrainedLeastSquares.m"/* pathName */
};

static emlrtRSInfo oc_emlrtRSI = { 25, /* lineNo */
  "cat",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m"/* pathName */
};

static emlrtRTEInfo s_emlrtRTEI = { 10,/* lineNo */
  14,                                  /* colNo */
  "ConstrainedLeastSquares",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\ConstrainedLeastSquares.m"/* pName */
};

static emlrtRTEInfo u_emlrtRTEI = { 13,/* lineNo */
  9,                                   /* colNo */
  "ConstrainedLeastSquares",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\ConstrainedLeastSquares.m"/* pName */
};

static emlrtRTEInfo v_emlrtRTEI = { 14,/* lineNo */
  9,                                   /* colNo */
  "ConstrainedLeastSquares",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\ConstrainedLeastSquares.m"/* pName */
};

static emlrtRTEInfo w_emlrtRTEI = { 26,/* lineNo */
  5,                                   /* colNo */
  "ConstrainedLeastSquares",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\ConstrainedLeastSquares.m"/* pName */
};

static emlrtRTEInfo x_emlrtRTEI = { 103,/* lineNo */
  1,                                   /* colNo */
  "cat",                               /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m"/* pName */
};

static emlrtDCInfo d_emlrtDCI = { 26,  /* lineNo */
  48,                                  /* colNo */
  "ConstrainedLeastSquares",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\ConstrainedLeastSquares.m",/* pName */
  4                                    /* checkKind */
};

/* Function Definitions */
void ConstrainedLeastSquares(const emlrtStack *sp, const emxArray_real_T *A,
  const emxArray_real_T *b, const emxArray_real_T *Aeq, const real_T beq_data[],
  const int32_T beq_size[1], real_T lambda, emxArray_real_T *x)
{
  emxArray_real_T *A_;
  emxArray_real_T *b_;
  emxArray_real_T *varargin_2;
  cell_wrap_1 reshapes[2];
  int32_T i5;
  int32_T loop_ub;
  boolean_T empty_non_axis_sizes;
  emxArray_real_T *Sinv;
  emxArray_real_T *U;
  emxArray_real_T *V;
  int32_T result;
  emxArray_real_T *r6;
  emxArray_real_T *r7;
  int32_T b_result;
  int32_T i6;
  int32_T b_loop_ub;
  int32_T Sinv_idx_1;
  cell_wrap_1 b_reshapes[2];
  emxArray_real_T *a;
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

  /*  x = ConstrainedLeastSquares(A, b, Aeq, beq, lambda) */
  /*  */
  /*  Fit linear function */
  /*  c2*x^2 + c1*x + c0 = y */
  /*  Perform through least squares as: min |Ax - y|^2  */
  /*  x = [c2, c1, c0]' */
  /*  A = [x1^2, x1, 1; x2^2, x2, 1; ...]; */
  /*  Which is the same as using the pseudo-inverse of A: A^+ */
  /*  x_hat = A^+ y */
  emxInit_real_T(sp, &A_, 2, &u_emlrtRTEI, true);
  emxInit_real_T1(sp, &b_, 1, &v_emlrtRTEI, true);
  emxInit_real_T(sp, &varargin_2, 2, &s_emlrtRTEI, true);
  emxInitMatrix_cell_wrap_1(sp, reshapes, &x_emlrtRTEI, true);
  if ((Aeq->size[0] == beq_size[0]) && (A->size[1] == Aeq->size[1])) {
    i5 = varargin_2->size[0] * varargin_2->size[1];
    varargin_2->size[0] = Aeq->size[0];
    varargin_2->size[1] = Aeq->size[1];
    emxEnsureCapacity_real_T1(sp, varargin_2, i5, &s_emlrtRTEI);
    loop_ub = Aeq->size[0] * Aeq->size[1];
    for (i5 = 0; i5 < loop_ub; i5++) {
      varargin_2->data[i5] = lambda * Aeq->data[i5];
    }

    st.site = &jc_emlrtRSI;
    b_st.site = &oc_emlrtRSI;
    c_st.site = &ic_emlrtRSI;
    empty_non_axis_sizes = true;
    if (varargin_2->size[1] != A->size[1]) {
      empty_non_axis_sizes = false;
    }

    if (!empty_non_axis_sizes) {
      emlrtErrorWithMessageIdR2018a(&c_st, &mb_emlrtRTEI,
        "MATLAB:catenate:matrixDimensionMismatch",
        "MATLAB:catenate:matrixDimensionMismatch", 0);
    }

    empty_non_axis_sizes = (A->size[1] == 0);
    if (empty_non_axis_sizes || (!((A->size[0] == 0) || (A->size[1] == 0)))) {
      result = A->size[0];
    } else {
      result = 0;
    }

    if (empty_non_axis_sizes || (!((varargin_2->size[0] == 0) ||
          (varargin_2->size[1] == 0)))) {
      b_result = varargin_2->size[0];
    } else {
      b_result = 0;
    }

    i5 = reshapes[1].f1->size[0] * reshapes[1].f1->size[1];
    reshapes[1].f1->size[0] = b_result;
    reshapes[1].f1->size[1] = A->size[1];
    emxEnsureCapacity_real_T1(&b_st, reshapes[1].f1, i5, &s_emlrtRTEI);
    loop_ub = b_result * A->size[1];
    for (i5 = 0; i5 < loop_ub; i5++) {
      reshapes[1].f1->data[i5] = varargin_2->data[i5];
    }

    b_result = A->size[1];
    i5 = A_->size[0] * A_->size[1];
    A_->size[0] = result + reshapes[1].f1->size[0];
    A_->size[1] = b_result;
    emxEnsureCapacity_real_T1(&b_st, A_, i5, &s_emlrtRTEI);
    for (i5 = 0; i5 < b_result; i5++) {
      for (i6 = 0; i6 < result; i6++) {
        A_->data[i6 + A_->size[0] * i5] = A->data[i6 + result * i5];
      }
    }

    loop_ub = reshapes[1].f1->size[1];
    for (i5 = 0; i5 < loop_ub; i5++) {
      b_loop_ub = reshapes[1].f1->size[0];
      for (i6 = 0; i6 < b_loop_ub; i6++) {
        A_->data[(i6 + result) + A_->size[0] * i5] = reshapes[1].f1->data[i6 +
          reshapes[1].f1->size[0] * i5];
      }
    }

    i5 = b_->size[0];
    b_->size[0] = b->size[0] + beq_size[0];
    emxEnsureCapacity_real_T(sp, b_, i5, &s_emlrtRTEI);
    loop_ub = b->size[0];
    for (i5 = 0; i5 < loop_ub; i5++) {
      b_->data[i5] = b->data[i5];
    }

    loop_ub = beq_size[0];
    for (i5 = 0; i5 < loop_ub; i5++) {
      b_->data[i5 + b->size[0]] = lambda * beq_data[i5];
    }
  } else {
    i5 = A_->size[0] * A_->size[1];
    A_->size[0] = A->size[0];
    A_->size[1] = A->size[1];
    emxEnsureCapacity_real_T1(sp, A_, i5, &s_emlrtRTEI);
    loop_ub = A->size[0] * A->size[1];
    for (i5 = 0; i5 < loop_ub; i5++) {
      A_->data[i5] = A->data[i5];
    }

    i5 = b_->size[0];
    b_->size[0] = b->size[0];
    emxEnsureCapacity_real_T(sp, b_, i5, &s_emlrtRTEI);
    loop_ub = b->size[0];
    for (i5 = 0; i5 < loop_ub; i5++) {
      b_->data[i5] = b->data[i5];
    }
  }

  emxFreeMatrix_cell_wrap_1(sp, reshapes);
  emxFree_real_T(sp, &varargin_2);
  emxInit_real_T(sp, &Sinv, 2, &w_emlrtRTEI, true);
  emxInit_real_T(sp, &U, 2, &s_emlrtRTEI, true);
  emxInit_real_T(sp, &V, 2, &s_emlrtRTEI, true);
  emxInit_real_T1(sp, &r6, 1, &s_emlrtRTEI, true);
  emxInit_real_T1(sp, &r7, 1, &s_emlrtRTEI, true);

  /*  Pseudo-inverse through numerically unstable way */
  /* A_invpseudo = inv(A_'*A_) * A_' */
  /*  Pseudo-inverse through MATLAB */
  /* A_invpseudo2 = pinv(A_)  % Moore-Penrose Pseudoinverse of matrix of A */
  /*  Pseudo-inverse through SVD */
  st.site = &kc_emlrtRSI;
  svd(&st, A_, U, Sinv, V);
  st.site = &lc_emlrtRSI;
  diag(&st, Sinv, r7);
  rdivide(sp, r7, r6);
  st.site = &lc_emlrtRSI;
  b_diag(&st, r6, A_);
  i5 = Sinv->size[0] - Sinv->size[1];
  if (!(i5 >= 0)) {
    emlrtNonNegativeCheckR2012b(i5, &d_emlrtDCI, sp);
  }

  st.site = &lc_emlrtRSI;
  b_st.site = &oc_emlrtRSI;
  emxFree_real_T(&b_st, &r7);
  emxFree_real_T(&b_st, &r6);
  if (!((A_->size[0] == 0) || (A_->size[1] == 0))) {
    b_loop_ub = A_->size[0];
  } else {
    b_result = Sinv->size[1];
    Sinv_idx_1 = Sinv->size[0] - Sinv->size[1];
    if (!((b_result == 0) || (Sinv_idx_1 == 0))) {
      b_loop_ub = Sinv->size[1];
    } else {
      b_loop_ub = muIntScalarMax_sint32(A_->size[0], 0);
      b_result = Sinv->size[1];
      if (b_result > b_loop_ub) {
        b_loop_ub = Sinv->size[1];
      }
    }
  }

  c_st.site = &ic_emlrtRSI;
  if ((A_->size[0] == b_loop_ub) || ((A_->size[0] == 0) || (A_->size[1] == 0)))
  {
    empty_non_axis_sizes = true;
  } else {
    empty_non_axis_sizes = false;
    emlrtErrorWithMessageIdR2018a(&c_st, &mb_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  b_result = Sinv->size[1];
  if (b_result == b_loop_ub) {
  } else {
    b_result = Sinv->size[1];
    Sinv_idx_1 = Sinv->size[0] - Sinv->size[1];
    if ((b_result == 0) || (Sinv_idx_1 == 0)) {
    } else {
      empty_non_axis_sizes = false;
    }
  }

  if (!empty_non_axis_sizes) {
    emlrtErrorWithMessageIdR2018a(&c_st, &mb_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  empty_non_axis_sizes = (b_loop_ub == 0);
  if (empty_non_axis_sizes || (!((A_->size[0] == 0) || (A_->size[1] == 0)))) {
    result = A_->size[1];
  } else {
    result = 0;
  }

  if (empty_non_axis_sizes) {
    Sinv_idx_1 = Sinv->size[0] - Sinv->size[1];
  } else {
    b_result = Sinv->size[1];
    Sinv_idx_1 = Sinv->size[0] - Sinv->size[1];
    if (!((b_result == 0) || (Sinv_idx_1 == 0))) {
      Sinv_idx_1 = Sinv->size[0] - Sinv->size[1];
    } else {
      Sinv_idx_1 = 0;
    }
  }

  emxInitMatrix_cell_wrap_1(&b_st, b_reshapes, &x_emlrtRTEI, true);
  i5 = b_reshapes[1].f1->size[0] * b_reshapes[1].f1->size[1];
  b_reshapes[1].f1->size[0] = b_loop_ub;
  b_reshapes[1].f1->size[1] = Sinv_idx_1;
  emxEnsureCapacity_real_T1(&b_st, b_reshapes[1].f1, i5, &s_emlrtRTEI);
  loop_ub = b_loop_ub * Sinv_idx_1;
  for (i5 = 0; i5 < loop_ub; i5++) {
    b_reshapes[1].f1->data[i5] = 0.0;
  }

  i5 = Sinv->size[0] * Sinv->size[1];
  Sinv->size[0] = b_loop_ub;
  Sinv->size[1] = result + b_reshapes[1].f1->size[1];
  emxEnsureCapacity_real_T1(&b_st, Sinv, i5, &s_emlrtRTEI);
  for (i5 = 0; i5 < result; i5++) {
    for (i6 = 0; i6 < b_loop_ub; i6++) {
      Sinv->data[i6 + Sinv->size[0] * i5] = A_->data[i6 + b_loop_ub * i5];
    }
  }

  loop_ub = b_reshapes[1].f1->size[1];
  for (i5 = 0; i5 < loop_ub; i5++) {
    b_loop_ub = b_reshapes[1].f1->size[0];
    for (i6 = 0; i6 < b_loop_ub; i6++) {
      Sinv->data[i6 + Sinv->size[0] * (i5 + result)] = b_reshapes[1].f1->data[i6
        + b_reshapes[1].f1->size[0] * i5];
    }
  }

  emxFreeMatrix_cell_wrap_1(&b_st, b_reshapes);
  st.site = &mc_emlrtRSI;
  b_st.site = &rd_emlrtRSI;
  if (!(V->size[1] == Sinv->size[0])) {
    if (((V->size[0] == 1) && (V->size[1] == 1)) || ((Sinv->size[0] == 1) &&
         (Sinv->size[1] == 1))) {
      emlrtErrorWithMessageIdR2018a(&b_st, &tb_emlrtRTEI,
        "Coder:toolbox:mtimes_noDynamicScalarExpansion",
        "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(&b_st, &ub_emlrtRTEI,
        "Coder:MATLAB:innerdim", "Coder:MATLAB:innerdim", 0);
    }
  }

  emxInit_real_T(&st, &a, 2, &s_emlrtRTEI, true);
  if ((V->size[1] == 1) || (Sinv->size[0] == 1)) {
    i5 = a->size[0] * a->size[1];
    a->size[0] = V->size[0];
    a->size[1] = Sinv->size[1];
    emxEnsureCapacity_real_T1(&st, a, i5, &s_emlrtRTEI);
    loop_ub = V->size[0];
    for (i5 = 0; i5 < loop_ub; i5++) {
      b_loop_ub = Sinv->size[1];
      for (i6 = 0; i6 < b_loop_ub; i6++) {
        a->data[i5 + a->size[0] * i6] = 0.0;
        b_result = V->size[1];
        for (Sinv_idx_1 = 0; Sinv_idx_1 < b_result; Sinv_idx_1++) {
          a->data[i5 + a->size[0] * i6] += V->data[i5 + V->size[0] * Sinv_idx_1]
            * Sinv->data[Sinv_idx_1 + Sinv->size[0] * i6];
        }
      }
    }
  } else {
    b_st.site = &qd_emlrtRSI;
    if ((V->size[0] == 0) || (V->size[1] == 0) || (Sinv->size[0] == 0) ||
        (Sinv->size[1] == 0)) {
      i5 = a->size[0] * a->size[1];
      a->size[0] = V->size[0];
      a->size[1] = Sinv->size[1];
      emxEnsureCapacity_real_T1(&b_st, a, i5, &s_emlrtRTEI);
      loop_ub = V->size[0] * Sinv->size[1];
      for (i5 = 0; i5 < loop_ub; i5++) {
        a->data[i5] = 0.0;
      }
    } else {
      c_st.site = &td_emlrtRSI;
      TRANSA = 'N';
      TRANSB = 'N';
      alpha1 = 1.0;
      beta1 = 0.0;
      m_t = (ptrdiff_t)V->size[0];
      n_t = (ptrdiff_t)Sinv->size[1];
      k_t = (ptrdiff_t)V->size[1];
      lda_t = (ptrdiff_t)V->size[0];
      ldb_t = (ptrdiff_t)V->size[1];
      ldc_t = (ptrdiff_t)V->size[0];
      i5 = a->size[0] * a->size[1];
      a->size[0] = V->size[0];
      a->size[1] = Sinv->size[1];
      emxEnsureCapacity_real_T1(&c_st, a, i5, &t_emlrtRTEI);
      dgemm(&TRANSA, &TRANSB, &m_t, &n_t, &k_t, &alpha1, &V->data[0], &lda_t,
            &Sinv->data[0], &ldb_t, &beta1, &a->data[0], &ldc_t);
    }
  }

  emxFree_real_T(&st, &V);
  st.site = &mc_emlrtRSI;
  i5 = Sinv->size[0] * Sinv->size[1];
  Sinv->size[0] = U->size[1];
  Sinv->size[1] = U->size[0];
  emxEnsureCapacity_real_T1(&st, Sinv, i5, &s_emlrtRTEI);
  loop_ub = U->size[0];
  for (i5 = 0; i5 < loop_ub; i5++) {
    b_loop_ub = U->size[1];
    for (i6 = 0; i6 < b_loop_ub; i6++) {
      Sinv->data[i6 + Sinv->size[0] * i5] = U->data[i5 + U->size[0] * i6];
    }
  }

  emxFree_real_T(&st, &U);
  b_st.site = &rd_emlrtRSI;
  if (!(a->size[1] == Sinv->size[0])) {
    if (((a->size[0] == 1) && (a->size[1] == 1)) || ((Sinv->size[0] == 1) &&
         (Sinv->size[1] == 1))) {
      emlrtErrorWithMessageIdR2018a(&b_st, &tb_emlrtRTEI,
        "Coder:toolbox:mtimes_noDynamicScalarExpansion",
        "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(&b_st, &ub_emlrtRTEI,
        "Coder:MATLAB:innerdim", "Coder:MATLAB:innerdim", 0);
    }
  }

  if ((a->size[1] == 1) || (Sinv->size[0] == 1)) {
    i5 = A_->size[0] * A_->size[1];
    A_->size[0] = a->size[0];
    A_->size[1] = Sinv->size[1];
    emxEnsureCapacity_real_T1(&st, A_, i5, &s_emlrtRTEI);
    loop_ub = a->size[0];
    for (i5 = 0; i5 < loop_ub; i5++) {
      b_loop_ub = Sinv->size[1];
      for (i6 = 0; i6 < b_loop_ub; i6++) {
        A_->data[i5 + A_->size[0] * i6] = 0.0;
        b_result = a->size[1];
        for (Sinv_idx_1 = 0; Sinv_idx_1 < b_result; Sinv_idx_1++) {
          A_->data[i5 + A_->size[0] * i6] += a->data[i5 + a->size[0] *
            Sinv_idx_1] * Sinv->data[Sinv_idx_1 + Sinv->size[0] * i6];
        }
      }
    }
  } else {
    b_st.site = &qd_emlrtRSI;
    if ((a->size[0] == 0) || (a->size[1] == 0) || (Sinv->size[0] == 0) ||
        (Sinv->size[1] == 0)) {
      i5 = A_->size[0] * A_->size[1];
      A_->size[0] = a->size[0];
      A_->size[1] = Sinv->size[1];
      emxEnsureCapacity_real_T1(&b_st, A_, i5, &s_emlrtRTEI);
      loop_ub = a->size[0] * Sinv->size[1];
      for (i5 = 0; i5 < loop_ub; i5++) {
        A_->data[i5] = 0.0;
      }
    } else {
      c_st.site = &td_emlrtRSI;
      TRANSA = 'N';
      TRANSB = 'N';
      alpha1 = 1.0;
      beta1 = 0.0;
      m_t = (ptrdiff_t)a->size[0];
      n_t = (ptrdiff_t)Sinv->size[1];
      k_t = (ptrdiff_t)a->size[1];
      lda_t = (ptrdiff_t)a->size[0];
      ldb_t = (ptrdiff_t)a->size[1];
      ldc_t = (ptrdiff_t)a->size[0];
      i5 = A_->size[0] * A_->size[1];
      A_->size[0] = a->size[0];
      A_->size[1] = Sinv->size[1];
      emxEnsureCapacity_real_T1(&c_st, A_, i5, &t_emlrtRTEI);
      dgemm(&TRANSA, &TRANSB, &m_t, &n_t, &k_t, &alpha1, &a->data[0], &lda_t,
            &Sinv->data[0], &ldb_t, &beta1, &A_->data[0], &ldc_t);
    }
  }

  emxFree_real_T(&st, &a);
  emxFree_real_T(&st, &Sinv);
  st.site = &nc_emlrtRSI;
  b_st.site = &rd_emlrtRSI;
  if (!(A_->size[1] == b_->size[0])) {
    if (((A_->size[0] == 1) && (A_->size[1] == 1)) || (b_->size[0] == 1)) {
      emlrtErrorWithMessageIdR2018a(&b_st, &tb_emlrtRTEI,
        "Coder:toolbox:mtimes_noDynamicScalarExpansion",
        "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(&b_st, &ub_emlrtRTEI,
        "Coder:MATLAB:innerdim", "Coder:MATLAB:innerdim", 0);
    }
  }

  if ((A_->size[1] == 1) || (b_->size[0] == 1)) {
    i5 = x->size[0];
    x->size[0] = A_->size[0];
    emxEnsureCapacity_real_T(&st, x, i5, &s_emlrtRTEI);
    loop_ub = A_->size[0];
    for (i5 = 0; i5 < loop_ub; i5++) {
      x->data[i5] = 0.0;
      b_loop_ub = A_->size[1];
      for (i6 = 0; i6 < b_loop_ub; i6++) {
        x->data[i5] += A_->data[i5 + A_->size[0] * i6] * b_->data[i6];
      }
    }
  } else {
    b_st.site = &qd_emlrtRSI;
    if ((A_->size[0] == 0) || (A_->size[1] == 0) || (b_->size[0] == 0)) {
      i5 = x->size[0];
      x->size[0] = A_->size[0];
      emxEnsureCapacity_real_T(&b_st, x, i5, &s_emlrtRTEI);
      loop_ub = A_->size[0];
      for (i5 = 0; i5 < loop_ub; i5++) {
        x->data[i5] = 0.0;
      }
    } else {
      c_st.site = &td_emlrtRSI;
      TRANSA = 'N';
      TRANSB = 'N';
      alpha1 = 1.0;
      beta1 = 0.0;
      m_t = (ptrdiff_t)A_->size[0];
      n_t = (ptrdiff_t)1;
      k_t = (ptrdiff_t)A_->size[1];
      lda_t = (ptrdiff_t)A_->size[0];
      ldb_t = (ptrdiff_t)A_->size[1];
      ldc_t = (ptrdiff_t)A_->size[0];
      i5 = x->size[0];
      x->size[0] = A_->size[0];
      emxEnsureCapacity_real_T(&c_st, x, i5, &t_emlrtRTEI);
      dgemm(&TRANSA, &TRANSB, &m_t, &n_t, &k_t, &alpha1, &A_->data[0], &lda_t,
            &b_->data[0], &ldb_t, &beta1, &x->data[0], &ldc_t);
    }
  }

  emxFree_real_T(&st, &b_);
  emxFree_real_T(&st, &A_);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void b_ConstrainedLeastSquares(const emlrtStack *sp, const emxArray_real_T *A,
  const emxArray_real_T *b, const emxArray_real_T *Aeq, const real_T beq[2],
  real_T lambda, emxArray_real_T *x)
{
  emxArray_real_T *A_;
  emxArray_real_T *b_;
  int32_T i22;
  emxArray_real_T *varargin_2;
  int32_T loop_ub;
  int32_T result;
  emxArray_real_T *Sinv;
  emxArray_real_T *U;
  boolean_T empty_non_axis_sizes;
  emxArray_real_T *V;
  emxArray_real_T *r13;
  emxArray_real_T *r14;
  int32_T b_result;
  int32_T i23;
  int32_T Sinv_idx_0;
  int32_T Sinv_idx_1;
  cell_wrap_1 reshapes[2];
  emxArray_real_T *a;
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

  /*  x = ConstrainedLeastSquares(A, b, Aeq, beq, lambda) */
  /*  */
  /*  Fit linear function */
  /*  c2*x^2 + c1*x + c0 = y */
  /*  Perform through least squares as: min |Ax - y|^2  */
  /*  x = [c2, c1, c0]' */
  /*  A = [x1^2, x1, 1; x2^2, x2, 1; ...]; */
  /*  Which is the same as using the pseudo-inverse of A: A^+ */
  /*  x_hat = A^+ y */
  emxInit_real_T(sp, &A_, 2, &u_emlrtRTEI, true);
  emxInit_real_T1(sp, &b_, 1, &v_emlrtRTEI, true);
  if (A->size[1] == Aeq->size[1]) {
    emxInit_real_T(sp, &varargin_2, 2, &s_emlrtRTEI, true);
    i22 = varargin_2->size[0] * varargin_2->size[1];
    varargin_2->size[0] = 2;
    varargin_2->size[1] = Aeq->size[1];
    emxEnsureCapacity_real_T1(sp, varargin_2, i22, &s_emlrtRTEI);
    loop_ub = Aeq->size[0] * Aeq->size[1];
    for (i22 = 0; i22 < loop_ub; i22++) {
      varargin_2->data[i22] = lambda * Aeq->data[i22];
    }

    st.site = &jc_emlrtRSI;
    b_st.site = &oc_emlrtRSI;
    if (!((A->size[0] == 0) || (A->size[1] == 0))) {
      result = A->size[1];
    } else {
      result = varargin_2->size[1];
    }

    c_st.site = &ic_emlrtRSI;
    if ((A->size[1] == result) || ((A->size[0] == 0) || (A->size[1] == 0))) {
      empty_non_axis_sizes = true;
    } else {
      empty_non_axis_sizes = false;
      emlrtErrorWithMessageIdR2018a(&c_st, &mb_emlrtRTEI,
        "MATLAB:catenate:matrixDimensionMismatch",
        "MATLAB:catenate:matrixDimensionMismatch", 0);
    }

    if (varargin_2->size[1] != result) {
      empty_non_axis_sizes = false;
    }

    if (!empty_non_axis_sizes) {
      emlrtErrorWithMessageIdR2018a(&c_st, &mb_emlrtRTEI,
        "MATLAB:catenate:matrixDimensionMismatch",
        "MATLAB:catenate:matrixDimensionMismatch", 0);
    }

    if ((result == 0) || (!((A->size[0] == 0) || (A->size[1] == 0)))) {
      b_result = A->size[0];
    } else {
      b_result = 0;
    }

    i22 = A_->size[0] * A_->size[1];
    A_->size[0] = b_result + 2;
    A_->size[1] = result;
    emxEnsureCapacity_real_T1(&b_st, A_, i22, &s_emlrtRTEI);
    for (i22 = 0; i22 < result; i22++) {
      for (i23 = 0; i23 < b_result; i23++) {
        A_->data[i23 + A_->size[0] * i22] = A->data[i23 + b_result * i22];
      }
    }

    for (i22 = 0; i22 < result; i22++) {
      for (i23 = 0; i23 < 2; i23++) {
        A_->data[(i23 + b_result) + A_->size[0] * i22] = varargin_2->data[i23 +
          (i22 << 1)];
      }
    }

    emxFree_real_T(&b_st, &varargin_2);
    i22 = b_->size[0];
    b_->size[0] = b->size[0] + 2;
    emxEnsureCapacity_real_T(sp, b_, i22, &s_emlrtRTEI);
    loop_ub = b->size[0];
    for (i22 = 0; i22 < loop_ub; i22++) {
      b_->data[i22] = b->data[i22];
    }

    for (i22 = 0; i22 < 2; i22++) {
      b_->data[i22 + b->size[0]] = lambda * beq[i22];
    }
  } else {
    i22 = A_->size[0] * A_->size[1];
    A_->size[0] = A->size[0];
    A_->size[1] = A->size[1];
    emxEnsureCapacity_real_T1(sp, A_, i22, &s_emlrtRTEI);
    loop_ub = A->size[0] * A->size[1];
    for (i22 = 0; i22 < loop_ub; i22++) {
      A_->data[i22] = A->data[i22];
    }

    i22 = b_->size[0];
    b_->size[0] = b->size[0];
    emxEnsureCapacity_real_T(sp, b_, i22, &s_emlrtRTEI);
    loop_ub = b->size[0];
    for (i22 = 0; i22 < loop_ub; i22++) {
      b_->data[i22] = b->data[i22];
    }
  }

  emxInit_real_T(sp, &Sinv, 2, &w_emlrtRTEI, true);
  emxInit_real_T(sp, &U, 2, &s_emlrtRTEI, true);
  emxInit_real_T(sp, &V, 2, &s_emlrtRTEI, true);
  emxInit_real_T1(sp, &r13, 1, &s_emlrtRTEI, true);
  emxInit_real_T1(sp, &r14, 1, &s_emlrtRTEI, true);

  /*  Pseudo-inverse through numerically unstable way */
  /* A_invpseudo = inv(A_'*A_) * A_' */
  /*  Pseudo-inverse through MATLAB */
  /* A_invpseudo2 = pinv(A_)  % Moore-Penrose Pseudoinverse of matrix of A */
  /*  Pseudo-inverse through SVD */
  st.site = &kc_emlrtRSI;
  svd(&st, A_, U, Sinv, V);
  st.site = &lc_emlrtRSI;
  diag(&st, Sinv, r14);
  rdivide(sp, r14, r13);
  st.site = &lc_emlrtRSI;
  b_diag(&st, r13, A_);
  i22 = Sinv->size[0] - Sinv->size[1];
  if (!(i22 >= 0)) {
    emlrtNonNegativeCheckR2012b(i22, &d_emlrtDCI, sp);
  }

  st.site = &lc_emlrtRSI;
  b_st.site = &oc_emlrtRSI;
  emxFree_real_T(&b_st, &r14);
  emxFree_real_T(&b_st, &r13);
  if (!((A_->size[0] == 0) || (A_->size[1] == 0))) {
    Sinv_idx_0 = A_->size[0];
  } else {
    b_result = Sinv->size[1];
    Sinv_idx_1 = Sinv->size[0] - Sinv->size[1];
    if (!((b_result == 0) || (Sinv_idx_1 == 0))) {
      Sinv_idx_0 = Sinv->size[1];
    } else {
      Sinv_idx_0 = muIntScalarMax_sint32(A_->size[0], 0);
      b_result = Sinv->size[1];
      if (b_result > Sinv_idx_0) {
        Sinv_idx_0 = Sinv->size[1];
      }
    }
  }

  c_st.site = &ic_emlrtRSI;
  if ((A_->size[0] == Sinv_idx_0) || ((A_->size[0] == 0) || (A_->size[1] == 0)))
  {
    empty_non_axis_sizes = true;
  } else {
    empty_non_axis_sizes = false;
    emlrtErrorWithMessageIdR2018a(&c_st, &mb_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  b_result = Sinv->size[1];
  if (b_result == Sinv_idx_0) {
  } else {
    b_result = Sinv->size[1];
    Sinv_idx_1 = Sinv->size[0] - Sinv->size[1];
    if ((b_result == 0) || (Sinv_idx_1 == 0)) {
    } else {
      empty_non_axis_sizes = false;
    }
  }

  if (!empty_non_axis_sizes) {
    emlrtErrorWithMessageIdR2018a(&c_st, &mb_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  empty_non_axis_sizes = (Sinv_idx_0 == 0);
  if (empty_non_axis_sizes || (!((A_->size[0] == 0) || (A_->size[1] == 0)))) {
    result = A_->size[1];
  } else {
    result = 0;
  }

  if (empty_non_axis_sizes) {
    Sinv_idx_1 = Sinv->size[0] - Sinv->size[1];
  } else {
    b_result = Sinv->size[1];
    Sinv_idx_1 = Sinv->size[0] - Sinv->size[1];
    if (!((b_result == 0) || (Sinv_idx_1 == 0))) {
      Sinv_idx_1 = Sinv->size[0] - Sinv->size[1];
    } else {
      Sinv_idx_1 = 0;
    }
  }

  emxInitMatrix_cell_wrap_1(&b_st, reshapes, &x_emlrtRTEI, true);
  i22 = reshapes[1].f1->size[0] * reshapes[1].f1->size[1];
  reshapes[1].f1->size[0] = Sinv_idx_0;
  reshapes[1].f1->size[1] = Sinv_idx_1;
  emxEnsureCapacity_real_T1(&b_st, reshapes[1].f1, i22, &s_emlrtRTEI);
  loop_ub = Sinv_idx_0 * Sinv_idx_1;
  for (i22 = 0; i22 < loop_ub; i22++) {
    reshapes[1].f1->data[i22] = 0.0;
  }

  i22 = Sinv->size[0] * Sinv->size[1];
  Sinv->size[0] = Sinv_idx_0;
  Sinv->size[1] = result + reshapes[1].f1->size[1];
  emxEnsureCapacity_real_T1(&b_st, Sinv, i22, &s_emlrtRTEI);
  for (i22 = 0; i22 < result; i22++) {
    for (i23 = 0; i23 < Sinv_idx_0; i23++) {
      Sinv->data[i23 + Sinv->size[0] * i22] = A_->data[i23 + Sinv_idx_0 * i22];
    }
  }

  loop_ub = reshapes[1].f1->size[1];
  for (i22 = 0; i22 < loop_ub; i22++) {
    Sinv_idx_0 = reshapes[1].f1->size[0];
    for (i23 = 0; i23 < Sinv_idx_0; i23++) {
      Sinv->data[i23 + Sinv->size[0] * (i22 + result)] = reshapes[1].f1->
        data[i23 + reshapes[1].f1->size[0] * i22];
    }
  }

  emxFreeMatrix_cell_wrap_1(&b_st, reshapes);
  st.site = &mc_emlrtRSI;
  b_st.site = &rd_emlrtRSI;
  if (!(V->size[1] == Sinv->size[0])) {
    if (((V->size[0] == 1) && (V->size[1] == 1)) || ((Sinv->size[0] == 1) &&
         (Sinv->size[1] == 1))) {
      emlrtErrorWithMessageIdR2018a(&b_st, &tb_emlrtRTEI,
        "Coder:toolbox:mtimes_noDynamicScalarExpansion",
        "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(&b_st, &ub_emlrtRTEI,
        "Coder:MATLAB:innerdim", "Coder:MATLAB:innerdim", 0);
    }
  }

  emxInit_real_T(&st, &a, 2, &s_emlrtRTEI, true);
  if ((V->size[1] == 1) || (Sinv->size[0] == 1)) {
    i22 = a->size[0] * a->size[1];
    a->size[0] = V->size[0];
    a->size[1] = Sinv->size[1];
    emxEnsureCapacity_real_T1(&st, a, i22, &s_emlrtRTEI);
    loop_ub = V->size[0];
    for (i22 = 0; i22 < loop_ub; i22++) {
      Sinv_idx_0 = Sinv->size[1];
      for (i23 = 0; i23 < Sinv_idx_0; i23++) {
        a->data[i22 + a->size[0] * i23] = 0.0;
        b_result = V->size[1];
        for (Sinv_idx_1 = 0; Sinv_idx_1 < b_result; Sinv_idx_1++) {
          a->data[i22 + a->size[0] * i23] += V->data[i22 + V->size[0] *
            Sinv_idx_1] * Sinv->data[Sinv_idx_1 + Sinv->size[0] * i23];
        }
      }
    }
  } else {
    b_st.site = &qd_emlrtRSI;
    if ((V->size[0] == 0) || (V->size[1] == 0) || (Sinv->size[0] == 0) ||
        (Sinv->size[1] == 0)) {
      i22 = a->size[0] * a->size[1];
      a->size[0] = V->size[0];
      a->size[1] = Sinv->size[1];
      emxEnsureCapacity_real_T1(&b_st, a, i22, &s_emlrtRTEI);
      loop_ub = V->size[0] * Sinv->size[1];
      for (i22 = 0; i22 < loop_ub; i22++) {
        a->data[i22] = 0.0;
      }
    } else {
      c_st.site = &td_emlrtRSI;
      TRANSA = 'N';
      TRANSB = 'N';
      alpha1 = 1.0;
      beta1 = 0.0;
      m_t = (ptrdiff_t)V->size[0];
      n_t = (ptrdiff_t)Sinv->size[1];
      k_t = (ptrdiff_t)V->size[1];
      lda_t = (ptrdiff_t)V->size[0];
      ldb_t = (ptrdiff_t)V->size[1];
      ldc_t = (ptrdiff_t)V->size[0];
      i22 = a->size[0] * a->size[1];
      a->size[0] = V->size[0];
      a->size[1] = Sinv->size[1];
      emxEnsureCapacity_real_T1(&c_st, a, i22, &t_emlrtRTEI);
      dgemm(&TRANSA, &TRANSB, &m_t, &n_t, &k_t, &alpha1, &V->data[0], &lda_t,
            &Sinv->data[0], &ldb_t, &beta1, &a->data[0], &ldc_t);
    }
  }

  emxFree_real_T(&st, &V);
  st.site = &mc_emlrtRSI;
  i22 = Sinv->size[0] * Sinv->size[1];
  Sinv->size[0] = U->size[1];
  Sinv->size[1] = U->size[0];
  emxEnsureCapacity_real_T1(&st, Sinv, i22, &s_emlrtRTEI);
  loop_ub = U->size[0];
  for (i22 = 0; i22 < loop_ub; i22++) {
    Sinv_idx_0 = U->size[1];
    for (i23 = 0; i23 < Sinv_idx_0; i23++) {
      Sinv->data[i23 + Sinv->size[0] * i22] = U->data[i22 + U->size[0] * i23];
    }
  }

  emxFree_real_T(&st, &U);
  b_st.site = &rd_emlrtRSI;
  if (!(a->size[1] == Sinv->size[0])) {
    if (((a->size[0] == 1) && (a->size[1] == 1)) || ((Sinv->size[0] == 1) &&
         (Sinv->size[1] == 1))) {
      emlrtErrorWithMessageIdR2018a(&b_st, &tb_emlrtRTEI,
        "Coder:toolbox:mtimes_noDynamicScalarExpansion",
        "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(&b_st, &ub_emlrtRTEI,
        "Coder:MATLAB:innerdim", "Coder:MATLAB:innerdim", 0);
    }
  }

  if ((a->size[1] == 1) || (Sinv->size[0] == 1)) {
    i22 = A_->size[0] * A_->size[1];
    A_->size[0] = a->size[0];
    A_->size[1] = Sinv->size[1];
    emxEnsureCapacity_real_T1(&st, A_, i22, &s_emlrtRTEI);
    loop_ub = a->size[0];
    for (i22 = 0; i22 < loop_ub; i22++) {
      Sinv_idx_0 = Sinv->size[1];
      for (i23 = 0; i23 < Sinv_idx_0; i23++) {
        A_->data[i22 + A_->size[0] * i23] = 0.0;
        b_result = a->size[1];
        for (Sinv_idx_1 = 0; Sinv_idx_1 < b_result; Sinv_idx_1++) {
          A_->data[i22 + A_->size[0] * i23] += a->data[i22 + a->size[0] *
            Sinv_idx_1] * Sinv->data[Sinv_idx_1 + Sinv->size[0] * i23];
        }
      }
    }
  } else {
    b_st.site = &qd_emlrtRSI;
    if ((a->size[0] == 0) || (a->size[1] == 0) || (Sinv->size[0] == 0) ||
        (Sinv->size[1] == 0)) {
      i22 = A_->size[0] * A_->size[1];
      A_->size[0] = a->size[0];
      A_->size[1] = Sinv->size[1];
      emxEnsureCapacity_real_T1(&b_st, A_, i22, &s_emlrtRTEI);
      loop_ub = a->size[0] * Sinv->size[1];
      for (i22 = 0; i22 < loop_ub; i22++) {
        A_->data[i22] = 0.0;
      }
    } else {
      c_st.site = &td_emlrtRSI;
      TRANSA = 'N';
      TRANSB = 'N';
      alpha1 = 1.0;
      beta1 = 0.0;
      m_t = (ptrdiff_t)a->size[0];
      n_t = (ptrdiff_t)Sinv->size[1];
      k_t = (ptrdiff_t)a->size[1];
      lda_t = (ptrdiff_t)a->size[0];
      ldb_t = (ptrdiff_t)a->size[1];
      ldc_t = (ptrdiff_t)a->size[0];
      i22 = A_->size[0] * A_->size[1];
      A_->size[0] = a->size[0];
      A_->size[1] = Sinv->size[1];
      emxEnsureCapacity_real_T1(&c_st, A_, i22, &t_emlrtRTEI);
      dgemm(&TRANSA, &TRANSB, &m_t, &n_t, &k_t, &alpha1, &a->data[0], &lda_t,
            &Sinv->data[0], &ldb_t, &beta1, &A_->data[0], &ldc_t);
    }
  }

  emxFree_real_T(&st, &a);
  emxFree_real_T(&st, &Sinv);
  st.site = &nc_emlrtRSI;
  b_st.site = &rd_emlrtRSI;
  if (!(A_->size[1] == b_->size[0])) {
    if (((A_->size[0] == 1) && (A_->size[1] == 1)) || (b_->size[0] == 1)) {
      emlrtErrorWithMessageIdR2018a(&b_st, &tb_emlrtRTEI,
        "Coder:toolbox:mtimes_noDynamicScalarExpansion",
        "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(&b_st, &ub_emlrtRTEI,
        "Coder:MATLAB:innerdim", "Coder:MATLAB:innerdim", 0);
    }
  }

  if ((A_->size[1] == 1) || (b_->size[0] == 1)) {
    i22 = x->size[0];
    x->size[0] = A_->size[0];
    emxEnsureCapacity_real_T(&st, x, i22, &s_emlrtRTEI);
    loop_ub = A_->size[0];
    for (i22 = 0; i22 < loop_ub; i22++) {
      x->data[i22] = 0.0;
      Sinv_idx_0 = A_->size[1];
      for (i23 = 0; i23 < Sinv_idx_0; i23++) {
        x->data[i22] += A_->data[i22 + A_->size[0] * i23] * b_->data[i23];
      }
    }
  } else {
    b_st.site = &qd_emlrtRSI;
    if ((A_->size[0] == 0) || (A_->size[1] == 0) || (b_->size[0] == 0)) {
      i22 = x->size[0];
      x->size[0] = A_->size[0];
      emxEnsureCapacity_real_T(&b_st, x, i22, &s_emlrtRTEI);
      loop_ub = A_->size[0];
      for (i22 = 0; i22 < loop_ub; i22++) {
        x->data[i22] = 0.0;
      }
    } else {
      c_st.site = &td_emlrtRSI;
      TRANSA = 'N';
      TRANSB = 'N';
      alpha1 = 1.0;
      beta1 = 0.0;
      m_t = (ptrdiff_t)A_->size[0];
      n_t = (ptrdiff_t)1;
      k_t = (ptrdiff_t)A_->size[1];
      lda_t = (ptrdiff_t)A_->size[0];
      ldb_t = (ptrdiff_t)A_->size[1];
      ldc_t = (ptrdiff_t)A_->size[0];
      i22 = x->size[0];
      x->size[0] = A_->size[0];
      emxEnsureCapacity_real_T(&c_st, x, i22, &t_emlrtRTEI);
      dgemm(&TRANSA, &TRANSB, &m_t, &n_t, &k_t, &alpha1, &A_->data[0], &lda_t,
            &b_->data[0], &ldb_t, &beta1, &x->data[0], &ldc_t);
    }
  }

  emxFree_real_T(&st, &b_);
  emxFree_real_T(&st, &A_);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void c_ConstrainedLeastSquares(const emlrtStack *sp, const emxArray_real_T *A,
  const real_T b[100], const emxArray_real_T *Aeq, const real_T beq_data[],
  const int32_T beq_size[1], emxArray_real_T *x)
{
  emxArray_real_T *A_;
  emxArray_real_T *varargin_2;
  int32_T i27;
  int32_T loop_ub;
  int32_T b__size_idx_0;
  real_T b__data[104];
  emxArray_real_T *Sinv;
  emxArray_real_T *U;
  emxArray_real_T *V;
  boolean_T empty_non_axis_sizes;
  emxArray_real_T *r19;
  emxArray_real_T *varargin_1;
  int32_T result;
  emxArray_real_T *b_b;
  int32_T unnamed_idx_1;
  int32_T result_idx_1;
  int32_T i28;
  int32_T Sinv_idx_0;
  cell_wrap_1 reshapes[2];
  emxArray_real_T *a;
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

  /*  x = ConstrainedLeastSquares(A, b, Aeq, beq, lambda) */
  /*  */
  /*  Fit linear function */
  /*  c2*x^2 + c1*x + c0 = y */
  /*  Perform through least squares as: min |Ax - y|^2  */
  /*  x = [c2, c1, c0]' */
  /*  A = [x1^2, x1, 1; x2^2, x2, 1; ...]; */
  /*  Which is the same as using the pseudo-inverse of A: A^+ */
  /*  x_hat = A^+ y */
  emxInit_real_T(sp, &A_, 2, &u_emlrtRTEI, true);
  emxInit_real_T(sp, &varargin_2, 2, &s_emlrtRTEI, true);
  if ((Aeq->size[0] == beq_size[0]) && (A->size[1] == Aeq->size[1])) {
    i27 = varargin_2->size[0] * varargin_2->size[1];
    varargin_2->size[0] = Aeq->size[0];
    varargin_2->size[1] = Aeq->size[1];
    emxEnsureCapacity_real_T1(sp, varargin_2, i27, &s_emlrtRTEI);
    loop_ub = Aeq->size[0] * Aeq->size[1];
    for (i27 = 0; i27 < loop_ub; i27++) {
      varargin_2->data[i27] = 1.0E+6 * Aeq->data[i27];
    }

    st.site = &jc_emlrtRSI;
    b_st.site = &oc_emlrtRSI;
    c_st.site = &ic_emlrtRSI;
    if ((varargin_2->size[1] == A->size[1]) || ((varargin_2->size[0] == 0) ||
         (varargin_2->size[1] == 0))) {
    } else {
      emlrtErrorWithMessageIdR2018a(&c_st, &mb_emlrtRTEI,
        "MATLAB:catenate:matrixDimensionMismatch",
        "MATLAB:catenate:matrixDimensionMismatch", 0);
    }

    empty_non_axis_sizes = (A->size[1] == 0);
    if (empty_non_axis_sizes || (!((varargin_2->size[0] == 0) ||
          (varargin_2->size[1] == 0)))) {
      result = varargin_2->size[0];
    } else {
      result = 0;
    }

    unnamed_idx_1 = A->size[1];
    result_idx_1 = A->size[1];
    i27 = A_->size[0] * A_->size[1];
    A_->size[0] = 100 + result;
    A_->size[1] = unnamed_idx_1;
    emxEnsureCapacity_real_T1(&b_st, A_, i27, &s_emlrtRTEI);
    for (i27 = 0; i27 < unnamed_idx_1; i27++) {
      for (i28 = 0; i28 < 100; i28++) {
        A_->data[i28 + A_->size[0] * i27] = A->data[i28 + 100 * i27];
      }
    }

    for (i27 = 0; i27 < result_idx_1; i27++) {
      for (i28 = 0; i28 < result; i28++) {
        A_->data[(i28 + A_->size[0] * i27) + 100] = varargin_2->data[i28 +
          result * i27];
      }
    }

    b__size_idx_0 = 100 + beq_size[0];
    memcpy(&b__data[0], &b[0], 100U * sizeof(real_T));
    loop_ub = beq_size[0];
    for (i27 = 0; i27 < loop_ub; i27++) {
      b__data[i27 + 100] = 1.0E+6 * beq_data[i27];
    }
  } else {
    i27 = A_->size[0] * A_->size[1];
    A_->size[0] = 100;
    A_->size[1] = A->size[1];
    emxEnsureCapacity_real_T1(sp, A_, i27, &s_emlrtRTEI);
    loop_ub = A->size[0] * A->size[1];
    for (i27 = 0; i27 < loop_ub; i27++) {
      A_->data[i27] = A->data[i27];
    }

    b__size_idx_0 = 100;
    memcpy(&b__data[0], &b[0], 100U * sizeof(real_T));
  }

  emxFree_real_T(sp, &varargin_2);
  emxInit_real_T(sp, &Sinv, 2, &w_emlrtRTEI, true);
  emxInit_real_T(sp, &U, 2, &s_emlrtRTEI, true);
  emxInit_real_T(sp, &V, 2, &s_emlrtRTEI, true);
  emxInit_real_T1(sp, &r19, 1, &s_emlrtRTEI, true);
  emxInit_real_T(sp, &varargin_1, 2, &s_emlrtRTEI, true);
  emxInit_real_T1(sp, &b_b, 1, &s_emlrtRTEI, true);

  /*  Pseudo-inverse through numerically unstable way */
  /* A_invpseudo = inv(A_'*A_) * A_' */
  /*  Pseudo-inverse through MATLAB */
  /* A_invpseudo2 = pinv(A_)  % Moore-Penrose Pseudoinverse of matrix of A */
  /*  Pseudo-inverse through SVD */
  st.site = &kc_emlrtRSI;
  svd(&st, A_, U, Sinv, V);
  st.site = &lc_emlrtRSI;
  diag(&st, Sinv, b_b);
  rdivide(sp, b_b, r19);
  st.site = &lc_emlrtRSI;
  b_diag(&st, r19, varargin_1);
  i27 = Sinv->size[0] - Sinv->size[1];
  if (!(i27 >= 0)) {
    emlrtNonNegativeCheckR2012b(i27, &d_emlrtDCI, sp);
  }

  st.site = &lc_emlrtRSI;
  b_st.site = &oc_emlrtRSI;
  emxFree_real_T(&b_st, &r19);
  emxFree_real_T(&b_st, &A_);
  if (!((varargin_1->size[0] == 0) || (varargin_1->size[1] == 0))) {
    Sinv_idx_0 = varargin_1->size[0];
  } else {
    unnamed_idx_1 = Sinv->size[1];
    result_idx_1 = Sinv->size[0] - Sinv->size[1];
    if (!((unnamed_idx_1 == 0) || (result_idx_1 == 0))) {
      Sinv_idx_0 = Sinv->size[1];
    } else {
      Sinv_idx_0 = muIntScalarMax_sint32(varargin_1->size[0], 0);
      unnamed_idx_1 = Sinv->size[1];
      if (unnamed_idx_1 > Sinv_idx_0) {
        Sinv_idx_0 = Sinv->size[1];
      }
    }
  }

  c_st.site = &ic_emlrtRSI;
  if ((varargin_1->size[0] == Sinv_idx_0) || ((varargin_1->size[0] == 0) ||
       (varargin_1->size[1] == 0))) {
    empty_non_axis_sizes = true;
  } else {
    empty_non_axis_sizes = false;
    emlrtErrorWithMessageIdR2018a(&c_st, &mb_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  unnamed_idx_1 = Sinv->size[1];
  if (unnamed_idx_1 == Sinv_idx_0) {
  } else {
    unnamed_idx_1 = Sinv->size[1];
    result_idx_1 = Sinv->size[0] - Sinv->size[1];
    if ((unnamed_idx_1 == 0) || (result_idx_1 == 0)) {
    } else {
      empty_non_axis_sizes = false;
    }
  }

  if (!empty_non_axis_sizes) {
    emlrtErrorWithMessageIdR2018a(&c_st, &mb_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  empty_non_axis_sizes = (Sinv_idx_0 == 0);
  if (empty_non_axis_sizes || (!((varargin_1->size[0] == 0) || (varargin_1->
         size[1] == 0)))) {
    result = varargin_1->size[1];
  } else {
    result = 0;
  }

  if (empty_non_axis_sizes) {
    result_idx_1 = Sinv->size[0] - Sinv->size[1];
  } else {
    unnamed_idx_1 = Sinv->size[1];
    result_idx_1 = Sinv->size[0] - Sinv->size[1];
    if (!((unnamed_idx_1 == 0) || (result_idx_1 == 0))) {
      result_idx_1 = Sinv->size[0] - Sinv->size[1];
    } else {
      result_idx_1 = 0;
    }
  }

  emxInitMatrix_cell_wrap_1(&b_st, reshapes, &x_emlrtRTEI, true);
  i27 = reshapes[1].f1->size[0] * reshapes[1].f1->size[1];
  reshapes[1].f1->size[0] = Sinv_idx_0;
  reshapes[1].f1->size[1] = result_idx_1;
  emxEnsureCapacity_real_T1(&b_st, reshapes[1].f1, i27, &s_emlrtRTEI);
  loop_ub = Sinv_idx_0 * result_idx_1;
  for (i27 = 0; i27 < loop_ub; i27++) {
    reshapes[1].f1->data[i27] = 0.0;
  }

  i27 = Sinv->size[0] * Sinv->size[1];
  Sinv->size[0] = Sinv_idx_0;
  Sinv->size[1] = result + reshapes[1].f1->size[1];
  emxEnsureCapacity_real_T1(&b_st, Sinv, i27, &s_emlrtRTEI);
  for (i27 = 0; i27 < result; i27++) {
    for (i28 = 0; i28 < Sinv_idx_0; i28++) {
      Sinv->data[i28 + Sinv->size[0] * i27] = varargin_1->data[i28 + Sinv_idx_0 *
        i27];
    }
  }

  loop_ub = reshapes[1].f1->size[1];
  for (i27 = 0; i27 < loop_ub; i27++) {
    Sinv_idx_0 = reshapes[1].f1->size[0];
    for (i28 = 0; i28 < Sinv_idx_0; i28++) {
      Sinv->data[i28 + Sinv->size[0] * (i27 + result)] = reshapes[1].f1->
        data[i28 + reshapes[1].f1->size[0] * i27];
    }
  }

  emxFreeMatrix_cell_wrap_1(&b_st, reshapes);
  st.site = &mc_emlrtRSI;
  b_st.site = &rd_emlrtRSI;
  if (!(V->size[1] == Sinv->size[0])) {
    if (((V->size[0] == 1) && (V->size[1] == 1)) || ((Sinv->size[0] == 1) &&
         (Sinv->size[1] == 1))) {
      emlrtErrorWithMessageIdR2018a(&b_st, &tb_emlrtRTEI,
        "Coder:toolbox:mtimes_noDynamicScalarExpansion",
        "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(&b_st, &ub_emlrtRTEI,
        "Coder:MATLAB:innerdim", "Coder:MATLAB:innerdim", 0);
    }
  }

  emxInit_real_T(&st, &a, 2, &s_emlrtRTEI, true);
  if ((V->size[1] == 1) || (Sinv->size[0] == 1)) {
    i27 = a->size[0] * a->size[1];
    a->size[0] = V->size[0];
    a->size[1] = Sinv->size[1];
    emxEnsureCapacity_real_T1(&st, a, i27, &s_emlrtRTEI);
    loop_ub = V->size[0];
    for (i27 = 0; i27 < loop_ub; i27++) {
      Sinv_idx_0 = Sinv->size[1];
      for (i28 = 0; i28 < Sinv_idx_0; i28++) {
        a->data[i27 + a->size[0] * i28] = 0.0;
        unnamed_idx_1 = V->size[1];
        for (result_idx_1 = 0; result_idx_1 < unnamed_idx_1; result_idx_1++) {
          a->data[i27 + a->size[0] * i28] += V->data[i27 + V->size[0] *
            result_idx_1] * Sinv->data[result_idx_1 + Sinv->size[0] * i28];
        }
      }
    }
  } else {
    b_st.site = &qd_emlrtRSI;
    if ((V->size[0] == 0) || (V->size[1] == 0) || (Sinv->size[0] == 0) ||
        (Sinv->size[1] == 0)) {
      i27 = a->size[0] * a->size[1];
      a->size[0] = V->size[0];
      a->size[1] = Sinv->size[1];
      emxEnsureCapacity_real_T1(&b_st, a, i27, &s_emlrtRTEI);
      loop_ub = V->size[0] * Sinv->size[1];
      for (i27 = 0; i27 < loop_ub; i27++) {
        a->data[i27] = 0.0;
      }
    } else {
      c_st.site = &td_emlrtRSI;
      TRANSA = 'N';
      TRANSB = 'N';
      alpha1 = 1.0;
      beta1 = 0.0;
      m_t = (ptrdiff_t)V->size[0];
      n_t = (ptrdiff_t)Sinv->size[1];
      k_t = (ptrdiff_t)V->size[1];
      lda_t = (ptrdiff_t)V->size[0];
      ldb_t = (ptrdiff_t)V->size[1];
      ldc_t = (ptrdiff_t)V->size[0];
      i27 = a->size[0] * a->size[1];
      a->size[0] = V->size[0];
      a->size[1] = Sinv->size[1];
      emxEnsureCapacity_real_T1(&c_st, a, i27, &t_emlrtRTEI);
      dgemm(&TRANSA, &TRANSB, &m_t, &n_t, &k_t, &alpha1, &V->data[0], &lda_t,
            &Sinv->data[0], &ldb_t, &beta1, &a->data[0], &ldc_t);
    }
  }

  emxFree_real_T(&st, &V);
  st.site = &mc_emlrtRSI;
  i27 = varargin_1->size[0] * varargin_1->size[1];
  varargin_1->size[0] = U->size[1];
  varargin_1->size[1] = U->size[0];
  emxEnsureCapacity_real_T1(&st, varargin_1, i27, &s_emlrtRTEI);
  loop_ub = U->size[0];
  for (i27 = 0; i27 < loop_ub; i27++) {
    Sinv_idx_0 = U->size[1];
    for (i28 = 0; i28 < Sinv_idx_0; i28++) {
      varargin_1->data[i28 + varargin_1->size[0] * i27] = U->data[i27 + U->size
        [0] * i28];
    }
  }

  emxFree_real_T(&st, &U);
  b_st.site = &rd_emlrtRSI;
  if (!(a->size[1] == varargin_1->size[0])) {
    if (((a->size[0] == 1) && (a->size[1] == 1)) || ((varargin_1->size[0] == 1) &&
         (varargin_1->size[1] == 1))) {
      emlrtErrorWithMessageIdR2018a(&b_st, &tb_emlrtRTEI,
        "Coder:toolbox:mtimes_noDynamicScalarExpansion",
        "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(&b_st, &ub_emlrtRTEI,
        "Coder:MATLAB:innerdim", "Coder:MATLAB:innerdim", 0);
    }
  }

  if ((a->size[1] == 1) || (varargin_1->size[0] == 1)) {
    i27 = Sinv->size[0] * Sinv->size[1];
    Sinv->size[0] = a->size[0];
    Sinv->size[1] = varargin_1->size[1];
    emxEnsureCapacity_real_T1(&st, Sinv, i27, &s_emlrtRTEI);
    loop_ub = a->size[0];
    for (i27 = 0; i27 < loop_ub; i27++) {
      Sinv_idx_0 = varargin_1->size[1];
      for (i28 = 0; i28 < Sinv_idx_0; i28++) {
        Sinv->data[i27 + Sinv->size[0] * i28] = 0.0;
        unnamed_idx_1 = a->size[1];
        for (result_idx_1 = 0; result_idx_1 < unnamed_idx_1; result_idx_1++) {
          Sinv->data[i27 + Sinv->size[0] * i28] += a->data[i27 + a->size[0] *
            result_idx_1] * varargin_1->data[result_idx_1 + varargin_1->size[0] *
            i28];
        }
      }
    }
  } else {
    b_st.site = &qd_emlrtRSI;
    if ((a->size[0] == 0) || (a->size[1] == 0) || (varargin_1->size[0] == 0) ||
        (varargin_1->size[1] == 0)) {
      i27 = Sinv->size[0] * Sinv->size[1];
      Sinv->size[0] = a->size[0];
      Sinv->size[1] = varargin_1->size[1];
      emxEnsureCapacity_real_T1(&b_st, Sinv, i27, &s_emlrtRTEI);
      loop_ub = a->size[0] * varargin_1->size[1];
      for (i27 = 0; i27 < loop_ub; i27++) {
        Sinv->data[i27] = 0.0;
      }
    } else {
      c_st.site = &td_emlrtRSI;
      TRANSA = 'N';
      TRANSB = 'N';
      alpha1 = 1.0;
      beta1 = 0.0;
      m_t = (ptrdiff_t)a->size[0];
      n_t = (ptrdiff_t)varargin_1->size[1];
      k_t = (ptrdiff_t)a->size[1];
      lda_t = (ptrdiff_t)a->size[0];
      ldb_t = (ptrdiff_t)a->size[1];
      ldc_t = (ptrdiff_t)a->size[0];
      i27 = Sinv->size[0] * Sinv->size[1];
      Sinv->size[0] = a->size[0];
      Sinv->size[1] = varargin_1->size[1];
      emxEnsureCapacity_real_T1(&c_st, Sinv, i27, &t_emlrtRTEI);
      dgemm(&TRANSA, &TRANSB, &m_t, &n_t, &k_t, &alpha1, &a->data[0], &lda_t,
            &varargin_1->data[0], &ldb_t, &beta1, &Sinv->data[0], &ldc_t);
    }
  }

  emxFree_real_T(&st, &a);
  emxFree_real_T(&st, &varargin_1);
  st.site = &nc_emlrtRSI;
  i27 = b_b->size[0];
  b_b->size[0] = b__size_idx_0;
  emxEnsureCapacity_real_T(&st, b_b, i27, &s_emlrtRTEI);
  for (i27 = 0; i27 < b__size_idx_0; i27++) {
    b_b->data[i27] = b__data[i27];
  }

  b_st.site = &rd_emlrtRSI;
  if (!(Sinv->size[1] == b__size_idx_0)) {
    if ((Sinv->size[0] == 1) && (Sinv->size[1] == 1)) {
      emlrtErrorWithMessageIdR2018a(&b_st, &tb_emlrtRTEI,
        "Coder:toolbox:mtimes_noDynamicScalarExpansion",
        "Coder:toolbox:mtimes_noDynamicScalarExpansion", 0);
    } else {
      emlrtErrorWithMessageIdR2018a(&b_st, &ub_emlrtRTEI,
        "Coder:MATLAB:innerdim", "Coder:MATLAB:innerdim", 0);
    }
  }

  b_st.site = &qd_emlrtRSI;
  if (Sinv->size[0] == 0) {
    i27 = x->size[0];
    x->size[0] = Sinv->size[0];
    emxEnsureCapacity_real_T(&b_st, x, i27, &s_emlrtRTEI);
    loop_ub = Sinv->size[0];
    for (i27 = 0; i27 < loop_ub; i27++) {
      x->data[i27] = 0.0;
    }
  } else {
    c_st.site = &td_emlrtRSI;
    TRANSA = 'N';
    TRANSB = 'N';
    alpha1 = 1.0;
    beta1 = 0.0;
    m_t = (ptrdiff_t)Sinv->size[0];
    n_t = (ptrdiff_t)1;
    k_t = (ptrdiff_t)Sinv->size[1];
    lda_t = (ptrdiff_t)Sinv->size[0];
    ldb_t = (ptrdiff_t)Sinv->size[1];
    ldc_t = (ptrdiff_t)Sinv->size[0];
    i27 = x->size[0];
    x->size[0] = Sinv->size[0];
    emxEnsureCapacity_real_T(&c_st, x, i27, &t_emlrtRTEI);
    dgemm(&TRANSA, &TRANSB, &m_t, &n_t, &k_t, &alpha1, &Sinv->data[0], &lda_t,
          &b_b->data[0], &ldb_t, &beta1, &x->data[0], &ldc_t);
  }

  emxFree_real_T(&b_st, &b_b);
  emxFree_real_T(&b_st, &Sinv);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (ConstrainedLeastSquares.c) */
