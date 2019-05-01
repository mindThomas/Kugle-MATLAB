/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * svd1.c
 *
 * Code generation for function 'svd1'
 *
 */

/* Include files */
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "FitReferencePathPolynomial.h"
#include "svd1.h"
#include "FitReferencePathPolynomial_emxutil.h"
#include "error.h"
#include "lapacke.h"

/* Type Definitions */
#include <stdlib.h>

/* Variable Definitions */
static emlrtRSInfo tc_emlrtRSI = { 53, /* lineNo */
  "svd",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\svd.m"/* pathName */
};

static emlrtRSInfo uc_emlrtRSI = { 75, /* lineNo */
  "svd",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\svd.m"/* pathName */
};

static emlrtRSInfo vc_emlrtRSI = { 83, /* lineNo */
  "svd",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\svd.m"/* pathName */
};

static emlrtRSInfo wc_emlrtRSI = { 105,/* lineNo */
  "svd",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\svd.m"/* pathName */
};

static emlrtRSInfo xc_emlrtRSI = { 205,/* lineNo */
  "xgesdd",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+lapack\\xgesdd.m"/* pathName */
};

static emlrtRSInfo yc_emlrtRSI = { 175,/* lineNo */
  "xgesdd",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+lapack\\xgesdd.m"/* pathName */
};

static emlrtRSInfo ad_emlrtRSI = { 64, /* lineNo */
  "xgesdd",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+lapack\\xgesdd.m"/* pathName */
};

static emlrtRSInfo bd_emlrtRSI = { 57, /* lineNo */
  "xgesdd",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+lapack\\xgesdd.m"/* pathName */
};

static emlrtRSInfo cd_emlrtRSI = { 54, /* lineNo */
  "xgesdd",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+lapack\\xgesdd.m"/* pathName */
};

static emlrtRSInfo dd_emlrtRSI = { 7,  /* lineNo */
  "int",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+lapack\\int.m"/* pathName */
};

static emlrtRSInfo ed_emlrtRSI = { 8,  /* lineNo */
  "majority",                          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+lapack\\majority.m"/* pathName */
};

static emlrtRSInfo fd_emlrtRSI = { 31, /* lineNo */
  "infocheck",                         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+lapack\\infocheck.m"/* pathName */
};

static emlrtRSInfo gd_emlrtRSI = { 45, /* lineNo */
  "infocheck",                         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+lapack\\infocheck.m"/* pathName */
};

static emlrtRSInfo hd_emlrtRSI = { 48, /* lineNo */
  "infocheck",                         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+lapack\\infocheck.m"/* pathName */
};

static emlrtRSInfo id_emlrtRSI = { 28, /* lineNo */
  "xgesvd",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+lapack\\xgesvd.m"/* pathName */
};

static emlrtRSInfo jd_emlrtRSI = { 193,/* lineNo */
  "xgesvd",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+lapack\\xgesvd.m"/* pathName */
};

static emlrtRSInfo kd_emlrtRSI = { 171,/* lineNo */
  "xgesvd",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+lapack\\xgesvd.m"/* pathName */
};

static emlrtRSInfo ld_emlrtRSI = { 114,/* lineNo */
  "xgesvd",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+lapack\\xgesvd.m"/* pathName */
};

static emlrtRSInfo md_emlrtRSI = { 107,/* lineNo */
  "xgesvd",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+lapack\\xgesvd.m"/* pathName */
};

static emlrtRSInfo nd_emlrtRSI = { 56, /* lineNo */
  "xgesvd",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+lapack\\xgesvd.m"/* pathName */
};

static emlrtRTEInfo bb_emlrtRTEI = { 1,/* lineNo */
  20,                                  /* colNo */
  "svd",                               /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\svd.m"/* pName */
};

static emlrtRTEInfo cb_emlrtRTEI = { 75,/* lineNo */
  9,                                   /* colNo */
  "svd",                               /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\svd.m"/* pName */
};

static emlrtRTEInfo db_emlrtRTEI = { 28,/* lineNo */
  5,                                   /* colNo */
  "xgesvd",                            /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+lapack\\xgesvd.m"/* pName */
};

static emlrtRTEInfo eb_emlrtRTEI = { 117,/* lineNo */
  9,                                   /* colNo */
  "xgesvd",                            /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+lapack\\xgesvd.m"/* pName */
};

/* Function Definitions */
void b_svd(const emlrtStack *sp, const emxArray_real_T *A, emxArray_real_T *U,
           emxArray_real_T *s, emxArray_real_T *V)
{
  emxArray_real_T *b_A;
  int32_T m;
  int32_T n;
  int32_T i7;
  int32_T loop_ub;
  emxArray_real_T *Vt;
  int32_T i8;
  ptrdiff_t info_t;
  emxArray_real_T *superb;
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
  emxInit_real_T(sp, &b_A, 2, &bb_emlrtRTEI, true);
  st.site = &tc_emlrtRSI;
  m = A->size[0];
  n = A->size[1];
  b_st.site = &uc_emlrtRSI;
  i7 = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = A->size[1];
  emxEnsureCapacity_real_T1(&b_st, b_A, i7, &bb_emlrtRTEI);
  loop_ub = A->size[0] * A->size[1];
  for (i7 = 0; i7 < loop_ub; i7++) {
    b_A->data[i7] = A->data[i7];
  }

  emxInit_real_T(&b_st, &Vt, 2, &bb_emlrtRTEI, true);
  i7 = U->size[0] * U->size[1];
  U->size[0] = A->size[0];
  U->size[1] = A->size[0];
  emxEnsureCapacity_real_T1(&b_st, U, i7, &cb_emlrtRTEI);
  i7 = Vt->size[0] * Vt->size[1];
  Vt->size[0] = A->size[1];
  Vt->size[1] = A->size[1];
  emxEnsureCapacity_real_T1(&b_st, Vt, i7, &cb_emlrtRTEI);
  i7 = s->size[0];
  s->size[0] = muIntScalarMin_sint32(n, m);
  emxEnsureCapacity_real_T(&b_st, s, i7, &cb_emlrtRTEI);
  c_st.site = &cd_emlrtRSI;
  if (!(A->size[0] == 0)) {
    c_st.site = &bd_emlrtRSI;
    d_st.site = &dd_emlrtRSI;
    c_st.site = &ad_emlrtRSI;
    d_st.site = &dd_emlrtRSI;
    c_st.site = &yc_emlrtRSI;
    d_st.site = &ed_emlrtRSI;
    info_t = LAPACKE_dgesdd(102, 'A', (ptrdiff_t)A->size[0], (ptrdiff_t)A->size
      [1], &b_A->data[0], (ptrdiff_t)A->size[0], &s->data[0], &U->data[0],
      (ptrdiff_t)A->size[0], &Vt->data[0], (ptrdiff_t)A->size[1]);
    m = (int32_T)info_t;
    c_st.site = &xc_emlrtRSI;
    d_st.site = &fd_emlrtRSI;
    if (m < 0) {
      if (m == -1010) {
        d_st.site = &gd_emlrtRSI;
        h_error(&d_st);
      } else {
        d_st.site = &hd_emlrtRSI;
        i_error(&d_st, m);
      }
    }
  } else {
    m = 0;
  }

  if (m > 0) {
    b_st.site = &vc_emlrtRSI;
    c_st.site = &id_emlrtRSI;
    i7 = b_A->size[0] * b_A->size[1];
    b_A->size[0] = A->size[0];
    b_A->size[1] = A->size[1];
    emxEnsureCapacity_real_T1(&c_st, b_A, i7, &bb_emlrtRTEI);
    loop_ub = A->size[0] * A->size[1];
    for (i7 = 0; i7 < loop_ub; i7++) {
      b_A->data[i7] = A->data[i7];
    }

    m = A->size[0];
    n = A->size[1];
    d_st.site = &nd_emlrtRSI;
    m = muIntScalarMin_sint32(n, m);
    i7 = U->size[0] * U->size[1];
    U->size[0] = A->size[0];
    U->size[1] = A->size[0];
    emxEnsureCapacity_real_T1(&c_st, U, i7, &db_emlrtRTEI);
    i7 = Vt->size[0] * Vt->size[1];
    Vt->size[0] = A->size[1];
    Vt->size[1] = A->size[1];
    emxEnsureCapacity_real_T1(&c_st, Vt, i7, &db_emlrtRTEI);
    i7 = s->size[0];
    s->size[0] = m;
    emxEnsureCapacity_real_T(&c_st, s, i7, &db_emlrtRTEI);
    if (!(A->size[0] == 0)) {
      d_st.site = &md_emlrtRSI;
      d_st.site = &ld_emlrtRSI;
      emxInit_real_T1(&c_st, &superb, 1, &eb_emlrtRTEI, true);
      if (m > 1) {
        i7 = superb->size[0];
        superb->size[0] = m - 1;
        emxEnsureCapacity_real_T(&c_st, superb, i7, &db_emlrtRTEI);
      } else {
        i7 = superb->size[0];
        superb->size[0] = 1;
        emxEnsureCapacity_real_T(&c_st, superb, i7, &db_emlrtRTEI);
      }

      d_st.site = &kd_emlrtRSI;
      info_t = LAPACKE_dgesvd(102, 'A', 'A', (ptrdiff_t)A->size[0], (ptrdiff_t)
        A->size[1], &b_A->data[0], (ptrdiff_t)A->size[0], &s->data[0], &U->data
        [0], (ptrdiff_t)A->size[0], &Vt->data[0], (ptrdiff_t)A->size[1],
        &superb->data[0]);
      m = (int32_T)info_t;
      emxFree_real_T(&c_st, &superb);
    } else {
      m = 0;
    }

    i7 = V->size[0] * V->size[1];
    V->size[0] = Vt->size[1];
    V->size[1] = Vt->size[0];
    emxEnsureCapacity_real_T1(&c_st, V, i7, &bb_emlrtRTEI);
    loop_ub = Vt->size[0];
    for (i7 = 0; i7 < loop_ub; i7++) {
      n = Vt->size[1];
      for (i8 = 0; i8 < n; i8++) {
        V->data[i8 + V->size[0] * i7] = Vt->data[i7 + Vt->size[0] * i8];
      }
    }

    d_st.site = &jd_emlrtRSI;
    if (m < 0) {
      if (m == -1010) {
        e_st.site = &gd_emlrtRSI;
        h_error(&e_st);
      } else {
        e_st.site = &hd_emlrtRSI;
        j_error(&e_st, m);
      }
    }
  } else {
    i7 = V->size[0] * V->size[1];
    V->size[0] = Vt->size[1];
    V->size[1] = Vt->size[0];
    emxEnsureCapacity_real_T1(&st, V, i7, &bb_emlrtRTEI);
    loop_ub = Vt->size[0];
    for (i7 = 0; i7 < loop_ub; i7++) {
      n = Vt->size[1];
      for (i8 = 0; i8 < n; i8++) {
        V->data[i8 + V->size[0] * i7] = Vt->data[i7 + Vt->size[0] * i8];
      }
    }
  }

  emxFree_real_T(&st, &b_A);
  emxFree_real_T(&st, &Vt);
  if (m > 0) {
    b_st.site = &wc_emlrtRSI;
    k_error(&b_st);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (svd1.c) */
