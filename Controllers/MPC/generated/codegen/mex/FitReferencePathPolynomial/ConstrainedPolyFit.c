/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ConstrainedPolyFit.c
 *
 * Code generation for function 'ConstrainedPolyFit'
 *
 */

/* Include files */
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "FitReferencePathPolynomial.h"
#include "ConstrainedPolyFit.h"
#include "FitReferencePathPolynomial_emxutil.h"
#include "power.h"
#include "ConstrainedLeastSquares.h"
#include "error1.h"
#include "FitReferencePathPolynomial_mexutil.h"
#include "FitReferencePathPolynomial_data.h"

/* Variable Definitions */
static emlrtRSInfo kb_emlrtRSI = { 4,  /* lineNo */
  "ConstrainedPolyFit",                /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m"/* pathName */
};

static emlrtRSInfo lb_emlrtRSI = { 6,  /* lineNo */
  "ConstrainedPolyFit",                /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m"/* pathName */
};

static emlrtRSInfo mb_emlrtRSI = { 12, /* lineNo */
  "ConstrainedPolyFit",                /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m"/* pathName */
};

static emlrtRSInfo nb_emlrtRSI = { 16, /* lineNo */
  "ConstrainedPolyFit",                /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m"/* pathName */
};

static emlrtRSInfo ob_emlrtRSI = { 23, /* lineNo */
  "ConstrainedPolyFit",                /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m"/* pathName */
};

static emlrtRSInfo pb_emlrtRSI = { 33, /* lineNo */
  "ConstrainedPolyFit",                /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m"/* pathName */
};

static emlrtRSInfo qb_emlrtRSI = { 38, /* lineNo */
  "ConstrainedPolyFit",                /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m"/* pathName */
};

static emlrtRSInfo rb_emlrtRSI = { 39, /* lineNo */
  "ConstrainedPolyFit",                /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m"/* pathName */
};

static emlrtRSInfo sb_emlrtRSI = { 40, /* lineNo */
  "ConstrainedPolyFit",                /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m"/* pathName */
};

static emlrtRSInfo tb_emlrtRSI = { 41, /* lineNo */
  "ConstrainedPolyFit",                /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m"/* pathName */
};

static emlrtRSInfo ub_emlrtRSI = { 48, /* lineNo */
  "ConstrainedPolyFit",                /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m"/* pathName */
};

static emlrtRSInfo vb_emlrtRSI = { 49, /* lineNo */
  "ConstrainedPolyFit",                /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m"/* pathName */
};

static emlrtRSInfo wb_emlrtRSI = { 64, /* lineNo */
  "ConstrainedPolyFit",                /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m"/* pathName */
};

static emlrtRSInfo cc_emlrtRSI = { 98, /* lineNo */
  "colon",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\ops\\colon.m"/* pathName */
};

static emlrtRSInfo dc_emlrtRSI = { 282,/* lineNo */
  "colon",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\ops\\colon.m"/* pathName */
};

static emlrtRSInfo ec_emlrtRSI = { 290,/* lineNo */
  "colon",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\ops\\colon.m"/* pathName */
};

static emlrtRTEInfo o_emlrtRTEI = { 2, /* lineNo */
  23,                                  /* colNo */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m"/* pName */
};

static emlrtRTEInfo p_emlrtRTEI = { 98,/* lineNo */
  9,                                   /* colNo */
  "colon",                             /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\ops\\colon.m"/* pName */
};

static emlrtRTEInfo q_emlrtRTEI = { 31,/* lineNo */
  5,                                   /* colNo */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m"/* pName */
};

static emlrtRTEInfo r_emlrtRTEI = { 38,/* lineNo */
  9,                                   /* colNo */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m"/* pName */
};

static emlrtRTEInfo rb_emlrtRTEI = { 388,/* lineNo */
  15,                                  /* colNo */
  "colon",                             /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\ops\\colon.m"/* pName */
};

static emlrtECInfo d_emlrtECI = { -1,  /* nDims */
  33,                                  /* lineNo */
  9,                                   /* colNo */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m"/* pName */
};

static emlrtBCInfo l_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  33,                                  /* lineNo */
  13,                                  /* colNo */
  "A",                                 /* aName */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo emlrtDCI = { 33,    /* lineNo */
  13,                                  /* colNo */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m",/* pName */
  1                                    /* checkKind */
};

static emlrtBCInfo m_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  50,                                  /* lineNo */
  22,                                  /* colNo */
  "b",                                 /* aName */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo n_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  49,                                  /* lineNo */
  22,                                  /* colNo */
  "t",                                 /* aName */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo o_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  45,                                  /* lineNo */
  44,                                  /* colNo */
  "t",                                 /* aName */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo p_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  45,                                  /* lineNo */
  37,                                  /* colNo */
  "t",                                 /* aName */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo q_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  45,                                  /* lineNo */
  26,                                  /* colNo */
  "b",                                 /* aName */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo r_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  45,                                  /* lineNo */
  19,                                  /* colNo */
  "b",                                 /* aName */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo s_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  44,                                  /* lineNo */
  31,                                  /* colNo */
  "t",                                 /* aName */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo t_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  44,                                  /* lineNo */
  24,                                  /* colNo */
  "b",                                 /* aName */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo u_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  44,                                  /* lineNo */
  19,                                  /* colNo */
  "b",                                 /* aName */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo v_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  42,                                  /* lineNo */
  18,                                  /* colNo */
  "b",                                 /* aName */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo e_emlrtECI = { 2,   /* nDims */
  41,                                  /* lineNo */
  16,                                  /* colNo */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m"/* pName */
};

static emlrtBCInfo w_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  41,                                  /* lineNo */
  35,                                  /* colNo */
  "t",                                 /* aName */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo f_emlrtECI = { 2,   /* nDims */
  40,                                  /* lineNo */
  16,                                  /* colNo */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m"/* pName */
};

static emlrtBCInfo x_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  39,                                  /* lineNo */
  18,                                  /* colNo */
  "t",                                 /* aName */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo sb_emlrtRTEI = { 32,/* lineNo */
  14,                                  /* colNo */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m"/* pName */
};

static emlrtBCInfo y_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  20,                                  /* lineNo */
  24,                                  /* colNo */
  "t",                                 /* aName */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m",/* pName */
  0                                    /* checkKind */
};

static emlrtDCInfo b_emlrtDCI = { 31,  /* lineNo */
  18,                                  /* colNo */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo c_emlrtDCI = { 31,  /* lineNo */
  18,                                  /* colNo */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m",/* pName */
  4                                    /* checkKind */
};

static emlrtBCInfo ab_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  21,                                  /* lineNo */
  16,                                  /* colNo */
  "t",                                 /* aName */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo bb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  21,                                  /* lineNo */
  26,                                  /* colNo */
  "t",                                 /* aName */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo cb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  51,                                  /* lineNo */
  22,                                  /* colNo */
  "b",                                 /* aName */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo db_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  43,                                  /* lineNo */
  18,                                  /* colNo */
  "b",                                 /* aName */
  "ConstrainedPolyFit",                /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ConstrainedPolyFit.m",/* pName */
  0                                    /* checkKind */
};

/* Function Definitions */
void ConstrainedPolyFit(const emlrtStack *sp, const emxArray_real_T *t, const
  emxArray_real_T *data, real_T order, emxArray_real_T *polyCoeffs)
{
  int32_T i4;
  boolean_T guard1 = false;
  int32_T nm1d2;
  emxArray_real_T *A;
  int32_T k;
  real_T ndbl;
  int32_T loop_ub;
  int32_T i;
  emxArray_int32_T *r0;
  emxArray_real_T *r1;
  real_T apnd;
  emxArray_real_T *Aeq;
  emxArray_real_T *varargin_1;
  emxArray_real_T *varargin_2;
  emxArray_real_T *r2;
  int32_T tmp_size[2];
  int32_T iv5[1];
  real_T tmp_data[1];
  emxArray_real_T b_tmp_data;
  emxArray_real_T *r3;
  int32_T b_tmp_size[2];
  real_T c_tmp_data[1];
  emxArray_real_T d_tmp_data;
  emxArray_real_T *r4;
  boolean_T b0;
  int32_T iv6[2];
  int32_T iv7[2];
  emxArray_real_T *varargin_4;
  real_T b_data[2];
  int32_T beq_size[1];
  real_T beq_data[4];
  emxArray_real_T *r5;
  emxArray_real_T *varargin_3;
  real_T c_data[4];
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

  /*  Constrained Least Squares polynomial fit for arbitrary function parameter however with t(1)=0 */
  if (order < 1.0) {
    st.site = &kb_emlrtRSI;
    c_error(&st);
  } else {
    if (order < 3.0) {
      st.site = &lb_emlrtRSI;
      d_error(&st);
    }
  }

  /*  should be the same as length(data) */
  if (t->size[0] != data->size[0]) {
    st.site = &mb_emlrtRSI;
    e_error(&st);
  }

  if (t->size[0] < order + 1.0) {
    /*  number of coefficients are order+1 */
    st.site = &nb_emlrtRSI;
    f_error(&st);
  }

  i4 = t->size[0];
  if (!(2 <= i4)) {
    emlrtDynamicBoundsCheckR2012b(2, 1, i4, &y_emlrtBCI, sp);
  }

  guard1 = false;
  if (t->data[0] == t->data[1]) {
    guard1 = true;
  } else {
    i4 = t->size[0];
    nm1d2 = t->size[0];
    if (!((nm1d2 >= 1) && (nm1d2 <= i4))) {
      emlrtDynamicBoundsCheckR2012b(nm1d2, 1, i4, &ab_emlrtBCI, sp);
    }

    i4 = t->size[0];
    k = t->size[0] - 1;
    if (!((k >= 1) && (k <= i4))) {
      emlrtDynamicBoundsCheckR2012b(k, 1, i4, &bb_emlrtBCI, sp);
    }

    if (t->data[nm1d2 - 1] == t->data[k - 1]) {
      guard1 = true;
    }
  }

  if (guard1) {
    st.site = &ob_emlrtRSI;
    g_error(&st);
  }

  emxInit_real_T(sp, &A, 2, &q_emlrtRTEI, true);

  /* A = [];     */
  /* for (j = order:-1:0) */
  /*     A = [A, t.^j]; */
  /* end */
  /* A = t.^(order:-1:0); */
  i4 = A->size[0] * A->size[1];
  A->size[0] = t->size[0];
  if (!(order + 1.0 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(order + 1.0, &c_emlrtDCI, sp);
  }

  ndbl = order + 1.0;
  if (ndbl != (int32_T)muDoubleScalarFloor(ndbl)) {
    emlrtIntegerCheckR2012b(ndbl, &b_emlrtDCI, sp);
  }

  A->size[1] = (int32_T)ndbl;
  emxEnsureCapacity_real_T1(sp, A, i4, &o_emlrtRTEI);
  if (!(order + 1.0 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(order + 1.0, &c_emlrtDCI, sp);
  }

  ndbl = order + 1.0;
  if (ndbl != (int32_T)muDoubleScalarFloor(ndbl)) {
    emlrtIntegerCheckR2012b(ndbl, &b_emlrtDCI, sp);
  }

  loop_ub = t->size[0] * (int32_T)ndbl;
  for (i4 = 0; i4 < loop_ub; i4++) {
    A->data[i4] = 0.0;
  }

  i4 = (int32_T)((-1.0 - order) / -1.0);
  emlrtForLoopVectorCheckR2012b(order, -1.0, 0.0, mxDOUBLE_CLASS, i4,
    &sb_emlrtRTEI, sp);
  i = 0;
  emxInit_int32_T(sp, &r0, 1, &o_emlrtRTEI, true);
  emxInit_real_T1(sp, &r1, 1, &o_emlrtRTEI, true);
  while (i <= i4 - 1) {
    apnd = order + -(real_T)i;
    loop_ub = A->size[0];
    nm1d2 = r0->size[0];
    r0->size[0] = loop_ub;
    emxEnsureCapacity_int32_T(sp, r0, nm1d2, &o_emlrtRTEI);
    for (nm1d2 = 0; nm1d2 < loop_ub; nm1d2++) {
      r0->data[nm1d2] = nm1d2;
    }

    nm1d2 = A->size[1];
    ndbl = (order - apnd) + 1.0;
    if (ndbl != (int32_T)muDoubleScalarFloor(ndbl)) {
      emlrtIntegerCheckR2012b(ndbl, &emlrtDCI, sp);
    }

    k = (int32_T)ndbl;
    if (!((k >= 1) && (k <= nm1d2))) {
      emlrtDynamicBoundsCheckR2012b(k, 1, nm1d2, &l_emlrtBCI, sp);
    }

    nm1d2 = k - 1;
    st.site = &pb_emlrtRSI;
    b_power(&st, t, apnd, r1);
    iv5[0] = r0->size[0];
    emlrtSubAssignSizeCheckR2012b(&iv5[0], 1, &(*(int32_T (*)[1])r1->size)[0], 1,
      &d_emlrtECI, sp);
    loop_ub = r1->size[0];
    for (k = 0; k < loop_ub; k++) {
      A->data[r0->data[k] + A->size[0] * nm1d2] = r1->data[k];
    }

    i++;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  emxFree_real_T(sp, &r1);
  emxFree_int32_T(sp, &r0);
  emxInit_real_T(sp, &Aeq, 2, &r_emlrtRTEI, true);
  emxInit_real_T(sp, &varargin_1, 2, &o_emlrtRTEI, true);
  emxInit_real_T(sp, &varargin_2, 2, &o_emlrtRTEI, true);
  if (order >= 3.0) {
    /*  enforce all constraints */
    /*  polynomial evaluated in t(1) */
    /*  polynomial evaluated in t(end) */
    /*  derivative polynomial evaluated in t(1) */
    st.site = &sb_emlrtRSI;
    emxInit_real_T(&st, &r2, 2, &o_emlrtRTEI, true);
    if (muDoubleScalarFloor(order) == order) {
      i4 = r2->size[0] * r2->size[1];
      r2->size[0] = 1;
      r2->size[1] = (int32_T)muDoubleScalarFloor(-(1.0 - order)) + 1;
      emxEnsureCapacity_real_T1(&st, r2, i4, &o_emlrtRTEI);
      loop_ub = (int32_T)muDoubleScalarFloor(-(1.0 - order));
      for (i4 = 0; i4 <= loop_ub; i4++) {
        r2->data[r2->size[0] * i4] = order - (real_T)i4;
      }
    } else {
      b_st.site = &cc_emlrtRSI;
      ndbl = muDoubleScalarFloor((1.0 - order) / -1.0 + 0.5);
      apnd = order + -ndbl;
      if (muDoubleScalarAbs(1.0 - apnd) < 4.4408920985006262E-16 * order) {
        ndbl++;
        apnd = 1.0;
      } else if (1.0 - apnd > 0.0) {
        apnd = order + -(ndbl - 1.0);
      } else {
        ndbl++;
      }

      loop_ub = (int32_T)ndbl;
      c_st.site = &dc_emlrtRSI;
      if (ndbl > 2.147483647E+9) {
        emlrtErrorWithMessageIdR2018a(&c_st, &rb_emlrtRTEI,
          "Coder:MATLAB:pmaxsize", "Coder:MATLAB:pmaxsize", 0);
      }

      i4 = r2->size[0] * r2->size[1];
      r2->size[0] = 1;
      r2->size[1] = loop_ub;
      emxEnsureCapacity_real_T1(&b_st, r2, i4, &p_emlrtRTEI);
      if (loop_ub > 0) {
        r2->data[0] = order;
        if (loop_ub > 1) {
          r2->data[loop_ub - 1] = apnd;
          nm1d2 = (loop_ub - 1) / 2;
          for (k = 1; k < nm1d2; k++) {
            r2->data[k] = order + -(real_T)k;
            r2->data[(loop_ub - k) - 1] = apnd - (-(real_T)k);
          }

          if (nm1d2 << 1 == loop_ub - 1) {
            r2->data[nm1d2] = (order + apnd) / 2.0;
          } else {
            r2->data[nm1d2] = order + -(real_T)nm1d2;
            r2->data[nm1d2 + 1] = apnd - (-(real_T)nm1d2);
          }
        }
      }
    }

    st.site = &sb_emlrtRSI;
    emxInit_real_T(&st, &r3, 2, &o_emlrtRTEI, true);
    if (muDoubleScalarFloor(order - 1.0) == order - 1.0) {
      i4 = r3->size[0] * r3->size[1];
      r3->size[0] = 1;
      r3->size[1] = (int32_T)muDoubleScalarFloor(-(0.0 - (order - 1.0))) + 1;
      emxEnsureCapacity_real_T1(&st, r3, i4, &o_emlrtRTEI);
      loop_ub = (int32_T)muDoubleScalarFloor(-(0.0 - (order - 1.0)));
      for (i4 = 0; i4 <= loop_ub; i4++) {
        r3->data[r3->size[0] * i4] = (order - 1.0) - (real_T)i4;
      }
    } else {
      b_st.site = &cc_emlrtRSI;
      ndbl = muDoubleScalarFloor((0.0 - (order - 1.0)) / -1.0 + 0.5);
      apnd = (order - 1.0) + -ndbl;
      if (muDoubleScalarAbs(0.0 - apnd) < 4.4408920985006262E-16 * (order - 1.0))
      {
        ndbl++;
        apnd = 0.0;
      } else if (0.0 - apnd > 0.0) {
        apnd = (order - 1.0) + -(ndbl - 1.0);
      } else {
        ndbl++;
      }

      loop_ub = (int32_T)ndbl;
      c_st.site = &dc_emlrtRSI;
      if (ndbl > 2.147483647E+9) {
        emlrtErrorWithMessageIdR2018a(&c_st, &rb_emlrtRTEI,
          "Coder:MATLAB:pmaxsize", "Coder:MATLAB:pmaxsize", 0);
      }

      i4 = r3->size[0] * r3->size[1];
      r3->size[0] = 1;
      r3->size[1] = loop_ub;
      emxEnsureCapacity_real_T1(&b_st, r3, i4, &p_emlrtRTEI);
      if (loop_ub > 0) {
        r3->data[0] = order - 1.0;
        if (loop_ub > 1) {
          r3->data[loop_ub - 1] = apnd;
          nm1d2 = (loop_ub - 1) / 2;
          for (k = 1; k < nm1d2; k++) {
            r3->data[k] = (order - 1.0) + -(real_T)k;
            r3->data[(loop_ub - k) - 1] = apnd - (-(real_T)k);
          }

          if (nm1d2 << 1 == loop_ub - 1) {
            r3->data[nm1d2] = ((order - 1.0) + apnd) / 2.0;
          } else {
            r3->data[nm1d2] = (order - 1.0) + -(real_T)nm1d2;
            r3->data[nm1d2 + 1] = apnd - (-(real_T)nm1d2);
          }
        }
      }
    }

    emxInit_real_T(&st, &r4, 2, &o_emlrtRTEI, true);
    st.site = &sb_emlrtRSI;
    c_power(&st, t->data[0], r3, r4);
    for (i4 = 0; i4 < 2; i4++) {
      iv6[i4] = r2->size[i4];
    }

    for (i4 = 0; i4 < 2; i4++) {
      iv7[i4] = r4->size[i4];
    }

    if ((iv6[0] != iv7[0]) || (iv6[1] != iv7[1])) {
      emlrtSizeEqCheckNDR2012b(&iv6[0], &iv7[0], &f_emlrtECI, sp);
    }

    st.site = &tb_emlrtRSI;
    if (muDoubleScalarFloor(order) == order) {
      i4 = r3->size[0] * r3->size[1];
      r3->size[0] = 1;
      r3->size[1] = (int32_T)muDoubleScalarFloor(-(1.0 - order)) + 1;
      emxEnsureCapacity_real_T1(&st, r3, i4, &o_emlrtRTEI);
      loop_ub = (int32_T)muDoubleScalarFloor(-(1.0 - order));
      for (i4 = 0; i4 <= loop_ub; i4++) {
        r3->data[r3->size[0] * i4] = order - (real_T)i4;
      }
    } else {
      b_st.site = &cc_emlrtRSI;
      ndbl = muDoubleScalarFloor((1.0 - order) / -1.0 + 0.5);
      apnd = order + -ndbl;
      if (muDoubleScalarAbs(1.0 - apnd) < 4.4408920985006262E-16 * order) {
        ndbl++;
        apnd = 1.0;
      } else if (1.0 - apnd > 0.0) {
        apnd = order + -(ndbl - 1.0);
      } else {
        ndbl++;
      }

      loop_ub = (int32_T)ndbl;
      c_st.site = &dc_emlrtRSI;
      if (ndbl > 2.147483647E+9) {
        emlrtErrorWithMessageIdR2018a(&c_st, &rb_emlrtRTEI,
          "Coder:MATLAB:pmaxsize", "Coder:MATLAB:pmaxsize", 0);
      }

      i4 = r3->size[0] * r3->size[1];
      r3->size[0] = 1;
      r3->size[1] = loop_ub;
      emxEnsureCapacity_real_T1(&b_st, r3, i4, &p_emlrtRTEI);
      if (loop_ub > 0) {
        r3->data[0] = order;
        if (loop_ub > 1) {
          r3->data[loop_ub - 1] = apnd;
          nm1d2 = (loop_ub - 1) / 2;
          for (k = 1; k < nm1d2; k++) {
            r3->data[k] = order + -(real_T)k;
            r3->data[(loop_ub - k) - 1] = apnd - (-(real_T)k);
          }

          if (nm1d2 << 1 == loop_ub - 1) {
            r3->data[nm1d2] = (order + apnd) / 2.0;
          } else {
            r3->data[nm1d2] = order + -(real_T)nm1d2;
            r3->data[nm1d2 + 1] = apnd - (-(real_T)nm1d2);
          }
        }
      }
    }

    i4 = t->size[0];
    nm1d2 = t->size[0];
    if (!((nm1d2 >= 1) && (nm1d2 <= i4))) {
      emlrtDynamicBoundsCheckR2012b(nm1d2, 1, i4, &w_emlrtBCI, sp);
    }

    st.site = &tb_emlrtRSI;
    emxInit_real_T(&st, &varargin_4, 2, &o_emlrtRTEI, true);
    if (muDoubleScalarFloor(order - 1.0) == order - 1.0) {
      i4 = varargin_4->size[0] * varargin_4->size[1];
      varargin_4->size[0] = 1;
      varargin_4->size[1] = (int32_T)muDoubleScalarFloor(-(0.0 - (order - 1.0)))
        + 1;
      emxEnsureCapacity_real_T1(&st, varargin_4, i4, &o_emlrtRTEI);
      loop_ub = (int32_T)muDoubleScalarFloor(-(0.0 - (order - 1.0)));
      for (i4 = 0; i4 <= loop_ub; i4++) {
        varargin_4->data[varargin_4->size[0] * i4] = (order - 1.0) - (real_T)i4;
      }
    } else {
      b_st.site = &cc_emlrtRSI;
      ndbl = muDoubleScalarFloor((0.0 - (order - 1.0)) / -1.0 + 0.5);
      apnd = (order - 1.0) + -ndbl;
      if (muDoubleScalarAbs(0.0 - apnd) < 4.4408920985006262E-16 * (order - 1.0))
      {
        ndbl++;
        apnd = 0.0;
      } else if (0.0 - apnd > 0.0) {
        apnd = (order - 1.0) + -(ndbl - 1.0);
      } else {
        ndbl++;
      }

      loop_ub = (int32_T)ndbl;
      c_st.site = &dc_emlrtRSI;
      if (ndbl > 2.147483647E+9) {
        emlrtErrorWithMessageIdR2018a(&c_st, &rb_emlrtRTEI,
          "Coder:MATLAB:pmaxsize", "Coder:MATLAB:pmaxsize", 0);
      }

      i4 = varargin_4->size[0] * varargin_4->size[1];
      varargin_4->size[0] = 1;
      varargin_4->size[1] = loop_ub;
      emxEnsureCapacity_real_T1(&b_st, varargin_4, i4, &p_emlrtRTEI);
      if (loop_ub > 0) {
        varargin_4->data[0] = order - 1.0;
        if (loop_ub > 1) {
          varargin_4->data[loop_ub - 1] = apnd;
          nm1d2 = (loop_ub - 1) / 2;
          for (k = 1; k < nm1d2; k++) {
            varargin_4->data[k] = (order - 1.0) + -(real_T)k;
            varargin_4->data[(loop_ub - k) - 1] = apnd - (-(real_T)k);
          }

          if (nm1d2 << 1 == loop_ub - 1) {
            varargin_4->data[nm1d2] = ((order - 1.0) + apnd) / 2.0;
          } else {
            varargin_4->data[nm1d2] = (order - 1.0) + -(real_T)nm1d2;
            varargin_4->data[nm1d2 + 1] = apnd - (-(real_T)nm1d2);
          }
        }
      }
    }

    emxInit_real_T(&st, &r5, 2, &o_emlrtRTEI, true);
    st.site = &tb_emlrtRSI;
    c_power(&st, t->data[t->size[0] - 1], varargin_4, r5);
    for (i4 = 0; i4 < 2; i4++) {
      iv6[i4] = r3->size[i4];
    }

    for (i4 = 0; i4 < 2; i4++) {
      iv7[i4] = r5->size[i4];
    }

    if ((iv6[0] != iv7[0]) || (iv6[1] != iv7[1])) {
      emlrtSizeEqCheckNDR2012b(&iv6[0], &iv7[0], &e_emlrtECI, sp);
    }

    st.site = &qb_emlrtRSI;
    if (muDoubleScalarFloor(order) == order) {
      i4 = varargin_4->size[0] * varargin_4->size[1];
      varargin_4->size[0] = 1;
      varargin_4->size[1] = (int32_T)muDoubleScalarFloor(-(0.0 - order)) + 1;
      emxEnsureCapacity_real_T1(&st, varargin_4, i4, &o_emlrtRTEI);
      loop_ub = (int32_T)muDoubleScalarFloor(-(0.0 - order));
      for (i4 = 0; i4 <= loop_ub; i4++) {
        varargin_4->data[varargin_4->size[0] * i4] = order - (real_T)i4;
      }
    } else {
      b_st.site = &cc_emlrtRSI;
      ndbl = muDoubleScalarFloor((0.0 - order) / -1.0 + 0.5);
      apnd = order + -ndbl;
      if (muDoubleScalarAbs(0.0 - apnd) < 4.4408920985006262E-16 * order) {
        ndbl++;
        apnd = 0.0;
      } else if (0.0 - apnd > 0.0) {
        apnd = order + -(ndbl - 1.0);
      } else {
        ndbl++;
      }

      loop_ub = (int32_T)ndbl;
      c_st.site = &dc_emlrtRSI;
      if (ndbl > 2.147483647E+9) {
        emlrtErrorWithMessageIdR2018a(&c_st, &rb_emlrtRTEI,
          "Coder:MATLAB:pmaxsize", "Coder:MATLAB:pmaxsize", 0);
      }

      i4 = varargin_4->size[0] * varargin_4->size[1];
      varargin_4->size[0] = 1;
      varargin_4->size[1] = loop_ub;
      emxEnsureCapacity_real_T1(&b_st, varargin_4, i4, &p_emlrtRTEI);
      if (loop_ub > 0) {
        varargin_4->data[0] = order;
        if (loop_ub > 1) {
          varargin_4->data[loop_ub - 1] = apnd;
          nm1d2 = (loop_ub - 1) / 2;
          for (k = 1; k < nm1d2; k++) {
            varargin_4->data[k] = order + -(real_T)k;
            varargin_4->data[(loop_ub - k) - 1] = apnd - (-(real_T)k);
          }

          if (nm1d2 << 1 == loop_ub - 1) {
            varargin_4->data[nm1d2] = (order + apnd) / 2.0;
          } else {
            varargin_4->data[nm1d2] = order + -(real_T)nm1d2;
            varargin_4->data[nm1d2 + 1] = apnd - (-(real_T)nm1d2);
          }
        }
      }
    }

    i4 = t->size[0];
    nm1d2 = t->size[0];
    if (!((nm1d2 >= 1) && (nm1d2 <= i4))) {
      emlrtDynamicBoundsCheckR2012b(nm1d2, 1, i4, &x_emlrtBCI, sp);
    }

    st.site = &rb_emlrtRSI;
    emxInit_real_T(&st, &varargin_3, 2, &o_emlrtRTEI, true);
    if (muDoubleScalarFloor(order) == order) {
      i4 = varargin_3->size[0] * varargin_3->size[1];
      varargin_3->size[0] = 1;
      varargin_3->size[1] = (int32_T)muDoubleScalarFloor(-(0.0 - order)) + 1;
      emxEnsureCapacity_real_T1(&st, varargin_3, i4, &o_emlrtRTEI);
      loop_ub = (int32_T)muDoubleScalarFloor(-(0.0 - order));
      for (i4 = 0; i4 <= loop_ub; i4++) {
        varargin_3->data[varargin_3->size[0] * i4] = order - (real_T)i4;
      }
    } else {
      b_st.site = &cc_emlrtRSI;
      ndbl = muDoubleScalarFloor((0.0 - order) / -1.0 + 0.5);
      apnd = order + -ndbl;
      if (muDoubleScalarAbs(0.0 - apnd) < 4.4408920985006262E-16 * order) {
        ndbl++;
        apnd = 0.0;
      } else if (0.0 - apnd > 0.0) {
        apnd = order + -(ndbl - 1.0);
      } else {
        ndbl++;
      }

      loop_ub = (int32_T)ndbl;
      c_st.site = &dc_emlrtRSI;
      if (ndbl > 2.147483647E+9) {
        emlrtErrorWithMessageIdR2018a(&c_st, &rb_emlrtRTEI,
          "Coder:MATLAB:pmaxsize", "Coder:MATLAB:pmaxsize", 0);
      }

      i4 = varargin_3->size[0] * varargin_3->size[1];
      varargin_3->size[0] = 1;
      varargin_3->size[1] = loop_ub;
      emxEnsureCapacity_real_T1(&b_st, varargin_3, i4, &p_emlrtRTEI);
      if (loop_ub > 0) {
        varargin_3->data[0] = order;
        if (loop_ub > 1) {
          varargin_3->data[loop_ub - 1] = apnd;
          nm1d2 = (loop_ub - 1) / 2;
          for (k = 1; k < nm1d2; k++) {
            varargin_3->data[k] = order + -(real_T)k;
            varargin_3->data[(loop_ub - k) - 1] = apnd - (-(real_T)k);
          }

          if (nm1d2 << 1 == loop_ub - 1) {
            varargin_3->data[nm1d2] = (order + apnd) / 2.0;
          } else {
            varargin_3->data[nm1d2] = order + -(real_T)nm1d2;
            varargin_3->data[nm1d2 + 1] = apnd - (-(real_T)nm1d2);
          }
        }
      }
    }

    st.site = &qb_emlrtRSI;
    c_power(&st, t->data[0], varargin_4, varargin_1);
    st.site = &rb_emlrtRSI;
    c_power(&st, t->data[t->size[0] - 1], varargin_3, varargin_2);
    st.site = &qb_emlrtRSI;
    i4 = varargin_3->size[0] * varargin_3->size[1];
    varargin_3->size[0] = 1;
    varargin_3->size[1] = r2->size[1] + 1;
    emxEnsureCapacity_real_T1(&st, varargin_3, i4, &o_emlrtRTEI);
    loop_ub = r2->size[1];
    for (i4 = 0; i4 < loop_ub; i4++) {
      varargin_3->data[varargin_3->size[0] * i4] = r2->data[r2->size[0] * i4] *
        r4->data[r4->size[0] * i4];
    }

    emxFree_real_T(&st, &r4);
    varargin_3->data[varargin_3->size[0] * r2->size[1]] = 0.0;
    i4 = varargin_4->size[0] * varargin_4->size[1];
    varargin_4->size[0] = 1;
    varargin_4->size[1] = r3->size[1] + 1;
    emxEnsureCapacity_real_T1(&st, varargin_4, i4, &o_emlrtRTEI);
    loop_ub = r3->size[1];
    emxFree_real_T(&st, &r2);
    for (i4 = 0; i4 < loop_ub; i4++) {
      varargin_4->data[varargin_4->size[0] * i4] = r3->data[r3->size[0] * i4] *
        r5->data[r5->size[0] * i4];
    }

    emxFree_real_T(&st, &r5);
    varargin_4->data[varargin_4->size[0] * r3->size[1]] = 0.0;
    b_st.site = &hc_emlrtRSI;
    c_st.site = &ic_emlrtRSI;
    b0 = true;
    emxFree_real_T(&c_st, &r3);
    if (varargin_2->size[1] != varargin_1->size[1]) {
      b0 = false;
    }

    if (!b0) {
      emlrtErrorWithMessageIdR2018a(&c_st, &mb_emlrtRTEI,
        "MATLAB:catenate:matrixDimensionMismatch",
        "MATLAB:catenate:matrixDimensionMismatch", 0);
    }

    if (varargin_3->size[1] != varargin_1->size[1]) {
      b0 = false;
    }

    if (!b0) {
      emlrtErrorWithMessageIdR2018a(&c_st, &mb_emlrtRTEI,
        "MATLAB:catenate:matrixDimensionMismatch",
        "MATLAB:catenate:matrixDimensionMismatch", 0);
    }

    if (varargin_4->size[1] != varargin_1->size[1]) {
      b0 = false;
    }

    if (!b0) {
      emlrtErrorWithMessageIdR2018a(&c_st, &mb_emlrtRTEI,
        "MATLAB:catenate:matrixDimensionMismatch",
        "MATLAB:catenate:matrixDimensionMismatch", 0);
    }

    i4 = Aeq->size[0] * Aeq->size[1];
    Aeq->size[0] = 4;
    Aeq->size[1] = varargin_1->size[1];
    emxEnsureCapacity_real_T1(sp, Aeq, i4, &o_emlrtRTEI);
    loop_ub = varargin_1->size[1];
    for (i4 = 0; i4 < loop_ub; i4++) {
      Aeq->data[Aeq->size[0] * i4] = varargin_1->data[varargin_1->size[0] * i4];
    }

    loop_ub = varargin_2->size[1];
    for (i4 = 0; i4 < loop_ub; i4++) {
      Aeq->data[1 + Aeq->size[0] * i4] = varargin_2->data[varargin_2->size[0] *
        i4];
    }

    loop_ub = varargin_3->size[1];
    for (i4 = 0; i4 < loop_ub; i4++) {
      Aeq->data[2 + Aeq->size[0] * i4] = varargin_3->data[varargin_3->size[0] *
        i4];
    }

    emxFree_real_T(sp, &varargin_3);
    loop_ub = varargin_4->size[1];
    for (i4 = 0; i4 < loop_ub; i4++) {
      Aeq->data[3 + Aeq->size[0] * i4] = varargin_4->data[varargin_4->size[0] *
        i4];
    }

    emxFree_real_T(sp, &varargin_4);

    /*  derivative polynomial evaluated in t(end) */
    i4 = data->size[0];
    if (!(1 <= i4)) {
      emlrtDynamicBoundsCheckR2012b(1, 1, i4, &v_emlrtBCI, sp);
    }

    i4 = data->size[0];
    if (!(2 <= i4)) {
      emlrtDynamicBoundsCheckR2012b(2, 1, i4, &u_emlrtBCI, sp);
    }

    i4 = data->size[0];
    if (!(1 <= i4)) {
      emlrtDynamicBoundsCheckR2012b(1, 1, i4, &t_emlrtBCI, sp);
    }

    i4 = t->size[0];
    if (!(2 <= i4)) {
      emlrtDynamicBoundsCheckR2012b(2, 1, i4, &s_emlrtBCI, sp);
    }

    i4 = data->size[0];
    nm1d2 = data->size[0];
    if (!((nm1d2 >= 1) && (nm1d2 <= i4))) {
      emlrtDynamicBoundsCheckR2012b(nm1d2, 1, i4, &r_emlrtBCI, sp);
    }

    i4 = data->size[0];
    nm1d2 = (int32_T)((real_T)data->size[0] - 1.0);
    if (!((nm1d2 >= 1) && (nm1d2 <= i4))) {
      emlrtDynamicBoundsCheckR2012b(nm1d2, 1, i4, &q_emlrtBCI, sp);
    }

    i4 = t->size[0];
    nm1d2 = t->size[0];
    if (!((nm1d2 >= 1) && (nm1d2 <= i4))) {
      emlrtDynamicBoundsCheckR2012b(nm1d2, 1, i4, &p_emlrtBCI, sp);
    }

    i4 = t->size[0];
    nm1d2 = (int32_T)((real_T)t->size[0] - 1.0);
    if (!((nm1d2 >= 1) && (nm1d2 <= i4))) {
      emlrtDynamicBoundsCheckR2012b(nm1d2, 1, i4, &o_emlrtBCI, sp);
    }

    c_data[0] = data->data[0];
    i4 = data->size[0];
    nm1d2 = data->size[0];
    if (!((nm1d2 >= 1) && (nm1d2 <= i4))) {
      emlrtDynamicBoundsCheckR2012b(nm1d2, 1, i4, &db_emlrtBCI, sp);
    }

    c_data[1] = data->data[nm1d2 - 1];
    c_data[2] = (data->data[1] - data->data[0]) / (t->data[1] - t->data[0]);
    c_data[3] = (data->data[data->size[0] - 1] - data->data[data->size[0] - 2]) /
      (t->data[t->size[0] - 1] - t->data[t->size[0] - 2]);
    beq_size[0] = 4;
    for (i4 = 0; i4 < 4; i4++) {
      beq_data[i4] = c_data[i4];
    }
  } else {
    /*  limit constraints due to reduced order such that we only require start and end point to be fulfilled */
    /*  polynomial evaluated in t(1) */
    st.site = &ub_emlrtRSI;
    i4 = t->size[0];
    nm1d2 = t->size[0];
    if (!((nm1d2 >= 1) && (nm1d2 <= i4))) {
      emlrtDynamicBoundsCheckR2012b(nm1d2, 1, i4, &n_emlrtBCI, sp);
    }

    st.site = &vb_emlrtRSI;
    tmp_size[0] = 1;
    tmp_size[1] = 1;
    tmp_data[0] = rtNaN;
    b_tmp_data.data = (real_T *)&tmp_data;
    b_tmp_data.size = (int32_T *)&tmp_size;
    b_tmp_data.allocatedSize = 1;
    b_tmp_data.numDimensions = 2;
    b_tmp_data.canFreeData = false;
    st.site = &ub_emlrtRSI;
    c_power(&st, t->data[0], &b_tmp_data, varargin_1);
    b_tmp_size[0] = 1;
    b_tmp_size[1] = 1;
    c_tmp_data[0] = rtNaN;
    d_tmp_data.data = (real_T *)&c_tmp_data;
    d_tmp_data.size = (int32_T *)&b_tmp_size;
    d_tmp_data.allocatedSize = 1;
    d_tmp_data.numDimensions = 2;
    d_tmp_data.canFreeData = false;
    st.site = &vb_emlrtRSI;
    c_power(&st, t->data[t->size[0] - 1], &d_tmp_data, varargin_2);
    st.site = &ub_emlrtRSI;
    b_st.site = &hc_emlrtRSI;
    c_st.site = &ic_emlrtRSI;
    b0 = true;
    if (varargin_2->size[1] != varargin_1->size[1]) {
      b0 = false;
    }

    if (!b0) {
      emlrtErrorWithMessageIdR2018a(&c_st, &mb_emlrtRTEI,
        "MATLAB:catenate:matrixDimensionMismatch",
        "MATLAB:catenate:matrixDimensionMismatch", 0);
    }

    i4 = Aeq->size[0] * Aeq->size[1];
    Aeq->size[0] = 2;
    Aeq->size[1] = varargin_1->size[1];
    emxEnsureCapacity_real_T1(sp, Aeq, i4, &o_emlrtRTEI);
    loop_ub = varargin_1->size[1];
    for (i4 = 0; i4 < loop_ub; i4++) {
      Aeq->data[Aeq->size[0] * i4] = varargin_1->data[varargin_1->size[0] * i4];
    }

    loop_ub = varargin_2->size[1];
    for (i4 = 0; i4 < loop_ub; i4++) {
      Aeq->data[1 + Aeq->size[0] * i4] = varargin_2->data[varargin_2->size[0] *
        i4];
    }

    /*  polynomial evaluated in t(end)  */
    i4 = data->size[0];
    if (!(1 <= i4)) {
      emlrtDynamicBoundsCheckR2012b(1, 1, i4, &m_emlrtBCI, sp);
    }

    b_data[0] = data->data[0];
    i4 = data->size[0];
    nm1d2 = data->size[0];
    if (!((nm1d2 >= 1) && (nm1d2 <= i4))) {
      emlrtDynamicBoundsCheckR2012b(nm1d2, 1, i4, &cb_emlrtBCI, sp);
    }

    b_data[1] = data->data[nm1d2 - 1];
    beq_size[0] = 2;
    for (i4 = 0; i4 < 2; i4++) {
      beq_data[i4] = b_data[i4];
    }
  }

  emxFree_real_T(sp, &varargin_2);
  emxFree_real_T(sp, &varargin_1);

  /* opts = optimset('display','off'); */
  /* polyCoeffs = lsqlin(A,b, [], [], Aeq, beq, [], [], [], opts); */
  st.site = &wb_emlrtRSI;
  ConstrainedLeastSquares(&st, A, data, Aeq, beq_data, beq_size, (real_T)t->
    size[0] * 10000.0, polyCoeffs);

  /*      t_visualize_fit = linspace(0,1,100); */
  /*      data_fit = polyval(polyCoeffs, t_visualize_fit); */
  /*      figure(10); */
  /*      plot(t,data,'r--'); */
  /*      hold on; */
  /*      plot(t_visualize_fit,data_fit,'b--'); */
  /*      hold off; */
  emxFree_real_T(sp, &Aeq);
  emxFree_real_T(sp, &A);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void b_ConstrainedPolyFit(const emlrtStack *sp, const emxArray_real_T *t, const
  emxArray_real_T *data, real_T order, emxArray_real_T *polyCoeffs)
{
  const mxArray *y;
  const mxArray *m5;
  static const int32_T iv8[2] = { 1, 39 };

  int32_T i21;
  static const int32_T iv9[2] = { 1, 57 };

  int32_T nm1d2;
  static const int32_T iv10[2] = { 1, 75 };

  int32_T k;
  emxArray_real_T *A;
  static const int32_T iv11[2] = { 1, 109 };

  real_T ndbl;
  int32_T loop_ub;
  int32_T i;
  emxArray_int32_T *r10;
  emxArray_real_T *r11;
  real_T apnd;
  emxArray_real_T *varargin_2;
  real_T absa;
  int32_T iv12[1];
  emxArray_real_T *r12;
  emxArray_real_T *varargin_1;
  boolean_T b1;
  emxArray_real_T *b_varargin_1;
  real_T b_data[2];
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

  /*  Constrained Least Squares polynomial fit for arbitrary function parameter however with t(1)=0 */
  if (order < 1.0) {
    st.site = &kb_emlrtRSI;
    y = NULL;
    m5 = emlrtCreateCharArray(2, iv8);
    emlrtInitCharArrayR2013a(&st, 39, m5, &cv0[0]);
    emlrtAssign(&y, m5);
    b_st.site = &ke_emlrtRSI;
    l_error(&b_st, y, &emlrtMCI);
  }

  /*  should be the same as length(data) */
  if (t->size[0] != data->size[0]) {
    st.site = &mb_emlrtRSI;
    y = NULL;
    m5 = emlrtCreateCharArray(2, iv9);
    emlrtInitCharArrayR2013a(&st, 57, m5, &cv1[0]);
    emlrtAssign(&y, m5);
    b_st.site = &ke_emlrtRSI;
    l_error(&b_st, y, &emlrtMCI);
  }

  if (t->size[0] < order + 1.0) {
    /*  number of coefficients are order+1 */
    st.site = &nb_emlrtRSI;
    y = NULL;
    m5 = emlrtCreateCharArray(2, iv10);
    emlrtInitCharArrayR2013a(&st, 75, m5, &cv2[0]);
    emlrtAssign(&y, m5);
    b_st.site = &ke_emlrtRSI;
    l_error(&b_st, y, &emlrtMCI);
  }

  i21 = t->size[0];
  nm1d2 = t->size[0];
  if (!((nm1d2 >= 1) && (nm1d2 <= i21))) {
    emlrtDynamicBoundsCheckR2012b(nm1d2, 1, i21, &ab_emlrtBCI, sp);
  }

  i21 = t->size[0];
  k = t->size[0] - 1;
  if (!((k >= 1) && (k <= i21))) {
    emlrtDynamicBoundsCheckR2012b(k, 1, i21, &bb_emlrtBCI, sp);
  }

  if (t->data[nm1d2 - 1] == t->data[k - 1]) {
    st.site = &ob_emlrtRSI;
    y = NULL;
    m5 = emlrtCreateCharArray(2, iv11);
    emlrtInitCharArrayR2013a(&st, 109, m5, &cv3[0]);
    emlrtAssign(&y, m5);
    b_st.site = &ke_emlrtRSI;
    l_error(&b_st, y, &emlrtMCI);
  }

  emxInit_real_T(sp, &A, 2, &q_emlrtRTEI, true);

  /* A = [];     */
  /* for (j = order:-1:0) */
  /*     A = [A, t.^j]; */
  /* end */
  /* A = t.^(order:-1:0); */
  i21 = A->size[0] * A->size[1];
  A->size[0] = t->size[0];
  if (!(order + 1.0 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(order + 1.0, &c_emlrtDCI, sp);
  }

  ndbl = order + 1.0;
  if (ndbl != (int32_T)muDoubleScalarFloor(ndbl)) {
    emlrtIntegerCheckR2012b(ndbl, &b_emlrtDCI, sp);
  }

  A->size[1] = (int32_T)ndbl;
  emxEnsureCapacity_real_T1(sp, A, i21, &o_emlrtRTEI);
  if (!(order + 1.0 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(order + 1.0, &c_emlrtDCI, sp);
  }

  ndbl = order + 1.0;
  if (ndbl != (int32_T)muDoubleScalarFloor(ndbl)) {
    emlrtIntegerCheckR2012b(ndbl, &b_emlrtDCI, sp);
  }

  loop_ub = t->size[0] * (int32_T)ndbl;
  for (i21 = 0; i21 < loop_ub; i21++) {
    A->data[i21] = 0.0;
  }

  i21 = (int32_T)((-1.0 - order) / -1.0);
  emlrtForLoopVectorCheckR2012b(order, -1.0, 0.0, mxDOUBLE_CLASS, i21,
    &sb_emlrtRTEI, sp);
  i = 0;
  emxInit_int32_T(sp, &r10, 1, &o_emlrtRTEI, true);
  emxInit_real_T1(sp, &r11, 1, &o_emlrtRTEI, true);
  while (i <= i21 - 1) {
    apnd = order + -(real_T)i;
    loop_ub = A->size[0];
    nm1d2 = r10->size[0];
    r10->size[0] = loop_ub;
    emxEnsureCapacity_int32_T(sp, r10, nm1d2, &o_emlrtRTEI);
    for (nm1d2 = 0; nm1d2 < loop_ub; nm1d2++) {
      r10->data[nm1d2] = nm1d2;
    }

    ndbl = (order - apnd) + 1.0;
    if (ndbl != (int32_T)muDoubleScalarFloor(ndbl)) {
      emlrtIntegerCheckR2012b(ndbl, &emlrtDCI, sp);
    }

    nm1d2 = A->size[1];
    k = (int32_T)ndbl;
    if (!((k >= 1) && (k <= nm1d2))) {
      emlrtDynamicBoundsCheckR2012b(k, 1, nm1d2, &l_emlrtBCI, sp);
    }

    nm1d2 = k - 1;
    st.site = &pb_emlrtRSI;
    b_power(&st, t, apnd, r11);
    iv12[0] = r10->size[0];
    emlrtSubAssignSizeCheckR2012b(&iv12[0], 1, &(*(int32_T (*)[1])r11->size)[0],
      1, &d_emlrtECI, sp);
    loop_ub = r11->size[0];
    for (k = 0; k < loop_ub; k++) {
      A->data[r10->data[k] + A->size[0] * nm1d2] = r11->data[k];
    }

    i++;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  emxFree_real_T(sp, &r11);
  emxFree_int32_T(sp, &r10);

  /*  limit constraints due to reduced order such that we only require start and end point to be fulfilled */
  /*  polynomial evaluated in t(1) */
  st.site = &ub_emlrtRSI;
  emxInit_real_T(&st, &varargin_2, 2, &o_emlrtRTEI, true);
  if (muDoubleScalarIsNaN(order)) {
    i21 = varargin_2->size[0] * varargin_2->size[1];
    varargin_2->size[0] = 1;
    varargin_2->size[1] = 1;
    emxEnsureCapacity_real_T1(&st, varargin_2, i21, &o_emlrtRTEI);
    varargin_2->data[0] = rtNaN;
  } else if (muDoubleScalarFloor(order) == order) {
    i21 = varargin_2->size[0] * varargin_2->size[1];
    varargin_2->size[0] = 1;
    varargin_2->size[1] = (int32_T)muDoubleScalarFloor(-(0.0 - order)) + 1;
    emxEnsureCapacity_real_T1(&st, varargin_2, i21, &o_emlrtRTEI);
    loop_ub = (int32_T)muDoubleScalarFloor(-(0.0 - order));
    for (i21 = 0; i21 <= loop_ub; i21++) {
      varargin_2->data[varargin_2->size[0] * i21] = order - (real_T)i21;
    }
  } else {
    b_st.site = &cc_emlrtRSI;
    ndbl = muDoubleScalarFloor((0.0 - order) / -1.0 + 0.5);
    apnd = order + -ndbl;
    absa = muDoubleScalarAbs(order);
    if (muDoubleScalarAbs(0.0 - apnd) < 4.4408920985006262E-16 *
        muDoubleScalarMax(absa, 0.0)) {
      ndbl++;
      apnd = 0.0;
    } else if (0.0 - apnd > 0.0) {
      apnd = order + -(ndbl - 1.0);
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      loop_ub = (int32_T)ndbl;
    } else {
      loop_ub = 0;
    }

    c_st.site = &dc_emlrtRSI;
    if (ndbl > 2.147483647E+9) {
      emlrtErrorWithMessageIdR2018a(&c_st, &rb_emlrtRTEI,
        "Coder:MATLAB:pmaxsize", "Coder:MATLAB:pmaxsize", 0);
    }

    i21 = varargin_2->size[0] * varargin_2->size[1];
    varargin_2->size[0] = 1;
    varargin_2->size[1] = loop_ub;
    emxEnsureCapacity_real_T1(&b_st, varargin_2, i21, &p_emlrtRTEI);
    if (loop_ub > 0) {
      varargin_2->data[0] = order;
      if (loop_ub > 1) {
        varargin_2->data[loop_ub - 1] = apnd;
        nm1d2 = (loop_ub - 1) / 2;
        c_st.site = &ec_emlrtRSI;
        for (k = 1; k < nm1d2; k++) {
          varargin_2->data[k] = order + -(real_T)k;
          varargin_2->data[(loop_ub - k) - 1] = apnd - (-(real_T)k);
        }

        if (nm1d2 << 1 == loop_ub - 1) {
          varargin_2->data[nm1d2] = (order + apnd) / 2.0;
        } else {
          varargin_2->data[nm1d2] = order + -(real_T)nm1d2;
          varargin_2->data[nm1d2 + 1] = apnd - (-(real_T)nm1d2);
        }
      }
    }
  }

  i21 = t->size[0];
  nm1d2 = t->size[0];
  if (!((nm1d2 >= 1) && (nm1d2 <= i21))) {
    emlrtDynamicBoundsCheckR2012b(nm1d2, 1, i21, &n_emlrtBCI, sp);
  }

  st.site = &vb_emlrtRSI;
  emxInit_real_T(&st, &r12, 2, &o_emlrtRTEI, true);
  if (muDoubleScalarIsNaN(order)) {
    i21 = r12->size[0] * r12->size[1];
    r12->size[0] = 1;
    r12->size[1] = 1;
    emxEnsureCapacity_real_T1(&st, r12, i21, &o_emlrtRTEI);
    r12->data[0] = rtNaN;
  } else if (muDoubleScalarFloor(order) == order) {
    i21 = r12->size[0] * r12->size[1];
    r12->size[0] = 1;
    r12->size[1] = (int32_T)muDoubleScalarFloor(-(0.0 - order)) + 1;
    emxEnsureCapacity_real_T1(&st, r12, i21, &o_emlrtRTEI);
    loop_ub = (int32_T)muDoubleScalarFloor(-(0.0 - order));
    for (i21 = 0; i21 <= loop_ub; i21++) {
      r12->data[r12->size[0] * i21] = order - (real_T)i21;
    }
  } else {
    b_st.site = &cc_emlrtRSI;
    ndbl = muDoubleScalarFloor((0.0 - order) / -1.0 + 0.5);
    apnd = order + -ndbl;
    absa = muDoubleScalarAbs(order);
    if (muDoubleScalarAbs(0.0 - apnd) < 4.4408920985006262E-16 *
        muDoubleScalarMax(absa, 0.0)) {
      ndbl++;
      apnd = 0.0;
    } else if (0.0 - apnd > 0.0) {
      apnd = order + -(ndbl - 1.0);
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      loop_ub = (int32_T)ndbl;
    } else {
      loop_ub = 0;
    }

    c_st.site = &dc_emlrtRSI;
    if (ndbl > 2.147483647E+9) {
      emlrtErrorWithMessageIdR2018a(&c_st, &rb_emlrtRTEI,
        "Coder:MATLAB:pmaxsize", "Coder:MATLAB:pmaxsize", 0);
    }

    i21 = r12->size[0] * r12->size[1];
    r12->size[0] = 1;
    r12->size[1] = loop_ub;
    emxEnsureCapacity_real_T1(&b_st, r12, i21, &p_emlrtRTEI);
    if (loop_ub > 0) {
      r12->data[0] = order;
      if (loop_ub > 1) {
        r12->data[loop_ub - 1] = apnd;
        nm1d2 = (loop_ub - 1) / 2;
        c_st.site = &ec_emlrtRSI;
        for (k = 1; k < nm1d2; k++) {
          r12->data[k] = order + -(real_T)k;
          r12->data[(loop_ub - k) - 1] = apnd - (-(real_T)k);
        }

        if (nm1d2 << 1 == loop_ub - 1) {
          r12->data[nm1d2] = (order + apnd) / 2.0;
        } else {
          r12->data[nm1d2] = order + -(real_T)nm1d2;
          r12->data[nm1d2 + 1] = apnd - (-(real_T)nm1d2);
        }
      }
    }
  }

  emxInit_real_T(&st, &varargin_1, 2, &o_emlrtRTEI, true);
  st.site = &ub_emlrtRSI;
  c_power(&st, t->data[0], varargin_2, varargin_1);
  st.site = &vb_emlrtRSI;
  c_power(&st, t->data[t->size[0] - 1], r12, varargin_2);
  st.site = &ub_emlrtRSI;
  b_st.site = &hc_emlrtRSI;
  c_st.site = &ic_emlrtRSI;
  b1 = true;
  emxFree_real_T(&c_st, &r12);
  if (varargin_2->size[1] != varargin_1->size[1]) {
    b1 = false;
  }

  if (!b1) {
    emlrtErrorWithMessageIdR2018a(&c_st, &mb_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  emxInit_real_T(&c_st, &b_varargin_1, 2, &o_emlrtRTEI, true);

  /*  polynomial evaluated in t(end)  */
  i21 = data->size[0];
  nm1d2 = data->size[0];
  if (!((nm1d2 >= 1) && (nm1d2 <= i21))) {
    emlrtDynamicBoundsCheckR2012b(nm1d2, 1, i21, &cb_emlrtBCI, sp);
  }

  /* opts = optimset('display','off'); */
  /* polyCoeffs = lsqlin(A,b, [], [], Aeq, beq, [], [], [], opts); */
  i21 = b_varargin_1->size[0] * b_varargin_1->size[1];
  b_varargin_1->size[0] = 2;
  b_varargin_1->size[1] = varargin_1->size[1];
  emxEnsureCapacity_real_T1(sp, b_varargin_1, i21, &o_emlrtRTEI);
  loop_ub = varargin_1->size[1];
  for (i21 = 0; i21 < loop_ub; i21++) {
    b_varargin_1->data[b_varargin_1->size[0] * i21] = varargin_1->
      data[varargin_1->size[0] * i21];
  }

  emxFree_real_T(sp, &varargin_1);
  loop_ub = varargin_2->size[1];
  for (i21 = 0; i21 < loop_ub; i21++) {
    b_varargin_1->data[1 + b_varargin_1->size[0] * i21] = varargin_2->
      data[varargin_2->size[0] * i21];
  }

  emxFree_real_T(sp, &varargin_2);
  b_data[0] = data->data[0];
  b_data[1] = data->data[data->size[0] - 1];
  st.site = &wb_emlrtRSI;
  b_ConstrainedLeastSquares(&st, A, data, b_varargin_1, b_data, (real_T)t->size
    [0] * 10000.0, polyCoeffs);

  /*      t_visualize_fit = linspace(0,1,100); */
  /*      data_fit = polyval(polyCoeffs, t_visualize_fit); */
  /*      figure(10); */
  /*      plot(t,data,'r--'); */
  /*      hold on; */
  /*      plot(t_visualize_fit,data_fit,'b--'); */
  /*      hold off; */
  emxFree_real_T(sp, &b_varargin_1);
  emxFree_real_T(sp, &A);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void c_ConstrainedPolyFit(const emlrtStack *sp, const real_T t[100], const
  real_T data[100], real_T order, emxArray_real_T *polyCoeffs)
{
  emxArray_real_T *A;
  int32_T i26;
  real_T ndbl;
  int32_T nm1d2;
  emxArray_real_T *Aeq;
  real_T apnd;
  emxArray_real_T *varargin_1;
  int32_T k;
  emxArray_real_T *varargin_2;
  int32_T n;
  emxArray_real_T *r15;
  int32_T tmp_size[2];
  real_T tmp_data[1];
  emxArray_real_T b_tmp_data;
  int32_T b_tmp_size[2];
  real_T c_tmp_data[1];
  emxArray_real_T *r16;
  emxArray_real_T d_tmp_data;
  boolean_T b2;
  emxArray_real_T *r17;
  int32_T iv13[2];
  int32_T iv14[2];
  real_T b_data[2];
  int32_T beq_size[1];
  emxArray_real_T *varargin_4;
  real_T beq_data[4];
  emxArray_real_T *r18;
  emxArray_real_T *varargin_3;
  real_T c_data[4];
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

  /*  Constrained Least Squares polynomial fit for arbitrary function parameter however with t(1)=0 */
  if (order < 1.0) {
    st.site = &kb_emlrtRSI;
    c_error(&st);
  } else {
    if (order < 3.0) {
      st.site = &lb_emlrtRSI;
      d_error(&st);
    }
  }

  /*  should be the same as length(data) */
  if (100.0 < order + 1.0) {
    /*  number of coefficients are order+1 */
    st.site = &nb_emlrtRSI;
    f_error(&st);
  }

  if ((t[0] == t[1]) || (t[99] == t[98])) {
    st.site = &ob_emlrtRSI;
    g_error(&st);
  }

  emxInit_real_T(sp, &A, 2, &q_emlrtRTEI, true);

  /* A = [];     */
  /* for (j = order:-1:0) */
  /*     A = [A, t.^j]; */
  /* end */
  /* A = t.^(order:-1:0); */
  i26 = A->size[0] * A->size[1];
  A->size[0] = 100;
  if (!(order + 1.0 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(order + 1.0, &c_emlrtDCI, sp);
  }

  ndbl = order + 1.0;
  if (ndbl != (int32_T)muDoubleScalarFloor(ndbl)) {
    emlrtIntegerCheckR2012b(ndbl, &b_emlrtDCI, sp);
  }

  A->size[1] = (int32_T)ndbl;
  emxEnsureCapacity_real_T1(sp, A, i26, &o_emlrtRTEI);
  if (!(order + 1.0 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(order + 1.0, &c_emlrtDCI, sp);
  }

  ndbl = order + 1.0;
  if (ndbl != (int32_T)muDoubleScalarFloor(ndbl)) {
    emlrtIntegerCheckR2012b(ndbl, &b_emlrtDCI, sp);
  }

  nm1d2 = 100 * (int32_T)ndbl;
  for (i26 = 0; i26 < nm1d2; i26++) {
    A->data[i26] = 0.0;
  }

  i26 = (int32_T)((-1.0 - order) / -1.0);
  emlrtForLoopVectorCheckR2012b(order, -1.0, 0.0, mxDOUBLE_CLASS, i26,
    &sb_emlrtRTEI, sp);
  nm1d2 = 0;
  while (nm1d2 <= i26 - 1) {
    apnd = order + -(real_T)nm1d2;
    k = A->size[1];
    ndbl = (order - apnd) + 1.0;
    if (ndbl != (int32_T)muDoubleScalarFloor(ndbl)) {
      emlrtIntegerCheckR2012b(ndbl, &emlrtDCI, sp);
    }

    n = (int32_T)ndbl;
    if (!((n >= 1) && (n <= k))) {
      emlrtDynamicBoundsCheckR2012b(n, 1, k, &l_emlrtBCI, sp);
    }

    st.site = &pb_emlrtRSI;
    d_power(&st, t, apnd, *(real_T (*)[100])&A->data[A->size[0] * (n - 1)]);
    nm1d2++;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  emxInit_real_T(sp, &Aeq, 2, &r_emlrtRTEI, true);
  emxInit_real_T(sp, &varargin_1, 2, &o_emlrtRTEI, true);
  emxInit_real_T(sp, &varargin_2, 2, &o_emlrtRTEI, true);
  if (order >= 3.0) {
    /*  enforce all constraints */
    /*  polynomial evaluated in t(1) */
    /*  polynomial evaluated in t(end) */
    /*  derivative polynomial evaluated in t(1) */
    st.site = &sb_emlrtRSI;
    emxInit_real_T(&st, &r15, 2, &o_emlrtRTEI, true);
    if (muDoubleScalarFloor(order) == order) {
      i26 = r15->size[0] * r15->size[1];
      r15->size[0] = 1;
      r15->size[1] = (int32_T)muDoubleScalarFloor(-(1.0 - order)) + 1;
      emxEnsureCapacity_real_T1(&st, r15, i26, &o_emlrtRTEI);
      nm1d2 = (int32_T)muDoubleScalarFloor(-(1.0 - order));
      for (i26 = 0; i26 <= nm1d2; i26++) {
        r15->data[r15->size[0] * i26] = order - (real_T)i26;
      }
    } else {
      b_st.site = &cc_emlrtRSI;
      ndbl = muDoubleScalarFloor((1.0 - order) / -1.0 + 0.5);
      apnd = order + -ndbl;
      if (muDoubleScalarAbs(1.0 - apnd) < 4.4408920985006262E-16 * order) {
        ndbl++;
        apnd = 1.0;
      } else if (1.0 - apnd > 0.0) {
        apnd = order + -(ndbl - 1.0);
      } else {
        ndbl++;
      }

      n = (int32_T)ndbl;
      c_st.site = &dc_emlrtRSI;
      if (ndbl > 2.147483647E+9) {
        emlrtErrorWithMessageIdR2018a(&c_st, &rb_emlrtRTEI,
          "Coder:MATLAB:pmaxsize", "Coder:MATLAB:pmaxsize", 0);
      }

      i26 = r15->size[0] * r15->size[1];
      r15->size[0] = 1;
      r15->size[1] = n;
      emxEnsureCapacity_real_T1(&b_st, r15, i26, &p_emlrtRTEI);
      if (n > 0) {
        r15->data[0] = order;
        if (n > 1) {
          r15->data[n - 1] = apnd;
          nm1d2 = (n - 1) / 2;
          for (k = 1; k < nm1d2; k++) {
            r15->data[k] = order + -(real_T)k;
            r15->data[(n - k) - 1] = apnd - (-(real_T)k);
          }

          if (nm1d2 << 1 == n - 1) {
            r15->data[nm1d2] = (order + apnd) / 2.0;
          } else {
            r15->data[nm1d2] = order + -(real_T)nm1d2;
            r15->data[nm1d2 + 1] = apnd - (-(real_T)nm1d2);
          }
        }
      }
    }

    st.site = &sb_emlrtRSI;
    emxInit_real_T(&st, &r16, 2, &o_emlrtRTEI, true);
    if (muDoubleScalarFloor(order - 1.0) == order - 1.0) {
      i26 = r16->size[0] * r16->size[1];
      r16->size[0] = 1;
      r16->size[1] = (int32_T)muDoubleScalarFloor(-(0.0 - (order - 1.0))) + 1;
      emxEnsureCapacity_real_T1(&st, r16, i26, &o_emlrtRTEI);
      nm1d2 = (int32_T)muDoubleScalarFloor(-(0.0 - (order - 1.0)));
      for (i26 = 0; i26 <= nm1d2; i26++) {
        r16->data[r16->size[0] * i26] = (order - 1.0) - (real_T)i26;
      }
    } else {
      b_st.site = &cc_emlrtRSI;
      ndbl = muDoubleScalarFloor((0.0 - (order - 1.0)) / -1.0 + 0.5);
      apnd = (order - 1.0) + -ndbl;
      if (muDoubleScalarAbs(0.0 - apnd) < 4.4408920985006262E-16 * (order - 1.0))
      {
        ndbl++;
        apnd = 0.0;
      } else if (0.0 - apnd > 0.0) {
        apnd = (order - 1.0) + -(ndbl - 1.0);
      } else {
        ndbl++;
      }

      n = (int32_T)ndbl;
      c_st.site = &dc_emlrtRSI;
      if (ndbl > 2.147483647E+9) {
        emlrtErrorWithMessageIdR2018a(&c_st, &rb_emlrtRTEI,
          "Coder:MATLAB:pmaxsize", "Coder:MATLAB:pmaxsize", 0);
      }

      i26 = r16->size[0] * r16->size[1];
      r16->size[0] = 1;
      r16->size[1] = n;
      emxEnsureCapacity_real_T1(&b_st, r16, i26, &p_emlrtRTEI);
      if (n > 0) {
        r16->data[0] = order - 1.0;
        if (n > 1) {
          r16->data[n - 1] = apnd;
          nm1d2 = (n - 1) / 2;
          for (k = 1; k < nm1d2; k++) {
            r16->data[k] = (order - 1.0) + -(real_T)k;
            r16->data[(n - k) - 1] = apnd - (-(real_T)k);
          }

          if (nm1d2 << 1 == n - 1) {
            r16->data[nm1d2] = ((order - 1.0) + apnd) / 2.0;
          } else {
            r16->data[nm1d2] = (order - 1.0) + -(real_T)nm1d2;
            r16->data[nm1d2 + 1] = apnd - (-(real_T)nm1d2);
          }
        }
      }
    }

    emxInit_real_T(&st, &r17, 2, &o_emlrtRTEI, true);
    st.site = &sb_emlrtRSI;
    c_power(&st, t[0], r16, r17);
    for (i26 = 0; i26 < 2; i26++) {
      iv13[i26] = r15->size[i26];
    }

    for (i26 = 0; i26 < 2; i26++) {
      iv14[i26] = r17->size[i26];
    }

    if ((iv13[0] != iv14[0]) || (iv13[1] != iv14[1])) {
      emlrtSizeEqCheckNDR2012b(&iv13[0], &iv14[0], &f_emlrtECI, sp);
    }

    st.site = &tb_emlrtRSI;
    if (muDoubleScalarFloor(order) == order) {
      i26 = r16->size[0] * r16->size[1];
      r16->size[0] = 1;
      r16->size[1] = (int32_T)muDoubleScalarFloor(-(1.0 - order)) + 1;
      emxEnsureCapacity_real_T1(&st, r16, i26, &o_emlrtRTEI);
      nm1d2 = (int32_T)muDoubleScalarFloor(-(1.0 - order));
      for (i26 = 0; i26 <= nm1d2; i26++) {
        r16->data[r16->size[0] * i26] = order - (real_T)i26;
      }
    } else {
      b_st.site = &cc_emlrtRSI;
      ndbl = muDoubleScalarFloor((1.0 - order) / -1.0 + 0.5);
      apnd = order + -ndbl;
      if (muDoubleScalarAbs(1.0 - apnd) < 4.4408920985006262E-16 * order) {
        ndbl++;
        apnd = 1.0;
      } else if (1.0 - apnd > 0.0) {
        apnd = order + -(ndbl - 1.0);
      } else {
        ndbl++;
      }

      n = (int32_T)ndbl;
      c_st.site = &dc_emlrtRSI;
      if (ndbl > 2.147483647E+9) {
        emlrtErrorWithMessageIdR2018a(&c_st, &rb_emlrtRTEI,
          "Coder:MATLAB:pmaxsize", "Coder:MATLAB:pmaxsize", 0);
      }

      i26 = r16->size[0] * r16->size[1];
      r16->size[0] = 1;
      r16->size[1] = n;
      emxEnsureCapacity_real_T1(&b_st, r16, i26, &p_emlrtRTEI);
      if (n > 0) {
        r16->data[0] = order;
        if (n > 1) {
          r16->data[n - 1] = apnd;
          nm1d2 = (n - 1) / 2;
          for (k = 1; k < nm1d2; k++) {
            r16->data[k] = order + -(real_T)k;
            r16->data[(n - k) - 1] = apnd - (-(real_T)k);
          }

          if (nm1d2 << 1 == n - 1) {
            r16->data[nm1d2] = (order + apnd) / 2.0;
          } else {
            r16->data[nm1d2] = order + -(real_T)nm1d2;
            r16->data[nm1d2 + 1] = apnd - (-(real_T)nm1d2);
          }
        }
      }
    }

    st.site = &tb_emlrtRSI;
    emxInit_real_T(&st, &varargin_4, 2, &o_emlrtRTEI, true);
    if (muDoubleScalarFloor(order - 1.0) == order - 1.0) {
      i26 = varargin_4->size[0] * varargin_4->size[1];
      varargin_4->size[0] = 1;
      varargin_4->size[1] = (int32_T)muDoubleScalarFloor(-(0.0 - (order - 1.0)))
        + 1;
      emxEnsureCapacity_real_T1(&st, varargin_4, i26, &o_emlrtRTEI);
      nm1d2 = (int32_T)muDoubleScalarFloor(-(0.0 - (order - 1.0)));
      for (i26 = 0; i26 <= nm1d2; i26++) {
        varargin_4->data[varargin_4->size[0] * i26] = (order - 1.0) - (real_T)
          i26;
      }
    } else {
      b_st.site = &cc_emlrtRSI;
      ndbl = muDoubleScalarFloor((0.0 - (order - 1.0)) / -1.0 + 0.5);
      apnd = (order - 1.0) + -ndbl;
      if (muDoubleScalarAbs(0.0 - apnd) < 4.4408920985006262E-16 * (order - 1.0))
      {
        ndbl++;
        apnd = 0.0;
      } else if (0.0 - apnd > 0.0) {
        apnd = (order - 1.0) + -(ndbl - 1.0);
      } else {
        ndbl++;
      }

      n = (int32_T)ndbl;
      c_st.site = &dc_emlrtRSI;
      if (ndbl > 2.147483647E+9) {
        emlrtErrorWithMessageIdR2018a(&c_st, &rb_emlrtRTEI,
          "Coder:MATLAB:pmaxsize", "Coder:MATLAB:pmaxsize", 0);
      }

      i26 = varargin_4->size[0] * varargin_4->size[1];
      varargin_4->size[0] = 1;
      varargin_4->size[1] = n;
      emxEnsureCapacity_real_T1(&b_st, varargin_4, i26, &p_emlrtRTEI);
      if (n > 0) {
        varargin_4->data[0] = order - 1.0;
        if (n > 1) {
          varargin_4->data[n - 1] = apnd;
          nm1d2 = (n - 1) / 2;
          for (k = 1; k < nm1d2; k++) {
            varargin_4->data[k] = (order - 1.0) + -(real_T)k;
            varargin_4->data[(n - k) - 1] = apnd - (-(real_T)k);
          }

          if (nm1d2 << 1 == n - 1) {
            varargin_4->data[nm1d2] = ((order - 1.0) + apnd) / 2.0;
          } else {
            varargin_4->data[nm1d2] = (order - 1.0) + -(real_T)nm1d2;
            varargin_4->data[nm1d2 + 1] = apnd - (-(real_T)nm1d2);
          }
        }
      }
    }

    emxInit_real_T(&st, &r18, 2, &o_emlrtRTEI, true);
    st.site = &tb_emlrtRSI;
    c_power(&st, t[99], varargin_4, r18);
    for (i26 = 0; i26 < 2; i26++) {
      iv13[i26] = r16->size[i26];
    }

    for (i26 = 0; i26 < 2; i26++) {
      iv14[i26] = r18->size[i26];
    }

    if ((iv13[0] != iv14[0]) || (iv13[1] != iv14[1])) {
      emlrtSizeEqCheckNDR2012b(&iv13[0], &iv14[0], &e_emlrtECI, sp);
    }

    st.site = &qb_emlrtRSI;
    if (muDoubleScalarFloor(order) == order) {
      i26 = varargin_4->size[0] * varargin_4->size[1];
      varargin_4->size[0] = 1;
      varargin_4->size[1] = (int32_T)muDoubleScalarFloor(-(0.0 - order)) + 1;
      emxEnsureCapacity_real_T1(&st, varargin_4, i26, &o_emlrtRTEI);
      nm1d2 = (int32_T)muDoubleScalarFloor(-(0.0 - order));
      for (i26 = 0; i26 <= nm1d2; i26++) {
        varargin_4->data[varargin_4->size[0] * i26] = order - (real_T)i26;
      }
    } else {
      b_st.site = &cc_emlrtRSI;
      ndbl = muDoubleScalarFloor((0.0 - order) / -1.0 + 0.5);
      apnd = order + -ndbl;
      if (muDoubleScalarAbs(0.0 - apnd) < 4.4408920985006262E-16 * order) {
        ndbl++;
        apnd = 0.0;
      } else if (0.0 - apnd > 0.0) {
        apnd = order + -(ndbl - 1.0);
      } else {
        ndbl++;
      }

      n = (int32_T)ndbl;
      c_st.site = &dc_emlrtRSI;
      if (ndbl > 2.147483647E+9) {
        emlrtErrorWithMessageIdR2018a(&c_st, &rb_emlrtRTEI,
          "Coder:MATLAB:pmaxsize", "Coder:MATLAB:pmaxsize", 0);
      }

      i26 = varargin_4->size[0] * varargin_4->size[1];
      varargin_4->size[0] = 1;
      varargin_4->size[1] = n;
      emxEnsureCapacity_real_T1(&b_st, varargin_4, i26, &p_emlrtRTEI);
      if (n > 0) {
        varargin_4->data[0] = order;
        if (n > 1) {
          varargin_4->data[n - 1] = apnd;
          nm1d2 = (n - 1) / 2;
          for (k = 1; k < nm1d2; k++) {
            varargin_4->data[k] = order + -(real_T)k;
            varargin_4->data[(n - k) - 1] = apnd - (-(real_T)k);
          }

          if (nm1d2 << 1 == n - 1) {
            varargin_4->data[nm1d2] = (order + apnd) / 2.0;
          } else {
            varargin_4->data[nm1d2] = order + -(real_T)nm1d2;
            varargin_4->data[nm1d2 + 1] = apnd - (-(real_T)nm1d2);
          }
        }
      }
    }

    st.site = &rb_emlrtRSI;
    emxInit_real_T(&st, &varargin_3, 2, &o_emlrtRTEI, true);
    if (muDoubleScalarFloor(order) == order) {
      i26 = varargin_3->size[0] * varargin_3->size[1];
      varargin_3->size[0] = 1;
      varargin_3->size[1] = (int32_T)muDoubleScalarFloor(-(0.0 - order)) + 1;
      emxEnsureCapacity_real_T1(&st, varargin_3, i26, &o_emlrtRTEI);
      nm1d2 = (int32_T)muDoubleScalarFloor(-(0.0 - order));
      for (i26 = 0; i26 <= nm1d2; i26++) {
        varargin_3->data[varargin_3->size[0] * i26] = order - (real_T)i26;
      }
    } else {
      b_st.site = &cc_emlrtRSI;
      ndbl = muDoubleScalarFloor((0.0 - order) / -1.0 + 0.5);
      apnd = order + -ndbl;
      if (muDoubleScalarAbs(0.0 - apnd) < 4.4408920985006262E-16 * order) {
        ndbl++;
        apnd = 0.0;
      } else if (0.0 - apnd > 0.0) {
        apnd = order + -(ndbl - 1.0);
      } else {
        ndbl++;
      }

      n = (int32_T)ndbl;
      c_st.site = &dc_emlrtRSI;
      if (ndbl > 2.147483647E+9) {
        emlrtErrorWithMessageIdR2018a(&c_st, &rb_emlrtRTEI,
          "Coder:MATLAB:pmaxsize", "Coder:MATLAB:pmaxsize", 0);
      }

      i26 = varargin_3->size[0] * varargin_3->size[1];
      varargin_3->size[0] = 1;
      varargin_3->size[1] = n;
      emxEnsureCapacity_real_T1(&b_st, varargin_3, i26, &p_emlrtRTEI);
      if (n > 0) {
        varargin_3->data[0] = order;
        if (n > 1) {
          varargin_3->data[n - 1] = apnd;
          nm1d2 = (n - 1) / 2;
          for (k = 1; k < nm1d2; k++) {
            varargin_3->data[k] = order + -(real_T)k;
            varargin_3->data[(n - k) - 1] = apnd - (-(real_T)k);
          }

          if (nm1d2 << 1 == n - 1) {
            varargin_3->data[nm1d2] = (order + apnd) / 2.0;
          } else {
            varargin_3->data[nm1d2] = order + -(real_T)nm1d2;
            varargin_3->data[nm1d2 + 1] = apnd - (-(real_T)nm1d2);
          }
        }
      }
    }

    st.site = &qb_emlrtRSI;
    c_power(&st, t[0], varargin_4, varargin_1);
    st.site = &rb_emlrtRSI;
    c_power(&st, t[99], varargin_3, varargin_2);
    st.site = &qb_emlrtRSI;
    i26 = varargin_3->size[0] * varargin_3->size[1];
    varargin_3->size[0] = 1;
    varargin_3->size[1] = r15->size[1] + 1;
    emxEnsureCapacity_real_T1(&st, varargin_3, i26, &o_emlrtRTEI);
    nm1d2 = r15->size[1];
    for (i26 = 0; i26 < nm1d2; i26++) {
      varargin_3->data[varargin_3->size[0] * i26] = r15->data[r15->size[0] * i26]
        * r17->data[r17->size[0] * i26];
    }

    emxFree_real_T(&st, &r17);
    varargin_3->data[varargin_3->size[0] * r15->size[1]] = 0.0;
    i26 = varargin_4->size[0] * varargin_4->size[1];
    varargin_4->size[0] = 1;
    varargin_4->size[1] = r16->size[1] + 1;
    emxEnsureCapacity_real_T1(&st, varargin_4, i26, &o_emlrtRTEI);
    nm1d2 = r16->size[1];
    emxFree_real_T(&st, &r15);
    for (i26 = 0; i26 < nm1d2; i26++) {
      varargin_4->data[varargin_4->size[0] * i26] = r16->data[r16->size[0] * i26]
        * r18->data[r18->size[0] * i26];
    }

    emxFree_real_T(&st, &r18);
    varargin_4->data[varargin_4->size[0] * r16->size[1]] = 0.0;
    b_st.site = &hc_emlrtRSI;
    c_st.site = &ic_emlrtRSI;
    b2 = true;
    emxFree_real_T(&c_st, &r16);
    if (varargin_2->size[1] != varargin_1->size[1]) {
      b2 = false;
    }

    if (!b2) {
      emlrtErrorWithMessageIdR2018a(&c_st, &mb_emlrtRTEI,
        "MATLAB:catenate:matrixDimensionMismatch",
        "MATLAB:catenate:matrixDimensionMismatch", 0);
    }

    if (varargin_3->size[1] != varargin_1->size[1]) {
      b2 = false;
    }

    if (!b2) {
      emlrtErrorWithMessageIdR2018a(&c_st, &mb_emlrtRTEI,
        "MATLAB:catenate:matrixDimensionMismatch",
        "MATLAB:catenate:matrixDimensionMismatch", 0);
    }

    if (varargin_4->size[1] != varargin_1->size[1]) {
      b2 = false;
    }

    if (!b2) {
      emlrtErrorWithMessageIdR2018a(&c_st, &mb_emlrtRTEI,
        "MATLAB:catenate:matrixDimensionMismatch",
        "MATLAB:catenate:matrixDimensionMismatch", 0);
    }

    i26 = Aeq->size[0] * Aeq->size[1];
    Aeq->size[0] = 4;
    Aeq->size[1] = varargin_1->size[1];
    emxEnsureCapacity_real_T1(sp, Aeq, i26, &o_emlrtRTEI);
    nm1d2 = varargin_1->size[1];
    for (i26 = 0; i26 < nm1d2; i26++) {
      Aeq->data[Aeq->size[0] * i26] = varargin_1->data[varargin_1->size[0] * i26];
    }

    nm1d2 = varargin_2->size[1];
    for (i26 = 0; i26 < nm1d2; i26++) {
      Aeq->data[1 + Aeq->size[0] * i26] = varargin_2->data[varargin_2->size[0] *
        i26];
    }

    nm1d2 = varargin_3->size[1];
    for (i26 = 0; i26 < nm1d2; i26++) {
      Aeq->data[2 + Aeq->size[0] * i26] = varargin_3->data[varargin_3->size[0] *
        i26];
    }

    emxFree_real_T(sp, &varargin_3);
    nm1d2 = varargin_4->size[1];
    for (i26 = 0; i26 < nm1d2; i26++) {
      Aeq->data[3 + Aeq->size[0] * i26] = varargin_4->data[varargin_4->size[0] *
        i26];
    }

    emxFree_real_T(sp, &varargin_4);

    /*  derivative polynomial evaluated in t(end) */
    c_data[0] = data[0];
    c_data[1] = data[99];
    c_data[2] = (data[1] - data[0]) / (t[1] - t[0]);
    c_data[3] = (data[99] - data[98]) / (t[99] - t[98]);
    beq_size[0] = 4;
    for (i26 = 0; i26 < 4; i26++) {
      beq_data[i26] = c_data[i26];
    }
  } else {
    /*  limit constraints due to reduced order such that we only require start and end point to be fulfilled */
    /*  polynomial evaluated in t(1) */
    st.site = &ub_emlrtRSI;
    st.site = &vb_emlrtRSI;
    tmp_size[0] = 1;
    tmp_size[1] = 1;
    tmp_data[0] = rtNaN;
    b_tmp_data.data = (real_T *)&tmp_data;
    b_tmp_data.size = (int32_T *)&tmp_size;
    b_tmp_data.allocatedSize = 1;
    b_tmp_data.numDimensions = 2;
    b_tmp_data.canFreeData = false;
    st.site = &ub_emlrtRSI;
    c_power(&st, t[0], &b_tmp_data, varargin_1);
    b_tmp_size[0] = 1;
    b_tmp_size[1] = 1;
    c_tmp_data[0] = rtNaN;
    d_tmp_data.data = (real_T *)&c_tmp_data;
    d_tmp_data.size = (int32_T *)&b_tmp_size;
    d_tmp_data.allocatedSize = 1;
    d_tmp_data.numDimensions = 2;
    d_tmp_data.canFreeData = false;
    st.site = &vb_emlrtRSI;
    c_power(&st, t[99], &d_tmp_data, varargin_2);
    st.site = &ub_emlrtRSI;
    b_st.site = &hc_emlrtRSI;
    c_st.site = &ic_emlrtRSI;
    b2 = true;
    if (varargin_2->size[1] != varargin_1->size[1]) {
      b2 = false;
    }

    if (!b2) {
      emlrtErrorWithMessageIdR2018a(&c_st, &mb_emlrtRTEI,
        "MATLAB:catenate:matrixDimensionMismatch",
        "MATLAB:catenate:matrixDimensionMismatch", 0);
    }

    i26 = Aeq->size[0] * Aeq->size[1];
    Aeq->size[0] = 2;
    Aeq->size[1] = varargin_1->size[1];
    emxEnsureCapacity_real_T1(sp, Aeq, i26, &o_emlrtRTEI);
    nm1d2 = varargin_1->size[1];
    for (i26 = 0; i26 < nm1d2; i26++) {
      Aeq->data[Aeq->size[0] * i26] = varargin_1->data[varargin_1->size[0] * i26];
    }

    nm1d2 = varargin_2->size[1];
    for (i26 = 0; i26 < nm1d2; i26++) {
      Aeq->data[1 + Aeq->size[0] * i26] = varargin_2->data[varargin_2->size[0] *
        i26];
    }

    /*  polynomial evaluated in t(end)  */
    b_data[0] = data[0];
    b_data[1] = data[99];
    beq_size[0] = 2;
    for (i26 = 0; i26 < 2; i26++) {
      beq_data[i26] = b_data[i26];
    }
  }

  emxFree_real_T(sp, &varargin_2);
  emxFree_real_T(sp, &varargin_1);

  /* opts = optimset('display','off'); */
  /* polyCoeffs = lsqlin(A,b, [], [], Aeq, beq, [], [], [], opts); */
  st.site = &wb_emlrtRSI;
  c_ConstrainedLeastSquares(&st, A, data, Aeq, beq_data, beq_size, polyCoeffs);

  /*      t_visualize_fit = linspace(0,1,100); */
  /*      data_fit = polyval(polyCoeffs, t_visualize_fit); */
  /*      figure(10); */
  /*      plot(t,data,'r--'); */
  /*      hold on; */
  /*      plot(t_visualize_fit,data_fit,'b--'); */
  /*      hold off; */
  emxFree_real_T(sp, &Aeq);
  emxFree_real_T(sp, &A);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (ConstrainedPolyFit.c) */
