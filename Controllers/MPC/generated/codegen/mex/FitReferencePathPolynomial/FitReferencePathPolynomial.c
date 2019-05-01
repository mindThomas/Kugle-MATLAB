/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * FitReferencePathPolynomial.c
 *
 * Code generation for function 'FitReferencePathPolynomial'
 *
 */

/* Include files */
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "FitReferencePathPolynomial.h"
#include "FitReferencePathPolynomial_emxutil.h"
#include "error.h"
#include "EvaluatePolynomial.h"
#include "ComputeSquaredPolynomialCoefficients.h"
#include "ComputeDerivativePolynomialCoefficients.h"
#include "ConstrainedPolyFit.h"
#include "linspace.h"
#include "eml_int_forloop_overflow_check.h"
#include "power.h"
#include "diff.h"
#include "FitReferencePathPolynomial_data.h"

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = { 13,    /* lineNo */
  "FitReferencePathPolynomial",        /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pathName */
};

static emlrtRSInfo b_emlrtRSI = { 14,  /* lineNo */
  "FitReferencePathPolynomial",        /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pathName */
};

static emlrtRSInfo c_emlrtRSI = { 16,  /* lineNo */
  "FitReferencePathPolynomial",        /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pathName */
};

static emlrtRSInfo d_emlrtRSI = { 17,  /* lineNo */
  "FitReferencePathPolynomial",        /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pathName */
};

static emlrtRSInfo e_emlrtRSI = { 22,  /* lineNo */
  "FitReferencePathPolynomial",        /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pathName */
};

static emlrtRSInfo f_emlrtRSI = { 29,  /* lineNo */
  "FitReferencePathPolynomial",        /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pathName */
};

static emlrtRSInfo g_emlrtRSI = { 32,  /* lineNo */
  "FitReferencePathPolynomial",        /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pathName */
};

static emlrtRSInfo h_emlrtRSI = { 42,  /* lineNo */
  "FitReferencePathPolynomial",        /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pathName */
};

static emlrtRSInfo i_emlrtRSI = { 43,  /* lineNo */
  "FitReferencePathPolynomial",        /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pathName */
};

static emlrtRSInfo j_emlrtRSI = { 46,  /* lineNo */
  "FitReferencePathPolynomial",        /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pathName */
};

static emlrtRSInfo k_emlrtRSI = { 47,  /* lineNo */
  "FitReferencePathPolynomial",        /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pathName */
};

static emlrtRSInfo l_emlrtRSI = { 54,  /* lineNo */
  "FitReferencePathPolynomial",        /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pathName */
};

static emlrtRSInfo m_emlrtRSI = { 55,  /* lineNo */
  "FitReferencePathPolynomial",        /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pathName */
};

static emlrtRSInfo n_emlrtRSI = { 61,  /* lineNo */
  "FitReferencePathPolynomial",        /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pathName */
};

static emlrtRSInfo o_emlrtRSI = { 101, /* lineNo */
  "FitReferencePathPolynomial",        /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pathName */
};

static emlrtRSInfo p_emlrtRSI = { 102, /* lineNo */
  "FitReferencePathPolynomial",        /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pathName */
};

static emlrtRSInfo q_emlrtRSI = { 104, /* lineNo */
  "FitReferencePathPolynomial",        /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pathName */
};

static emlrtRSInfo bb_emlrtRSI = { 12, /* lineNo */
  "sqrt",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elfun\\sqrt.m"/* pathName */
};

static emlrtRSInfo cb_emlrtRSI = { 15, /* lineNo */
  "sqrt",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elfun\\sqrt.m"/* pathName */
};

static emlrtRSInfo db_emlrtRSI = { 31, /* lineNo */
  "applyScalarFunctionInPlace",        /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\applyScalarFunctionInPlace.m"/* pathName */
};

static emlrtRSInfo eb_emlrtRSI = { 14, /* lineNo */
  "cumsum",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\cumsum.m"/* pathName */
};

static emlrtRSInfo fb_emlrtRSI = { 11, /* lineNo */
  "cumop",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\private\\cumop.m"/* pathName */
};

static emlrtRSInfo ud_emlrtRSI = { 9,  /* lineNo */
  "ArcLengthApproximation",            /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ArcLengthApproximation.m"/* pathName */
};

static emlrtRSInfo vd_emlrtRSI = { 10, /* lineNo */
  "ArcLengthApproximation",            /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ArcLengthApproximation.m"/* pathName */
};

static emlrtRSInfo wd_emlrtRSI = { 11, /* lineNo */
  "ArcLengthApproximation",            /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ArcLengthApproximation.m"/* pathName */
};

static emlrtRSInfo xd_emlrtRSI = { 12, /* lineNo */
  "ArcLengthApproximation",            /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ArcLengthApproximation.m"/* pathName */
};

static emlrtRSInfo yd_emlrtRSI = { 19, /* lineNo */
  "ArcLengthApproximation",            /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ArcLengthApproximation.m"/* pathName */
};

static emlrtRSInfo ge_emlrtRSI = { 3,  /* lineNo */
  "PolynomialMinimumFinderWithBounds", /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\PolynomialMinimumFinderWithBounds.m"/* pathName */
};

static emlrtRSInfo he_emlrtRSI = { 4,  /* lineNo */
  "PolynomialMinimumFinderWithBounds", /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\PolynomialMinimumFinderWithBounds.m"/* pathName */
};

static emlrtRSInfo ie_emlrtRSI = { 11, /* lineNo */
  "PolynomialMinimumFinderWithBounds", /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\PolynomialMinimumFinderWithBounds.m"/* pathName */
};

static emlrtRSInfo je_emlrtRSI = { 12, /* lineNo */
  "PolynomialMinimumFinderWithBounds", /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\PolynomialMinimumFinderWithBounds.m"/* pathName */
};

static emlrtRTEInfo emlrtRTEI = { 1,   /* lineNo */
  93,                                  /* colNo */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pName */
};

static emlrtRTEInfo b_emlrtRTEI = { 13,/* lineNo */
  5,                                   /* colNo */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pName */
};

static emlrtRTEInfo c_emlrtRTEI = { 14,/* lineNo */
  5,                                   /* colNo */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pName */
};

static emlrtRTEInfo d_emlrtRTEI = { 16,/* lineNo */
  5,                                   /* colNo */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pName */
};

static emlrtRTEInfo e_emlrtRTEI = { 17,/* lineNo */
  5,                                   /* colNo */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pName */
};

static emlrtRTEInfo f_emlrtRTEI = { 20,/* lineNo */
  5,                                   /* colNo */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pName */
};

static emlrtRTEInfo g_emlrtRTEI = { 29,/* lineNo */
  5,                                   /* colNo */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pName */
};

static emlrtRTEInfo h_emlrtRTEI = { 54,/* lineNo */
  5,                                   /* colNo */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pName */
};

static emlrtRTEInfo i_emlrtRTEI = { 102,/* lineNo */
  5,                                   /* colNo */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pName */
};

static emlrtBCInfo emlrtBCI = { -1,    /* iFirst */
  -1,                                  /* iLast */
  14,                                  /* lineNo */
  45,                                  /* colNo */
  "diffWindowTrajectory",              /* aName */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo b_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  14,                                  /* lineNo */
  47,                                  /* colNo */
  "diffWindowTrajectory",              /* aName */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  14,                                  /* lineNo */
  105,                                 /* colNo */
  "diffWindowTrajectory",              /* aName */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  14,                                  /* lineNo */
  107,                                 /* colNo */
  "diffWindowTrajectory",              /* aName */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo emlrtECI = { -1,    /* nDims */
  14,                                  /* lineNo */
  24,                                  /* colNo */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pName */
};

static emlrtBCInfo e_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  16,                                  /* lineNo */
  60,                                  /* colNo */
  "WindowTrajectoryPoints",            /* aName */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo f_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  16,                                  /* lineNo */
  62,                                  /* colNo */
  "WindowTrajectoryPoints",            /* aName */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo g_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  17,                                  /* lineNo */
  60,                                  /* colNo */
  "WindowTrajectoryPoints",            /* aName */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo h_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  17,                                  /* lineNo */
  62,                                  /* colNo */
  "WindowTrajectoryPoints",            /* aName */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo i_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  22,                                  /* lineNo */
  58,                                  /* colNo */
  "t",                                 /* aName */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo b_emlrtECI = { -1,  /* nDims */
  59,                                  /* lineNo */
  26,                                  /* colNo */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m"/* pName */
};

static emlrtECInfo c_emlrtECI = { -1,  /* nDims */
  13,                                  /* lineNo */
  15,                                  /* colNo */
  "ArcLengthApproximation",            /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ArcLengthApproximation.m"/* pName */
};

static emlrtBCInfo j_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  26,                                  /* lineNo */
  17,                                  /* colNo */
  "s",                                 /* aName */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo k_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  22,                                  /* lineNo */
  10,                                  /* colNo */
  "s",                                 /* aName */
  "FitReferencePathPolynomial",        /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\FitReferencePathPolynomial.m",/* pName */
  0                                    /* checkKind */
};

/* Function Definitions */
void FitReferencePathPolynomial(const emlrtStack *sp, const emxArray_real_T
  *WindowTrajectoryPoints, real_T approximation_order, real_T velocity, real_T
  ts, real_T N, emxArray_real_T *TrajectoryPoints, emxArray_real_T *coeff_xs,
  emxArray_real_T *coeff_ys, real_T *windowTrajectoryLength, real_T
  *minDistancePoint)
{
  emxArray_real_T *diffWindowTrajectory;
  int32_T nx;
  int32_T i0;
  int32_T n;
  emxArray_real_T *y_ref;
  emxArray_real_T *xs_squared_coeff;
  emxArray_real_T *coeff_ts;
  boolean_T overflow;
  emxArray_real_T *t;
  emxArray_real_T *coeff_x;
  emxArray_real_T *coeff_y;
  emxArray_real_T *s;
  real_T s_total;
  real_T s_eval[100];
  real_T t_eval[100];
  real_T x_eval[100];
  real_T y_eval[100];
  real_T FirstDerivative;
  real_T SecondDerivative;
  real_T b_s;
  emxArray_real_T *y;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  emxInit_real_T(sp, &diffWindowTrajectory, 2, &b_emlrtRTEI, true);

  /*  ts = sample time */
  /*  N = MPC horizon (samples) */
  /*  Parameters */
  /*  x(t), y(t) */
  /*  t(s) */
  /*  x(s), y(s) */
  /*     %% Fit window points to two polynomials, x(t) and y(t), using parameters t starting with t0=0 and spaced with the distance between each point (chordal parameterization) */
  st.site = &emlrtRSI;
  diff(&st, WindowTrajectoryPoints, diffWindowTrajectory);
  if (diffWindowTrajectory->size[0] == 0) {
    nx = 0;
  } else {
    nx = muIntScalarMax_sint32(diffWindowTrajectory->size[0], 2);
  }

  if (1 > nx) {
    nx = 0;
  } else {
    i0 = diffWindowTrajectory->size[0];
    if (!(1 <= i0)) {
      emlrtDynamicBoundsCheckR2012b(1, 1, i0, &emlrtBCI, sp);
    }

    i0 = diffWindowTrajectory->size[0];
    if (!(nx <= i0)) {
      emlrtDynamicBoundsCheckR2012b(nx, 1, i0, &b_emlrtBCI, sp);
    }
  }

  if (diffWindowTrajectory->size[0] == 0) {
    n = 0;
  } else {
    n = muIntScalarMax_sint32(diffWindowTrajectory->size[0], 2);
  }

  if (1 > n) {
    n = 0;
  } else {
    i0 = diffWindowTrajectory->size[0];
    if (!(1 <= i0)) {
      emlrtDynamicBoundsCheckR2012b(1, 1, i0, &c_emlrtBCI, sp);
    }

    i0 = diffWindowTrajectory->size[0];
    if (!(n <= i0)) {
      emlrtDynamicBoundsCheckR2012b(n, 1, i0, &d_emlrtBCI, sp);
    }
  }

  emxInit_real_T1(sp, &y_ref, 1, &i_emlrtRTEI, true);
  i0 = y_ref->size[0];
  y_ref->size[0] = nx;
  emxEnsureCapacity_real_T(sp, y_ref, i0, &emlrtRTEI);
  for (i0 = 0; i0 < nx; i0++) {
    y_ref->data[i0] = diffWindowTrajectory->data[i0];
  }

  emxInit_real_T1(sp, &xs_squared_coeff, 1, &h_emlrtRTEI, true);
  st.site = &b_emlrtRSI;
  power(&st, y_ref, xs_squared_coeff);
  i0 = y_ref->size[0];
  y_ref->size[0] = n;
  emxEnsureCapacity_real_T(sp, y_ref, i0, &emlrtRTEI);
  for (i0 = 0; i0 < n; i0++) {
    y_ref->data[i0] = diffWindowTrajectory->data[i0 + diffWindowTrajectory->
      size[0]];
  }

  emxFree_real_T(sp, &diffWindowTrajectory);
  emxInit_real_T1(sp, &coeff_ts, 1, &g_emlrtRTEI, true);
  st.site = &b_emlrtRSI;
  power(&st, y_ref, coeff_ts);
  i0 = xs_squared_coeff->size[0];
  nx = coeff_ts->size[0];
  if (i0 != nx) {
    emlrtSizeEqCheck1DR2012b(i0, nx, &emlrtECI, sp);
  }

  st.site = &b_emlrtRSI;
  i0 = xs_squared_coeff->size[0];
  emxEnsureCapacity_real_T(&st, xs_squared_coeff, i0, &emlrtRTEI);
  nx = xs_squared_coeff->size[0];
  for (i0 = 0; i0 < nx; i0++) {
    xs_squared_coeff->data[i0] += coeff_ts->data[i0];
  }

  overflow = false;
  for (n = 0; n < xs_squared_coeff->size[0]; n++) {
    if (overflow || (xs_squared_coeff->data[n] < 0.0)) {
      overflow = true;
    } else {
      overflow = false;
    }
  }

  if (overflow) {
    b_st.site = &bb_emlrtRSI;
    b_error(&b_st);
  }

  b_st.site = &cb_emlrtRSI;
  nx = xs_squared_coeff->size[0];
  c_st.site = &db_emlrtRSI;
  overflow = ((!(1 > xs_squared_coeff->size[0])) && (xs_squared_coeff->size[0] >
    2147483646));
  if (overflow) {
    d_st.site = &t_emlrtRSI;
    check_forloop_overflow_error(&d_st);
  }

  for (n = 0; n < nx; n++) {
    xs_squared_coeff->data[n] = muDoubleScalarSqrt(xs_squared_coeff->data[n]);
  }

  st.site = &b_emlrtRSI;
  b_st.site = &eb_emlrtRSI;
  nx = 2;
  if (xs_squared_coeff->size[0] != 1) {
    nx = 1;
  }

  c_st.site = &fb_emlrtRSI;
  if ((1 == nx) && (xs_squared_coeff->size[0] != 0)) {
    nx = xs_squared_coeff->size[0];
    if (xs_squared_coeff->size[0] != 1) {
      for (n = 1; n < nx; n++) {
        xs_squared_coeff->data[n] += xs_squared_coeff->data[n - 1];
      }
    }
  }

  emxInit_real_T1(&c_st, &t, 1, &c_emlrtRTEI, true);
  i0 = t->size[0];
  t->size[0] = 1 + xs_squared_coeff->size[0];
  emxEnsureCapacity_real_T(sp, t, i0, &emlrtRTEI);
  t->data[0] = 0.0;
  nx = xs_squared_coeff->size[0];
  for (i0 = 0; i0 < nx; i0++) {
    t->data[i0 + 1] = xs_squared_coeff->data[i0];
  }

  if (WindowTrajectoryPoints->size[0] == 0) {
    nx = 0;
  } else {
    nx = muIntScalarMax_sint32(WindowTrajectoryPoints->size[0], 2);
  }

  if (1 > nx) {
    nx = 0;
  } else {
    i0 = WindowTrajectoryPoints->size[0];
    if (!(1 <= i0)) {
      emlrtDynamicBoundsCheckR2012b(1, 1, i0, &e_emlrtBCI, sp);
    }

    i0 = WindowTrajectoryPoints->size[0];
    if (!(nx <= i0)) {
      emlrtDynamicBoundsCheckR2012b(nx, 1, i0, &f_emlrtBCI, sp);
    }
  }

  i0 = y_ref->size[0];
  y_ref->size[0] = nx;
  emxEnsureCapacity_real_T(sp, y_ref, i0, &emlrtRTEI);
  for (i0 = 0; i0 < nx; i0++) {
    y_ref->data[i0] = WindowTrajectoryPoints->data[i0];
  }

  emxInit_real_T1(sp, &coeff_x, 1, &d_emlrtRTEI, true);
  st.site = &c_emlrtRSI;
  ConstrainedPolyFit(&st, t, y_ref, approximation_order, coeff_x);
  if (WindowTrajectoryPoints->size[0] == 0) {
    nx = 0;
  } else {
    nx = muIntScalarMax_sint32(WindowTrajectoryPoints->size[0], 2);
  }

  if (1 > nx) {
    nx = 0;
  } else {
    i0 = WindowTrajectoryPoints->size[0];
    if (!(1 <= i0)) {
      emlrtDynamicBoundsCheckR2012b(1, 1, i0, &g_emlrtBCI, sp);
    }

    i0 = WindowTrajectoryPoints->size[0];
    if (!(nx <= i0)) {
      emlrtDynamicBoundsCheckR2012b(nx, 1, i0, &h_emlrtBCI, sp);
    }
  }

  i0 = y_ref->size[0];
  y_ref->size[0] = nx;
  emxEnsureCapacity_real_T(sp, y_ref, i0, &emlrtRTEI);
  for (i0 = 0; i0 < nx; i0++) {
    y_ref->data[i0] = WindowTrajectoryPoints->data[i0 +
      WindowTrajectoryPoints->size[0]];
  }

  emxInit_real_T1(sp, &coeff_y, 1, &e_emlrtRTEI, true);
  emxInit_real_T1(sp, &s, 1, &f_emlrtRTEI, true);
  st.site = &d_emlrtRSI;
  ConstrainedPolyFit(&st, t, y_ref, approximation_order, coeff_y);

  /*     %% Compute numerical approximation of arc length at the points, t, using the fitted polynomial */
  i0 = s->size[0];
  s->size[0] = t->size[0];
  emxEnsureCapacity_real_T(sp, s, i0, &emlrtRTEI);
  nx = t->size[0];
  for (i0 = 0; i0 < nx; i0++) {
    s->data[i0] = 0.0;
  }

  n = 0;
  while (n <= t->size[0] - 1) {
    st.site = &e_emlrtRSI;
    i0 = t->size[0];
    nx = n + 1;
    if (!((nx >= 1) && (nx <= i0))) {
      emlrtDynamicBoundsCheckR2012b(nx, 1, i0, &i_emlrtBCI, &st);
    }

    /*  Based on http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.5.2912&rep=rep1&type=pdf */
    /*  Arc length is given as: */
    /*  integral( sqrt( dxdt^2 + dydt^2 ) ) dt */
    /*  This can however be approximated by using the velocity polynomial */
    /*  Q = sqrt( (df_x/dx)^2 + (df_dy)^2 ); */
    /*  First we compute the coefficients of the inner polynomial, f */
    /*  f = (df_x/dx)^2 + (df_dy)^2    */
    b_st.site = &ud_emlrtRSI;
    c_ComputeDerivativePolynomialCo(&b_st, coeff_x, coeff_ts);

    /*  taking the difference of a polynomial, moves the coefficients */
    b_st.site = &vd_emlrtRSI;
    c_ComputeDerivativePolynomialCo(&b_st, coeff_y, xs_squared_coeff);
    b_st.site = &wd_emlrtRSI;
    c_ComputeSquaredPolynomialCoeff(&b_st, coeff_ts, y_ref);
    b_st.site = &xd_emlrtRSI;
    c_ComputeSquaredPolynomialCoeff(&b_st, xs_squared_coeff, coeff_ts);
    i0 = y_ref->size[0];
    nx = coeff_ts->size[0];
    if (i0 != nx) {
      emlrtSizeEqCheck1DR2012b(i0, nx, &c_emlrtECI, &st);
    }

    i0 = y_ref->size[0];
    emxEnsureCapacity_real_T(&st, y_ref, i0, &emlrtRTEI);
    nx = y_ref->size[0];
    for (i0 = 0; i0 < nx; i0++) {
      y_ref->data[i0] += coeff_ts->data[i0];
    }

    /*  Such that */
    /*  Q = sqrt(EvaluatePolynomial(f_coeff, t))    */
    /*  Now the approximation (see paper above) is given by */
    /*  s(t) = t/2 * (5/9 * Q(1.774597*t/2) + 8/9 * Q(t/2) + 5/9 * Q(0.225403*t/2)) */
    b_st.site = &yd_emlrtRSI;
    FirstDerivative = EvaluatePolynomial(&b_st, y_ref, 1.774597 * t->data[n] /
      2.0);
    b_st.site = &yd_emlrtRSI;
    SecondDerivative = EvaluatePolynomial(&b_st, y_ref, t->data[n] / 2.0);
    b_st.site = &yd_emlrtRSI;
    s_total = EvaluatePolynomial(&b_st, y_ref, 0.225403 * t->data[n] / 2.0);
    b_st.site = &yd_emlrtRSI;
    if (FirstDerivative < 0.0) {
      c_st.site = &bb_emlrtRSI;
      b_error(&c_st);
    }

    b_st.site = &yd_emlrtRSI;
    if (SecondDerivative < 0.0) {
      c_st.site = &bb_emlrtRSI;
      b_error(&c_st);
    }

    b_st.site = &yd_emlrtRSI;
    if (s_total < 0.0) {
      c_st.site = &bb_emlrtRSI;
      b_error(&c_st);
    }

    i0 = s->size[0];
    nx = 1 + n;
    if (!((nx >= 1) && (nx <= i0))) {
      emlrtDynamicBoundsCheckR2012b(nx, 1, i0, &k_emlrtBCI, sp);
    }

    s->data[nx - 1] = t->data[n] / 2.0 * ((0.55555555555555558 *
      muDoubleScalarSqrt(FirstDerivative) + 0.88888888888888884 *
      muDoubleScalarSqrt(SecondDerivative)) + 0.55555555555555558 *
      muDoubleScalarSqrt(s_total));
    n++;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  /* s = ArcLengthApproximation(coeff_x, coeff_y, t); */
  i0 = s->size[0];
  nx = s->size[0];
  if (!((nx >= 1) && (nx <= i0))) {
    emlrtDynamicBoundsCheckR2012b(nx, 1, i0, &j_emlrtBCI, sp);
  }

  s_total = s->data[nx - 1];

  /*  total length of approximated trajectory */
  /*     %% Fit t(s) polynomial */
  st.site = &f_emlrtRSI;
  b_ConstrainedPolyFit(&st, s, t, approximation_order + 1.0, coeff_ts);
  linspace(s->data[s->size[0] - 1], s_eval);
  emxFree_real_T(sp, &t);

  /*  make 100 linearly seperated distance points for plotting */
  st.site = &g_emlrtRSI;
  b_EvaluatePolynomial(&st, coeff_ts, s_eval, t_eval);

  /*      figure(100); */
  /*      plot(s, t, s_eval, t_eval); */
  /*      legend('Fitting points', 'Polynomial approximation'); */
  /*      xlabel('s'); */
  /*      ylabel('t'); */
  /*      title('t(s) relationship'); */
  /*     %% Use the evenly spaced parameter values to generate new points from the two fitted polynomials, x_0,...,x_n = x(t_0 ),...,x(t_n) and same for y(t) */
  st.site = &h_emlrtRSI;
  b_EvaluatePolynomial(&st, coeff_x, t_eval, x_eval);
  st.site = &i_emlrtRSI;
  b_EvaluatePolynomial(&st, coeff_y, t_eval, y_eval);

  /*     %% Fit x(s) and y(s) on the newly generated points, x_0,...,x_n and y_0,...,y_n using the evenly spaced distance parameters, s_0,...,s_n, as the parameter */
  st.site = &j_emlrtRSI;
  c_ConstrainedPolyFit(&st, s_eval, x_eval, approximation_order + 1.0, coeff_xs);
  st.site = &k_emlrtRSI;
  c_ConstrainedPolyFit(&st, s_eval, y_eval, approximation_order + 1.0, coeff_ys);

  /*     %% Use the newly fitted polynomial to find the distance, s, closest to the current robot position, which is (0,0) because we are using window trajectory points */
  /*  This only works if RobotInWindow = [0;0] since the distance between robot and trajectory polynomial the */
  /*  is then given by just the distance from origin to the polynomial */
  /*  dist(s) = sqrt( f_x(s)^2 + f_y(s)^2 ) */
  /*  Create a distance polynomial and differentiate it  */
  st.site = &l_emlrtRSI;
  c_ComputeSquaredPolynomialCoeff(&st, coeff_xs, xs_squared_coeff);
  st.site = &m_emlrtRSI;
  c_ComputeSquaredPolynomialCoeff(&st, coeff_ys, coeff_ts);

  /*  And since we want to find the closest distance, we can also just */
  /*  minimize the squared distance: dist(s)^2 */
  /*  dist(s)^2 = f_x(s)^2 + f_y(s)^2 */
  i0 = xs_squared_coeff->size[0];
  nx = coeff_ts->size[0];
  if (i0 != nx) {
    emlrtSizeEqCheck1DR2012b(i0, nx, &b_emlrtECI, sp);
  }

  /*  find the distance, s, corresponding to the smallest distance */
  st.site = &n_emlrtRSI;

  /*  Find the minimum by Newton minimization given upper and lower parameter bounds */
  i0 = y_ref->size[0];
  y_ref->size[0] = xs_squared_coeff->size[0];
  emxEnsureCapacity_real_T(&st, y_ref, i0, &emlrtRTEI);
  nx = xs_squared_coeff->size[0];
  emxFree_real_T(&st, &coeff_y);
  emxFree_real_T(&st, &coeff_x);
  for (i0 = 0; i0 < nx; i0++) {
    y_ref->data[i0] = xs_squared_coeff->data[i0] + coeff_ts->data[i0];
  }

  b_st.site = &ge_emlrtRSI;
  c_ComputeDerivativePolynomialCo(&b_st, y_ref, coeff_ts);
  b_st.site = &he_emlrtRSI;
  c_ComputeDerivativePolynomialCo(&b_st, coeff_ts, xs_squared_coeff);

  /*  Newton's method - https://en.wikipedia.org/wiki/Newton%27s_method_in_optimization */
  overflow = false;
  nx = 0;
  b_s = 0.0;
  while ((!overflow) && (nx < 100)) {
    b_st.site = &ie_emlrtRSI;
    FirstDerivative = EvaluatePolynomial(&b_st, coeff_ts, b_s);

    /*  first derivative, for higher dimensions it would be the Gradient */
    b_st.site = &je_emlrtRSI;
    SecondDerivative = EvaluatePolynomial(&b_st, xs_squared_coeff, b_s);

    /*  second derivative, for higher dimensions it would be the Hessian */
    FirstDerivative = -FirstDerivative / SecondDerivative;
    b_s += FirstDerivative;
    if (b_s < 0.0) {
      b_s = 0.0;
    }

    if (b_s > s_total) {
      b_s = s_total;
    }

    if (muDoubleScalarAbs(FirstDerivative) < 0.001) {
      overflow = true;
    }

    nx++;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(&st);
    }
  }

  emxInit_real_T(&st, &y, 2, &emlrtRTEI, true);

  /*     %% Visualization/debugging only */
  /*      xs_eval = polyval(coeff_xs, s_eval); */
  /*      ys_eval = polyval(coeff_ys, s_eval); */
  /*      */
  /*      figure(2); */
  /*      subplot(2,1,1); */
  /*      plot(s_eval, x_eval, s_eval, xs_eval); */
  /*      subplot(2,1,2); */
  /*      plot(s_eval, y_eval, s_eval, ys_eval); */
  /*      */
  /*      figure(3); */
  /*      plot(WindowTrajectory(:,1), WindowTrajectory(:,2)); */
  /*      hold on; */
  /*      plot(x_eval, y_eval); */
  /*      plot(xs_eval, ys_eval); */
  /*      hold off; */
  /*      legend('Window trajectory', 't-approximation', 's-approximation'); */
  /*          */
  /*      %% Compute numerical approximation of arc length at the points, s, using the final polynomial (as confirmation/test) */
  /*      % Arc length is given as: */
  /*      % integral( sqrt( dxdt^2 + dydt^2 ) ) dt */
  /*      dxs_coeff = coeff_xs(1:end-1) .* (order_fs:-1:1)'; % taking the difference of a polynomial, moves the coefficients */
  /*      dys_coeff = coeff_ys(1:end-1) .* (order_fs:-1:1)'; */
  /*       */
  /*      dxdt = @(t) t.^(order_fs-1:-1:0) * dxs_coeff; */
  /*      dydt = @(t) t.^(order_fs-1:-1:0) * dys_coeff; */
  /*      arclength = @(t) sqrt( dxdt(t)^2 + dydt(t)^2 ); */
  /*      s = zeros(length(s_eval), 1); */
  /*      for (i = 1:length(s_eval)) */
  /*          s(i) = integral(arclength, 0, s_eval(i), 'ArrayValued', true); */
  /*      end        */
  /*      s-s_eval' */
  /*      mean(s-s_eval') */
  /*      3*sqrt(var(s-s_eval')) */
  /*     %% Generate trajectory reference points based on velocity, sample rate and horizon length, starting in the location closest to the robot */
  /*  Using constant velocity model */
  if (muDoubleScalarIsNaN(N - 1.0)) {
    i0 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = 1;
    emxEnsureCapacity_real_T1(sp, y, i0, &emlrtRTEI);
    y->data[0] = rtNaN;
  } else if (N - 1.0 < 0.0) {
    i0 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = 0;
    emxEnsureCapacity_real_T1(sp, y, i0, &emlrtRTEI);
  } else if (muDoubleScalarIsInf(N - 1.0) && (0.0 == N - 1.0)) {
    i0 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = 1;
    emxEnsureCapacity_real_T1(sp, y, i0, &emlrtRTEI);
    y->data[0] = rtNaN;
  } else {
    i0 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = (int32_T)muDoubleScalarFloor(N - 1.0) + 1;
    emxEnsureCapacity_real_T1(sp, y, i0, &emlrtRTEI);
    nx = (int32_T)muDoubleScalarFloor(N - 1.0);
    for (i0 = 0; i0 <= nx; i0++) {
      y->data[y->size[0] * i0] = i0;
    }
  }

  i0 = coeff_ts->size[0];
  coeff_ts->size[0] = y->size[1];
  emxEnsureCapacity_real_T(sp, coeff_ts, i0, &emlrtRTEI);
  nx = y->size[1];
  for (i0 = 0; i0 < nx; i0++) {
    coeff_ts->data[i0] = y->data[y->size[0] * i0] * ts * velocity + b_s;
  }

  emxFree_real_T(sp, &y);
  st.site = &o_emlrtRSI;
  c_EvaluatePolynomial(&st, coeff_xs, coeff_ts, xs_squared_coeff);
  st.site = &p_emlrtRSI;
  c_EvaluatePolynomial(&st, coeff_ys, coeff_ts, y_ref);
  st.site = &q_emlrtRSI;
  b_st.site = &hc_emlrtRSI;
  c_st.site = &ic_emlrtRSI;
  overflow = true;
  emxFree_real_T(&c_st, &coeff_ts);
  if (y_ref->size[0] != xs_squared_coeff->size[0]) {
    overflow = false;
  }

  if (!overflow) {
    emlrtErrorWithMessageIdR2018a(&c_st, &mb_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  i0 = TrajectoryPoints->size[0] * TrajectoryPoints->size[1];
  TrajectoryPoints->size[0] = xs_squared_coeff->size[0];
  TrajectoryPoints->size[1] = 2;
  emxEnsureCapacity_real_T1(&b_st, TrajectoryPoints, i0, &emlrtRTEI);
  nx = xs_squared_coeff->size[0];
  for (i0 = 0; i0 < nx; i0++) {
    TrajectoryPoints->data[i0] = xs_squared_coeff->data[i0];
  }

  emxFree_real_T(&b_st, &xs_squared_coeff);
  nx = y_ref->size[0];
  for (i0 = 0; i0 < nx; i0++) {
    TrajectoryPoints->data[i0 + TrajectoryPoints->size[0]] = y_ref->data[i0];
  }

  emxFree_real_T(&b_st, &y_ref);
  *windowTrajectoryLength = s->data[s->size[0] - 1];
  *minDistancePoint = b_s;
  emxFree_real_T(sp, &s);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (FitReferencePathPolynomial.c) */
