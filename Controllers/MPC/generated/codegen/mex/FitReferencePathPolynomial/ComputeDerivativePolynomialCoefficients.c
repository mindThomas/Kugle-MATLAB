/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ComputeDerivativePolynomialCoefficients.c
 *
 * Code generation for function 'ComputeDerivativePolynomialCoefficients'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "FitReferencePathPolynomial.h"
#include "ComputeDerivativePolynomialCoefficients.h"
#include "FitReferencePathPolynomial_emxutil.h"

/* Variable Definitions */
static emlrtRSInfo ae_emlrtRSI = { 8,  /* lineNo */
  "ComputeDerivativePolynomialCoefficients",/* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ComputeDerivativePolynomialCoefficients.m"/* pathName */
};

static emlrtRSInfo be_emlrtRSI = { 18, /* lineNo */
  "indexShapeCheck",                   /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\indexShapeCheck.m"/* pathName */
};

static emlrtRTEInfo hb_emlrtRTEI = { 6,/* lineNo */
  19,                                  /* colNo */
  "ComputeDerivativePolynomialCoefficients",/* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ComputeDerivativePolynomialCoefficients.m"/* pName */
};

static emlrtBCInfo eb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  8,                                   /* lineNo */
  31,                                  /* colNo */
  "coeff",                             /* aName */
  "ComputeDerivativePolynomialCoefficients",/* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ComputeDerivativePolynomialCoefficients.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo fb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  8,                                   /* lineNo */
  33,                                  /* colNo */
  "coeff",                             /* aName */
  "ComputeDerivativePolynomialCoefficients",/* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ComputeDerivativePolynomialCoefficients.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo g_emlrtECI = { -1,  /* nDims */
  8,                                   /* lineNo */
  14,                                  /* colNo */
  "ComputeDerivativePolynomialCoefficients",/* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ComputeDerivativePolynomialCoefficients.m"/* pName */
};

static emlrtRTEInfo wb_emlrtRTEI = { 88,/* lineNo */
  9,                                   /* colNo */
  "indexShapeCheck",                   /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\indexShapeCheck.m"/* pName */
};

/* Function Definitions */
void c_ComputeDerivativePolynomialCo(const emlrtStack *sp, const emxArray_real_T
  *coeff, emxArray_real_T *dcoeff)
{
  int32_T i11;
  int32_T i12;
  boolean_T nonSingletonDimFound;
  emxArray_real_T *y;
  real_T d0;
  int32_T loop_ub;
  emxArray_real_T *r9;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);

  /*  Given polynomial coefficients for: */
  /*  f(x) = c_n*x^n + c_n-1*x^(n-1) + ... c_1*x + c_0 */
  /*  This function computes the coefficients of the polynomial f'(x) */
  /*  f'(x) = df/dx = n*c_n*x^(n-1) + (n-1)*c_n-1*x^(n-2) + ... +  */
  /*  Coefficients are ordered such that coeff(1) = c_n  (highest order) */
  if (1 > coeff->size[0] - 1) {
    i12 = 0;
  } else {
    i11 = coeff->size[0];
    if (!(1 <= i11)) {
      emlrtDynamicBoundsCheckR2012b(1, 1, i11, &eb_emlrtBCI, sp);
    }

    i11 = coeff->size[0];
    i12 = coeff->size[0] - 1;
    if (!((i12 >= 1) && (i12 <= i11))) {
      emlrtDynamicBoundsCheckR2012b(i12, 1, i11, &fb_emlrtBCI, sp);
    }
  }

  st.site = &ae_emlrtRSI;
  nonSingletonDimFound = !(coeff->size[0] != 1);
  if (nonSingletonDimFound) {
    nonSingletonDimFound = false;
    if (i12 != 1) {
      nonSingletonDimFound = true;
    }

    if (nonSingletonDimFound) {
      nonSingletonDimFound = true;
    } else {
      nonSingletonDimFound = false;
    }
  } else {
    nonSingletonDimFound = false;
  }

  b_st.site = &be_emlrtRSI;
  if (nonSingletonDimFound) {
    emlrtErrorWithMessageIdR2018a(&b_st, &wb_emlrtRTEI,
      "Coder:FE:PotentialVectorVector", "Coder:FE:PotentialVectorVector", 0);
  }

  emxInit_real_T(&b_st, &y, 2, &hb_emlrtRTEI, true);
  st.site = &ae_emlrtRSI;
  if (coeff->size[0] - 1 < 1) {
    i11 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = 0;
    emxEnsureCapacity_real_T1(&st, y, i11, &hb_emlrtRTEI);
  } else {
    d0 = (real_T)coeff->size[0] - 1.0;
    i11 = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = (int32_T)-(1.0 - d0) + 1;
    emxEnsureCapacity_real_T1(&st, y, i11, &hb_emlrtRTEI);
    loop_ub = (int32_T)-(1.0 - d0);
    for (i11 = 0; i11 <= loop_ub; i11++) {
      y->data[y->size[0] * i11] = d0 - (real_T)i11;
    }
  }

  emxInit_real_T1(&st, &r9, 1, &hb_emlrtRTEI, true);
  i11 = r9->size[0];
  r9->size[0] = y->size[1];
  emxEnsureCapacity_real_T(sp, r9, i11, &hb_emlrtRTEI);
  loop_ub = y->size[1];
  for (i11 = 0; i11 < loop_ub; i11++) {
    r9->data[i11] = y->data[y->size[0] * i11];
  }

  emxFree_real_T(sp, &y);
  i11 = r9->size[0];
  if (i11 != i12) {
    emlrtSizeEqCheck1DR2012b(i11, i12, &g_emlrtECI, sp);
  }

  i11 = dcoeff->size[0];
  dcoeff->size[0] = r9->size[0];
  emxEnsureCapacity_real_T(sp, dcoeff, i11, &hb_emlrtRTEI);
  loop_ub = r9->size[0];
  for (i11 = 0; i11 < loop_ub; i11++) {
    dcoeff->data[i11] = r9->data[i11] * coeff->data[i11];
  }

  emxFree_real_T(sp, &r9);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (ComputeDerivativePolynomialCoefficients.c) */
