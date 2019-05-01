/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ComputeSquaredPolynomialCoefficients.c
 *
 * Code generation for function 'ComputeSquaredPolynomialCoefficients'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "FitReferencePathPolynomial.h"
#include "ComputeSquaredPolynomialCoefficients.h"
#include "FitReferencePathPolynomial_emxutil.h"
#include "FitReferencePathPolynomial_data.h"

/* Variable Definitions */
static emlrtRTEInfo ib_emlrtRTEI = { 5,/* lineNo */
  26,                                  /* colNo */
  "ComputeSquaredPolynomialCoefficients",/* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ComputeSquaredPolynomialCoefficients.m"/* pName */
};

static emlrtRTEInfo xb_emlrtRTEI = { 12,/* lineNo */
  14,                                  /* colNo */
  "ComputeSquaredPolynomialCoefficients",/* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ComputeSquaredPolynomialCoefficients.m"/* pName */
};

static emlrtDCInfo e_emlrtDCI = { 11,  /* lineNo */
  27,                                  /* colNo */
  "ComputeSquaredPolynomialCoefficients",/* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ComputeSquaredPolynomialCoefficients.m",/* pName */
  1                                    /* checkKind */
};

static emlrtDCInfo f_emlrtDCI = { 11,  /* lineNo */
  27,                                  /* colNo */
  "ComputeSquaredPolynomialCoefficients",/* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ComputeSquaredPolynomialCoefficients.m",/* pName */
  4                                    /* checkKind */
};

static emlrtBCInfo gb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  16,                                  /* lineNo */
  58,                                  /* colNo */
  "coeff_squared",                     /* aName */
  "ComputeSquaredPolynomialCoefficients",/* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ComputeSquaredPolynomialCoefficients.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo hb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  16,                                  /* lineNo */
  73,                                  /* colNo */
  "coeff",                             /* aName */
  "ComputeSquaredPolynomialCoefficients",/* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ComputeSquaredPolynomialCoefficients.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo ib_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  16,                                  /* lineNo */
  86,                                  /* colNo */
  "coeff",                             /* aName */
  "ComputeSquaredPolynomialCoefficients",/* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ComputeSquaredPolynomialCoefficients.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo jb_emlrtBCI = { -1, /* iFirst */
  -1,                                  /* iLast */
  16,                                  /* lineNo */
  35,                                  /* colNo */
  "coeff_squared",                     /* aName */
  "ComputeSquaredPolynomialCoefficients",/* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\polynomial\\ComputeSquaredPolynomialCoefficients.m",/* pName */
  0                                    /* checkKind */
};

/* Function Definitions */
void c_ComputeSquaredPolynomialCoeff(const emlrtStack *sp, const emxArray_real_T
  *coeff, emxArray_real_T *coeff_squared)
{
  int32_T varargin_1;
  int32_T n;
  real_T m;
  int32_T i13;
  real_T d1;
  int32_T loop_ub;
  int32_T i;
  int32_T j;
  int32_T i14;
  int32_T i15;
  int32_T i16;
  int32_T i17;

  /*  Given polynomial coefficients for: */
  /*  f(x) = c_n*x^n + c_n-1*x^(n-1) + ... c_1*x + c_0 */
  /*  This function computes the coefficients of the polynomial f^2(x) */
  /*  Coefficients are ordered such that coeff(1) = c_n  (highest order) */
  /*  f^2(x) = sum(sum(c_i * c_j * x^(i+j)) */
  /*  The resulting coefficients, coeff_k = c_i*c_j | i+j=k */
  varargin_1 = coeff->size[0] - 1;
  n = coeff->size[0];

  /*  input polynomial order */
  m = 2.0 * ((real_T)coeff->size[0] - 1.0);

  /*  squared (output) polynomial order */
  i13 = coeff_squared->size[0];
  if (!(m + 1.0 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(m + 1.0, &f_emlrtDCI, sp);
  }

  d1 = m + 1.0;
  if (d1 != (int32_T)d1) {
    emlrtIntegerCheckR2012b(d1, &e_emlrtDCI, sp);
  }

  coeff_squared->size[0] = (int32_T)d1;
  emxEnsureCapacity_real_T(sp, coeff_squared, i13, &ib_emlrtRTEI);
  if (!(m + 1.0 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(m + 1.0, &f_emlrtDCI, sp);
  }

  d1 = m + 1.0;
  if (d1 != (int32_T)d1) {
    emlrtIntegerCheckR2012b(d1, &e_emlrtDCI, sp);
  }

  loop_ub = (int32_T)d1;
  for (i13 = 0; i13 < loop_ub; i13++) {
    coeff_squared->data[i13] = 0.0;
  }

  emlrtForLoopVectorCheckR2012b(0.0, 1.0, m, mxDOUBLE_CLASS, (int32_T)(m + 1.0),
    &xb_emlrtRTEI, sp);
  loop_ub = 0;
  while (loop_ub <= (int32_T)(m + 1.0) - 1) {
    i = 0;
    while (i <= varargin_1) {
      j = 0;
      while (j <= varargin_1) {
        if ((real_T)((uint32_T)i + j) == loop_ub) {
          i13 = coeff_squared->size[0];
          i14 = (int32_T)((m - (real_T)loop_ub) + 1.0);
          if (!((i14 >= 1) && (i14 <= i13))) {
            emlrtDynamicBoundsCheckR2012b(i14, 1, i13, &gb_emlrtBCI, sp);
          }

          i13 = coeff->size[0];
          i15 = n - i;
          if (!((i15 >= 1) && (i15 <= i13))) {
            emlrtDynamicBoundsCheckR2012b(i15, 1, i13, &hb_emlrtBCI, sp);
          }

          i13 = coeff->size[0];
          i16 = n - j;
          if (!((i16 >= 1) && (i16 <= i13))) {
            emlrtDynamicBoundsCheckR2012b(i16, 1, i13, &ib_emlrtBCI, sp);
          }

          i13 = coeff_squared->size[0];
          i17 = (int32_T)((m - (real_T)loop_ub) + 1.0);
          if (!((i17 >= 1) && (i17 <= i13))) {
            emlrtDynamicBoundsCheckR2012b(i17, 1, i13, &jb_emlrtBCI, sp);
          }

          coeff_squared->data[i17 - 1] = coeff_squared->data[i14 - 1] +
            coeff->data[i15 - 1] * coeff->data[i16 - 1];
        }

        j++;
        if (*emlrtBreakCheckR2012bFlagVar != 0) {
          emlrtBreakCheckR2012b(sp);
        }
      }

      i++;
      if (*emlrtBreakCheckR2012bFlagVar != 0) {
        emlrtBreakCheckR2012b(sp);
      }
    }

    loop_ub++;
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }
}

/* End of code generation (ComputeSquaredPolynomialCoefficients.c) */
