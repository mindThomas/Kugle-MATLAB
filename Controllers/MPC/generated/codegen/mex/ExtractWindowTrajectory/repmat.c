/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * repmat.c
 *
 * Code generation for function 'repmat'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "ExtractWindowTrajectory.h"
#include "repmat.h"
#include "ExtractWindowTrajectory_emxutil.h"
#include "eml_int_forloop_overflow_check.h"
#include "assertValidSizeArg.h"
#include "ExtractWindowTrajectory_data.h"

/* Variable Definitions */
static emlrtRSInfo u_emlrtRSI = { 63,  /* lineNo */
  "repmat",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m"/* pathName */
};

static emlrtRSInfo v_emlrtRSI = { 58,  /* lineNo */
  "repmat",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m"/* pathName */
};

static emlrtRSInfo w_emlrtRSI = { 22,  /* lineNo */
  "repmat",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m"/* pathName */
};

static emlrtRSInfo kb_emlrtRSI = { 65, /* lineNo */
  "repmat",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m"/* pathName */
};

static emlrtMCInfo emlrtMCI = { 41,    /* lineNo */
  5,                                   /* colNo */
  "repmat",                            /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m"/* pName */
};

static emlrtRTEInfo m_emlrtRTEI = { 1, /* lineNo */
  14,                                  /* colNo */
  "repmat",                            /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m"/* pName */
};

static emlrtRSInfo nd_emlrtRSI = { 41, /* lineNo */
  "repmat",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m"/* pathName */
};

/* Function Declarations */
static void b_error(const emlrtStack *sp, const mxArray *b, emlrtMCInfo
                    *location);

/* Function Definitions */
static void b_error(const emlrtStack *sp, const mxArray *b, emlrtMCInfo
                    *location)
{
  const mxArray *pArray;
  pArray = b;
  emlrtCallMATLABR2012b(sp, 0, NULL, 1, &pArray, "error", true, location);
}

void b_repmat(const emlrtStack *sp, const emxArray_real_T *a, const real_T
              varargin_1[2], emxArray_real_T *b)
{
  int32_T outsize_idx_0;
  int32_T itilerow;
  const mxArray *y;
  const mxArray *m0;
  static const int32_T iv0[2] = { 1, 15 };

  static const char_T u[15] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'p', 'm', 'a',
    'x', 's', 'i', 'z', 'e' };

  int32_T ibcol;
  int32_T k;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &w_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  assertValidSizeArg(&st, varargin_1);
  outsize_idx_0 = a->size[0] * (int32_T)varargin_1[0];
  if (!(outsize_idx_0 == (real_T)a->size[0] * (real_T)(int32_T)varargin_1[0])) {
    y = NULL;
    m0 = emlrtCreateCharArray(2, iv0);
    emlrtInitCharArrayR2013a(sp, 15, m0, &u[0]);
    emlrtAssign(&y, m0);
    st.site = &nd_emlrtRSI;
    b_error(&st, y, &emlrtMCI);
  }

  itilerow = b->size[0] * b->size[1];
  b->size[0] = outsize_idx_0;
  b->size[1] = 1;
  emxEnsureCapacity_real_T(sp, b, itilerow, &m_emlrtRTEI);
  outsize_idx_0 = a->size[0];
  st.site = &v_emlrtRSI;
  st.site = &u_emlrtRSI;
  if ((!(1 > (int32_T)varargin_1[0])) && ((int32_T)varargin_1[0] > 2147483646))
  {
    b_st.site = &x_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (itilerow = 1; itilerow <= (int32_T)varargin_1[0]; itilerow++) {
    ibcol = (itilerow - 1) * outsize_idx_0;
    st.site = &kb_emlrtRSI;
    if ((!(1 > outsize_idx_0)) && (outsize_idx_0 > 2147483646)) {
      b_st.site = &x_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (k = 0; k < outsize_idx_0; k++) {
      b->data[ibcol + k] = a->data[k];
    }
  }
}

void repmat(const emlrtStack *sp, const real_T a[2], const real_T varargin_1[2],
            emxArray_real_T *b)
{
  int32_T ntilerows;
  boolean_T overflow;
  int32_T jcol;
  int32_T ibmat;
  int32_T itilerow;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &w_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  assertValidSizeArg(&st, varargin_1);
  ntilerows = b->size[0] * b->size[1];
  b->size[0] = (int32_T)varargin_1[0];
  b->size[1] = 2;
  emxEnsureCapacity_real_T(sp, b, ntilerows, &m_emlrtRTEI);
  ntilerows = (int32_T)varargin_1[0];
  st.site = &v_emlrtRSI;
  overflow = ((!(1 > (int32_T)varargin_1[0])) && ((int32_T)varargin_1[0] >
    2147483646));
  for (jcol = 0; jcol < 2; jcol++) {
    ibmat = jcol * ntilerows;
    st.site = &u_emlrtRSI;
    if (overflow) {
      b_st.site = &x_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (itilerow = 1; itilerow <= ntilerows; itilerow++) {
      b->data[(ibmat + itilerow) - 1] = a[jcol];
    }
  }
}

/* End of code generation (repmat.c) */
