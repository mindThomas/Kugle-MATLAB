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
#include "ExtractDistanceTrajectory.h"
#include "repmat.h"
#include "ExtractDistanceTrajectory_emxutil.h"
#include "eml_int_forloop_overflow_check.h"
#include "ExtractDistanceTrajectory_data.h"

/* Variable Definitions */
static emlrtRSInfo i_emlrtRSI = { 63,  /* lineNo */
  "repmat",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m"/* pathName */
};

static emlrtRSInfo j_emlrtRSI = { 58,  /* lineNo */
  "repmat",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m"/* pathName */
};

static emlrtRSInfo k_emlrtRSI = { 22,  /* lineNo */
  "repmat",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m"/* pathName */
};

static emlrtRTEInfo h_emlrtRTEI = { 1, /* lineNo */
  14,                                  /* colNo */
  "repmat",                            /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m"/* pName */
};

static emlrtRTEInfo x_emlrtRTEI = { 46,/* lineNo */
  19,                                  /* colNo */
  "assertValidSizeArg",                /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\assertValidSizeArg.m"/* pName */
};

static emlrtRTEInfo y_emlrtRTEI = { 61,/* lineNo */
  15,                                  /* colNo */
  "assertValidSizeArg",                /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\assertValidSizeArg.m"/* pName */
};

/* Function Definitions */
void repmat(const emlrtStack *sp, const real_T a[2], const real_T varargin_1[2],
            emxArray_real_T *b)
{
  int32_T ntilerows;
  boolean_T exitg1;
  real_T n;
  boolean_T overflow;
  int32_T jcol;
  int32_T ibmat;
  int32_T itilerow;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &k_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  ntilerows = 0;
  exitg1 = false;
  while ((!exitg1) && (ntilerows < 2)) {
    if (varargin_1[ntilerows] != varargin_1[ntilerows]) {
      emlrtErrorWithMessageIdR2018a(&st, &x_emlrtRTEI,
        "Coder:toolbox:eml_assert_valid_size_arg_invalidSizeVector",
        "Coder:toolbox:eml_assert_valid_size_arg_invalidSizeVector", 4, 12,
        MIN_int32_T, 12, MAX_int32_T);
      exitg1 = true;
    } else {
      ntilerows++;
    }
  }

  n = 1.0;
  for (ntilerows = 0; ntilerows < 2; ntilerows++) {
    if (varargin_1[ntilerows] <= 0.0) {
      n = 0.0;
    } else {
      n *= varargin_1[ntilerows];
    }
  }

  if (!(n <= 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(&st, &y_emlrtRTEI, "Coder:MATLAB:pmaxsize",
      "Coder:MATLAB:pmaxsize", 0);
  }

  ntilerows = b->size[0] * b->size[1];
  b->size[0] = (int32_T)varargin_1[0];
  b->size[1] = 2;
  emxEnsureCapacity_real_T(sp, b, ntilerows, &h_emlrtRTEI);
  ntilerows = (int32_T)varargin_1[0];
  st.site = &j_emlrtRSI;
  overflow = ((!(1 > (int32_T)varargin_1[0])) && ((int32_T)varargin_1[0] >
    2147483646));
  for (jcol = 0; jcol < 2; jcol++) {
    ibmat = jcol * ntilerows;
    st.site = &i_emlrtRSI;
    if (overflow) {
      b_st.site = &l_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (itilerow = 1; itilerow <= ntilerows; itilerow++) {
      b->data[(ibmat + itilerow) - 1] = a[jcol];
    }
  }
}

/* End of code generation (repmat.c) */
