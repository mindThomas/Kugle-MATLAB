/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ifWhileCond.c
 *
 * Code generation for function 'ifWhileCond'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "ExtractWindowTrajectory.h"
#include "ifWhileCond.h"
#include "eml_int_forloop_overflow_check.h"
#include "ExtractWindowTrajectory_data.h"

/* Variable Definitions */
static emlrtRSInfo cd_emlrtRSI = { 17, /* lineNo */
  "ifWhileCond",                       /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\ifWhileCond.m"/* pathName */
};

static emlrtRSInfo dd_emlrtRSI = { 30, /* lineNo */
  "ifWhileCond",                       /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\ifWhileCond.m"/* pathName */
};

/* Function Definitions */
boolean_T ifWhileCond(const emlrtStack *sp, const emxArray_boolean_T *x)
{
  boolean_T y;
  boolean_T overflow;
  int32_T k;
  boolean_T exitg1;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  y = !(x->size[0] == 0);
  if (y) {
    st.site = &cd_emlrtRSI;
    b_st.site = &dd_emlrtRSI;
    overflow = ((!(1 > x->size[0])) && (x->size[0] > 2147483646));
    if (overflow) {
      c_st.site = &x_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }

    k = 1;
    exitg1 = false;
    while ((!exitg1) && (k <= x->size[0])) {
      if (!x->data[k - 1]) {
        y = false;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  return y;
}

/* End of code generation (ifWhileCond.c) */
