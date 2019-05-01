/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * diff.c
 *
 * Code generation for function 'diff'
 *
 */

/* Include files */
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "FitReferencePathPolynomial.h"
#include "diff.h"
#include "FitReferencePathPolynomial_emxutil.h"
#include "eml_int_forloop_overflow_check.h"
#include "FitReferencePathPolynomial_data.h"

/* Variable Definitions */
static emlrtRSInfo r_emlrtRSI = { 108, /* lineNo */
  "diff",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\diff.m"/* pathName */
};

static emlrtRSInfo s_emlrtRSI = { 106, /* lineNo */
  "diff",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\diff.m"/* pathName */
};

static emlrtRTEInfo j_emlrtRTEI = { 1, /* lineNo */
  14,                                  /* colNo */
  "diff",                              /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\diff.m"/* pName */
};

static emlrtRTEInfo nb_emlrtRTEI = { 51,/* lineNo */
  19,                                  /* colNo */
  "diff",                              /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\diff.m"/* pName */
};

/* Function Definitions */
void diff(const emlrtStack *sp, const emxArray_real_T *x, emxArray_real_T *y)
{
  int32_T dimSize;
  int32_T i1;
  int32_T iyStart;
  boolean_T overflow;
  int32_T ySize_idx_0;
  int32_T r;
  int32_T ixLead;
  int32_T iyLead;
  real_T work_data_idx_0;
  int32_T m;
  real_T tmp1;
  real_T tmp2;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  dimSize = x->size[0];
  if (x->size[0] == 0) {
    iyStart = y->size[0] * y->size[1];
    y->size[0] = 0;
    y->size[1] = 2;
    emxEnsureCapacity_real_T1(sp, y, iyStart, &j_emlrtRTEI);
  } else {
    i1 = x->size[0] - 1;
    if (muIntScalarMin_sint32(i1, 1) < 1) {
      iyStart = y->size[0] * y->size[1];
      y->size[0] = 0;
      y->size[1] = 2;
      emxEnsureCapacity_real_T1(sp, y, iyStart, &j_emlrtRTEI);
    } else {
      overflow = (x->size[0] != 1);
      if (!overflow) {
        emlrtErrorWithMessageIdR2018a(sp, &nb_emlrtRTEI,
          "Coder:toolbox:autoDimIncompatibility",
          "Coder:toolbox:autoDimIncompatibility", 0);
      }

      ySize_idx_0 = x->size[0] - 1;
      iyStart = y->size[0] * y->size[1];
      y->size[0] = ySize_idx_0;
      y->size[1] = 2;
      emxEnsureCapacity_real_T1(sp, y, iyStart, &j_emlrtRTEI);
      if (!(y->size[0] == 0)) {
        ySize_idx_0 = 0;
        iyStart = 0;
        overflow = ((!(2 > dimSize)) && (dimSize > 2147483646));
        for (r = 0; r < 2; r++) {
          ixLead = ySize_idx_0 + 1;
          iyLead = iyStart;
          work_data_idx_0 = x->data[ySize_idx_0];
          st.site = &s_emlrtRSI;
          if (overflow) {
            b_st.site = &t_emlrtRSI;
            check_forloop_overflow_error(&b_st);
          }

          for (m = 2; m <= dimSize; m++) {
            tmp1 = x->data[ixLead];
            st.site = &r_emlrtRSI;
            tmp2 = work_data_idx_0;
            work_data_idx_0 = tmp1;
            tmp1 -= tmp2;
            ixLead++;
            y->data[iyLead] = tmp1;
            iyLead++;
          }

          ySize_idx_0 += dimSize;
          iyStart = (iyStart + dimSize) - 1;
        }
      }
    }
  }
}

/* End of code generation (diff.c) */
