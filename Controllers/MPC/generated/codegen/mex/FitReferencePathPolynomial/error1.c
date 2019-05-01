/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * error1.c
 *
 * Code generation for function 'error1'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "FitReferencePathPolynomial.h"
#include "error1.h"
#include "FitReferencePathPolynomial_mexutil.h"
#include "FitReferencePathPolynomial_data.h"

/* Function Definitions */
void c_error(const emlrtStack *sp)
{
  const mxArray *y;
  const mxArray *m0;
  static const int32_T iv0[2] = { 1, 39 };

  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  y = NULL;
  m0 = emlrtCreateCharArray(2, iv0);
  emlrtInitCharArrayR2013a(sp, 39, m0, &cv0[0]);
  emlrtAssign(&y, m0);
  st.site = &ke_emlrtRSI;
  l_error(&st, y, &emlrtMCI);
}

void d_error(const emlrtStack *sp)
{
  const mxArray *y;
  const mxArray *m1;
  static const int32_T iv1[2] = { 1, 60 };

  static const char_T varargin_1[60] = { 'P', 'o', 'l', 'y', 'n', 'o', 'm', 'i',
    'a', 'l', ' ', 'o', 'r', 'd', 'e', 'r', ' ', 't', 'o', 'o', ' ', 'l', 'o',
    'w', ' ', 't', 'o', ' ', 'e', 'n', 'f', 'o', 'r', 'c', 'e', ' ', 'b', 'o',
    't', 'h', ' ', 't', 'y', 'p', 'e', ' ', 'o', 'f', ' ', 'c', 'o', 'n', 's',
    't', 'r', 'a', 'i', 'n', 't', 's' };

  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  y = NULL;
  m1 = emlrtCreateCharArray(2, iv1);
  emlrtInitCharArrayR2013a(sp, 60, m1, &varargin_1[0]);
  emlrtAssign(&y, m1);
  st.site = &ke_emlrtRSI;
  l_error(&st, y, &emlrtMCI);
}

void e_error(const emlrtStack *sp)
{
  const mxArray *y;
  const mxArray *m2;
  static const int32_T iv2[2] = { 1, 57 };

  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  y = NULL;
  m2 = emlrtCreateCharArray(2, iv2);
  emlrtInitCharArrayR2013a(sp, 57, m2, &cv1[0]);
  emlrtAssign(&y, m2);
  st.site = &ke_emlrtRSI;
  l_error(&st, y, &emlrtMCI);
}

void f_error(const emlrtStack *sp)
{
  const mxArray *y;
  const mxArray *m3;
  static const int32_T iv3[2] = { 1, 75 };

  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  y = NULL;
  m3 = emlrtCreateCharArray(2, iv3);
  emlrtInitCharArrayR2013a(sp, 75, m3, &cv2[0]);
  emlrtAssign(&y, m3);
  st.site = &ke_emlrtRSI;
  l_error(&st, y, &emlrtMCI);
}

void g_error(const emlrtStack *sp)
{
  const mxArray *y;
  const mxArray *m4;
  static const int32_T iv4[2] = { 1, 109 };

  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  y = NULL;
  m4 = emlrtCreateCharArray(2, iv4);
  emlrtInitCharArrayR2013a(sp, 109, m4, &cv3[0]);
  emlrtAssign(&y, m4);
  st.site = &ke_emlrtRSI;
  l_error(&st, y, &emlrtMCI);
}

/* End of code generation (error1.c) */
