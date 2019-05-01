/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_FitReferencePathPolynomial_api.c
 *
 * Code generation for function '_coder_FitReferencePathPolynomial_api'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "FitReferencePathPolynomial.h"
#include "_coder_FitReferencePathPolynomial_api.h"
#include "FitReferencePathPolynomial_emxutil.h"
#include "FitReferencePathPolynomial_data.h"

/* Variable Definitions */
static emlrtRTEInfo lb_emlrtRTEI = { 1,/* lineNo */
  1,                                   /* colNo */
  "_coder_FitReferencePathPolynomial_api",/* fName */
  ""                                   /* pName */
};

/* Function Declarations */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_real_T *y);
static const mxArray *b_emlrt_marshallOut(const emxArray_real_T *u);
static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *approximation_order, const char_T *identifier);
static const mxArray *c_emlrt_marshallOut(const real_T u);
static real_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, emxArray_real_T *ret);
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *WindowTrajectoryPoints, const char_T *identifier, emxArray_real_T *y);
static const mxArray *emlrt_marshallOut(const emxArray_real_T *u);
static real_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);

/* Function Definitions */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_real_T *y)
{
  e_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static const mxArray *b_emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *y;
  const mxArray *m7;
  static const int32_T iv17[1] = { 0 };

  y = NULL;
  m7 = emlrtCreateNumericArray(1, iv17, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m7, (void *)&u->data[0]);
  emlrtSetDimensions((mxArray *)m7, u->size, 1);
  emlrtAssign(&y, m7);
  return y;
}

static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *approximation_order, const char_T *identifier)
{
  real_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(approximation_order), &thisId);
  emlrtDestroyArray(&approximation_order);
  return y;
}

static const mxArray *c_emlrt_marshallOut(const real_T u)
{
  const mxArray *y;
  const mxArray *m8;
  y = NULL;
  m8 = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m8);
  return y;
}

static real_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = f_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, emxArray_real_T *ret)
{
  static const int32_T dims[2] = { -1, 2 };

  const boolean_T bv0[2] = { true, false };

  int32_T iv18[2];
  emlrtCheckVsBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims, &bv0[0],
    iv18);
  ret->size[0] = iv18[0];
  ret->size[1] = iv18[1];
  ret->allocatedSize = ret->size[0] * ret->size[1];
  ret->data = (real_T *)emlrtMxGetData(src);
  ret->canFreeData = false;
  emlrtDestroyArray(&src);
}

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *WindowTrajectoryPoints, const char_T *identifier, emxArray_real_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  b_emlrt_marshallIn(sp, emlrtAlias(WindowTrajectoryPoints), &thisId, y);
  emlrtDestroyArray(&WindowTrajectoryPoints);
}

static const mxArray *emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *y;
  const mxArray *m6;
  static const int32_T iv16[2] = { 0, 0 };

  y = NULL;
  m6 = emlrtCreateNumericArray(2, iv16, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m6, (void *)&u->data[0]);
  emlrtSetDimensions((mxArray *)m6, u->size, 2);
  emlrtAssign(&y, m6);
  return y;
}

static real_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  real_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, &dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

void FitReferencePathPolynomial_api(const mxArray * const prhs[5], int32_T nlhs,
  const mxArray *plhs[5])
{
  emxArray_real_T *WindowTrajectoryPoints;
  emxArray_real_T *TrajectoryPoints;
  emxArray_real_T *coeff_xs;
  emxArray_real_T *coeff_ys;
  real_T approximation_order;
  real_T velocity;
  real_T ts;
  real_T N;
  real_T windowTrajectoryLength;
  real_T minDistancePoint;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  emxInit_real_T(&st, &WindowTrajectoryPoints, 2, &lb_emlrtRTEI, true);
  emxInit_real_T(&st, &TrajectoryPoints, 2, &lb_emlrtRTEI, true);
  emxInit_real_T1(&st, &coeff_xs, 1, &lb_emlrtRTEI, true);
  emxInit_real_T1(&st, &coeff_ys, 1, &lb_emlrtRTEI, true);

  /* Marshall function inputs */
  emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "WindowTrajectoryPoints",
                   WindowTrajectoryPoints);
  approximation_order = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]),
    "approximation_order");
  velocity = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "velocity");
  ts = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "ts");
  N = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "N");

  /* Invoke the target function */
  FitReferencePathPolynomial(&st, WindowTrajectoryPoints, approximation_order,
    velocity, ts, N, TrajectoryPoints, coeff_xs, coeff_ys,
    &windowTrajectoryLength, &minDistancePoint);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(TrajectoryPoints);
  TrajectoryPoints->canFreeData = false;
  emxFree_real_T(&st, &TrajectoryPoints);
  WindowTrajectoryPoints->canFreeData = false;
  emxFree_real_T(&st, &WindowTrajectoryPoints);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(coeff_xs);
  }

  coeff_xs->canFreeData = false;
  emxFree_real_T(&st, &coeff_xs);
  if (nlhs > 2) {
    plhs[2] = b_emlrt_marshallOut(coeff_ys);
  }

  coeff_ys->canFreeData = false;
  emxFree_real_T(&st, &coeff_ys);
  if (nlhs > 3) {
    plhs[3] = c_emlrt_marshallOut(windowTrajectoryLength);
  }

  if (nlhs > 4) {
    plhs[4] = c_emlrt_marshallOut(minDistancePoint);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

/* End of code generation (_coder_FitReferencePathPolynomial_api.c) */
