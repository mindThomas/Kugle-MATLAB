/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_ExtractDistanceTrajectory_api.c
 *
 * Code generation for function '_coder_ExtractDistanceTrajectory_api'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "ExtractDistanceTrajectory.h"
#include "_coder_ExtractDistanceTrajectory_api.h"
#include "ExtractDistanceTrajectory_emxutil.h"
#include "ExtractDistanceTrajectory_data.h"

/* Variable Definitions */
static emlrtRTEInfo q_emlrtRTEI = { 1, /* lineNo */
  1,                                   /* colNo */
  "_coder_ExtractDistanceTrajectory_api",/* fName */
  ""                                   /* pName */
};

/* Function Declarations */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_real_T *y);
static const mxArray *b_emlrt_marshallOut(const real_T u);
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *RobotPos,
  const char_T *identifier))[2];
static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[2];
static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *RobotYaw,
  const char_T *identifier);
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *TrajectoryPoints, const char_T *identifier, emxArray_real_T *y);
static const mxArray *emlrt_marshallOut(const emxArray_real_T *u);
static real_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, emxArray_real_T *ret);
static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[2];
static real_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);

/* Function Definitions */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, emxArray_real_T *y)
{
  g_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static const mxArray *b_emlrt_marshallOut(const real_T u)
{
  const mxArray *y;
  const mxArray *m1;
  y = NULL;
  m1 = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m1);
  return y;
}

static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *RobotPos,
  const char_T *identifier))[2]
{
  real_T (*y)[2];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(RobotPos), &thisId);
  emlrtDestroyArray(&RobotPos);
  return y;
}
  static real_T (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[2]
{
  real_T (*y)[2];
  y = h_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *RobotYaw,
  const char_T *identifier)
{
  real_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(RobotYaw), &thisId);
  emlrtDestroyArray(&RobotYaw);
  return y;
}

static void emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *TrajectoryPoints, const char_T *identifier, emxArray_real_T *y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  b_emlrt_marshallIn(sp, emlrtAlias(TrajectoryPoints), &thisId, y);
  emlrtDestroyArray(&TrajectoryPoints);
}

static const mxArray *emlrt_marshallOut(const emxArray_real_T *u)
{
  const mxArray *y;
  const mxArray *m0;
  static const int32_T iv0[2] = { 0, 0 };

  y = NULL;
  m0 = emlrtCreateNumericArray(2, iv0, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m0, (void *)&u->data[0]);
  emlrtSetDimensions((mxArray *)m0, u->size, 2);
  emlrtAssign(&y, m0);
  return y;
}

static real_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = i_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, emxArray_real_T *ret)
{
  static const int32_T dims[2] = { -1, 2 };

  const boolean_T bv0[2] = { true, false };

  int32_T iv1[2];
  emlrtCheckVsBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims, &bv0[0],
    iv1);
  ret->size[0] = iv1[0];
  ret->size[1] = iv1[1];
  ret->allocatedSize = ret->size[0] * ret->size[1];
  ret->data = (real_T *)emlrtMxGetData(src);
  ret->canFreeData = false;
  emlrtDestroyArray(&src);
}

static real_T (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[2]
{
  real_T (*ret)[2];
  static const int32_T dims[2] = { 1, 2 };

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  ret = (real_T (*)[2])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
  static real_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  real_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, &dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

void ExtractDistanceTrajectory_api(const mxArray * const prhs[7], int32_T nlhs,
  const mxArray *plhs[4])
{
  emxArray_real_T *TrajectoryPoints;
  emxArray_real_T *WindowTrajectory;
  real_T (*RobotPos)[2];
  real_T RobotYaw;
  real_T (*Velocity)[2];
  real_T ExtractDist;
  real_T OrientationSelection;
  real_T PreviousClosestIndex;
  real_T nTrajPoints;
  real_T WindowOrientation;
  real_T ClosestIdx;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtHeapReferenceStackEnterFcnR2012b(&st);
  emxInit_real_T(&st, &TrajectoryPoints, 2, &q_emlrtRTEI, true);
  emxInit_real_T(&st, &WindowTrajectory, 2, &q_emlrtRTEI, true);

  /* Marshall function inputs */
  emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "TrajectoryPoints",
                   TrajectoryPoints);
  RobotPos = c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "RobotPos");
  RobotYaw = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "RobotYaw");
  Velocity = c_emlrt_marshallIn(&st, emlrtAlias(prhs[3]), "Velocity");
  ExtractDist = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "ExtractDist");
  OrientationSelection = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[5]),
    "OrientationSelection");
  PreviousClosestIndex = e_emlrt_marshallIn(&st, emlrtAliasP(prhs[6]),
    "PreviousClosestIndex");

  /* Invoke the target function */
  ExtractDistanceTrajectory(&st, TrajectoryPoints, *RobotPos, RobotYaw,
    *Velocity, ExtractDist, OrientationSelection, PreviousClosestIndex,
    WindowTrajectory, &nTrajPoints, &WindowOrientation, &ClosestIdx);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(WindowTrajectory);
  WindowTrajectory->canFreeData = false;
  emxFree_real_T(&st, &WindowTrajectory);
  TrajectoryPoints->canFreeData = false;
  emxFree_real_T(&st, &TrajectoryPoints);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(nTrajPoints);
  }

  if (nlhs > 2) {
    plhs[2] = b_emlrt_marshallOut(WindowOrientation);
  }

  if (nlhs > 3) {
    plhs[3] = b_emlrt_marshallOut(ClosestIdx);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(&st);
}

/* End of code generation (_coder_ExtractDistanceTrajectory_api.c) */
