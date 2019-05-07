/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ExtractDistanceTrajectory.c
 *
 * Code generation for function 'ExtractDistanceTrajectory'
 *
 */

/* Include files */
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "ExtractDistanceTrajectory.h"
#include "ExtractDistanceTrajectory_emxutil.h"
#include "eml_int_forloop_overflow_check.h"
#include "error.h"
#include "power.h"
#include "sort1.h"
#include "mod.h"
#include "RotateTrajectory.h"
#include "repmat.h"
#include "ExtractDistanceTrajectory_data.h"

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = { 20,    /* lineNo */
  "ExtractDistanceTrajectory",         /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pathName */
};

static emlrtRSInfo b_emlrtRSI = { 21,  /* lineNo */
  "ExtractDistanceTrajectory",         /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pathName */
};

static emlrtRSInfo c_emlrtRSI = { 24,  /* lineNo */
  "ExtractDistanceTrajectory",         /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pathName */
};

static emlrtRSInfo d_emlrtRSI = { 25,  /* lineNo */
  "ExtractDistanceTrajectory",         /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pathName */
};

static emlrtRSInfo e_emlrtRSI = { 29,  /* lineNo */
  "ExtractDistanceTrajectory",         /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pathName */
};

static emlrtRSInfo f_emlrtRSI = { 31,  /* lineNo */
  "ExtractDistanceTrajectory",         /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pathName */
};

static emlrtRSInfo g_emlrtRSI = { 48,  /* lineNo */
  "ExtractDistanceTrajectory",         /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pathName */
};

static emlrtRSInfo h_emlrtRSI = { 49,  /* lineNo */
  "ExtractDistanceTrajectory",         /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pathName */
};

static emlrtRSInfo v_emlrtRSI = { 15,  /* lineNo */
  "min",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\min.m"/* pathName */
};

static emlrtRSInfo w_emlrtRSI = { 16,  /* lineNo */
  "minOrMax",                          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

static emlrtRSInfo x_emlrtRSI = { 38,  /* lineNo */
  "minOrMax",                          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

static emlrtRSInfo y_emlrtRSI = { 112, /* lineNo */
  "unaryMinOrMax",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo ab_emlrtRSI = { 852,/* lineNo */
  "unaryMinOrMax",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo bb_emlrtRSI = { 844,/* lineNo */
  "unaryMinOrMax",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo cb_emlrtRSI = { 894,/* lineNo */
  "unaryMinOrMax",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo db_emlrtRSI = { 910,/* lineNo */
  "unaryMinOrMax",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo eb_emlrtRSI = { 23, /* lineNo */
  "sort",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\sort.m"/* pathName */
};

static emlrtRSInfo fc_emlrtRSI = { 108,/* lineNo */
  "diff",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\diff.m"/* pathName */
};

static emlrtRSInfo gc_emlrtRSI = { 106,/* lineNo */
  "diff",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\diff.m"/* pathName */
};

static emlrtRSInfo hc_emlrtRSI = { 12, /* lineNo */
  "sqrt",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elfun\\sqrt.m"/* pathName */
};

static emlrtRSInfo ic_emlrtRSI = { 15, /* lineNo */
  "sqrt",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elfun\\sqrt.m"/* pathName */
};

static emlrtRSInfo jc_emlrtRSI = { 31, /* lineNo */
  "applyScalarFunctionInPlace",        /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\applyScalarFunctionInPlace.m"/* pathName */
};

static emlrtRSInfo kc_emlrtRSI = { 14, /* lineNo */
  "cumsum",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\cumsum.m"/* pathName */
};

static emlrtRSInfo lc_emlrtRSI = { 11, /* lineNo */
  "cumop",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\private\\cumop.m"/* pathName */
};

static emlrtRSInfo mc_emlrtRSI = { 32, /* lineNo */
  "useConstantDim",                    /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\useConstantDim.m"/* pathName */
};

static emlrtRSInfo nc_emlrtRSI = { 93, /* lineNo */
  "cumop",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\private\\cumop.m"/* pathName */
};

static emlrtRSInfo oc_emlrtRSI = { 119,/* lineNo */
  "cumop",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\private\\cumop.m"/* pathName */
};

static emlrtRSInfo pc_emlrtRSI = { 286,/* lineNo */
  "cumop",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\private\\cumop.m"/* pathName */
};

static emlrtRTEInfo emlrtRTEI = { 1,   /* lineNo */
  75,                                  /* colNo */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pName */
};

static emlrtRTEInfo b_emlrtRTEI = { 48,/* lineNo */
  29,                                  /* colNo */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pName */
};

static emlrtRTEInfo c_emlrtRTEI = { 21,/* lineNo */
  5,                                   /* colNo */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pName */
};

static emlrtRTEInfo d_emlrtRTEI = { 24,/* lineNo */
  5,                                   /* colNo */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pName */
};

static emlrtRTEInfo e_emlrtRTEI = { 30,/* lineNo */
  9,                                   /* colNo */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pName */
};

static emlrtRTEInfo f_emlrtRTEI = { 39,/* lineNo */
  5,                                   /* colNo */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pName */
};

static emlrtRTEInfo g_emlrtRTEI = { 49,/* lineNo */
  9,                                   /* colNo */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pName */
};

static emlrtRTEInfo u_emlrtRTEI = { 51,/* lineNo */
  19,                                  /* colNo */
  "diff",                              /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\diff.m"/* pName */
};

static emlrtRTEInfo v_emlrtRTEI = { 77,/* lineNo */
  27,                                  /* colNo */
  "unaryMinOrMax",                     /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pName */
};

static emlrtRTEInfo w_emlrtRTEI = { 22,/* lineNo */
  27,                                  /* colNo */
  "unaryMinOrMax",                     /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pName */
};

static emlrtBCInfo emlrtBCI = { -1,    /* iFirst */
  -1,                                  /* iLast */
  51,                                  /* lineNo */
  59,                                  /* colNo */
  "WindowTrajectory_tmp",              /* aName */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo emlrtECI = { -1,    /* nDims */
  49,                                  /* lineNo */
  34,                                  /* colNo */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pName */
};

static emlrtBCInfo b_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  39,                                  /* lineNo */
  102,                                 /* colNo */
  "RotatedCenteredTrajectory",         /* aName */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  39,                                  /* lineNo */
  100,                                 /* colNo */
  "RotatedCenteredTrajectory",         /* aName */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  39,                                  /* lineNo */
  66,                                  /* colNo */
  "RotatedCenteredTrajectory",         /* aName */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo e_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  39,                                  /* lineNo */
  55,                                  /* colNo */
  "RotatedCenteredTrajectory",         /* aName */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo f_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  30,                                  /* lineNo */
  58,                                  /* colNo */
  "distSorted",                        /* aName */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo b_emlrtECI = { -1,  /* nDims */
  24,                                  /* lineNo */
  12,                                  /* colNo */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pName */
};

static emlrtECInfo c_emlrtECI = { 2,   /* nDims */
  20,                                  /* lineNo */
  36,                                  /* colNo */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pName */
};

static emlrtBCInfo g_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  30,                                  /* lineNo */
  32,                                  /* colNo */
  "idxSorted",                         /* aName */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo h_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  32,                                  /* lineNo */
  33,                                  /* colNo */
  "idxSorted2",                        /* aName */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m",/* pName */
  0                                    /* checkKind */
};

/* Function Definitions */
void ExtractDistanceTrajectory(const emlrtStack *sp, const emxArray_real_T
  *TrajectoryPoints, const real_T RobotPos[2], real_T RobotYaw, const real_T
  Velocity[2], real_T ExtractDist, real_T OrientationSelection, real_T
  PreviousClosestIndex, emxArray_real_T *WindowTrajectory, real_T *nTrajPoints,
  real_T *WindowOrientation, real_T *ClosestIdx)
{
  int32_T ySize_idx_0;
  emxArray_real_T *RotatedCenteredTrajectory;
  real_T b_RobotPos[2];
  real_T n[2];
  int32_T m;
  int32_T b_TrajectoryPoints[2];
  emxArray_real_T *c_TrajectoryPoints;
  int32_T b_RotatedCenteredTrajectory[2];
  emxArray_real_T *c_RotatedCenteredTrajectory;
  emxArray_real_T *Dist;
  emxArray_real_T *r0;
  int32_T iyStart;
  boolean_T overflow;
  real_T tmp1;
  int32_T k;
  boolean_T exitg1;
  emxArray_real_T *idxSorted2;
  emxArray_real_T *ApproxDist;
  emxArray_int32_T *iidx;
  emxArray_real_T *WindowTrajectory_tmp;
  int32_T ixLead;
  int32_T iyLead;
  int32_T dimSize;
  int32_T i0;
  real_T work_data_idx_0;
  real_T tmp2;
  emxArray_int32_T *r1;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);

  /*  OrientationSelection = 0; */
  /*  RobotPos = [4.20, 0.8]; */
  /*  Velocity = [1, 0.2]; */
  if (OrientationSelection == 0.0) {
    /*  inertial frame */
    *WindowOrientation = 0.0;
  } else if (OrientationSelection == 1.0) {
    /*  robot yaw (heading) */
    *WindowOrientation = RobotYaw;
  } else if (OrientationSelection == 2.0) {
    /*  velocity direction */
    *WindowOrientation = muDoubleScalarAtan2(Velocity[1], Velocity[0]);
  } else {
    *WindowOrientation = 0.0;
  }

  /*  Rotate trajectory into robot heading */
  if (TrajectoryPoints->size[0] == 0) {
    ySize_idx_0 = 0;
  } else {
    ySize_idx_0 = muIntScalarMax_sint32(TrajectoryPoints->size[0], 2);
  }

  emxInit_real_T(sp, &RotatedCenteredTrajectory, 2, &c_emlrtRTEI, true);
  b_RobotPos[0] = RobotPos[0];
  b_RobotPos[1] = RobotPos[1];
  n[0] = ySize_idx_0;
  n[1] = 1.0;
  st.site = &emlrtRSI;
  repmat(&st, b_RobotPos, n, RotatedCenteredTrajectory);
  for (m = 0; m < 2; m++) {
    b_TrajectoryPoints[m] = TrajectoryPoints->size[m];
  }

  for (m = 0; m < 2; m++) {
    b_RotatedCenteredTrajectory[m] = RotatedCenteredTrajectory->size[m];
  }

  emxInit_real_T(sp, &c_TrajectoryPoints, 2, &emlrtRTEI, true);
  if ((b_TrajectoryPoints[0] != b_RotatedCenteredTrajectory[0]) ||
      (b_TrajectoryPoints[1] != b_RotatedCenteredTrajectory[1])) {
    emlrtSizeEqCheckNDR2012b(&b_TrajectoryPoints[0],
      &b_RotatedCenteredTrajectory[0], &c_emlrtECI, sp);
  }

  m = c_TrajectoryPoints->size[0] * c_TrajectoryPoints->size[1];
  c_TrajectoryPoints->size[0] = TrajectoryPoints->size[0];
  c_TrajectoryPoints->size[1] = 2;
  emxEnsureCapacity_real_T(sp, c_TrajectoryPoints, m, &emlrtRTEI);
  ySize_idx_0 = TrajectoryPoints->size[0] * TrajectoryPoints->size[1];
  for (m = 0; m < ySize_idx_0; m++) {
    c_TrajectoryPoints->data[m] = TrajectoryPoints->data[m] -
      RotatedCenteredTrajectory->data[m];
  }

  emxInit_real_T1(sp, &c_RotatedCenteredTrajectory, 1, &emlrtRTEI, true);
  st.site = &b_emlrtRSI;
  RotateTrajectory(&st, c_TrajectoryPoints, *WindowOrientation,
                   RotatedCenteredTrajectory);

  /*  Find continuous sequence of points closest to robot */
  ySize_idx_0 = RotatedCenteredTrajectory->size[0];
  m = c_RotatedCenteredTrajectory->size[0];
  c_RotatedCenteredTrajectory->size[0] = ySize_idx_0;
  emxEnsureCapacity_real_T1(sp, c_RotatedCenteredTrajectory, m, &emlrtRTEI);
  emxFree_real_T(sp, &c_TrajectoryPoints);
  for (m = 0; m < ySize_idx_0; m++) {
    c_RotatedCenteredTrajectory->data[m] = RotatedCenteredTrajectory->data[m];
  }

  emxInit_real_T1(sp, &Dist, 1, &d_emlrtRTEI, true);
  st.site = &c_emlrtRSI;
  power(&st, c_RotatedCenteredTrajectory, Dist);
  ySize_idx_0 = RotatedCenteredTrajectory->size[0];
  m = c_RotatedCenteredTrajectory->size[0];
  c_RotatedCenteredTrajectory->size[0] = ySize_idx_0;
  emxEnsureCapacity_real_T1(sp, c_RotatedCenteredTrajectory, m, &emlrtRTEI);
  for (m = 0; m < ySize_idx_0; m++) {
    c_RotatedCenteredTrajectory->data[m] = RotatedCenteredTrajectory->data[m +
      RotatedCenteredTrajectory->size[0]];
  }

  emxInit_real_T1(sp, &r0, 1, &emlrtRTEI, true);
  st.site = &c_emlrtRSI;
  power(&st, c_RotatedCenteredTrajectory, r0);
  m = Dist->size[0];
  iyStart = r0->size[0];
  if (m != iyStart) {
    emlrtSizeEqCheck1DR2012b(m, iyStart, &b_emlrtECI, sp);
  }

  m = Dist->size[0];
  emxEnsureCapacity_real_T1(sp, Dist, m, &emlrtRTEI);
  ySize_idx_0 = Dist->size[0];
  for (m = 0; m < ySize_idx_0; m++) {
    Dist->data[m] += r0->data[m];
  }

  st.site = &d_emlrtRSI;
  b_st.site = &v_emlrtRSI;
  c_st.site = &w_emlrtRSI;
  d_st.site = &x_emlrtRSI;
  if ((Dist->size[0] == 1) || (Dist->size[0] != 1)) {
  } else {
    emlrtErrorWithMessageIdR2018a(&d_st, &w_emlrtRTEI,
      "Coder:toolbox:autoDimIncompatibility",
      "Coder:toolbox:autoDimIncompatibility", 0);
  }

  if (!(Dist->size[0] >= 1)) {
    emlrtErrorWithMessageIdR2018a(&d_st, &v_emlrtRTEI,
      "Coder:toolbox:eml_min_or_max_varDimZero",
      "Coder:toolbox:eml_min_or_max_varDimZero", 0);
  }

  e_st.site = &y_emlrtRSI;
  if (Dist->size[0] <= 2) {
    if (Dist->size[0] == 1) {
      iyStart = 1;
    } else if ((Dist->data[0] > Dist->data[1]) || (muDoubleScalarIsNaN
                (Dist->data[0]) && (!muDoubleScalarIsNaN(Dist->data[1])))) {
      iyStart = 2;
    } else {
      iyStart = 1;
    }
  } else {
    f_st.site = &bb_emlrtRSI;
    if (!muDoubleScalarIsNaN(Dist->data[0])) {
      iyStart = 1;
    } else {
      iyStart = 0;
      g_st.site = &cb_emlrtRSI;
      overflow = (Dist->size[0] > 2147483646);
      if (overflow) {
        h_st.site = &l_emlrtRSI;
        check_forloop_overflow_error(&h_st);
      }

      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= Dist->size[0])) {
        if (!muDoubleScalarIsNaN(Dist->data[k - 1])) {
          iyStart = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (iyStart == 0) {
      iyStart = 1;
    } else {
      f_st.site = &ab_emlrtRSI;
      tmp1 = Dist->data[iyStart - 1];
      g_st.site = &db_emlrtRSI;
      overflow = ((!(iyStart + 1 > Dist->size[0])) && (Dist->size[0] >
        2147483646));
      if (overflow) {
        h_st.site = &l_emlrtRSI;
        check_forloop_overflow_error(&h_st);
      }

      for (k = iyStart; k < Dist->size[0]; k++) {
        if (tmp1 > Dist->data[k]) {
          tmp1 = Dist->data[k];
          iyStart = k + 1;
        }
      }
    }
  }

  *ClosestIdx = iyStart;
  emxInit_real_T1(sp, &idxSorted2, 1, &e_emlrtRTEI, true);
  emxInit_real_T1(sp, &ApproxDist, 1, &g_emlrtRTEI, true);
  emxInit_int32_T(sp, &iidx, 1, &emlrtRTEI, true);
  if (PreviousClosestIndex > 0.0) {
    if (TrajectoryPoints->size[0] == 0) {
      ySize_idx_0 = 0;
    } else {
      ySize_idx_0 = muIntScalarMax_sint32(TrajectoryPoints->size[0], 2);
    }

    if (b_mod((real_T)iyStart - PreviousClosestIndex, ySize_idx_0) > 50.0) {
      /*  index jump detected */
      /*  Instead of just taking the minimum, which will lead to problems with */
      /*  overlapping trajectories, we find the minimum closest to the previous index */
      st.site = &e_emlrtRSI;
      b_st.site = &eb_emlrtRSI;
      sort(&b_st, Dist, iidx);
      m = ApproxDist->size[0];
      ApproxDist->size[0] = iidx->size[0];
      emxEnsureCapacity_real_T1(&st, ApproxDist, m, &emlrtRTEI);
      ySize_idx_0 = iidx->size[0];
      for (m = 0; m < ySize_idx_0; m++) {
        ApproxDist->data[m] = iidx->data[m];
      }

      m = Dist->size[0];
      if (!(1 <= m)) {
        emlrtDynamicBoundsCheckR2012b(1, 1, m, &f_emlrtBCI, sp);
      }

      iyStart = Dist->size[0] - 1;
      ySize_idx_0 = 0;
      for (k = 0; k <= iyStart; k++) {
        if (Dist->data[k] < 2.0 * Dist->data[0]) {
          ySize_idx_0++;
        }
      }

      m = idxSorted2->size[0];
      idxSorted2->size[0] = ySize_idx_0;
      emxEnsureCapacity_real_T1(sp, idxSorted2, m, &emlrtRTEI);
      ySize_idx_0 = 0;
      for (k = 0; k <= iyStart; k++) {
        if (Dist->data[k] < 2.0 * Dist->data[0]) {
          m = ApproxDist->size[0];
          if (!((k + 1 >= 1) && (k + 1 <= m))) {
            emlrtDynamicBoundsCheckR2012b(k + 1, 1, m, &g_emlrtBCI, sp);
          }

          idxSorted2->data[ySize_idx_0] = ApproxDist->data[k];
          ySize_idx_0++;
        }
      }

      st.site = &f_emlrtRSI;
      m = ApproxDist->size[0];
      ApproxDist->size[0] = idxSorted2->size[0];
      emxEnsureCapacity_real_T1(&st, ApproxDist, m, &emlrtRTEI);
      ySize_idx_0 = idxSorted2->size[0];
      for (m = 0; m < ySize_idx_0; m++) {
        ApproxDist->data[m] = idxSorted2->data[m] - PreviousClosestIndex;
      }

      b_st.site = &v_emlrtRSI;
      c_st.site = &w_emlrtRSI;
      d_st.site = &x_emlrtRSI;
      if ((ApproxDist->size[0] == 1) || (ApproxDist->size[0] != 1)) {
      } else {
        emlrtErrorWithMessageIdR2018a(&d_st, &w_emlrtRTEI,
          "Coder:toolbox:autoDimIncompatibility",
          "Coder:toolbox:autoDimIncompatibility", 0);
      }

      if (!(ApproxDist->size[0] >= 1)) {
        emlrtErrorWithMessageIdR2018a(&d_st, &v_emlrtRTEI,
          "Coder:toolbox:eml_min_or_max_varDimZero",
          "Coder:toolbox:eml_min_or_max_varDimZero", 0);
      }

      e_st.site = &y_emlrtRSI;
      if (ApproxDist->size[0] <= 2) {
        if (ApproxDist->size[0] == 1) {
          iyStart = 1;
        } else if (ApproxDist->data[0] > ApproxDist->data[1]) {
          iyStart = 2;
        } else {
          iyStart = 1;
        }
      } else {
        f_st.site = &bb_emlrtRSI;
        f_st.site = &ab_emlrtRSI;
        tmp1 = ApproxDist->data[0];
        iyStart = 1;
        g_st.site = &db_emlrtRSI;
        overflow = (ApproxDist->size[0] > 2147483646);
        if (overflow) {
          h_st.site = &l_emlrtRSI;
          check_forloop_overflow_error(&h_st);
        }

        for (k = 2; k <= ApproxDist->size[0]; k++) {
          if (tmp1 > ApproxDist->data[k - 1]) {
            tmp1 = ApproxDist->data[k - 1];
            iyStart = k;
          }
        }
      }

      m = idxSorted2->size[0];
      if (!((iyStart >= 1) && (iyStart <= m))) {
        emlrtDynamicBoundsCheckR2012b(iyStart, 1, m, &h_emlrtBCI, sp);
      }

      *ClosestIdx = idxSorted2->data[iyStart - 1];

      /* ClosestIdx = PreviousClosestIndex+10; % note that this jump value of 10 is dependent on the density of the trajectory */
      /* if (ClosestIdx > length(TrajectoryPoints)) */
      /*     ClosestIdx = 1;% + mod(ClosestIdx, length(TrajectoryPoints)); */
      /* end */
    }
  }

  emxFree_int32_T(sp, &iidx);
  emxFree_real_T(sp, &idxSorted2);
  emxFree_real_T(sp, &Dist);
  if ((int32_T)*ClosestIdx > RotatedCenteredTrajectory->size[0]) {
    m = 0;
    k = 0;
  } else {
    m = RotatedCenteredTrajectory->size[0];
    iyStart = (int32_T)*ClosestIdx;
    if (!((iyStart >= 1) && (iyStart <= m))) {
      emlrtDynamicBoundsCheckR2012b(iyStart, 1, m, &e_emlrtBCI, sp);
    }

    m = iyStart - 1;
    iyStart = RotatedCenteredTrajectory->size[0];
    k = RotatedCenteredTrajectory->size[0];
    if (!((k >= 1) && (k <= iyStart))) {
      emlrtDynamicBoundsCheckR2012b(k, 1, iyStart, &d_emlrtBCI, sp);
    }
  }

  if (1.0 > *ClosestIdx - 1.0) {
    ySize_idx_0 = -1;
  } else {
    iyStart = RotatedCenteredTrajectory->size[0];
    if (!(1 <= iyStart)) {
      emlrtDynamicBoundsCheckR2012b(1, 1, iyStart, &c_emlrtBCI, sp);
    }

    iyStart = RotatedCenteredTrajectory->size[0];
    ixLead = (int32_T)*ClosestIdx - 1;
    if (!((ixLead >= 1) && (ixLead <= iyStart))) {
      emlrtDynamicBoundsCheckR2012b(ixLead, 1, iyStart, &b_emlrtBCI, sp);
    }

    ySize_idx_0 = ixLead - 1;
  }

  emxInit_real_T(sp, &WindowTrajectory_tmp, 2, &f_emlrtRTEI, true);
  iyStart = WindowTrajectory_tmp->size[0] * WindowTrajectory_tmp->size[1];
  WindowTrajectory_tmp->size[0] = ((k - m) + ySize_idx_0) + 1;
  WindowTrajectory_tmp->size[1] = 2;
  emxEnsureCapacity_real_T(sp, WindowTrajectory_tmp, iyStart, &emlrtRTEI);
  iyLead = k - m;
  for (iyStart = 0; iyStart < 2; iyStart++) {
    for (ixLead = 0; ixLead < iyLead; ixLead++) {
      WindowTrajectory_tmp->data[ixLead + WindowTrajectory_tmp->size[0] *
        iyStart] = RotatedCenteredTrajectory->data[(m + ixLead) +
        RotatedCenteredTrajectory->size[0] * iyStart];
    }
  }

  for (iyStart = 0; iyStart < 2; iyStart++) {
    for (ixLead = 0; ixLead <= ySize_idx_0; ixLead++) {
      WindowTrajectory_tmp->data[((ixLead + k) - m) + WindowTrajectory_tmp->
        size[0] * iyStart] = RotatedCenteredTrajectory->data[ixLead +
        RotatedCenteredTrajectory->size[0] * iyStart];
    }
  }

  /* if (SeqJumpIdx1 > SeqJumpIdx0) */
  /* else */
  /*     WindowTrajectory = RotatedCenteredTrajectory(WindowIdxReordered(SeqJumpIdx1+1:SeqJumpIdx0),:); */
  /* end */
  if (ExtractDist > 0.0) {
    /*  only extract certain future distance of trajectory based on a crude distance approximation */
    st.site = &g_emlrtRSI;
    dimSize = WindowTrajectory_tmp->size[0];
    if (WindowTrajectory_tmp->size[0] == 0) {
      m = RotatedCenteredTrajectory->size[0] * RotatedCenteredTrajectory->size[1];
      RotatedCenteredTrajectory->size[0] = 0;
      RotatedCenteredTrajectory->size[1] = 2;
      emxEnsureCapacity_real_T(&st, RotatedCenteredTrajectory, m, &emlrtRTEI);
    } else {
      i0 = WindowTrajectory_tmp->size[0] - 1;
      if (muIntScalarMin_sint32(i0, 1) < 1) {
        m = RotatedCenteredTrajectory->size[0] * RotatedCenteredTrajectory->
          size[1];
        RotatedCenteredTrajectory->size[0] = 0;
        RotatedCenteredTrajectory->size[1] = 2;
        emxEnsureCapacity_real_T(&st, RotatedCenteredTrajectory, m, &emlrtRTEI);
      } else {
        overflow = (WindowTrajectory_tmp->size[0] != 1);
        if (!overflow) {
          emlrtErrorWithMessageIdR2018a(&st, &u_emlrtRTEI,
            "Coder:toolbox:autoDimIncompatibility",
            "Coder:toolbox:autoDimIncompatibility", 0);
        }

        ySize_idx_0 = WindowTrajectory_tmp->size[0] - 1;
        m = RotatedCenteredTrajectory->size[0] * RotatedCenteredTrajectory->
          size[1];
        RotatedCenteredTrajectory->size[0] = ySize_idx_0;
        RotatedCenteredTrajectory->size[1] = 2;
        emxEnsureCapacity_real_T(&st, RotatedCenteredTrajectory, m, &b_emlrtRTEI);
        if (!(RotatedCenteredTrajectory->size[0] == 0)) {
          ySize_idx_0 = 0;
          iyStart = 0;
          overflow = ((!(2 > dimSize)) && (dimSize > 2147483646));
          for (k = 0; k < 2; k++) {
            ixLead = ySize_idx_0 + 1;
            iyLead = iyStart;
            work_data_idx_0 = WindowTrajectory_tmp->data[ySize_idx_0];
            b_st.site = &gc_emlrtRSI;
            if (overflow) {
              c_st.site = &l_emlrtRSI;
              check_forloop_overflow_error(&c_st);
            }

            for (m = 2; m <= dimSize; m++) {
              tmp1 = WindowTrajectory_tmp->data[ixLead];
              b_st.site = &fc_emlrtRSI;
              tmp2 = work_data_idx_0;
              work_data_idx_0 = tmp1;
              tmp1 -= tmp2;
              ixLead++;
              RotatedCenteredTrajectory->data[iyLead] = tmp1;
              iyLead++;
            }

            ySize_idx_0 += dimSize;
            iyStart = (iyStart + dimSize) - 1;
          }
        }
      }
    }

    ySize_idx_0 = RotatedCenteredTrajectory->size[0];
    m = c_RotatedCenteredTrajectory->size[0];
    c_RotatedCenteredTrajectory->size[0] = ySize_idx_0;
    emxEnsureCapacity_real_T1(sp, c_RotatedCenteredTrajectory, m, &emlrtRTEI);
    for (m = 0; m < ySize_idx_0; m++) {
      c_RotatedCenteredTrajectory->data[m] = RotatedCenteredTrajectory->data[m];
    }

    st.site = &h_emlrtRSI;
    power(&st, c_RotatedCenteredTrajectory, ApproxDist);
    ySize_idx_0 = RotatedCenteredTrajectory->size[0];
    m = c_RotatedCenteredTrajectory->size[0];
    c_RotatedCenteredTrajectory->size[0] = ySize_idx_0;
    emxEnsureCapacity_real_T1(sp, c_RotatedCenteredTrajectory, m, &emlrtRTEI);
    for (m = 0; m < ySize_idx_0; m++) {
      c_RotatedCenteredTrajectory->data[m] = RotatedCenteredTrajectory->data[m +
        RotatedCenteredTrajectory->size[0]];
    }

    st.site = &h_emlrtRSI;
    power(&st, c_RotatedCenteredTrajectory, r0);
    m = ApproxDist->size[0];
    iyStart = r0->size[0];
    if (m != iyStart) {
      emlrtSizeEqCheck1DR2012b(m, iyStart, &emlrtECI, sp);
    }

    st.site = &h_emlrtRSI;
    m = ApproxDist->size[0];
    emxEnsureCapacity_real_T1(&st, ApproxDist, m, &emlrtRTEI);
    ySize_idx_0 = ApproxDist->size[0];
    for (m = 0; m < ySize_idx_0; m++) {
      ApproxDist->data[m] += r0->data[m];
    }

    overflow = false;
    for (k = 0; k < ApproxDist->size[0]; k++) {
      if (overflow || (ApproxDist->data[k] < 0.0)) {
        overflow = true;
      } else {
        overflow = false;
      }
    }

    if (overflow) {
      b_st.site = &hc_emlrtRSI;
      error(&b_st);
    }

    b_st.site = &ic_emlrtRSI;
    ySize_idx_0 = ApproxDist->size[0];
    c_st.site = &jc_emlrtRSI;
    overflow = ((!(1 > ApproxDist->size[0])) && (ApproxDist->size[0] >
      2147483646));
    if (overflow) {
      d_st.site = &l_emlrtRSI;
      check_forloop_overflow_error(&d_st);
    }

    for (k = 0; k < ySize_idx_0; k++) {
      ApproxDist->data[k] = muDoubleScalarSqrt(ApproxDist->data[k]);
    }

    st.site = &h_emlrtRSI;
    b_st.site = &kc_emlrtRSI;
    ySize_idx_0 = 2;
    if (ApproxDist->size[0] != 1) {
      ySize_idx_0 = 1;
    }

    c_st.site = &lc_emlrtRSI;
    if (1 == ySize_idx_0) {
      d_st.site = &mc_emlrtRSI;
      e_st.site = &nc_emlrtRSI;
      if (ApproxDist->size[0] != 0) {
        f_st.site = &oc_emlrtRSI;
        ySize_idx_0 = ApproxDist->size[0];
        if (ApproxDist->size[0] != 1) {
          g_st.site = &pc_emlrtRSI;
          for (k = 1; k < ySize_idx_0; k++) {
            ApproxDist->data[k] += ApproxDist->data[k - 1];
          }
        }
      }
    }

    /* ExtractDist = N*ts*velocity; */
    iyStart = ApproxDist->size[0];
    for (k = 0; k < iyStart; k++) {
      if (ApproxDist->data[k] < ExtractDist) {
        m = WindowTrajectory_tmp->size[0];
        if (!((k + 1 >= 1) && (k + 1 <= m))) {
          emlrtDynamicBoundsCheckR2012b(k + 1, 1, m, &emlrtBCI, sp);
        }
      }
    }

    iyStart = ApproxDist->size[0] - 1;
    ySize_idx_0 = 0;
    for (k = 0; k <= iyStart; k++) {
      if (ApproxDist->data[k] < ExtractDist) {
        ySize_idx_0++;
      }
    }

    emxInit_int32_T(sp, &r1, 1, &emlrtRTEI, true);
    m = r1->size[0];
    r1->size[0] = ySize_idx_0;
    emxEnsureCapacity_int32_T(sp, r1, m, &emlrtRTEI);
    ySize_idx_0 = 0;
    for (k = 0; k <= iyStart; k++) {
      if (ApproxDist->data[k] < ExtractDist) {
        r1->data[ySize_idx_0] = k + 1;
        ySize_idx_0++;
      }
    }

    m = WindowTrajectory->size[0] * WindowTrajectory->size[1];
    WindowTrajectory->size[0] = r1->size[0];
    WindowTrajectory->size[1] = 2;
    emxEnsureCapacity_real_T(sp, WindowTrajectory, m, &emlrtRTEI);
    for (m = 0; m < 2; m++) {
      ySize_idx_0 = r1->size[0];
      for (iyStart = 0; iyStart < ySize_idx_0; iyStart++) {
        WindowTrajectory->data[iyStart + WindowTrajectory->size[0] * m] =
          WindowTrajectory_tmp->data[(r1->data[iyStart] +
          WindowTrajectory_tmp->size[0] * m) - 1];
      }
    }

    emxFree_int32_T(sp, &r1);
  } else {
    m = WindowTrajectory->size[0] * WindowTrajectory->size[1];
    WindowTrajectory->size[0] = WindowTrajectory_tmp->size[0];
    WindowTrajectory->size[1] = 2;
    emxEnsureCapacity_real_T(sp, WindowTrajectory, m, &emlrtRTEI);
    ySize_idx_0 = WindowTrajectory_tmp->size[0] * WindowTrajectory_tmp->size[1];
    for (m = 0; m < ySize_idx_0; m++) {
      WindowTrajectory->data[m] = WindowTrajectory_tmp->data[m];
    }
  }

  emxFree_real_T(sp, &c_RotatedCenteredTrajectory);
  emxFree_real_T(sp, &r0);
  emxFree_real_T(sp, &ApproxDist);
  emxFree_real_T(sp, &WindowTrajectory_tmp);
  emxFree_real_T(sp, &RotatedCenteredTrajectory);
  if (WindowTrajectory->size[0] == 0) {
    ySize_idx_0 = 0;
  } else {
    ySize_idx_0 = muIntScalarMax_sint32(WindowTrajectory->size[0], 2);
  }

  /*      splitIdx = find(diff(WindowIdx) > 1); % correct if/when the trajectory start and end is in the window, which messes up the order due to indexing (recommended to make trajectory object with sequence id) */
  /*      if (~isempty(splitIdx))  % correct (by reordering) trajectory index list */
  /*          splitIdx = splitIdx(1); */
  /*          WindowIdx = [WindowIdx(splitIdx+1:end); WindowIdx(1:splitIdx)]; */
  /*      end */
  /*      WindowTrajectory = RotatedCenteredTrajectory(WindowIdx,:);        */
  /* SquaredDistanceToPointsInWindow = RotatedCenteredTrajectory(:,1).^2 + RotatedCenteredTrajectory(:,2).^2; */
  /* [y, ClosestPointWithinWindowIdx] = min(SquaredDistanceToPointsInWindow);     */
  /*  Select continuous series of points which includes this closest point */
  /* WindowTrajectory = RotatedCenteredTrajectory(ClosestPointWithinWindowIdx:end);                */
  *nTrajPoints = ySize_idx_0;
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (ExtractDistanceTrajectory.c) */
