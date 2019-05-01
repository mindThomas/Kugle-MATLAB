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

static emlrtRSInfo e_emlrtRSI = { 36,  /* lineNo */
  "ExtractDistanceTrajectory",         /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pathName */
};

static emlrtRSInfo f_emlrtRSI = { 37,  /* lineNo */
  "ExtractDistanceTrajectory",         /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pathName */
};

static emlrtRSInfo t_emlrtRSI = { 15,  /* lineNo */
  "min",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\min.m"/* pathName */
};

static emlrtRSInfo u_emlrtRSI = { 16,  /* lineNo */
  "minOrMax",                          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

static emlrtRSInfo v_emlrtRSI = { 38,  /* lineNo */
  "minOrMax",                          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

static emlrtRSInfo w_emlrtRSI = { 112, /* lineNo */
  "unaryMinOrMax",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo x_emlrtRSI = { 852, /* lineNo */
  "unaryMinOrMax",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo y_emlrtRSI = { 844, /* lineNo */
  "unaryMinOrMax",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo ab_emlrtRSI = { 894,/* lineNo */
  "unaryMinOrMax",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo bb_emlrtRSI = { 910,/* lineNo */
  "unaryMinOrMax",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo cb_emlrtRSI = { 108,/* lineNo */
  "diff",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\diff.m"/* pathName */
};

static emlrtRSInfo db_emlrtRSI = { 106,/* lineNo */
  "diff",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\diff.m"/* pathName */
};

static emlrtRSInfo eb_emlrtRSI = { 12, /* lineNo */
  "sqrt",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elfun\\sqrt.m"/* pathName */
};

static emlrtRSInfo fb_emlrtRSI = { 15, /* lineNo */
  "sqrt",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elfun\\sqrt.m"/* pathName */
};

static emlrtRSInfo gb_emlrtRSI = { 31, /* lineNo */
  "applyScalarFunctionInPlace",        /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\applyScalarFunctionInPlace.m"/* pathName */
};

static emlrtRSInfo hb_emlrtRSI = { 14, /* lineNo */
  "cumsum",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\cumsum.m"/* pathName */
};

static emlrtRSInfo ib_emlrtRSI = { 11, /* lineNo */
  "cumop",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\private\\cumop.m"/* pathName */
};

static emlrtRSInfo jb_emlrtRSI = { 32, /* lineNo */
  "useConstantDim",                    /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\useConstantDim.m"/* pathName */
};

static emlrtRSInfo kb_emlrtRSI = { 93, /* lineNo */
  "cumop",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\private\\cumop.m"/* pathName */
};

static emlrtRSInfo lb_emlrtRSI = { 119,/* lineNo */
  "cumop",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\private\\cumop.m"/* pathName */
};

static emlrtRSInfo mb_emlrtRSI = { 286,/* lineNo */
  "cumop",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\private\\cumop.m"/* pathName */
};

static emlrtRTEInfo emlrtRTEI = { 1,   /* lineNo */
  63,                                  /* colNo */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pName */
};

static emlrtRTEInfo b_emlrtRTEI = { 36,/* lineNo */
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

static emlrtRTEInfo e_emlrtRTEI = { 27,/* lineNo */
  5,                                   /* colNo */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pName */
};

static emlrtRTEInfo n_emlrtRTEI = { 51,/* lineNo */
  19,                                  /* colNo */
  "diff",                              /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\diff.m"/* pName */
};

static emlrtRTEInfo o_emlrtRTEI = { 77,/* lineNo */
  27,                                  /* colNo */
  "unaryMinOrMax",                     /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pName */
};

static emlrtRTEInfo p_emlrtRTEI = { 22,/* lineNo */
  27,                                  /* colNo */
  "unaryMinOrMax",                     /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pName */
};

static emlrtBCInfo emlrtBCI = { -1,    /* iFirst */
  -1,                                  /* iLast */
  39,                                  /* lineNo */
  59,                                  /* colNo */
  "WindowTrajectory_tmp",              /* aName */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo emlrtECI = { -1,    /* nDims */
  37,                                  /* lineNo */
  34,                                  /* colNo */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m"/* pName */
};

static emlrtBCInfo b_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  27,                                  /* lineNo */
  102,                                 /* colNo */
  "RotatedCenteredTrajectory",         /* aName */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  27,                                  /* lineNo */
  100,                                 /* colNo */
  "RotatedCenteredTrajectory",         /* aName */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  27,                                  /* lineNo */
  66,                                  /* colNo */
  "RotatedCenteredTrajectory",         /* aName */
  "ExtractDistanceTrajectory",         /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractDistanceTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo e_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  27,                                  /* lineNo */
  55,                                  /* colNo */
  "RotatedCenteredTrajectory",         /* aName */
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

/* Function Definitions */
void ExtractDistanceTrajectory(const emlrtStack *sp, const emxArray_real_T
  *TrajectoryPoints, const real_T RobotPos[2], real_T RobotYaw, const real_T
  Velocity[2], real_T ExtractDist, real_T OrientationSelection, emxArray_real_T *
  WindowTrajectory, real_T *nTrajPoints, real_T *WindowOrientation)
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
  int32_T ixLead;
  boolean_T overflow;
  real_T tmp1;
  int32_T iyStart;
  boolean_T exitg1;
  int32_T iyLead;
  emxArray_real_T *WindowTrajectory_tmp;
  int32_T loop_ub;
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
  ixLead = r0->size[0];
  if (m != ixLead) {
    emlrtSizeEqCheck1DR2012b(m, ixLead, &b_emlrtECI, sp);
  }

  m = Dist->size[0];
  emxEnsureCapacity_real_T1(sp, Dist, m, &emlrtRTEI);
  ySize_idx_0 = Dist->size[0];
  for (m = 0; m < ySize_idx_0; m++) {
    Dist->data[m] += r0->data[m];
  }

  st.site = &d_emlrtRSI;
  b_st.site = &t_emlrtRSI;
  c_st.site = &u_emlrtRSI;
  d_st.site = &v_emlrtRSI;
  if ((Dist->size[0] == 1) || (Dist->size[0] != 1)) {
  } else {
    emlrtErrorWithMessageIdR2018a(&d_st, &p_emlrtRTEI,
      "Coder:toolbox:autoDimIncompatibility",
      "Coder:toolbox:autoDimIncompatibility", 0);
  }

  if (!(Dist->size[0] >= 1)) {
    emlrtErrorWithMessageIdR2018a(&d_st, &o_emlrtRTEI,
      "Coder:toolbox:eml_min_or_max_varDimZero",
      "Coder:toolbox:eml_min_or_max_varDimZero", 0);
  }

  e_st.site = &w_emlrtRSI;
  if (Dist->size[0] <= 2) {
    if (Dist->size[0] == 1) {
      ySize_idx_0 = 1;
    } else if ((Dist->data[0] > Dist->data[1]) || (muDoubleScalarIsNaN
                (Dist->data[0]) && (!muDoubleScalarIsNaN(Dist->data[1])))) {
      ySize_idx_0 = 2;
    } else {
      ySize_idx_0 = 1;
    }
  } else {
    f_st.site = &y_emlrtRSI;
    if (!muDoubleScalarIsNaN(Dist->data[0])) {
      ySize_idx_0 = 1;
    } else {
      ySize_idx_0 = 0;
      g_st.site = &ab_emlrtRSI;
      overflow = (Dist->size[0] > 2147483646);
      if (overflow) {
        h_st.site = &j_emlrtRSI;
        check_forloop_overflow_error(&h_st);
      }

      iyStart = 2;
      exitg1 = false;
      while ((!exitg1) && (iyStart <= Dist->size[0])) {
        if (!muDoubleScalarIsNaN(Dist->data[iyStart - 1])) {
          ySize_idx_0 = iyStart;
          exitg1 = true;
        } else {
          iyStart++;
        }
      }
    }

    if (ySize_idx_0 == 0) {
      ySize_idx_0 = 1;
    } else {
      f_st.site = &x_emlrtRSI;
      tmp1 = Dist->data[ySize_idx_0 - 1];
      g_st.site = &bb_emlrtRSI;
      overflow = ((!(ySize_idx_0 + 1 > Dist->size[0])) && (Dist->size[0] >
        2147483646));
      if (overflow) {
        h_st.site = &j_emlrtRSI;
        check_forloop_overflow_error(&h_st);
      }

      for (iyStart = ySize_idx_0; iyStart < Dist->size[0]; iyStart++) {
        if (tmp1 > Dist->data[iyStart]) {
          tmp1 = Dist->data[iyStart];
          ySize_idx_0 = iyStart + 1;
        }
      }
    }
  }

  if (ySize_idx_0 > RotatedCenteredTrajectory->size[0]) {
    m = 0;
    iyLead = 0;
  } else {
    m = RotatedCenteredTrajectory->size[0];
    if (!((ySize_idx_0 >= 1) && (ySize_idx_0 <= m))) {
      emlrtDynamicBoundsCheckR2012b(ySize_idx_0, 1, m, &e_emlrtBCI, sp);
    }

    m = ySize_idx_0 - 1;
    ixLead = RotatedCenteredTrajectory->size[0];
    iyLead = RotatedCenteredTrajectory->size[0];
    if (!((iyLead >= 1) && (iyLead <= ixLead))) {
      emlrtDynamicBoundsCheckR2012b(iyLead, 1, ixLead, &d_emlrtBCI, sp);
    }
  }

  if (1.0 > (real_T)ySize_idx_0 - 1.0) {
    ySize_idx_0 = -1;
  } else {
    ixLead = RotatedCenteredTrajectory->size[0];
    if (!(1 <= ixLead)) {
      emlrtDynamicBoundsCheckR2012b(1, 1, ixLead, &c_emlrtBCI, sp);
    }

    ixLead = RotatedCenteredTrajectory->size[0];
    iyStart = (int32_T)((real_T)ySize_idx_0 - 1.0);
    if (!((iyStart >= 1) && (iyStart <= ixLead))) {
      emlrtDynamicBoundsCheckR2012b(iyStart, 1, ixLead, &b_emlrtBCI, sp);
    }

    ySize_idx_0 = iyStart - 1;
  }

  emxInit_real_T(sp, &WindowTrajectory_tmp, 2, &e_emlrtRTEI, true);
  ixLead = WindowTrajectory_tmp->size[0] * WindowTrajectory_tmp->size[1];
  WindowTrajectory_tmp->size[0] = ((iyLead - m) + ySize_idx_0) + 1;
  WindowTrajectory_tmp->size[1] = 2;
  emxEnsureCapacity_real_T(sp, WindowTrajectory_tmp, ixLead, &emlrtRTEI);
  loop_ub = iyLead - m;
  for (ixLead = 0; ixLead < 2; ixLead++) {
    for (iyStart = 0; iyStart < loop_ub; iyStart++) {
      WindowTrajectory_tmp->data[iyStart + WindowTrajectory_tmp->size[0] *
        ixLead] = RotatedCenteredTrajectory->data[(m + iyStart) +
        RotatedCenteredTrajectory->size[0] * ixLead];
    }
  }

  for (ixLead = 0; ixLead < 2; ixLead++) {
    for (iyStart = 0; iyStart <= ySize_idx_0; iyStart++) {
      WindowTrajectory_tmp->data[((iyStart + iyLead) - m) +
        WindowTrajectory_tmp->size[0] * ixLead] =
        RotatedCenteredTrajectory->data[iyStart +
        RotatedCenteredTrajectory->size[0] * ixLead];
    }
  }

  /* if (SeqJumpIdx1 > SeqJumpIdx0) */
  /* else */
  /*     WindowTrajectory = RotatedCenteredTrajectory(WindowIdxReordered(SeqJumpIdx1+1:SeqJumpIdx0),:); */
  /* end */
  if (ExtractDist > 0.0) {
    /*  only extract certain future distance of trajectory based on a crude distance approximation */
    st.site = &e_emlrtRSI;
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
          emlrtErrorWithMessageIdR2018a(&st, &n_emlrtRTEI,
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
          for (loop_ub = 0; loop_ub < 2; loop_ub++) {
            ixLead = ySize_idx_0 + 1;
            iyLead = iyStart;
            work_data_idx_0 = WindowTrajectory_tmp->data[ySize_idx_0];
            b_st.site = &db_emlrtRSI;
            if (overflow) {
              c_st.site = &j_emlrtRSI;
              check_forloop_overflow_error(&c_st);
            }

            for (m = 2; m <= dimSize; m++) {
              tmp1 = WindowTrajectory_tmp->data[ixLead];
              b_st.site = &cb_emlrtRSI;
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

    st.site = &f_emlrtRSI;
    power(&st, c_RotatedCenteredTrajectory, Dist);
    ySize_idx_0 = RotatedCenteredTrajectory->size[0];
    m = c_RotatedCenteredTrajectory->size[0];
    c_RotatedCenteredTrajectory->size[0] = ySize_idx_0;
    emxEnsureCapacity_real_T1(sp, c_RotatedCenteredTrajectory, m, &emlrtRTEI);
    for (m = 0; m < ySize_idx_0; m++) {
      c_RotatedCenteredTrajectory->data[m] = RotatedCenteredTrajectory->data[m +
        RotatedCenteredTrajectory->size[0]];
    }

    st.site = &f_emlrtRSI;
    power(&st, c_RotatedCenteredTrajectory, r0);
    m = Dist->size[0];
    ixLead = r0->size[0];
    if (m != ixLead) {
      emlrtSizeEqCheck1DR2012b(m, ixLead, &emlrtECI, sp);
    }

    st.site = &f_emlrtRSI;
    m = Dist->size[0];
    emxEnsureCapacity_real_T1(&st, Dist, m, &emlrtRTEI);
    ySize_idx_0 = Dist->size[0];
    for (m = 0; m < ySize_idx_0; m++) {
      Dist->data[m] += r0->data[m];
    }

    overflow = false;
    for (iyStart = 0; iyStart < Dist->size[0]; iyStart++) {
      if (overflow || (Dist->data[iyStart] < 0.0)) {
        overflow = true;
      } else {
        overflow = false;
      }
    }

    if (overflow) {
      b_st.site = &eb_emlrtRSI;
      error(&b_st);
    }

    b_st.site = &fb_emlrtRSI;
    ySize_idx_0 = Dist->size[0];
    c_st.site = &gb_emlrtRSI;
    overflow = ((!(1 > Dist->size[0])) && (Dist->size[0] > 2147483646));
    if (overflow) {
      d_st.site = &j_emlrtRSI;
      check_forloop_overflow_error(&d_st);
    }

    for (iyStart = 0; iyStart < ySize_idx_0; iyStart++) {
      Dist->data[iyStart] = muDoubleScalarSqrt(Dist->data[iyStart]);
    }

    st.site = &f_emlrtRSI;
    b_st.site = &hb_emlrtRSI;
    ySize_idx_0 = 2;
    if (Dist->size[0] != 1) {
      ySize_idx_0 = 1;
    }

    c_st.site = &ib_emlrtRSI;
    if (1 == ySize_idx_0) {
      d_st.site = &jb_emlrtRSI;
      e_st.site = &kb_emlrtRSI;
      if (Dist->size[0] != 0) {
        f_st.site = &lb_emlrtRSI;
        ySize_idx_0 = Dist->size[0];
        if (Dist->size[0] != 1) {
          g_st.site = &mb_emlrtRSI;
          for (iyStart = 1; iyStart < ySize_idx_0; iyStart++) {
            Dist->data[iyStart] += Dist->data[iyStart - 1];
          }
        }
      }
    }

    /* ExtractDist = N*ts*velocity; */
    iyStart = Dist->size[0];
    for (loop_ub = 0; loop_ub < iyStart; loop_ub++) {
      if (Dist->data[loop_ub] < ExtractDist) {
        m = WindowTrajectory_tmp->size[0];
        if (!((loop_ub + 1 >= 1) && (loop_ub + 1 <= m))) {
          emlrtDynamicBoundsCheckR2012b(loop_ub + 1, 1, m, &emlrtBCI, sp);
        }
      }
    }

    iyStart = Dist->size[0] - 1;
    ySize_idx_0 = 0;
    for (loop_ub = 0; loop_ub <= iyStart; loop_ub++) {
      if (Dist->data[loop_ub] < ExtractDist) {
        ySize_idx_0++;
      }
    }

    emxInit_int32_T(sp, &r1, 1, &emlrtRTEI, true);
    m = r1->size[0];
    r1->size[0] = ySize_idx_0;
    emxEnsureCapacity_int32_T(sp, r1, m, &emlrtRTEI);
    ySize_idx_0 = 0;
    for (loop_ub = 0; loop_ub <= iyStart; loop_ub++) {
      if (Dist->data[loop_ub] < ExtractDist) {
        r1->data[ySize_idx_0] = loop_ub + 1;
        ySize_idx_0++;
      }
    }

    m = WindowTrajectory->size[0] * WindowTrajectory->size[1];
    WindowTrajectory->size[0] = r1->size[0];
    WindowTrajectory->size[1] = 2;
    emxEnsureCapacity_real_T(sp, WindowTrajectory, m, &emlrtRTEI);
    for (m = 0; m < 2; m++) {
      ySize_idx_0 = r1->size[0];
      for (ixLead = 0; ixLead < ySize_idx_0; ixLead++) {
        WindowTrajectory->data[ixLead + WindowTrajectory->size[0] * m] =
          WindowTrajectory_tmp->data[(r1->data[ixLead] +
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
  emxFree_real_T(sp, &WindowTrajectory_tmp);
  emxFree_real_T(sp, &Dist);
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
  /* WindowTrajectory = RotatedCenteredTrajectory(ClosestPointWithinWindowIdx:end);     */
  /*  figure(1); */
  /*  clf; */
  /*  ax1 = axes; */
  /*  plot(ax1, TrajectoryPoints(:,1), TrajectoryPoints(:,2), 'k-', 'MarkerSize', 10); */
  /*  hold(ax1,'on');    */
  /*  InertialWindowTrajectory = RotateTrajectory(WindowTrajectory, -WindowOrientation) + repmat([RobotPos(1),RobotPos(2)], [length(WindowTrajectory),1]); */
  /*  plot(InertialWindowTrajectory(:,1), InertialWindowTrajectory(:,2), 'k*', 'MarkerSize', 2); */
  /*  PlotAxRobotWithTiltAndVelocity(ax1, [RobotPos(1),RobotPos(2)], WindowOrientation, 0.05, [Velocity(1),Velocity(2)], [0,0]); */
  /*  %plot(InertialReferencePoints(:,1), InertialReferencePoints(:,2), 'g*', 'MarkerSize', 3); */
  /*  %plot(InertialMPCtrajectory(:,1), InertialMPCtrajectory(:,2), 'r*', 'MarkerSize', 3);   */
  /*  WindowCornersCentered = [-WindowHeight/2, -WindowWidth/2 */
  /*                            WindowHeight/2, -WindowWidth/2 */
  /*                            WindowHeight/2, WindowWidth/2 */
  /*                            -WindowHeight/2, WindowWidth/2 */
  /*                            -WindowHeight/2, -WindowWidth/2] + WindowOffset;                      */
  /*  WindowCorners = RotateTrajectory(WindowCornersCentered, -WindowOrientation) + repmat([RobotPos(1), RobotPos(2)], [5,1]); */
  /*  plot(WindowCorners(:,1), WindowCorners(:,2), 'k--');  */
  /*  hold('off');    */
  /*  axis equal; */
  /*  xlim([min(TrajectoryPoints(:,1))*2, max(TrajectoryPoints(:,1))*2]); */
  /*  ylim([min(TrajectoryPoints(:,2))*2, max(TrajectoryPoints(:,2))*2]); */
  /*   */
  /*  figure(2); */
  /*  clf; */
  /*  ax2 = axes; */
  /*  plot(ax2, -WindowTrajectory(:,2), WindowTrajectory(:,1), 'k*'); % plot rotated */
  /*  hold(ax2, 'on'); */
  /*  Vel = R_orientation * [Velocity(1);Velocity(2)];        */
  /*  PlotAxRobotWithTiltAndVelocity(ax2, [0,0], deg2rad(90)+0, 0.05, [-Vel(2),Vel(1)], [0,0]);     */
  /*  % if (trajectoryLength > 0) */
  /*  %     s = (0:0.01:trajectoryLength)'; */
  /*  %     trajectory_x = EvaluatePolynomial(coeff_trajectory_x, s); */
  /*  %     trajectory_y = EvaluatePolynomial(coeff_trajectory_y, s); */
  /*  %     plot(ax2, -trajectory_y, trajectory_x, 'b-'); */
  /*  % end     */
  /*  %plot(ax2, -ReferencePoints(:,2), ReferencePoints(:,1), 'g*'); */
  /*  %plot(ax2, -MPCtrajectory(:,2), MPCtrajectory(:,1), 'r*');   */
  /*  hold(ax2, 'off'); */
  /*  axis(ax2, 'equal'); */
  /*  xlim(ax2, [-WindowWidth/2 + WindowOffset(2), WindowWidth/2 + WindowOffset(2)]); */
  /*  ylim(ax2, [-WindowHeight/2 + WindowOffset(1), WindowHeight/2 + WindowOffset(1)]); */
  *nTrajPoints = ySize_idx_0;
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (ExtractDistanceTrajectory.c) */
