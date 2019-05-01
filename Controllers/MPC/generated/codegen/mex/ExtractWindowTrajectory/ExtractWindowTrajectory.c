/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ExtractWindowTrajectory.c
 *
 * Code generation for function 'ExtractWindowTrajectory'
 *
 */

/* Include files */
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "ExtractWindowTrajectory.h"
#include "ExtractWindowTrajectory_emxutil.h"
#include "eml_int_forloop_overflow_check.h"
#include "error.h"
#include "power.h"
#include "diff.h"
#include "indexShapeCheck.h"
#include "ifWhileCond.h"
#include "sort1.h"
#include "abs.h"
#include "repmat.h"
#include "RotateTrajectory.h"
#include "ExtractWindowTrajectory_data.h"

/* Variable Definitions */
static emlrtRSInfo emlrtRSI = { 20,    /* lineNo */
  "ExtractWindowTrajectory",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pathName */
};

static emlrtRSInfo b_emlrtRSI = { 21,  /* lineNo */
  "ExtractWindowTrajectory",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pathName */
};

static emlrtRSInfo c_emlrtRSI = { 24,  /* lineNo */
  "ExtractWindowTrajectory",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pathName */
};

static emlrtRSInfo d_emlrtRSI = { 29,  /* lineNo */
  "ExtractWindowTrajectory",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pathName */
};

static emlrtRSInfo e_emlrtRSI = { 30,  /* lineNo */
  "ExtractWindowTrajectory",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pathName */
};

static emlrtRSInfo f_emlrtRSI = { 31,  /* lineNo */
  "ExtractWindowTrajectory",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pathName */
};

static emlrtRSInfo g_emlrtRSI = { 34,  /* lineNo */
  "ExtractWindowTrajectory",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pathName */
};

static emlrtRSInfo h_emlrtRSI = { 39,  /* lineNo */
  "ExtractWindowTrajectory",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pathName */
};

static emlrtRSInfo i_emlrtRSI = { 44,  /* lineNo */
  "ExtractWindowTrajectory",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pathName */
};

static emlrtRSInfo j_emlrtRSI = { 45,  /* lineNo */
  "ExtractWindowTrajectory",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pathName */
};

static emlrtRSInfo k_emlrtRSI = { 52,  /* lineNo */
  "ExtractWindowTrajectory",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pathName */
};

static emlrtRSInfo l_emlrtRSI = { 53,  /* lineNo */
  "ExtractWindowTrajectory",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pathName */
};

static emlrtRSInfo m_emlrtRSI = { 54,  /* lineNo */
  "ExtractWindowTrajectory",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pathName */
};

static emlrtRSInfo n_emlrtRSI = { 56,  /* lineNo */
  "ExtractWindowTrajectory",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pathName */
};

static emlrtRSInfo o_emlrtRSI = { 57,  /* lineNo */
  "ExtractWindowTrajectory",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pathName */
};

static emlrtRSInfo p_emlrtRSI = { 59,  /* lineNo */
  "ExtractWindowTrajectory",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pathName */
};

static emlrtRSInfo q_emlrtRSI = { 68,  /* lineNo */
  "ExtractWindowTrajectory",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pathName */
};

static emlrtRSInfo r_emlrtRSI = { 81,  /* lineNo */
  "ExtractWindowTrajectory",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pathName */
};

static emlrtRSInfo s_emlrtRSI = { 87,  /* lineNo */
  "ExtractWindowTrajectory",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pathName */
};

static emlrtRSInfo t_emlrtRSI = { 88,  /* lineNo */
  "ExtractWindowTrajectory",           /* fcnName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pathName */
};

static emlrtRSInfo db_emlrtRSI = { 41, /* lineNo */
  "find",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elmat\\find.m"/* pathName */
};

static emlrtRSInfo eb_emlrtRSI = { 153,/* lineNo */
  "find",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elmat\\find.m"/* pathName */
};

static emlrtRSInfo fb_emlrtRSI = { 377,/* lineNo */
  "find",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elmat\\find.m"/* pathName */
};

static emlrtRSInfo gb_emlrtRSI = { 397,/* lineNo */
  "find",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elmat\\find.m"/* pathName */
};

static emlrtRSInfo nb_emlrtRSI = { 15, /* lineNo */
  "min",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\min.m"/* pathName */
};

static emlrtRSInfo ob_emlrtRSI = { 16, /* lineNo */
  "minOrMax",                          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

static emlrtRSInfo pb_emlrtRSI = { 38, /* lineNo */
  "minOrMax",                          /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\minOrMax.m"/* pathName */
};

static emlrtRSInfo qb_emlrtRSI = { 112,/* lineNo */
  "unaryMinOrMax",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo rb_emlrtRSI = { 852,/* lineNo */
  "unaryMinOrMax",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo sb_emlrtRSI = { 844,/* lineNo */
  "unaryMinOrMax",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo tb_emlrtRSI = { 894,/* lineNo */
  "unaryMinOrMax",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo ub_emlrtRSI = { 910,/* lineNo */
  "unaryMinOrMax",                     /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pathName */
};

static emlrtRSInfo bc_emlrtRSI = { 28, /* lineNo */
  "sort",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\sort.m"/* pathName */
};

static emlrtRSInfo ed_emlrtRSI = { 12, /* lineNo */
  "sqrt",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elfun\\sqrt.m"/* pathName */
};

static emlrtRSInfo fd_emlrtRSI = { 15, /* lineNo */
  "sqrt",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elfun\\sqrt.m"/* pathName */
};

static emlrtRSInfo gd_emlrtRSI = { 31, /* lineNo */
  "applyScalarFunctionInPlace",        /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\applyScalarFunctionInPlace.m"/* pathName */
};

static emlrtRSInfo hd_emlrtRSI = { 14, /* lineNo */
  "cumsum",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\cumsum.m"/* pathName */
};

static emlrtRSInfo id_emlrtRSI = { 11, /* lineNo */
  "cumop",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\private\\cumop.m"/* pathName */
};

static emlrtRSInfo jd_emlrtRSI = { 32, /* lineNo */
  "useConstantDim",                    /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\useConstantDim.m"/* pathName */
};

static emlrtRSInfo kd_emlrtRSI = { 93, /* lineNo */
  "cumop",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\private\\cumop.m"/* pathName */
};

static emlrtRSInfo ld_emlrtRSI = { 119,/* lineNo */
  "cumop",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\private\\cumop.m"/* pathName */
};

static emlrtRSInfo md_emlrtRSI = { 286,/* lineNo */
  "cumop",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\private\\cumop.m"/* pathName */
};

static emlrtRTEInfo emlrtRTEI = { 1,   /* lineNo */
  63,                                  /* colNo */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pName */
};

static emlrtRTEInfo b_emlrtRTEI = { 153,/* lineNo */
  13,                                  /* colNo */
  "find",                              /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elmat\\find.m"/* pName */
};

static emlrtRTEInfo c_emlrtRTEI = { 397,/* lineNo */
  5,                                   /* colNo */
  "find",                              /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elmat\\find.m"/* pName */
};

static emlrtRTEInfo d_emlrtRTEI = { 21,/* lineNo */
  5,                                   /* colNo */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pName */
};

static emlrtRTEInfo e_emlrtRTEI = { 24,/* lineNo */
  5,                                   /* colNo */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pName */
};

static emlrtRTEInfo f_emlrtRTEI = { 29,/* lineNo */
  5,                                   /* colNo */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pName */
};

static emlrtRTEInfo g_emlrtRTEI = { 30,/* lineNo */
  5,                                   /* colNo */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pName */
};

static emlrtRTEInfo h_emlrtRTEI = { 31,/* lineNo */
  5,                                   /* colNo */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pName */
};

static emlrtRTEInfo i_emlrtRTEI = { 44,/* lineNo */
  9,                                   /* colNo */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pName */
};

static emlrtRTEInfo j_emlrtRTEI = { 52,/* lineNo */
  5,                                   /* colNo */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pName */
};

static emlrtRTEInfo k_emlrtRTEI = { 81,/* lineNo */
  9,                                   /* colNo */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pName */
};

static emlrtRTEInfo l_emlrtRTEI = { 33,/* lineNo */
  6,                                   /* colNo */
  "find",                              /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elmat\\find.m"/* pName */
};

static emlrtECInfo emlrtECI = { 2,     /* nDims */
  20,                                  /* lineNo */
  36,                                  /* colNo */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pName */
};

static emlrtECInfo b_emlrtECI = { -1,  /* nDims */
  24,                                  /* lineNo */
  22,                                  /* colNo */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pName */
};

static emlrtECInfo c_emlrtECI = { -1,  /* nDims */
  34,                                  /* lineNo */
  27,                                  /* colNo */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pName */
};

static emlrtBCInfo emlrtBCI = { -1,    /* iFirst */
  -1,                                  /* iLast */
  35,                                  /* lineNo */
  37,                                  /* colNo */
  "JumpIdx",                           /* aName */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo d_emlrtECI = { -1,  /* nDims */
  39,                                  /* lineNo */
  27,                                  /* colNo */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pName */
};

static emlrtBCInfo b_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  40,                                  /* lineNo */
  39,                                  /* colNo */
  "JumpIdx",                           /* aName */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo c_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  44,                                  /* lineNo */
  41,                                  /* colNo */
  "WindowIdx",                         /* aName */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo d_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  44,                                  /* lineNo */
  59,                                  /* colNo */
  "WindowIdx",                         /* aName */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo e_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  44,                                  /* lineNo */
  80,                                  /* colNo */
  "WindowIdx",                         /* aName */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo f_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  44,                                  /* lineNo */
  92,                                  /* colNo */
  "WindowIdx",                         /* aName */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo g_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  45,                                  /* lineNo */
  53,                                  /* colNo */
  "WindowIdx",                         /* aName */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo h_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  45,                                  /* lineNo */
  71,                                  /* colNo */
  "WindowIdx",                         /* aName */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo i_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  45,                                  /* lineNo */
  117,                                 /* colNo */
  "WindowIdx",                         /* aName */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo j_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  45,                                  /* lineNo */
  129,                                 /* colNo */
  "WindowIdx",                         /* aName */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo e_emlrtECI = { -1,  /* nDims */
  52,                                  /* lineNo */
  12,                                  /* colNo */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pName */
};

static emlrtBCInfo k_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  61,                                  /* lineNo */
  47,                                  /* colNo */
  "CenterIdx",                         /* aName */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo l_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  69,                                  /* lineNo */
  47,                                  /* colNo */
  "CenterIdx",                         /* aName */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo m_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  81,                                  /* lineNo */
  77,                                  /* colNo */
  "WindowIdxReordered",                /* aName */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo n_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  81,                                  /* lineNo */
  89,                                  /* colNo */
  "WindowIdxReordered",                /* aName */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtECInfo f_emlrtECI = { -1,  /* nDims */
  88,                                  /* lineNo */
  34,                                  /* colNo */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m"/* pName */
};

static emlrtBCInfo o_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  90,                                  /* lineNo */
  59,                                  /* colNo */
  "WindowTrajectory_tmp",              /* aName */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtRTEInfo cb_emlrtRTEI = { 387,/* lineNo */
  1,                                   /* colNo */
  "find",                              /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\elmat\\find.m"/* pName */
};

static emlrtRTEInfo db_emlrtRTEI = { 22,/* lineNo */
  27,                                  /* colNo */
  "unaryMinOrMax",                     /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pName */
};

static emlrtRTEInfo eb_emlrtRTEI = { 77,/* lineNo */
  27,                                  /* colNo */
  "unaryMinOrMax",                     /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\unaryMinOrMax.m"/* pName */
};

static emlrtBCInfo p_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  52,                                  /* lineNo */
  38,                                  /* colNo */
  "RotatedCenteredTrajectory",         /* aName */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo q_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  52,                                  /* lineNo */
  91,                                  /* colNo */
  "RotatedCenteredTrajectory",         /* aName */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo r_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  61,                                  /* lineNo */
  37,                                  /* colNo */
  "SortedJumpIdx",                     /* aName */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo s_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  71,                                  /* lineNo */
  37,                                  /* colNo */
  "SortedJumpIdx",                     /* aName */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo t_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  69,                                  /* lineNo */
  37,                                  /* colNo */
  "SortedJumpIdx",                     /* aName */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m",/* pName */
  0                                    /* checkKind */
};

static emlrtBCInfo u_emlrtBCI = { -1,  /* iFirst */
  -1,                                  /* iLast */
  81,                                  /* lineNo */
  58,                                  /* colNo */
  "RotatedCenteredTrajectory",         /* aName */
  "ExtractWindowTrajectory",           /* fName */
  "C:\\Users\\Thomas\\Documents\\Kugle-MATLAB\\Controllers\\MPC\\functions\\trajectory\\ExtractWindowTrajectory.m",/* pName */
  0                                    /* checkKind */
};

/* Function Definitions */
void ExtractWindowTrajectory(const emlrtStack *sp, const emxArray_real_T
  *TrajectoryPoints, const real_T RobotPos[2], real_T RobotYaw, const real_T
  Velocity[2], real_T ExtractDist, real_T WindowWidth, real_T WindowHeight,
  const real_T WindowOffset[2], real_T OrientationSelection, emxArray_real_T
  *WindowTrajectory, real_T *nTrajPoints, real_T *WindowOrientation)
{
  int32_T n;
  emxArray_real_T *RotatedCenteredTrajectory;
  real_T b_RobotPos[2];
  real_T b_n[2];
  int32_T i0;
  int32_T b_TrajectoryPoints[2];
  emxArray_real_T *c_TrajectoryPoints;
  int32_T b_RotatedCenteredTrajectory[2];
  int32_T loop_ub;
  emxArray_boolean_T *x;
  real_T SeqJumpIdx0;
  emxArray_boolean_T *r0;
  int32_T i1;
  emxArray_int32_T *ii;
  int32_T nx;
  int32_T idx;
  boolean_T overflow;
  boolean_T exitg1;
  emxArray_real_T *WindowIdx;
  emxArray_real_T *StartIdx;
  emxArray_real_T *JumpIdx;
  emxArray_real_T *EndIdx;
  emxArray_int32_T *WindowIdxReordered;
  emxArray_real_T *Dist;
  emxArray_real_T *r1;
  emxArray_real_T *b_JumpIdx;
  int32_T b_idx;
  int32_T k;
  int32_T i2;
  real_T SeqJumpIdx1;
  emxArray_real_T *WindowTrajectory_tmp;
  emxArray_int32_T *r2;
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
    n = 0;
  } else {
    n = muIntScalarMax_sint32(TrajectoryPoints->size[0], 2);
  }

  emxInit_real_T(sp, &RotatedCenteredTrajectory, 2, &d_emlrtRTEI, true);
  b_RobotPos[0] = RobotPos[0];
  b_RobotPos[1] = RobotPos[1];
  b_n[0] = n;
  b_n[1] = 1.0;
  st.site = &emlrtRSI;
  repmat(&st, b_RobotPos, b_n, RotatedCenteredTrajectory);
  for (i0 = 0; i0 < 2; i0++) {
    b_TrajectoryPoints[i0] = TrajectoryPoints->size[i0];
  }

  for (i0 = 0; i0 < 2; i0++) {
    b_RotatedCenteredTrajectory[i0] = RotatedCenteredTrajectory->size[i0];
  }

  emxInit_real_T(sp, &c_TrajectoryPoints, 2, &emlrtRTEI, true);
  if ((b_TrajectoryPoints[0] != b_RotatedCenteredTrajectory[0]) ||
      (b_TrajectoryPoints[1] != b_RotatedCenteredTrajectory[1])) {
    emlrtSizeEqCheckNDR2012b(&b_TrajectoryPoints[0],
      &b_RotatedCenteredTrajectory[0], &emlrtECI, sp);
  }

  i0 = c_TrajectoryPoints->size[0] * c_TrajectoryPoints->size[1];
  c_TrajectoryPoints->size[0] = TrajectoryPoints->size[0];
  c_TrajectoryPoints->size[1] = 2;
  emxEnsureCapacity_real_T(sp, c_TrajectoryPoints, i0, &emlrtRTEI);
  loop_ub = TrajectoryPoints->size[0] * TrajectoryPoints->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    c_TrajectoryPoints->data[i0] = TrajectoryPoints->data[i0] -
      RotatedCenteredTrajectory->data[i0];
  }

  emxInit_boolean_T(sp, &x, 1, &emlrtRTEI, true);
  st.site = &b_emlrtRSI;
  RotateTrajectory(&st, c_TrajectoryPoints, *WindowOrientation,
                   RotatedCenteredTrajectory);

  /*  Only extract points within window centered at robot (-WindowWidth/2 to WindowWidth/2, -WindowHeight/2 to WindowHeight) */
  loop_ub = RotatedCenteredTrajectory->size[0];
  SeqJumpIdx0 = -WindowHeight / 2.0 + WindowOffset[0];
  i0 = x->size[0];
  x->size[0] = loop_ub;
  emxEnsureCapacity_boolean_T(sp, x, i0, &emlrtRTEI);
  emxFree_real_T(sp, &c_TrajectoryPoints);
  for (i0 = 0; i0 < loop_ub; i0++) {
    x->data[i0] = (RotatedCenteredTrajectory->data[i0] >= SeqJumpIdx0);
  }

  emxInit_boolean_T(sp, &r0, 1, &emlrtRTEI, true);
  loop_ub = RotatedCenteredTrajectory->size[0];
  SeqJumpIdx0 = WindowHeight / 2.0 + WindowOffset[0];
  i0 = r0->size[0];
  r0->size[0] = loop_ub;
  emxEnsureCapacity_boolean_T(sp, r0, i0, &emlrtRTEI);
  for (i0 = 0; i0 < loop_ub; i0++) {
    r0->data[i0] = (RotatedCenteredTrajectory->data[i0] <= SeqJumpIdx0);
  }

  i0 = x->size[0];
  i1 = r0->size[0];
  if (i0 != i1) {
    emlrtSizeEqCheck1DR2012b(i0, i1, &b_emlrtECI, sp);
  }

  i0 = x->size[0];
  emxEnsureCapacity_boolean_T(sp, x, i0, &emlrtRTEI);
  loop_ub = x->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    x->data[i0] = (x->data[i0] && r0->data[i0]);
  }

  loop_ub = RotatedCenteredTrajectory->size[0];
  SeqJumpIdx0 = -WindowWidth / 2.0 + WindowOffset[1];
  i0 = r0->size[0];
  r0->size[0] = loop_ub;
  emxEnsureCapacity_boolean_T(sp, r0, i0, &emlrtRTEI);
  for (i0 = 0; i0 < loop_ub; i0++) {
    r0->data[i0] = (RotatedCenteredTrajectory->data[i0 +
                    RotatedCenteredTrajectory->size[0]] >= SeqJumpIdx0);
  }

  i0 = x->size[0];
  i1 = r0->size[0];
  if (i0 != i1) {
    emlrtSizeEqCheck1DR2012b(i0, i1, &b_emlrtECI, sp);
  }

  i0 = x->size[0];
  emxEnsureCapacity_boolean_T(sp, x, i0, &emlrtRTEI);
  loop_ub = x->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    x->data[i0] = (x->data[i0] && r0->data[i0]);
  }

  loop_ub = RotatedCenteredTrajectory->size[0];
  SeqJumpIdx0 = WindowWidth / 2.0 + WindowOffset[1];
  i0 = r0->size[0];
  r0->size[0] = loop_ub;
  emxEnsureCapacity_boolean_T(sp, r0, i0, &emlrtRTEI);
  for (i0 = 0; i0 < loop_ub; i0++) {
    r0->data[i0] = (RotatedCenteredTrajectory->data[i0 +
                    RotatedCenteredTrajectory->size[0]] <= SeqJumpIdx0);
  }

  i0 = x->size[0];
  i1 = r0->size[0];
  if (i0 != i1) {
    emlrtSizeEqCheck1DR2012b(i0, i1, &b_emlrtECI, sp);
  }

  st.site = &c_emlrtRSI;
  i0 = x->size[0];
  emxEnsureCapacity_boolean_T(&st, x, i0, &emlrtRTEI);
  loop_ub = x->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    x->data[i0] = (x->data[i0] && r0->data[i0]);
  }

  emxFree_boolean_T(&st, &r0);
  emxInit_int32_T(&st, &ii, 1, &l_emlrtRTEI, true);
  b_st.site = &db_emlrtRSI;
  nx = x->size[0];
  c_st.site = &eb_emlrtRSI;
  idx = 0;
  i0 = ii->size[0];
  ii->size[0] = x->size[0];
  emxEnsureCapacity_int32_T(&c_st, ii, i0, &b_emlrtRTEI);
  d_st.site = &fb_emlrtRSI;
  overflow = ((!(1 > x->size[0])) && (x->size[0] > 2147483646));
  if (overflow) {
    e_st.site = &x_emlrtRSI;
    check_forloop_overflow_error(&e_st);
  }

  n = 1;
  exitg1 = false;
  while ((!exitg1) && (n <= nx)) {
    if (x->data[n - 1]) {
      idx++;
      ii->data[idx - 1] = n;
      if (idx >= nx) {
        exitg1 = true;
      } else {
        n++;
      }
    } else {
      n++;
    }
  }

  if (!(idx <= x->size[0])) {
    emlrtErrorWithMessageIdR2018a(&c_st, &cb_emlrtRTEI,
      "Coder:builtins:AssertionFailed", "Coder:builtins:AssertionFailed", 0);
  }

  if (x->size[0] == 1) {
    if (idx == 0) {
      i0 = ii->size[0];
      ii->size[0] = 0;
      emxEnsureCapacity_int32_T(&c_st, ii, i0, &emlrtRTEI);
    }
  } else {
    if (1 > idx) {
      i0 = 0;
    } else {
      i0 = idx;
    }

    b_TrajectoryPoints[0] = 1;
    b_TrajectoryPoints[1] = i0;
    d_st.site = &gb_emlrtRSI;
    indexShapeCheck(&d_st, ii->size[0], b_TrajectoryPoints);
    i1 = ii->size[0];
    ii->size[0] = i0;
    emxEnsureCapacity_int32_T(&c_st, ii, i1, &c_emlrtRTEI);
  }

  emxInit_real_T1(&c_st, &WindowIdx, 1, &e_emlrtRTEI, true);
  i0 = WindowIdx->size[0];
  WindowIdx->size[0] = ii->size[0];
  emxEnsureCapacity_real_T1(&st, WindowIdx, i0, &emlrtRTEI);
  loop_ub = ii->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    WindowIdx->data[i0] = ii->data[i0];
  }

  emxInit_real_T1(&st, &StartIdx, 1, &g_emlrtRTEI, true);
  st.site = &d_emlrtRSI;
  b_st.site = &d_emlrtRSI;
  diff(&b_st, WindowIdx, StartIdx);
  i0 = x->size[0];
  x->size[0] = StartIdx->size[0];
  emxEnsureCapacity_boolean_T(&st, x, i0, &emlrtRTEI);
  loop_ub = StartIdx->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    x->data[i0] = (StartIdx->data[i0] > 1.0);
  }

  b_st.site = &db_emlrtRSI;
  nx = x->size[0];
  c_st.site = &eb_emlrtRSI;
  idx = 0;
  i0 = ii->size[0];
  ii->size[0] = x->size[0];
  emxEnsureCapacity_int32_T(&c_st, ii, i0, &b_emlrtRTEI);
  d_st.site = &fb_emlrtRSI;
  overflow = ((!(1 > x->size[0])) && (x->size[0] > 2147483646));
  if (overflow) {
    e_st.site = &x_emlrtRSI;
    check_forloop_overflow_error(&e_st);
  }

  n = 1;
  exitg1 = false;
  while ((!exitg1) && (n <= nx)) {
    if (x->data[n - 1]) {
      idx++;
      ii->data[idx - 1] = n;
      if (idx >= nx) {
        exitg1 = true;
      } else {
        n++;
      }
    } else {
      n++;
    }
  }

  if (!(idx <= x->size[0])) {
    emlrtErrorWithMessageIdR2018a(&c_st, &cb_emlrtRTEI,
      "Coder:builtins:AssertionFailed", "Coder:builtins:AssertionFailed", 0);
  }

  if (x->size[0] == 1) {
    if (idx == 0) {
      i0 = ii->size[0];
      ii->size[0] = 0;
      emxEnsureCapacity_int32_T(&c_st, ii, i0, &emlrtRTEI);
    }
  } else {
    if (1 > idx) {
      i0 = 0;
    } else {
      i0 = idx;
    }

    b_TrajectoryPoints[0] = 1;
    b_TrajectoryPoints[1] = i0;
    d_st.site = &gb_emlrtRSI;
    indexShapeCheck(&d_st, ii->size[0], b_TrajectoryPoints);
    i1 = ii->size[0];
    ii->size[0] = i0;
    emxEnsureCapacity_int32_T(&c_st, ii, i1, &c_emlrtRTEI);
  }

  emxInit_real_T1(&c_st, &JumpIdx, 1, &f_emlrtRTEI, true);
  i0 = JumpIdx->size[0];
  JumpIdx->size[0] = ii->size[0];
  emxEnsureCapacity_real_T1(&st, JumpIdx, i0, &emlrtRTEI);
  loop_ub = ii->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    JumpIdx->data[i0] = ii->data[i0];
  }

  /*  find indices where there is a jump in the sequence index */
  st.site = &e_emlrtRSI;
  i0 = x->size[0];
  x->size[0] = WindowIdx->size[0];
  emxEnsureCapacity_boolean_T(&st, x, i0, &emlrtRTEI);
  loop_ub = WindowIdx->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    x->data[i0] = (WindowIdx->data[i0] == 1.0);
  }

  b_st.site = &db_emlrtRSI;
  nx = x->size[0];
  c_st.site = &eb_emlrtRSI;
  idx = 0;
  i0 = ii->size[0];
  ii->size[0] = x->size[0];
  emxEnsureCapacity_int32_T(&c_st, ii, i0, &b_emlrtRTEI);
  d_st.site = &fb_emlrtRSI;
  overflow = ((!(1 > x->size[0])) && (x->size[0] > 2147483646));
  if (overflow) {
    e_st.site = &x_emlrtRSI;
    check_forloop_overflow_error(&e_st);
  }

  n = 1;
  exitg1 = false;
  while ((!exitg1) && (n <= nx)) {
    if (x->data[n - 1]) {
      idx++;
      ii->data[idx - 1] = n;
      if (idx >= nx) {
        exitg1 = true;
      } else {
        n++;
      }
    } else {
      n++;
    }
  }

  if (!(idx <= x->size[0])) {
    emlrtErrorWithMessageIdR2018a(&c_st, &cb_emlrtRTEI,
      "Coder:builtins:AssertionFailed", "Coder:builtins:AssertionFailed", 0);
  }

  if (x->size[0] == 1) {
    if (idx == 0) {
      i0 = ii->size[0];
      ii->size[0] = 0;
      emxEnsureCapacity_int32_T(&c_st, ii, i0, &emlrtRTEI);
    }
  } else {
    if (1 > idx) {
      i0 = 0;
    } else {
      i0 = idx;
    }

    b_TrajectoryPoints[0] = 1;
    b_TrajectoryPoints[1] = i0;
    d_st.site = &gb_emlrtRSI;
    indexShapeCheck(&d_st, ii->size[0], b_TrajectoryPoints);
    i1 = ii->size[0];
    ii->size[0] = i0;
    emxEnsureCapacity_int32_T(&c_st, ii, i1, &c_emlrtRTEI);
  }

  i0 = StartIdx->size[0];
  StartIdx->size[0] = ii->size[0];
  emxEnsureCapacity_real_T1(&st, StartIdx, i0, &emlrtRTEI);
  loop_ub = ii->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    StartIdx->data[i0] = ii->data[i0];
  }

  if (TrajectoryPoints->size[0] == 0) {
    n = 0;
  } else {
    n = muIntScalarMax_sint32(TrajectoryPoints->size[0], 2);
  }

  st.site = &f_emlrtRSI;
  i0 = x->size[0];
  x->size[0] = WindowIdx->size[0];
  emxEnsureCapacity_boolean_T(&st, x, i0, &emlrtRTEI);
  loop_ub = WindowIdx->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    x->data[i0] = ((int32_T)WindowIdx->data[i0] == n);
  }

  b_st.site = &db_emlrtRSI;
  nx = x->size[0];
  c_st.site = &eb_emlrtRSI;
  idx = 0;
  i0 = ii->size[0];
  ii->size[0] = x->size[0];
  emxEnsureCapacity_int32_T(&c_st, ii, i0, &b_emlrtRTEI);
  d_st.site = &fb_emlrtRSI;
  overflow = ((!(1 > x->size[0])) && (x->size[0] > 2147483646));
  if (overflow) {
    e_st.site = &x_emlrtRSI;
    check_forloop_overflow_error(&e_st);
  }

  n = 1;
  exitg1 = false;
  while ((!exitg1) && (n <= nx)) {
    if (x->data[n - 1]) {
      idx++;
      ii->data[idx - 1] = n;
      if (idx >= nx) {
        exitg1 = true;
      } else {
        n++;
      }
    } else {
      n++;
    }
  }

  if (!(idx <= x->size[0])) {
    emlrtErrorWithMessageIdR2018a(&c_st, &cb_emlrtRTEI,
      "Coder:builtins:AssertionFailed", "Coder:builtins:AssertionFailed", 0);
  }

  if (x->size[0] == 1) {
    if (idx == 0) {
      i0 = ii->size[0];
      ii->size[0] = 0;
      emxEnsureCapacity_int32_T(&c_st, ii, i0, &emlrtRTEI);
    }
  } else {
    if (1 > idx) {
      i0 = 0;
    } else {
      i0 = idx;
    }

    b_TrajectoryPoints[0] = 1;
    b_TrajectoryPoints[1] = i0;
    d_st.site = &gb_emlrtRSI;
    indexShapeCheck(&d_st, ii->size[0], b_TrajectoryPoints);
    i1 = ii->size[0];
    ii->size[0] = i0;
    emxEnsureCapacity_int32_T(&c_st, ii, i1, &c_emlrtRTEI);
  }

  emxInit_real_T1(&c_st, &EndIdx, 1, &h_emlrtRTEI, true);
  i0 = EndIdx->size[0];
  EndIdx->size[0] = ii->size[0];
  emxEnsureCapacity_real_T1(&st, EndIdx, i0, &emlrtRTEI);
  loop_ub = ii->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    EndIdx->data[i0] = ii->data[i0];
  }

  emxInit_int32_T(sp, &WindowIdxReordered, 1, &i_emlrtRTEI, true);
  emxInit_real_T1(sp, &Dist, 1, &j_emlrtRTEI, true);
  emxInit_real_T(sp, &r1, 2, &emlrtRTEI, true);
  emxInit_real_T1(sp, &b_JumpIdx, 1, &emlrtRTEI, true);
  if ((!(StartIdx->size[0] == 0)) && (!(EndIdx->size[0] == 0))) {
    /*  extracted window includes both start and end index - we need to merge these such that the part belonging to the end index comes first */
    /*  Find jump index closest to the end index */
    b_RobotPos[0] = JumpIdx->size[0];
    b_RobotPos[1] = 1.0;
    st.site = &g_emlrtRSI;
    b_repmat(&st, EndIdx, b_RobotPos, r1);
    i0 = JumpIdx->size[0];
    i1 = r1->size[0];
    if (i0 != i1) {
      emlrtSizeEqCheck1DR2012b(i0, i1, &c_emlrtECI, sp);
    }

    i0 = b_JumpIdx->size[0];
    b_JumpIdx->size[0] = JumpIdx->size[0];
    emxEnsureCapacity_real_T1(sp, b_JumpIdx, i0, &emlrtRTEI);
    loop_ub = JumpIdx->size[0];
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_JumpIdx->data[i0] = JumpIdx->data[i0] - r1->data[i0];
    }

    st.site = &g_emlrtRSI;
    b_abs(&st, b_JumpIdx, Dist);
    st.site = &g_emlrtRSI;
    b_st.site = &nb_emlrtRSI;
    c_st.site = &ob_emlrtRSI;
    d_st.site = &pb_emlrtRSI;
    if ((Dist->size[0] == 1) || (Dist->size[0] != 1)) {
    } else {
      emlrtErrorWithMessageIdR2018a(&d_st, &db_emlrtRTEI,
        "Coder:toolbox:autoDimIncompatibility",
        "Coder:toolbox:autoDimIncompatibility", 0);
    }

    if (!(Dist->size[0] >= 1)) {
      emlrtErrorWithMessageIdR2018a(&d_st, &eb_emlrtRTEI,
        "Coder:toolbox:eml_min_or_max_varDimZero",
        "Coder:toolbox:eml_min_or_max_varDimZero", 0);
    }

    e_st.site = &qb_emlrtRSI;
    if (Dist->size[0] <= 2) {
      if (Dist->size[0] == 1) {
        idx = 0;
      } else if ((Dist->data[0] > Dist->data[1]) || (muDoubleScalarIsNaN
                  (Dist->data[0]) && (!muDoubleScalarIsNaN(Dist->data[1])))) {
        idx = 1;
      } else {
        idx = 0;
      }
    } else {
      f_st.site = &sb_emlrtRSI;
      if (!muDoubleScalarIsNaN(Dist->data[0])) {
        idx = 1;
      } else {
        idx = 0;
        g_st.site = &tb_emlrtRSI;
        overflow = (Dist->size[0] > 2147483646);
        if (overflow) {
          h_st.site = &x_emlrtRSI;
          check_forloop_overflow_error(&h_st);
        }

        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k <= Dist->size[0])) {
          if (!muDoubleScalarIsNaN(Dist->data[k - 1])) {
            idx = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }

      if (idx != 0) {
        f_st.site = &rb_emlrtRSI;
        SeqJumpIdx0 = Dist->data[idx - 1];
        b_idx = idx - 1;
        g_st.site = &ub_emlrtRSI;
        overflow = ((!(idx + 1 > Dist->size[0])) && (Dist->size[0] > 2147483646));
        if (overflow) {
          h_st.site = &x_emlrtRSI;
          check_forloop_overflow_error(&h_st);
        }

        while (idx + 1 <= Dist->size[0]) {
          if (SeqJumpIdx0 > Dist->data[idx]) {
            SeqJumpIdx0 = Dist->data[idx];
            b_idx = idx;
          }

          idx++;
        }

        idx = b_idx;
      }
    }

    i0 = JumpIdx->size[0];
    i1 = idx + 1;
    if (!((i1 >= 1) && (i1 <= i0))) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i0, &emlrtBCI, sp);
    }

    /* EndPortion = RotatedCenteredTrajectory(WindowIdx(EndPortionJumpIdx:EndIdx),:); */
    /*  Find jump index closest to the start index */
    b_RobotPos[0] = JumpIdx->size[0];
    b_RobotPos[1] = 1.0;
    st.site = &h_emlrtRSI;
    b_repmat(&st, StartIdx, b_RobotPos, r1);
    i0 = JumpIdx->size[0];
    i1 = r1->size[0];
    if (i0 != i1) {
      emlrtSizeEqCheck1DR2012b(i0, i1, &d_emlrtECI, sp);
    }

    i0 = b_JumpIdx->size[0];
    b_JumpIdx->size[0] = JumpIdx->size[0];
    emxEnsureCapacity_real_T1(sp, b_JumpIdx, i0, &emlrtRTEI);
    loop_ub = JumpIdx->size[0];
    for (i0 = 0; i0 < loop_ub; i0++) {
      b_JumpIdx->data[i0] = JumpIdx->data[i0] - r1->data[i0];
    }

    st.site = &h_emlrtRSI;
    b_abs(&st, b_JumpIdx, Dist);
    st.site = &h_emlrtRSI;
    b_st.site = &nb_emlrtRSI;
    c_st.site = &ob_emlrtRSI;
    d_st.site = &pb_emlrtRSI;
    if ((Dist->size[0] == 1) || (Dist->size[0] != 1)) {
    } else {
      emlrtErrorWithMessageIdR2018a(&d_st, &db_emlrtRTEI,
        "Coder:toolbox:autoDimIncompatibility",
        "Coder:toolbox:autoDimIncompatibility", 0);
    }

    if (!(Dist->size[0] >= 1)) {
      emlrtErrorWithMessageIdR2018a(&d_st, &eb_emlrtRTEI,
        "Coder:toolbox:eml_min_or_max_varDimZero",
        "Coder:toolbox:eml_min_or_max_varDimZero", 0);
    }

    e_st.site = &qb_emlrtRSI;
    if (Dist->size[0] <= 2) {
      if (Dist->size[0] == 1) {
        b_idx = 1;
      } else if ((Dist->data[0] > Dist->data[1]) || (muDoubleScalarIsNaN
                  (Dist->data[0]) && (!muDoubleScalarIsNaN(Dist->data[1])))) {
        b_idx = 2;
      } else {
        b_idx = 1;
      }
    } else {
      f_st.site = &sb_emlrtRSI;
      if (!muDoubleScalarIsNaN(Dist->data[0])) {
        b_idx = 1;
      } else {
        b_idx = 0;
        g_st.site = &tb_emlrtRSI;
        overflow = (Dist->size[0] > 2147483646);
        if (overflow) {
          h_st.site = &x_emlrtRSI;
          check_forloop_overflow_error(&h_st);
        }

        k = 2;
        exitg1 = false;
        while ((!exitg1) && (k <= Dist->size[0])) {
          if (!muDoubleScalarIsNaN(Dist->data[k - 1])) {
            b_idx = k;
            exitg1 = true;
          } else {
            k++;
          }
        }
      }

      if (b_idx == 0) {
        b_idx = 1;
      } else {
        f_st.site = &rb_emlrtRSI;
        SeqJumpIdx0 = Dist->data[b_idx - 1];
        g_st.site = &ub_emlrtRSI;
        overflow = ((!(b_idx + 1 > Dist->size[0])) && (Dist->size[0] >
          2147483646));
        if (overflow) {
          h_st.site = &x_emlrtRSI;
          check_forloop_overflow_error(&h_st);
        }

        for (k = b_idx; k < Dist->size[0]; k++) {
          if (SeqJumpIdx0 > Dist->data[k]) {
            SeqJumpIdx0 = Dist->data[k];
            b_idx = k + 1;
          }
        }
      }
    }

    i0 = JumpIdx->size[0];
    if (!((b_idx >= 1) && (b_idx <= i0))) {
      emlrtDynamicBoundsCheckR2012b(b_idx, 1, i0, &b_emlrtBCI, sp);
    }

    /* StartPortion = RotatedCenteredTrajectory(WindowIdx(StartIdx:EndPortionJumpIdx-1),:); */
    /* WindowTrajectory = [EndPortion; StartPortion]; */
    if (JumpIdx->data[idx] + 1.0 > EndIdx->data[0]) {
      i1 = 1;
      i0 = 1;
    } else {
      i0 = WindowIdx->size[0];
      i1 = (int32_T)(JumpIdx->data[idx] + 1.0);
      if (!((i1 >= 1) && (i1 <= i0))) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i0, &c_emlrtBCI, sp);
      }

      i0 = WindowIdx->size[0];
      i2 = (int32_T)EndIdx->data[0];
      if (!((i2 >= 1) && (i2 <= i0))) {
        emlrtDynamicBoundsCheckR2012b(i2, 1, i0, &d_emlrtBCI, sp);
      }

      i0 = i2 + 1;
    }

    b_TrajectoryPoints[0] = 1;
    b_TrajectoryPoints[1] = i0 - i1;
    st.site = &i_emlrtRSI;
    indexShapeCheck(&st, WindowIdx->size[0], b_TrajectoryPoints);
    if (StartIdx->data[0] > (JumpIdx->data[idx] + 1.0) - 1.0) {
      nx = 1;
      i2 = 1;
    } else {
      i2 = WindowIdx->size[0];
      nx = (int32_T)StartIdx->data[0];
      if (!((nx >= 1) && (nx <= i2))) {
        emlrtDynamicBoundsCheckR2012b(nx, 1, i2, &e_emlrtBCI, sp);
      }

      i2 = WindowIdx->size[0];
      k = (int32_T)((JumpIdx->data[idx] + 1.0) - 1.0);
      if (!((k >= 1) && (k <= i2))) {
        emlrtDynamicBoundsCheckR2012b(k, 1, i2, &f_emlrtBCI, sp);
      }

      i2 = k + 1;
    }

    b_TrajectoryPoints[0] = 1;
    b_TrajectoryPoints[1] = i2 - nx;
    st.site = &i_emlrtRSI;
    indexShapeCheck(&st, WindowIdx->size[0], b_TrajectoryPoints);
    k = WindowIdxReordered->size[0];
    WindowIdxReordered->size[0] = ((i0 - i1) + i2) - nx;
    emxEnsureCapacity_int32_T(sp, WindowIdxReordered, k, &emlrtRTEI);
    loop_ub = i0 - i1;
    for (k = 0; k < loop_ub; k++) {
      WindowIdxReordered->data[k] = (int32_T)WindowIdx->data[(i1 + k) - 1];
    }

    loop_ub = i2 - nx;
    for (i2 = 0; i2 < loop_ub; i2++) {
      WindowIdxReordered->data[(i2 + i0) - i1] = (int32_T)WindowIdx->data[(nx +
        i2) - 1];
    }

    if (JumpIdx->data[idx] + 1.0 > EndIdx->data[0]) {
      i1 = 1;
      i0 = 1;
    } else {
      i0 = WindowIdx->size[0];
      i1 = (int32_T)(JumpIdx->data[idx] + 1.0);
      if (!((i1 >= 1) && (i1 <= i0))) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i0, &g_emlrtBCI, sp);
      }

      i0 = WindowIdx->size[0];
      i2 = (int32_T)EndIdx->data[0];
      if (!((i2 >= 1) && (i2 <= i0))) {
        emlrtDynamicBoundsCheckR2012b(i2, 1, i0, &h_emlrtBCI, sp);
      }

      i0 = i2 + 1;
    }

    b_TrajectoryPoints[0] = 1;
    b_TrajectoryPoints[1] = i0 - i1;
    st.site = &j_emlrtRSI;
    indexShapeCheck(&st, WindowIdx->size[0], b_TrajectoryPoints);
    if (StartIdx->data[0] > (JumpIdx->data[idx] + 1.0) - 1.0) {
      nx = 1;
      i2 = 1;
    } else {
      i2 = WindowIdx->size[0];
      nx = (int32_T)StartIdx->data[0];
      if (!((nx >= 1) && (nx <= i2))) {
        emlrtDynamicBoundsCheckR2012b(nx, 1, i2, &i_emlrtBCI, sp);
      }

      i2 = WindowIdx->size[0];
      k = (int32_T)((JumpIdx->data[idx] + 1.0) - 1.0);
      if (!((k >= 1) && (k <= i2))) {
        emlrtDynamicBoundsCheckR2012b(k, 1, i2, &j_emlrtBCI, sp);
      }

      i2 = k + 1;
    }

    b_TrajectoryPoints[0] = 1;
    b_TrajectoryPoints[1] = i2 - nx;
    st.site = &j_emlrtRSI;
    indexShapeCheck(&st, WindowIdx->size[0], b_TrajectoryPoints);
    if (TrajectoryPoints->size[0] == 0) {
      n = 0;
    } else {
      n = muIntScalarMax_sint32(TrajectoryPoints->size[0], 2);
    }

    k = JumpIdx->size[0];
    JumpIdx->size[0] = ((i0 - i1) + i2) - nx;
    emxEnsureCapacity_real_T1(sp, JumpIdx, k, &emlrtRTEI);
    loop_ub = i0 - i1;
    for (k = 0; k < loop_ub; k++) {
      JumpIdx->data[k] = WindowIdx->data[(i1 + k) - 1] - (real_T)n;
    }

    loop_ub = i2 - nx;
    for (i2 = 0; i2 < loop_ub; i2++) {
      JumpIdx->data[(i2 + i0) - i1] = WindowIdx->data[(nx + i2) - 1];
    }
  } else {
    i0 = WindowIdxReordered->size[0];
    WindowIdxReordered->size[0] = WindowIdx->size[0];
    emxEnsureCapacity_int32_T(sp, WindowIdxReordered, i0, &emlrtRTEI);
    loop_ub = WindowIdx->size[0];
    for (i0 = 0; i0 < loop_ub; i0++) {
      WindowIdxReordered->data[i0] = (int32_T)WindowIdx->data[i0];
    }

    i0 = JumpIdx->size[0];
    JumpIdx->size[0] = WindowIdx->size[0];
    emxEnsureCapacity_real_T1(sp, JumpIdx, i0, &emlrtRTEI);
    loop_ub = WindowIdx->size[0];
    for (i0 = 0; i0 < loop_ub; i0++) {
      JumpIdx->data[i0] = WindowIdx->data[i0];
    }
  }

  emxFree_real_T(sp, &b_JumpIdx);
  emxFree_real_T(sp, &r1);
  emxFree_real_T(sp, &EndIdx);

  /*  Find continuous sequence of points closest to robot */
  b_idx = RotatedCenteredTrajectory->size[0];
  i0 = WindowIdx->size[0];
  WindowIdx->size[0] = WindowIdxReordered->size[0];
  emxEnsureCapacity_real_T1(sp, WindowIdx, i0, &emlrtRTEI);
  loop_ub = WindowIdxReordered->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    i1 = WindowIdxReordered->data[i0];
    if (!((i1 >= 1) && (i1 <= b_idx))) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, b_idx, &p_emlrtBCI, sp);
    }

    WindowIdx->data[i0] = RotatedCenteredTrajectory->data[i1 - 1];
  }

  st.site = &k_emlrtRSI;
  power(&st, WindowIdx, Dist);
  b_idx = RotatedCenteredTrajectory->size[0];
  i0 = WindowIdx->size[0];
  WindowIdx->size[0] = WindowIdxReordered->size[0];
  emxEnsureCapacity_real_T1(sp, WindowIdx, i0, &emlrtRTEI);
  loop_ub = WindowIdxReordered->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    i1 = WindowIdxReordered->data[i0];
    if (!((i1 >= 1) && (i1 <= b_idx))) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, b_idx, &q_emlrtBCI, sp);
    }

    WindowIdx->data[i0] = RotatedCenteredTrajectory->data[(i1 +
      RotatedCenteredTrajectory->size[0]) - 1];
  }

  st.site = &k_emlrtRSI;
  power(&st, WindowIdx, StartIdx);
  i0 = Dist->size[0];
  i1 = StartIdx->size[0];
  if (i0 != i1) {
    emlrtSizeEqCheck1DR2012b(i0, i1, &e_emlrtECI, sp);
  }

  i0 = Dist->size[0];
  emxEnsureCapacity_real_T1(sp, Dist, i0, &emlrtRTEI);
  loop_ub = Dist->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    Dist->data[i0] += StartIdx->data[i0];
  }

  st.site = &l_emlrtRSI;
  b_st.site = &nb_emlrtRSI;
  c_st.site = &ob_emlrtRSI;
  d_st.site = &pb_emlrtRSI;
  if ((Dist->size[0] == 1) || (Dist->size[0] != 1)) {
  } else {
    emlrtErrorWithMessageIdR2018a(&d_st, &db_emlrtRTEI,
      "Coder:toolbox:autoDimIncompatibility",
      "Coder:toolbox:autoDimIncompatibility", 0);
  }

  if (!(Dist->size[0] >= 1)) {
    emlrtErrorWithMessageIdR2018a(&d_st, &eb_emlrtRTEI,
      "Coder:toolbox:eml_min_or_max_varDimZero",
      "Coder:toolbox:eml_min_or_max_varDimZero", 0);
  }

  e_st.site = &qb_emlrtRSI;
  if (Dist->size[0] <= 2) {
    if (Dist->size[0] == 1) {
      idx = 1;
    } else if ((Dist->data[0] > Dist->data[1]) || (muDoubleScalarIsNaN
                (Dist->data[0]) && (!muDoubleScalarIsNaN(Dist->data[1])))) {
      idx = 2;
    } else {
      idx = 1;
    }
  } else {
    f_st.site = &sb_emlrtRSI;
    if (!muDoubleScalarIsNaN(Dist->data[0])) {
      idx = 1;
    } else {
      idx = 0;
      g_st.site = &tb_emlrtRSI;
      overflow = (Dist->size[0] > 2147483646);
      if (overflow) {
        h_st.site = &x_emlrtRSI;
        check_forloop_overflow_error(&h_st);
      }

      k = 2;
      exitg1 = false;
      while ((!exitg1) && (k <= Dist->size[0])) {
        if (!muDoubleScalarIsNaN(Dist->data[k - 1])) {
          idx = k;
          exitg1 = true;
        } else {
          k++;
        }
      }
    }

    if (idx == 0) {
      idx = 1;
    } else {
      f_st.site = &rb_emlrtRSI;
      SeqJumpIdx0 = Dist->data[idx - 1];
      g_st.site = &ub_emlrtRSI;
      overflow = ((!(idx + 1 > Dist->size[0])) && (Dist->size[0] > 2147483646));
      if (overflow) {
        h_st.site = &x_emlrtRSI;
        check_forloop_overflow_error(&h_st);
      }

      for (k = idx; k < Dist->size[0]; k++) {
        if (SeqJumpIdx0 > Dist->data[k]) {
          SeqJumpIdx0 = Dist->data[k];
          idx = k + 1;
        }
      }
    }
  }

  st.site = &m_emlrtRSI;
  b_st.site = &m_emlrtRSI;
  diff(&b_st, JumpIdx, StartIdx);
  i0 = x->size[0];
  x->size[0] = StartIdx->size[0];
  emxEnsureCapacity_boolean_T(&st, x, i0, &emlrtRTEI);
  loop_ub = StartIdx->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    x->data[i0] = (StartIdx->data[i0] > 1.0);
  }

  b_st.site = &db_emlrtRSI;
  nx = x->size[0];
  c_st.site = &eb_emlrtRSI;
  b_idx = 0;
  i0 = ii->size[0];
  ii->size[0] = x->size[0];
  emxEnsureCapacity_int32_T(&c_st, ii, i0, &b_emlrtRTEI);
  d_st.site = &fb_emlrtRSI;
  overflow = ((!(1 > x->size[0])) && (x->size[0] > 2147483646));
  if (overflow) {
    e_st.site = &x_emlrtRSI;
    check_forloop_overflow_error(&e_st);
  }

  n = 1;
  exitg1 = false;
  while ((!exitg1) && (n <= nx)) {
    if (x->data[n - 1]) {
      b_idx++;
      ii->data[b_idx - 1] = n;
      if (b_idx >= nx) {
        exitg1 = true;
      } else {
        n++;
      }
    } else {
      n++;
    }
  }

  if (!(b_idx <= x->size[0])) {
    emlrtErrorWithMessageIdR2018a(&c_st, &cb_emlrtRTEI,
      "Coder:builtins:AssertionFailed", "Coder:builtins:AssertionFailed", 0);
  }

  if (x->size[0] == 1) {
    if (b_idx == 0) {
      i0 = ii->size[0];
      ii->size[0] = 0;
      emxEnsureCapacity_int32_T(&c_st, ii, i0, &emlrtRTEI);
    }
  } else {
    if (1 > b_idx) {
      i0 = 0;
    } else {
      i0 = b_idx;
    }

    b_TrajectoryPoints[0] = 1;
    b_TrajectoryPoints[1] = i0;
    d_st.site = &gb_emlrtRSI;
    indexShapeCheck(&d_st, ii->size[0], b_TrajectoryPoints);
    i1 = ii->size[0];
    ii->size[0] = i0;
    emxEnsureCapacity_int32_T(&c_st, ii, i1, &c_emlrtRTEI);
  }

  /*  find indices where there is a jump in the sequence index */
  st.site = &n_emlrtRSI;
  i0 = Dist->size[0];
  Dist->size[0] = 3 + ii->size[0];
  emxEnsureCapacity_real_T1(&st, Dist, i0, &emlrtRTEI);
  Dist->data[0] = 1.0;
  loop_ub = ii->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    Dist->data[i0 + 1] = (real_T)ii->data[i0] + 1.0;
  }

  Dist->data[1 + ii->size[0]] = WindowIdxReordered->size[0];
  Dist->data[2 + ii->size[0]] = idx;
  b_st.site = &bc_emlrtRSI;
  sort(&b_st, Dist);
  st.site = &o_emlrtRSI;
  i0 = x->size[0];
  x->size[0] = Dist->size[0];
  emxEnsureCapacity_boolean_T(&st, x, i0, &emlrtRTEI);
  loop_ub = Dist->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    x->data[i0] = (Dist->data[i0] == idx);
  }

  b_st.site = &db_emlrtRSI;
  nx = x->size[0];
  c_st.site = &eb_emlrtRSI;
  idx = 0;
  i0 = ii->size[0];
  ii->size[0] = x->size[0];
  emxEnsureCapacity_int32_T(&c_st, ii, i0, &b_emlrtRTEI);
  d_st.site = &fb_emlrtRSI;
  overflow = ((!(1 > x->size[0])) && (x->size[0] > 2147483646));
  if (overflow) {
    e_st.site = &x_emlrtRSI;
    check_forloop_overflow_error(&e_st);
  }

  n = 1;
  exitg1 = false;
  while ((!exitg1) && (n <= nx)) {
    if (x->data[n - 1]) {
      idx++;
      ii->data[idx - 1] = n;
      if (idx >= nx) {
        exitg1 = true;
      } else {
        n++;
      }
    } else {
      n++;
    }
  }

  if (!(idx <= x->size[0])) {
    emlrtErrorWithMessageIdR2018a(&c_st, &cb_emlrtRTEI,
      "Coder:builtins:AssertionFailed", "Coder:builtins:AssertionFailed", 0);
  }

  if (x->size[0] == 1) {
    if (idx == 0) {
      i0 = ii->size[0];
      ii->size[0] = 0;
      emxEnsureCapacity_int32_T(&c_st, ii, i0, &emlrtRTEI);
    }
  } else {
    if (1 > idx) {
      i0 = 0;
    } else {
      i0 = idx;
    }

    b_TrajectoryPoints[0] = 1;
    b_TrajectoryPoints[1] = i0;
    d_st.site = &gb_emlrtRSI;
    indexShapeCheck(&d_st, ii->size[0], b_TrajectoryPoints);
    i1 = ii->size[0];
    ii->size[0] = i0;
    emxEnsureCapacity_int32_T(&c_st, ii, i1, &c_emlrtRTEI);
  }

  i0 = JumpIdx->size[0];
  JumpIdx->size[0] = ii->size[0];
  emxEnsureCapacity_real_T1(&st, JumpIdx, i0, &emlrtRTEI);
  loop_ub = ii->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    JumpIdx->data[i0] = ii->data[i0];
  }

  emxFree_int32_T(&st, &ii);
  i0 = x->size[0];
  x->size[0] = JumpIdx->size[0];
  emxEnsureCapacity_boolean_T(sp, x, i0, &emlrtRTEI);
  loop_ub = JumpIdx->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    x->data[i0] = (JumpIdx->data[i0] > 1.0);
  }

  st.site = &p_emlrtRSI;
  if (ifWhileCond(&st, x)) {
    /* SeqJumpIdx0 = SortedJumpIdx(CenterIdx(1)-1); % include old points as long as they are connected         */
    i0 = JumpIdx->size[0];
    if (!(1 <= i0)) {
      emlrtDynamicBoundsCheckR2012b(1, 1, i0, &k_emlrtBCI, sp);
    }

    i0 = Dist->size[0];
    i1 = (int32_T)JumpIdx->data[0];
    if (!((i1 >= 1) && (i1 <= i0))) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i0, &r_emlrtBCI, sp);
    }

    SeqJumpIdx0 = Dist->data[i1 - 1] - 2.0;

    /*  only include future trajectory points (+ 2 previous/old points) */
    if (Dist->data[(int32_T)JumpIdx->data[0] - 1] - 2.0 < 1.0) {
      SeqJumpIdx0 = 1.0;
    }
  } else {
    SeqJumpIdx0 = Dist->data[0];
  }

  n = Dist->size[0];
  i0 = x->size[0];
  x->size[0] = JumpIdx->size[0];
  emxEnsureCapacity_boolean_T(sp, x, i0, &emlrtRTEI);
  loop_ub = JumpIdx->size[0];
  for (i0 = 0; i0 < loop_ub; i0++) {
    x->data[i0] = ((int32_T)JumpIdx->data[i0] < n);
  }

  st.site = &q_emlrtRSI;
  if (ifWhileCond(&st, x)) {
    i0 = JumpIdx->size[0];
    if (!(1 <= i0)) {
      emlrtDynamicBoundsCheckR2012b(1, 1, i0, &l_emlrtBCI, sp);
    }

    i0 = Dist->size[0];
    i1 = (int32_T)(JumpIdx->data[0] + 1.0);
    if (!((i1 >= 1) && (i1 <= i0))) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i0, &t_emlrtBCI, sp);
    }

    SeqJumpIdx1 = Dist->data[i1 - 1];
  } else {
    i0 = Dist->size[0];
    i1 = Dist->size[0];
    if (!((i1 >= 1) && (i1 <= i0))) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i0, &s_emlrtBCI, sp);
    }

    SeqJumpIdx1 = Dist->data[i1 - 1];
  }

  emxFree_boolean_T(sp, &x);
  emxFree_real_T(sp, &JumpIdx);

  /*      [y,idx] = min(abs(JumpIdx-ClosestIdx)); */
  /*      SeqJumpIdx0 = JumpIdx(idx); */
  /*      JumpIdx(idx) = []; % remove first jump index     */
  /*      [y,idx] = min(abs(JumpIdx-ClosestIdx)); % find second jump index */
  /*      SeqJumpIdx1 = JumpIdx(idx);    */
  /* if (SeqJumpIdx1 > SeqJumpIdx0) */
  if (SeqJumpIdx0 > SeqJumpIdx1) {
    i1 = 1;
    i0 = 1;
  } else {
    i0 = WindowIdxReordered->size[0];
    i1 = (int32_T)SeqJumpIdx0;
    if (!((i1 >= 1) && (i1 <= i0))) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i0, &m_emlrtBCI, sp);
    }

    i0 = WindowIdxReordered->size[0];
    i2 = (int32_T)SeqJumpIdx1;
    if (!((i2 >= 1) && (i2 <= i0))) {
      emlrtDynamicBoundsCheckR2012b(i2, 1, i0, &n_emlrtBCI, sp);
    }

    i0 = i2 + 1;
  }

  emxInit_real_T(sp, &WindowTrajectory_tmp, 2, &k_emlrtRTEI, true);
  b_TrajectoryPoints[0] = 1;
  b_TrajectoryPoints[1] = i0 - i1;
  st.site = &r_emlrtRSI;
  indexShapeCheck(&st, WindowIdxReordered->size[0], b_TrajectoryPoints);
  b_idx = RotatedCenteredTrajectory->size[0];
  i2 = WindowTrajectory_tmp->size[0] * WindowTrajectory_tmp->size[1];
  WindowTrajectory_tmp->size[0] = i0 - i1;
  WindowTrajectory_tmp->size[1] = 2;
  emxEnsureCapacity_real_T(sp, WindowTrajectory_tmp, i2, &emlrtRTEI);
  loop_ub = i0 - i1;
  for (i2 = 0; i2 < 2; i2++) {
    for (nx = 0; nx < loop_ub; nx++) {
      k = WindowIdxReordered->data[(i1 + nx) - 1];
      if (!((k >= 1) && (k <= b_idx))) {
        emlrtDynamicBoundsCheckR2012b(k, 1, b_idx, &u_emlrtBCI, sp);
      }

      WindowTrajectory_tmp->data[nx + WindowTrajectory_tmp->size[0] * i2] =
        RotatedCenteredTrajectory->data[(k + RotatedCenteredTrajectory->size[0] *
        i2) - 1];
    }
  }

  emxFree_int32_T(sp, &WindowIdxReordered);

  /* else */
  /*     WindowTrajectory = RotatedCenteredTrajectory(WindowIdxReordered(SeqJumpIdx1+1:SeqJumpIdx0),:); */
  /* end */
  if (ExtractDist > 0.0) {
    /*  only extract certain future distance of trajectory based on a crude distance approximation */
    st.site = &s_emlrtRSI;
    b_diff(&st, WindowTrajectory_tmp, RotatedCenteredTrajectory);
    loop_ub = RotatedCenteredTrajectory->size[0];
    i2 = WindowIdx->size[0];
    WindowIdx->size[0] = loop_ub;
    emxEnsureCapacity_real_T1(sp, WindowIdx, i2, &emlrtRTEI);
    for (i2 = 0; i2 < loop_ub; i2++) {
      WindowIdx->data[i2] = RotatedCenteredTrajectory->data[i2];
    }

    st.site = &t_emlrtRSI;
    power(&st, WindowIdx, Dist);
    loop_ub = RotatedCenteredTrajectory->size[0];
    i2 = WindowIdx->size[0];
    WindowIdx->size[0] = loop_ub;
    emxEnsureCapacity_real_T1(sp, WindowIdx, i2, &emlrtRTEI);
    for (i2 = 0; i2 < loop_ub; i2++) {
      WindowIdx->data[i2] = RotatedCenteredTrajectory->data[i2 +
        RotatedCenteredTrajectory->size[0]];
    }

    st.site = &t_emlrtRSI;
    power(&st, WindowIdx, StartIdx);
    i2 = Dist->size[0];
    nx = StartIdx->size[0];
    if (i2 != nx) {
      emlrtSizeEqCheck1DR2012b(i2, nx, &f_emlrtECI, sp);
    }

    st.site = &t_emlrtRSI;
    i2 = Dist->size[0];
    emxEnsureCapacity_real_T1(&st, Dist, i2, &emlrtRTEI);
    loop_ub = Dist->size[0];
    for (i2 = 0; i2 < loop_ub; i2++) {
      Dist->data[i2] += StartIdx->data[i2];
    }

    overflow = false;
    for (k = 0; k < Dist->size[0]; k++) {
      if (overflow || (Dist->data[k] < 0.0)) {
        overflow = true;
      } else {
        overflow = false;
      }
    }

    if (overflow) {
      b_st.site = &ed_emlrtRSI;
      error(&b_st);
    }

    b_st.site = &fd_emlrtRSI;
    nx = Dist->size[0];
    c_st.site = &gd_emlrtRSI;
    overflow = ((!(1 > Dist->size[0])) && (Dist->size[0] > 2147483646));
    if (overflow) {
      d_st.site = &x_emlrtRSI;
      check_forloop_overflow_error(&d_st);
    }

    for (k = 0; k < nx; k++) {
      Dist->data[k] = muDoubleScalarSqrt(Dist->data[k]);
    }

    st.site = &t_emlrtRSI;
    b_st.site = &hd_emlrtRSI;
    n = 2;
    if (Dist->size[0] != 1) {
      n = 1;
    }

    c_st.site = &id_emlrtRSI;
    if (1 == n) {
      d_st.site = &jd_emlrtRSI;
      e_st.site = &kd_emlrtRSI;
      if (Dist->size[0] != 0) {
        f_st.site = &ld_emlrtRSI;
        n = Dist->size[0];
        if (Dist->size[0] != 1) {
          g_st.site = &md_emlrtRSI;
          for (k = 1; k < n; k++) {
            Dist->data[k] += Dist->data[k - 1];
          }
        }
      }
    }

    /* ExtractDist = N*ts*velocity; */
    b_idx = Dist->size[0];
    for (nx = 0; nx < b_idx; nx++) {
      if (Dist->data[nx] < ExtractDist) {
        i2 = i0 - i1;
        if (!((nx + 1 >= 1) && (nx + 1 <= i2))) {
          emlrtDynamicBoundsCheckR2012b(nx + 1, 1, i2, &o_emlrtBCI, sp);
        }
      }
    }

    b_idx = Dist->size[0] - 1;
    n = 0;
    for (nx = 0; nx <= b_idx; nx++) {
      if (Dist->data[nx] < ExtractDist) {
        n++;
      }
    }

    emxInit_int32_T(sp, &r2, 1, &emlrtRTEI, true);
    i0 = r2->size[0];
    r2->size[0] = n;
    emxEnsureCapacity_int32_T(sp, r2, i0, &emlrtRTEI);
    n = 0;
    for (nx = 0; nx <= b_idx; nx++) {
      if (Dist->data[nx] < ExtractDist) {
        r2->data[n] = nx + 1;
        n++;
      }
    }

    i0 = WindowTrajectory->size[0] * WindowTrajectory->size[1];
    WindowTrajectory->size[0] = r2->size[0];
    WindowTrajectory->size[1] = 2;
    emxEnsureCapacity_real_T(sp, WindowTrajectory, i0, &emlrtRTEI);
    for (i0 = 0; i0 < 2; i0++) {
      loop_ub = r2->size[0];
      for (i1 = 0; i1 < loop_ub; i1++) {
        WindowTrajectory->data[i1 + WindowTrajectory->size[0] * i0] =
          WindowTrajectory_tmp->data[(r2->data[i1] + WindowTrajectory_tmp->size
          [0] * i0) - 1];
      }
    }

    emxFree_int32_T(sp, &r2);
  } else {
    i0 = WindowTrajectory->size[0] * WindowTrajectory->size[1];
    WindowTrajectory->size[0] = WindowTrajectory_tmp->size[0];
    WindowTrajectory->size[1] = 2;
    emxEnsureCapacity_real_T(sp, WindowTrajectory, i0, &emlrtRTEI);
    loop_ub = WindowTrajectory_tmp->size[0] * WindowTrajectory_tmp->size[1];
    for (i0 = 0; i0 < loop_ub; i0++) {
      WindowTrajectory->data[i0] = WindowTrajectory_tmp->data[i0];
    }
  }

  emxFree_real_T(sp, &WindowTrajectory_tmp);
  emxFree_real_T(sp, &Dist);
  emxFree_real_T(sp, &StartIdx);
  emxFree_real_T(sp, &WindowIdx);
  emxFree_real_T(sp, &RotatedCenteredTrajectory);
  if (WindowTrajectory->size[0] == 0) {
    n = 0;
  } else {
    n = muIntScalarMax_sint32(WindowTrajectory->size[0], 2);
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
  *nTrajPoints = n;
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (ExtractWindowTrajectory.c) */
