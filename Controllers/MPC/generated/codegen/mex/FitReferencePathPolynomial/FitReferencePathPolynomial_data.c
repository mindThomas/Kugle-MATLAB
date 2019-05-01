/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * FitReferencePathPolynomial_data.c
 *
 * Code generation for function 'FitReferencePathPolynomial_data'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "FitReferencePathPolynomial.h"
#include "FitReferencePathPolynomial_data.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
const volatile char_T *emlrtBreakCheckR2012bFlagVar = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131466U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "FitReferencePathPolynomial",        /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

emlrtRSInfo t_emlrtRSI = { 21,         /* lineNo */
  "eml_int_forloop_overflow_check",    /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\eml\\eml_int_forloop_overflow_check.m"/* pathName */
};

emlrtRSInfo gb_emlrtRSI = { 32,        /* lineNo */
  "useConstantDim",                    /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\useConstantDim.m"/* pathName */
};

emlrtRSInfo hb_emlrtRSI = { 93,        /* lineNo */
  "cumop",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\private\\cumop.m"/* pathName */
};

emlrtRSInfo ib_emlrtRSI = { 119,       /* lineNo */
  "cumop",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\private\\cumop.m"/* pathName */
};

emlrtRSInfo jb_emlrtRSI = { 286,       /* lineNo */
  "cumop",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\datafun\\private\\cumop.m"/* pathName */
};

emlrtRSInfo ac_emlrtRSI = { 36,        /* lineNo */
  "vAllOrAny",                         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\vAllOrAny.m"/* pathName */
};

emlrtRSInfo bc_emlrtRSI = { 96,        /* lineNo */
  "vAllOrAny",                         /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\vAllOrAny.m"/* pathName */
};

emlrtRSInfo hc_emlrtRSI = { 20,        /* lineNo */
  "cat",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m"/* pathName */
};

emlrtRSInfo ic_emlrtRSI = { 100,       /* lineNo */
  "cat",                               /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m"/* pathName */
};

emlrtRSInfo od_emlrtRSI = { 49,        /* lineNo */
  "xdot",                              /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\xdot.m"/* pathName */
};

emlrtRSInfo qd_emlrtRSI = { 52,        /* lineNo */
  "eml_mtimes_helper",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_helper.m"/* pathName */
};

emlrtRSInfo rd_emlrtRSI = { 21,        /* lineNo */
  "eml_mtimes_helper",                 /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_helper.m"/* pathName */
};

emlrtRSInfo sd_emlrtRSI = { 114,       /* lineNo */
  "mtimes",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\mtimes.m"/* pathName */
};

emlrtRSInfo td_emlrtRSI = { 118,       /* lineNo */
  "mtimes",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\mtimes.m"/* pathName */
};

emlrtRSInfo ee_emlrtRSI = { 88,        /* lineNo */
  "mtimes",                            /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\mtimes.m"/* pathName */
};

emlrtRSInfo fe_emlrtRSI = { 32,        /* lineNo */
  "xdotu",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\xdotu.m"/* pathName */
};

emlrtMCInfo emlrtMCI = { 27,           /* lineNo */
  5,                                   /* colNo */
  "error",                             /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\lang\\error.m"/* pName */
};

emlrtRTEInfo t_emlrtRTEI = { 118,      /* lineNo */
  13,                                  /* colNo */
  "mtimes",                            /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\+blas\\mtimes.m"/* pName */
};

emlrtRTEInfo mb_emlrtRTEI = { 281,     /* lineNo */
  27,                                  /* colNo */
  "cat",                               /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\cat.m"/* pName */
};

emlrtRTEInfo tb_emlrtRTEI = { 83,      /* lineNo */
  23,                                  /* colNo */
  "eml_mtimes_helper",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_helper.m"/* pName */
};

emlrtRTEInfo ub_emlrtRTEI = { 88,      /* lineNo */
  23,                                  /* colNo */
  "eml_mtimes_helper",                 /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\ops\\eml_mtimes_helper.m"/* pName */
};

const char_T cv0[39] = { 'P', 'o', 'l', 'y', 'n', 'o', 'm', 'i', 'a', 'l', ' ',
  'o', 'r', 'd', 'e', 'r', ' ', 'n', 'e', 'e', 'd', 's', ' ', 't', 'o', ' ', 'b',
  'e', ' ', 'a', 't', ' ', 'l', 'e', 'a', 's', 't', ' ', '1' };

const char_T cv1[57] = { 'M', 'i', 's', 'm', 'a', 't', 'c', 'h', ' ', 'i', 'n',
  ' ', 'd', 'a', 't', 'a', ' ', 'l', 'e', 'n', 'g', 't', 'h', ' ', 'a', 'n', 'd',
  ' ', 'l', 'e', 'n', 'g', 't', 'h', ' ', 'o', 'f', ' ', 'p', 'a', 'r', 'a', 'm',
  'e', 't', 'e', 'r', ' ', 'v', 'a', 'l', 'u', 'e', 's', ',', ' ', 't' };

const char_T cv2[75] = { 'U', 'n', 'd', 'e', 'r', 'd', 'e', 't', 'e', 'r', 'm',
  'i', 'n', 'e', 'd', ' ', 'l', 'e', 'a', 's', 't', ' ', 's', 'q', 'u', 'a', 'r',
  'e', 's', ' ', 'p', 'o', 'l', 'y', 'n', 'o', 'm', 'i', 'a', 'l', ' ', 'f', 'i',
  't', 't', 'i', 'n', 'g', ' ', 'd', 'u', 'e', ' ', 't', 'o', ' ', 't', 'o', 'o',
  ' ', 'f', 'e', 'w', ' ', 'd', 'a', 't', 'a', ' ', 'p', 'o', 'i', 'n', 't', 's'
};

const char_T cv3[109] = { 'C', 'a', 'n', ' ', 'n', 'o', 't', ' ', 'e', 'n', 'f',
  'o', 'r', 'c', 'e', ' ', 'b', 'e', 'g', 'i', 'n', '-', 'e', 'n', 'd', ' ', 'a',
  'n', 'g', 'l', 'e', ' ', 'c', 'o', 'n', 's', 't', 'r', 'a', 'i', 'n', 't', ' ',
  'w', 'h', 'e', 'n', ' ', 'i', 'n', 'p', 'u', 't', ' ', 'p', 'a', 'r', 'a', 'm',
  'e', 't', 'e', 'r', ',', ' ', 't', ',', ' ', 'i', 's', ' ', 'n', 'o', 't', ' ',
  'c', 'h', 'a', 'n', 'g', 'i', 'n', 'g', ' ', 'n', 'e', 'a', 'r', ' ', 't', 'h',
  'e', ' ', 'b', 'e', 'g', 'i', 'n', 'n', 'i', 'n', 'g', ' ', 'o', 'r', ' ', 'e',
  'n', 'd' };

emlrtRSInfo ke_emlrtRSI = { 27,        /* lineNo */
  "error",                             /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\lib\\matlab\\lang\\error.m"/* pathName */
};

/* End of code generation (FitReferencePathPolynomial_data.c) */
