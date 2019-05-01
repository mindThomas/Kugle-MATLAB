/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_QEKF_api.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 26-Apr-2019 13:33:55
 */

/* Include Files */
#include "tmwtypes.h"
#include "_coder_QEKF_api.h"
#include "_coder_QEKF_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131466U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "QEKF",                              /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[10]);
static const mxArray *b_emlrt_marshallOut(const real32_T u[100]);
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *P_prev,
  const char_T *identifier, real32_T y[100]);
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[100]);
static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *Gyroscope,
  const char_T *identifier, real32_T y[3]);
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *X, const
  char_T *identifier, real32_T y[10]);
static const mxArray *emlrt_marshallOut(const real32_T u[10]);
static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[3]);
static real32_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *Heading,
  const char_T *identifier);
static real32_T h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static boolean_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *UseHeadingForCorrection, const char_T *identifier);
static boolean_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId);
static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *cov_gyro,
  const char_T *identifier, real32_T y[9]);
static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[9]);
static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[10]);
static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[100]);
static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[3]);
static real32_T p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);
static boolean_T q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);
static void r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[9]);

/* Function Definitions */

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real32_T y[10]
 * Return Type  : void
 */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[10])
{
  m_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const real32_T u[100]
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(const real32_T u[100])
{
  const mxArray *y;
  int32_T i5;
  const mxArray *m1;
  static const int32_T iv1[2] = { 10, 10 };

  int32_T i;
  real32_T *pData;
  real32_T fv2[100];
  int32_T b_i;
  y = NULL;
  for (i5 = 0; i5 < 10; i5++) {
    for (i = 0; i < 10; i++) {
      fv2[i + 10 * i5] = u[i5 + 10 * i];
    }
  }

  m1 = emlrtCreateNumericArray(2, iv1, mxSINGLE_CLASS, mxREAL);
  pData = (real32_T *)emlrtMxGetData(m1);
  i5 = 0;
  for (i = 0; i < 10; i++) {
    for (b_i = 0; b_i < 10; b_i++) {
      pData[i5] = fv2[b_i + 10 * i];
      i5++;
    }
  }

  emlrtAssign(&y, m1);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *P_prev
 *                const char_T *identifier
 *                real32_T y[100]
 * Return Type  : void
 */
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *P_prev,
  const char_T *identifier, real32_T y[100])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  d_emlrt_marshallIn(sp, emlrtAlias(P_prev), &thisId, y);
  emlrtDestroyArray(&P_prev);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real32_T y[100]
 * Return Type  : void
 */
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[100])
{
  real32_T fv0[100];
  int32_T i0;
  int32_T i1;
  n_emlrt_marshallIn(sp, emlrtAlias(u), parentId, fv0);
  for (i0 = 0; i0 < 10; i0++) {
    for (i1 = 0; i1 < 10; i1++) {
      y[i1 + 10 * i0] = fv0[i0 + 10 * i1];
    }
  }

  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *Gyroscope
 *                const char_T *identifier
 *                real32_T y[3]
 * Return Type  : void
 */
static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *Gyroscope,
  const char_T *identifier, real32_T y[3])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  f_emlrt_marshallIn(sp, emlrtAlias(Gyroscope), &thisId, y);
  emlrtDestroyArray(&Gyroscope);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *X
 *                const char_T *identifier
 *                real32_T y[10]
 * Return Type  : void
 */
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *X, const
  char_T *identifier, real32_T y[10])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  b_emlrt_marshallIn(sp, emlrtAlias(X), &thisId, y);
  emlrtDestroyArray(&X);
}

/*
 * Arguments    : const real32_T u[10]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real32_T u[10])
{
  const mxArray *y;
  const mxArray *m0;
  static const int32_T iv0[1] = { 10 };

  real32_T *pData;
  int32_T i4;
  int32_T i;
  y = NULL;
  m0 = emlrtCreateNumericArray(1, iv0, mxSINGLE_CLASS, mxREAL);
  pData = (real32_T *)emlrtMxGetData(m0);
  i4 = 0;
  for (i = 0; i < 10; i++) {
    pData[i4] = u[i];
    i4++;
  }

  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real32_T y[3]
 * Return Type  : void
 */
static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[3])
{
  o_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *Heading
 *                const char_T *identifier
 * Return Type  : real32_T
 */
static real32_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *Heading,
  const char_T *identifier)
{
  real32_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = h_emlrt_marshallIn(sp, emlrtAlias(Heading), &thisId);
  emlrtDestroyArray(&Heading);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real32_T
 */
static real32_T h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real32_T y;
  y = p_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *UseHeadingForCorrection
 *                const char_T *identifier
 * Return Type  : boolean_T
 */
static boolean_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *UseHeadingForCorrection, const char_T *identifier)
{
  boolean_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = j_emlrt_marshallIn(sp, emlrtAlias(UseHeadingForCorrection), &thisId);
  emlrtDestroyArray(&UseHeadingForCorrection);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : boolean_T
 */
static boolean_T j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId)
{
  boolean_T y;
  y = q_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *cov_gyro
 *                const char_T *identifier
 *                real32_T y[9]
 * Return Type  : void
 */
static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *cov_gyro,
  const char_T *identifier, real32_T y[9])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  l_emlrt_marshallIn(sp, emlrtAlias(cov_gyro), &thisId, y);
  emlrtDestroyArray(&cov_gyro);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real32_T y[9]
 * Return Type  : void
 */
static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[9])
{
  real32_T fv1[9];
  int32_T i2;
  int32_T i3;
  r_emlrt_marshallIn(sp, emlrtAlias(u), parentId, fv1);
  for (i2 = 0; i2 < 3; i2++) {
    for (i3 = 0; i3 < 3; i3++) {
      y[i3 + 3 * i2] = fv1[i2 + 3 * i3];
    }
  }

  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real32_T ret[10]
 * Return Type  : void
 */
static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[10])
{
  static const int32_T dims[1] = { 10 };

  int32_T i6;
  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "single",
    false, 1U, *(int32_T (*)[1])&dims[0]);
  for (i6 = 0; i6 < 10; i6++) {
    ret[i6] = (*(real32_T (*)[10])emlrtMxGetData(src))[i6];
  }

  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real32_T ret[100]
 * Return Type  : void
 */
static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[100])
{
  static const int32_T dims[2] = { 10, 10 };

  int32_T i7;
  int32_T i8;
  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "single",
    false, 2U, *(int32_T (*)[2])&dims[0]);
  for (i7 = 0; i7 < 10; i7++) {
    for (i8 = 0; i8 < 10; i8++) {
      ret[i8 + 10 * i7] = (*(real32_T (*)[100])emlrtMxGetData(src))[i8 + 10 * i7];
    }
  }

  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real32_T ret[3]
 * Return Type  : void
 */
static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[3])
{
  static const int32_T dims[1] = { 3 };

  int32_T i9;
  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "single",
    false, 1U, *(int32_T (*)[1])&dims[0]);
  for (i9 = 0; i9 < 3; i9++) {
    ret[i9] = (*(real32_T (*)[3])emlrtMxGetData(src))[i9];
  }

  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real32_T
 */
static real32_T p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  real32_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "single",
    false, 0U, (int32_T *)&dims);
  ret = *(real32_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : boolean_T
 */
static boolean_T q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  boolean_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "logical",
    false, 0U, (int32_T *)&dims);
  ret = *emlrtMxGetLogicals(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real32_T ret[9]
 * Return Type  : void
 */
static void r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[9])
{
  static const int32_T dims[2] = { 3, 3 };

  int32_T i10;
  int32_T i11;
  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "single",
    false, 2U, *(int32_T (*)[2])&dims[0]);
  for (i10 = 0; i10 < 3; i10++) {
    for (i11 = 0; i11 < 3; i11++) {
      ret[i11 + 3 * i10] = (*(real32_T (*)[9])emlrtMxGetData(src))[i11 + 3 * i10];
    }
  }

  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const mxArray * const prhs[22]
 *                int32_T nlhs
 *                const mxArray *plhs[2]
 * Return Type  : void
 */
void QEKF_api(const mxArray * const prhs[22], int32_T nlhs, const mxArray *plhs
              [2])
{
  real32_T X[10];
  real32_T P_prev[100];
  real32_T Gyroscope[3];
  real32_T Accelerometer[3];
  real32_T Heading;
  boolean_T UseHeadingForCorrection;
  real32_T SamplePeriod;
  boolean_T SensorDriven;
  boolean_T BiasEstimationEnabled;
  boolean_T YawBiasEstimationEnabled;
  boolean_T NormalizeAccelerometer;
  real32_T cov_gyro[9];
  real32_T cov_acc[9];
  real32_T GyroscopeTrustFactor;
  real32_T sigma2_omega;
  real32_T sigma2_heading;
  real32_T sigma2_bias;
  boolean_T AccelerometerVibrationDetectionEnabled;
  real32_T AccelerometerVibrationNormLPFtau;
  real32_T AccelerometerVibrationCovarianceVaryFactor;
  real32_T MaxVaryFactor;
  real32_T g;
  real32_T X_out[10];
  real32_T P_out[100];
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Marshall function inputs */
  emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "X", X);
  c_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "P_prev", P_prev);
  e_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "Gyroscope", Gyroscope);
  e_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "Accelerometer", Accelerometer);
  Heading = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "Heading");
  UseHeadingForCorrection = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[5]),
    "UseHeadingForCorrection");
  SamplePeriod = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "SamplePeriod");
  SensorDriven = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[7]), "SensorDriven");
  BiasEstimationEnabled = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[8]),
    "BiasEstimationEnabled");
  YawBiasEstimationEnabled = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[9]),
    "YawBiasEstimationEnabled");
  NormalizeAccelerometer = i_emlrt_marshallIn(&st, emlrtAliasP(prhs[10]),
    "NormalizeAccelerometer");
  k_emlrt_marshallIn(&st, emlrtAliasP(prhs[11]), "cov_gyro", cov_gyro);
  k_emlrt_marshallIn(&st, emlrtAliasP(prhs[12]), "cov_acc", cov_acc);
  GyroscopeTrustFactor = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[13]),
    "GyroscopeTrustFactor");
  sigma2_omega = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[14]), "sigma2_omega");
  sigma2_heading = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[15]),
    "sigma2_heading");
  sigma2_bias = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[16]), "sigma2_bias");
  AccelerometerVibrationDetectionEnabled = i_emlrt_marshallIn(&st, emlrtAliasP
    (prhs[17]), "AccelerometerVibrationDetectionEnabled");
  AccelerometerVibrationNormLPFtau = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[18]),
    "AccelerometerVibrationNormLPFtau");
  AccelerometerVibrationCovarianceVaryFactor = g_emlrt_marshallIn(&st,
    emlrtAliasP(prhs[19]), "AccelerometerVibrationCovarianceVaryFactor");
  MaxVaryFactor = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[20]), "MaxVaryFactor");
  g = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[21]), "g");

  /* Invoke the target function */
  QEKF(X, P_prev, Gyroscope, Accelerometer, Heading, UseHeadingForCorrection,
       SamplePeriod, SensorDriven, BiasEstimationEnabled,
       YawBiasEstimationEnabled, NormalizeAccelerometer, cov_gyro, cov_acc,
       GyroscopeTrustFactor, sigma2_omega, sigma2_heading, sigma2_bias,
       AccelerometerVibrationDetectionEnabled, AccelerometerVibrationNormLPFtau,
       AccelerometerVibrationCovarianceVaryFactor, MaxVaryFactor, g, X_out,
       P_out);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(X_out);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(P_out);
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void QEKF_atexit(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  QEKF_xil_terminate();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void QEKF_initialize(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void QEKF_terminate(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_QEKF_api.c
 *
 * [EOF]
 */
