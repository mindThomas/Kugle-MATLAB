/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_VelocityEKF_api.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 26-Apr-2019 13:40:20
 */

/* Include Files */
#include "tmwtypes.h"
#include "_coder_VelocityEKF_api.h"
#include "_coder_VelocityEKF_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131466U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "VelocityEKF",                       /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[7]);
static const mxArray *b_emlrt_marshallOut(const real32_T u[49]);
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *P_prev,
  const char_T *identifier, real32_T y[49]);
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[49]);
static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *EncoderDiffMeas, const char_T *identifier, real32_T y[3]);
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *X, const
  char_T *identifier, real32_T y[7]);
static const mxArray *emlrt_marshallOut(const real32_T u[7]);
static void f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[3]);
static real32_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *eta_encoder, const char_T *identifier);
static real32_T h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *cov_acc,
  const char_T *identifier, real32_T y[9]);
static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[9]);
static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *qQEKF, const
  char_T *identifier, real32_T y[4]);
static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[4]);
static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *cov_qQEKF,
  const char_T *identifier, real32_T y[16]);
static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[16]);
static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[7]);
static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[49]);
static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[3]);
static real32_T r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);
static void s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[9]);
static void t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[4]);
static void u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[16]);

/* Function Definitions */

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real32_T y[7]
 * Return Type  : void
 */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[7])
{
  o_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const real32_T u[49]
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(const real32_T u[49])
{
  const mxArray *y;
  int32_T i7;
  const mxArray *m1;
  static const int32_T iv1[2] = { 7, 7 };

  int32_T i;
  real32_T *pData;
  real32_T fv3[49];
  int32_T b_i;
  y = NULL;
  for (i7 = 0; i7 < 7; i7++) {
    for (i = 0; i < 7; i++) {
      fv3[i + 7 * i7] = u[i7 + 7 * i];
    }
  }

  m1 = emlrtCreateNumericArray(2, iv1, mxSINGLE_CLASS, mxREAL);
  pData = (real32_T *)emlrtMxGetData(m1);
  i7 = 0;
  for (i = 0; i < 7; i++) {
    for (b_i = 0; b_i < 7; b_i++) {
      pData[i7] = fv3[b_i + 7 * i];
      i7++;
    }
  }

  emlrtAssign(&y, m1);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *P_prev
 *                const char_T *identifier
 *                real32_T y[49]
 * Return Type  : void
 */
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *P_prev,
  const char_T *identifier, real32_T y[49])
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
 *                real32_T y[49]
 * Return Type  : void
 */
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[49])
{
  real32_T fv0[49];
  int32_T i0;
  int32_T i1;
  p_emlrt_marshallIn(sp, emlrtAlias(u), parentId, fv0);
  for (i0 = 0; i0 < 7; i0++) {
    for (i1 = 0; i1 < 7; i1++) {
      y[i1 + 7 * i0] = fv0[i0 + 7 * i1];
    }
  }

  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *EncoderDiffMeas
 *                const char_T *identifier
 *                real32_T y[3]
 * Return Type  : void
 */
static void e_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *EncoderDiffMeas, const char_T *identifier, real32_T y[3])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  f_emlrt_marshallIn(sp, emlrtAlias(EncoderDiffMeas), &thisId, y);
  emlrtDestroyArray(&EncoderDiffMeas);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *X
 *                const char_T *identifier
 *                real32_T y[7]
 * Return Type  : void
 */
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *X, const
  char_T *identifier, real32_T y[7])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  b_emlrt_marshallIn(sp, emlrtAlias(X), &thisId, y);
  emlrtDestroyArray(&X);
}

/*
 * Arguments    : const real32_T u[7]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real32_T u[7])
{
  const mxArray *y;
  const mxArray *m0;
  static const int32_T iv0[1] = { 7 };

  real32_T *pData;
  int32_T i6;
  int32_T i;
  y = NULL;
  m0 = emlrtCreateNumericArray(1, iv0, mxSINGLE_CLASS, mxREAL);
  pData = (real32_T *)emlrtMxGetData(m0);
  i6 = 0;
  for (i = 0; i < 7; i++) {
    pData[i6] = u[i];
    i6++;
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
  q_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *eta_encoder
 *                const char_T *identifier
 * Return Type  : real32_T
 */
static real32_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *eta_encoder, const char_T *identifier)
{
  real32_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = h_emlrt_marshallIn(sp, emlrtAlias(eta_encoder), &thisId);
  emlrtDestroyArray(&eta_encoder);
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
  y = r_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *cov_acc
 *                const char_T *identifier
 *                real32_T y[9]
 * Return Type  : void
 */
static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *cov_acc,
  const char_T *identifier, real32_T y[9])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  j_emlrt_marshallIn(sp, emlrtAlias(cov_acc), &thisId, y);
  emlrtDestroyArray(&cov_acc);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real32_T y[9]
 * Return Type  : void
 */
static void j_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[9])
{
  real32_T fv1[9];
  int32_T i2;
  int32_T i3;
  s_emlrt_marshallIn(sp, emlrtAlias(u), parentId, fv1);
  for (i2 = 0; i2 < 3; i2++) {
    for (i3 = 0; i3 < 3; i3++) {
      y[i3 + 3 * i2] = fv1[i2 + 3 * i3];
    }
  }

  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *qQEKF
 *                const char_T *identifier
 *                real32_T y[4]
 * Return Type  : void
 */
static void k_emlrt_marshallIn(const emlrtStack *sp, const mxArray *qQEKF, const
  char_T *identifier, real32_T y[4])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  l_emlrt_marshallIn(sp, emlrtAlias(qQEKF), &thisId, y);
  emlrtDestroyArray(&qQEKF);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real32_T y[4]
 * Return Type  : void
 */
static void l_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[4])
{
  t_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *cov_qQEKF
 *                const char_T *identifier
 *                real32_T y[16]
 * Return Type  : void
 */
static void m_emlrt_marshallIn(const emlrtStack *sp, const mxArray *cov_qQEKF,
  const char_T *identifier, real32_T y[16])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  n_emlrt_marshallIn(sp, emlrtAlias(cov_qQEKF), &thisId, y);
  emlrtDestroyArray(&cov_qQEKF);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real32_T y[16]
 * Return Type  : void
 */
static void n_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[16])
{
  real32_T fv2[16];
  int32_T i4;
  int32_T i5;
  u_emlrt_marshallIn(sp, emlrtAlias(u), parentId, fv2);
  for (i4 = 0; i4 < 4; i4++) {
    for (i5 = 0; i5 < 4; i5++) {
      y[i5 + (i4 << 2)] = fv2[i4 + (i5 << 2)];
    }
  }

  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real32_T ret[7]
 * Return Type  : void
 */
static void o_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[7])
{
  static const int32_T dims[1] = { 7 };

  int32_T i8;
  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "single",
    false, 1U, *(int32_T (*)[1])&dims[0]);
  for (i8 = 0; i8 < 7; i8++) {
    ret[i8] = (*(real32_T (*)[7])emlrtMxGetData(src))[i8];
  }

  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real32_T ret[49]
 * Return Type  : void
 */
static void p_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[49])
{
  static const int32_T dims[2] = { 7, 7 };

  int32_T i9;
  int32_T i10;
  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "single",
    false, 2U, *(int32_T (*)[2])&dims[0]);
  for (i9 = 0; i9 < 7; i9++) {
    for (i10 = 0; i10 < 7; i10++) {
      ret[i10 + 7 * i9] = (*(real32_T (*)[49])emlrtMxGetData(src))[i10 + 7 * i9];
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
static void q_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[3])
{
  static const int32_T dims[1] = { 3 };

  int32_T i11;
  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "single",
    false, 1U, *(int32_T (*)[1])&dims[0]);
  for (i11 = 0; i11 < 3; i11++) {
    ret[i11] = (*(real32_T (*)[3])emlrtMxGetData(src))[i11];
  }

  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real32_T
 */
static real32_T r_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
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
 *                real32_T ret[9]
 * Return Type  : void
 */
static void s_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[9])
{
  static const int32_T dims[2] = { 3, 3 };

  int32_T i12;
  int32_T i13;
  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "single",
    false, 2U, *(int32_T (*)[2])&dims[0]);
  for (i12 = 0; i12 < 3; i12++) {
    for (i13 = 0; i13 < 3; i13++) {
      ret[i13 + 3 * i12] = (*(real32_T (*)[9])emlrtMxGetData(src))[i13 + 3 * i12];
    }
  }

  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real32_T ret[4]
 * Return Type  : void
 */
static void t_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[4])
{
  static const int32_T dims[1] = { 4 };

  int32_T i14;
  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "single",
    false, 1U, *(int32_T (*)[1])&dims[0]);
  for (i14 = 0; i14 < 4; i14++) {
    ret[i14] = (*(real32_T (*)[4])emlrtMxGetData(src))[i14];
  }

  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real32_T ret[16]
 * Return Type  : void
 */
static void u_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[16])
{
  static const int32_T dims[2] = { 4, 4 };

  int32_T i15;
  int32_T i16;
  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "single",
    false, 2U, *(int32_T (*)[2])&dims[0]);
  for (i15 = 0; i15 < 4; i15++) {
    for (i16 = 0; i16 < 4; i16++) {
      ret[i16 + (i15 << 2)] = (*(real32_T (*)[16])emlrtMxGetData(src))[i16 +
        (i15 << 2)];
    }
  }

  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const mxArray * const prhs[17]
 *                int32_T nlhs
 *                const mxArray *plhs[2]
 * Return Type  : void
 */
void VelocityEKF_api(const mxArray * const prhs[17], int32_T nlhs, const mxArray
                     *plhs[2])
{
  real32_T X[7];
  real32_T P_prev[49];
  real32_T EncoderDiffMeas[3];
  real32_T eta_encoder;
  real32_T Accelerometer[3];
  real32_T cov_acc[9];
  real32_T eta_accelerometer;
  real32_T eta_bias;
  real32_T qQEKF[4];
  real32_T cov_qQEKF[16];
  real32_T qdotQEKF[4];
  real32_T eta_acceleration;
  real32_T SamplePeriod;
  real32_T TicksPrRev;
  real32_T rk;
  real32_T rw;
  real32_T g;
  real32_T X_out[7];
  real32_T P_out[49];
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Marshall function inputs */
  emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "X", X);
  c_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "P_prev", P_prev);
  e_emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "EncoderDiffMeas",
                     EncoderDiffMeas);
  eta_encoder = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "eta_encoder");
  e_emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "Accelerometer", Accelerometer);
  i_emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "cov_acc", cov_acc);
  eta_accelerometer = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[6]),
    "eta_accelerometer");
  eta_bias = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[7]), "eta_bias");
  k_emlrt_marshallIn(&st, emlrtAliasP(prhs[8]), "qQEKF", qQEKF);
  m_emlrt_marshallIn(&st, emlrtAliasP(prhs[9]), "cov_qQEKF", cov_qQEKF);
  k_emlrt_marshallIn(&st, emlrtAliasP(prhs[10]), "qdotQEKF", qdotQEKF);
  eta_acceleration = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[11]),
    "eta_acceleration");
  SamplePeriod = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[12]), "SamplePeriod");
  TicksPrRev = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[13]), "TicksPrRev");
  rk = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[14]), "rk");
  rw = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[15]), "rw");
  g = g_emlrt_marshallIn(&st, emlrtAliasP(prhs[16]), "g");

  /* Invoke the target function */
  VelocityEKF(X, P_prev, EncoderDiffMeas, eta_encoder, Accelerometer, cov_acc,
              eta_accelerometer, eta_bias, qQEKF, cov_qQEKF, qdotQEKF,
              eta_acceleration, SamplePeriod, TicksPrRev, rk, rw, g, X_out,
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
void VelocityEKF_atexit(void)
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
  VelocityEKF_xil_terminate();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void VelocityEKF_initialize(void)
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
void VelocityEKF_terminate(void)
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
 * File trailer for _coder_VelocityEKF_api.c
 *
 * [EOF]
 */
