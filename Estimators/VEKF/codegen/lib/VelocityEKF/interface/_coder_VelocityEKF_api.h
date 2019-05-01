/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_VelocityEKF_api.h
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 26-Apr-2019 13:40:20
 */

#ifndef _CODER_VELOCITYEKF_API_H
#define _CODER_VELOCITYEKF_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_VelocityEKF_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void VelocityEKF(real32_T X[7], real32_T P_prev[49], real32_T
  EncoderDiffMeas[3], real32_T eta_encoder, real32_T Accelerometer[3], real32_T
  cov_acc[9], real32_T eta_accelerometer, real32_T eta_bias, real32_T qQEKF[4],
  real32_T cov_qQEKF[16], real32_T qdotQEKF[4], real32_T eta_acceleration,
  real32_T SamplePeriod, real32_T TicksPrRev, real32_T rk, real32_T rw, real32_T
  g, real32_T X_out[7], real32_T P_out[49]);
extern void VelocityEKF_api(const mxArray * const prhs[17], int32_T nlhs, const
  mxArray *plhs[2]);
extern void VelocityEKF_atexit(void);
extern void VelocityEKF_initialize(void);
extern void VelocityEKF_terminate(void);
extern void VelocityEKF_xil_terminate(void);

#endif

/*
 * File trailer for _coder_VelocityEKF_api.h
 *
 * [EOF]
 */
