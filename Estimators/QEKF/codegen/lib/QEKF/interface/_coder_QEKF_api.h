/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_QEKF_api.h
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 26-Apr-2019 13:33:55
 */

#ifndef _CODER_QEKF_API_H
#define _CODER_QEKF_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_QEKF_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void QEKF(real32_T X[10], real32_T P_prev[100], real32_T Gyroscope[3],
                 real32_T Accelerometer[3], real32_T Heading, boolean_T
                 UseHeadingForCorrection, real32_T SamplePeriod, boolean_T
                 SensorDriven, boolean_T BiasEstimationEnabled, boolean_T
                 YawBiasEstimationEnabled, boolean_T NormalizeAccelerometer,
                 real32_T cov_gyro[9], real32_T cov_acc[9], real32_T
                 GyroscopeTrustFactor, real32_T sigma2_omega, real32_T
                 sigma2_heading, real32_T sigma2_bias, boolean_T
                 AccelerometerVibrationDetectionEnabled, real32_T
                 AccelerometerVibrationNormLPFtau, real32_T
                 AccelerometerVibrationCovarianceVaryFactor, real32_T
                 MaxVaryFactor, real32_T g, real32_T X_out[10], real32_T P_out
                 [100]);
extern void QEKF_api(const mxArray * const prhs[22], int32_T nlhs, const mxArray
                     *plhs[2]);
extern void QEKF_atexit(void);
extern void QEKF_initialize(void);
extern void QEKF_terminate(void);
extern void QEKF_xil_terminate(void);

#endif

/*
 * File trailer for _coder_QEKF_api.h
 *
 * [EOF]
 */
