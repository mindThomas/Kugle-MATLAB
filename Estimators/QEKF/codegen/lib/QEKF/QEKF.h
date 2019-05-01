//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: QEKF.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 26-Apr-2019 13:33:55
//
#ifndef QEKF_H
#define QEKF_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "QEKF_types.h"

// Function Declarations
extern void QEKF(const float X[10], const float P_prev[100], const float
                 Gyroscope[3], const float Accelerometer[3], float Heading,
                 boolean_T UseHeadingForCorrection, float SamplePeriod,
                 boolean_T SensorDriven, boolean_T BiasEstimationEnabled,
                 boolean_T YawBiasEstimationEnabled, boolean_T
                 NormalizeAccelerometer, const float cov_gyro[9], const float
                 cov_acc[9], float GyroscopeTrustFactor, float sigma2_omega,
                 float sigma2_heading, float sigma2_bias, boolean_T
                 AccelerometerVibrationDetectionEnabled, float
                 AccelerometerVibrationNormLPFtau, float
                 AccelerometerVibrationCovarianceVaryFactor, float MaxVaryFactor,
                 float g, float X_out[10], float P_out[100]);
extern void acc_norm_filtered_not_empty_init();
extern void acc_norm_old_not_empty_init();

#endif

//
// File trailer for QEKF.h
//
// [EOF]
//
