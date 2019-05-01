//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: VelocityEKF.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 26-Apr-2019 13:40:20
//
#ifndef VELOCITYEKF_H
#define VELOCITYEKF_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "VelocityEKF_types.h"

// Function Declarations
extern void VelocityEKF(const float X[7], const float P_prev[49], const float
  EncoderDiffMeas[3], float eta_encoder, const float Accelerometer[3], const
  float cov_acc[9], float eta_accelerometer, float eta_bias, const float qQEKF[4],
  const float cov_qQEKF[16], const float qdotQEKF[4], float eta_acceleration,
  float SamplePeriod, float TicksPrRev, float rk, float rw, float g, float
  X_out[7], float P_out[49]);

#endif

//
// File trailer for VelocityEKF.h
//
// [EOF]
//
