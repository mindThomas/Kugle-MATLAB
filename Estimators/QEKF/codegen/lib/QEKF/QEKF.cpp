//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: QEKF.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 26-Apr-2019 13:33:55
//

// Include Files
#include <math.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <string.h>
#include "QEKF.h"
#include "norm.h"
#include "eye.h"
#include "mrdivide.h"

// Variable Definitions
static float acc_norm_filtered;
static boolean_T acc_norm_filtered_not_empty;
static float acc_norm_old;
static boolean_T acc_norm_old_not_empty;

// Function Declarations
static float rt_atan2f_snf(float u0, float u1);

// Function Definitions

//
// Arguments    : float u0
//                float u1
// Return Type  : float
//
static float rt_atan2f_snf(float u0, float u1)
{
  float y;
  int b_u0;
  int b_u1;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = ((real32_T)rtNaN);
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    if (u0 > 0.0F) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0F) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = (float)atan2((double)(float)b_u0, (double)(float)b_u1);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = (float)atan2((double)u0, (double)u1);
  }

  return y;
}

//
// function [X_out, P_out] = QEKF(X, P_prev, Gyroscope, Accelerometer, Heading, UseHeadingForCorrection, SamplePeriod, SensorDriven, BiasEstimationEnabled, YawBiasEstimationEnabled, NormalizeAccelerometer, cov_gyro, cov_acc, GyroscopeTrustFactor, sigma2_omega, sigma2_heading, sigma2_bias, AccelerometerVibrationDetectionEnabled, AccelerometerVibrationNormLPFtau, AccelerometerVibrationCovarianceVaryFactor, MaxVaryFactor, g)
// for q o p = Phi(q) * p
// Arguments    : const float X[10]
//                const float P_prev[100]
//                const float Gyroscope[3]
//                const float Accelerometer[3]
//                float Heading
//                boolean_T UseHeadingForCorrection
//                float SamplePeriod
//                boolean_T SensorDriven
//                boolean_T BiasEstimationEnabled
//                boolean_T YawBiasEstimationEnabled
//                boolean_T NormalizeAccelerometer
//                const float cov_gyro[9]
//                const float cov_acc[9]
//                float GyroscopeTrustFactor
//                float sigma2_omega
//                float sigma2_heading
//                float sigma2_bias
//                boolean_T AccelerometerVibrationDetectionEnabled
//                float AccelerometerVibrationNormLPFtau
//                float AccelerometerVibrationCovarianceVaryFactor
//                float MaxVaryFactor
//                float g
//                float X_out[10]
//                float P_out[100]
// Return Type  : void
//
void QEKF(const float X[10], const float P_prev[100], const float Gyroscope[3],
          const float Accelerometer[3], float Heading, boolean_T
          UseHeadingForCorrection, float SamplePeriod, boolean_T SensorDriven,
          boolean_T BiasEstimationEnabled, boolean_T YawBiasEstimationEnabled,
          boolean_T NormalizeAccelerometer, const float cov_gyro[9], const float
          cov_acc[9], float GyroscopeTrustFactor, float sigma2_omega, float
          sigma2_heading, float sigma2_bias, boolean_T
          AccelerometerVibrationDetectionEnabled, float
          AccelerometerVibrationNormLPFtau, float
          AccelerometerVibrationCovarianceVaryFactor, float MaxVaryFactor, float
          g, float X_out[10], float P_out[100])
{
  float q[4];
  float gyro_bias[3];
  float gyro_input[3];
  int i0;
  signed char BiasEstimationEnabledMat[9];
  float cov_omega[9];
  static const signed char iv0[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  float a;
  float coeff_b;
  int i;
  static float Q[100];
  float z_acc[3];
  float R[36];
  float z[6];
  float X_apriori[10];
  float fv0[16];
  float b_X[4];
  float dq[12];
  int i1;
  float b_dq[4];
  static const signed char iv1[12] = { 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static float F_prev[100];
  double dv0[16];
  float y[16];
  float b_y[4];
  double dv1[9];
  static float b_P_prev[100];
  float c_y[16];
  static float P_apriori[100];
  float d_y[16];
  float c_dq[12];
  static const signed char iv2[12] = { 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1 };

  static const signed char iv3[4] = { 0, 0, 0, -1 };

  float b_cov_gyro[12];
  float fv1[4];
  float fv2[4];
  float fv3[4];
  static const signed char iv4[16] = { 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0,
    0, 0, -1 };

  float H_acc[30];
  float z_hat[6];
  static float H[60];
  static float b_H[60];
  float c_H[36];
  static float K[60];
  float b_z[6];
  static double dv2[100];
  float c_z[10];
  float d_dq[8];
  float heading_vector_hat[2];
  float e_dq[8];
  static const signed char iv5[8] = { 0, 1, 0, 0, 0, 0, 1, 0 };

  float b_heading_vector_hat[2];
  static const signed char iv6[4] = { 0, 1, 0, 0 };

  float fv4[20];
  static float H2[70];
  static float b_H2[70];
  static float K2[70];
  static float b_R[49];
  static float c_H2[49];
  static float d_H2[49];
  float d_z[7];
  float b_z_hat[7];
  float e_z[7];

  // 'QEKF:3' Phi = @(q)[q(1) -q(2) -q(3) -q(4);     % for q o p = Phi(q) * p
  // 'QEKF:4'               q(2) q(1)  -q(4) q(3);
  // 'QEKF:5'               q(3) q(4)  q(1)  -q(2);
  // 'QEKF:6'               q(4) -q(3) q(2)  q(1)];
  //  for q o p = Gamma(p) * q
  // 'QEKF:7' Gamma = @(p)[p(1) -p(2) -p(3) -p(4);   % for q o p = Gamma(p) * q
  // 'QEKF:8'                  p(2) p(1) p(4) -p(3);
  // 'QEKF:9'                  p(3) -p(4) p(1) p(2);
  // 'QEKF:10'                  p(4) p(3) -p(2) p(1)];
  // 'QEKF:12' devec = [0,1,0,0;0,0,1,0;0,0,0,1];
  //  'v' in notes
  // 'QEKF:13' vec = [0,0,0;1,0,0;0,1,0;0,0,1];
  //  '^' in notes
  // 'QEKF:14' I_conj = diag([1,-1,-1,-1]);
  // 'QEKF:16' dt = SamplePeriod;
  //  Split state vector, X[k-1], into individual variables
  // 'QEKF:19' q1 = X(1);
  // 'QEKF:20' q2 = X(2);
  // 'QEKF:21' q3 = X(3);
  // 'QEKF:22' q4 = X(4);
  // 'QEKF:23' q = [q1, q2, q3, q4]';
  q[0] = X[0];
  q[1] = X[1];
  q[2] = X[2];
  q[3] = X[3];

  // 'QEKF:25' omega = X(5:7);
  // 'QEKF:27' gyro_bias_x = X(8);
  // 'QEKF:28' gyro_bias_y = X(9);
  // 'QEKF:29' gyro_bias_z = X(10);
  // 'QEKF:30' gyro_bias = [gyro_bias_x; gyro_bias_y; gyro_bias_z];
  gyro_bias[0] = X[7];
  gyro_bias[1] = X[8];
  gyro_bias[2] = X[9];

  // 'QEKF:32' gyro_x = Gyroscope(1);
  // 'QEKF:33' gyro_y = Gyroscope(2);
  // 'QEKF:34' gyro_z = Gyroscope(3);
  // 'QEKF:35' gyro_input = [gyro_x; gyro_y; gyro_z];
  gyro_input[0] = Gyroscope[0];
  gyro_input[1] = Gyroscope[1];
  gyro_input[2] = Gyroscope[2];

  // 'QEKF:37' BiasEstimationEnabledMat = single(zeros(3,3));
  // 'QEKF:38' BiasEstimationEnabledMat(1,1) = BiasEstimationEnabled;
  for (i0 = 0; i0 < 9; i0++) {
    BiasEstimationEnabledMat[i0] = 0;
    cov_omega[i0] = sigma2_omega * (float)iv0[i0];
  }

  BiasEstimationEnabledMat[0] = (signed char)BiasEstimationEnabled;

  // 'QEKF:39' BiasEstimationEnabledMat(2,2) = BiasEstimationEnabled;
  BiasEstimationEnabledMat[4] = (signed char)BiasEstimationEnabled;

  // 'QEKF:40' BiasEstimationEnabledMat(3,3) = BiasEstimationEnabled*YawBiasEstimationEnabled; 
  BiasEstimationEnabledMat[8] = (signed char)(BiasEstimationEnabled *
    YawBiasEstimationEnabled);

  //  Process covariances
  // 'QEKF:43' cov_q = single(zeros(4,4));
  //  quaternion kinematics is correct since we propagate with Quaternion exponential 
  // 'QEKF:44' cov_omega = sigma2_omega * eye(3);
  // 'QEKF:45' cov_bias = sigma2_bias*dt*BiasEstimationEnabledMat + zeros(3,3);
  a = sigma2_bias * SamplePeriod;

  //  bias stays constant
  //  Vary gyroscope covariance based on accelerometer vibrations noise -
  //  trust gyroscope more at high accelerometer vibrations
  // 'QEKF:50' if (isempty(acc_norm_filtered))
  if (!acc_norm_filtered_not_empty) {
    // 'QEKF:51' acc_norm_filtered = g;
    acc_norm_filtered = g;
    acc_norm_filtered_not_empty = true;
  }

  // 'QEKF:55' if (isempty(acc_norm_old))
  if (!acc_norm_old_not_empty) {
    // 'QEKF:56' acc_norm_old = g;
    acc_norm_old = g;
    acc_norm_old_not_empty = true;
  }

  // 'QEKF:59' if (AccelerometerVibrationDetectionEnabled)
  if (AccelerometerVibrationDetectionEnabled) {
    // 'QEKF:60' coeff_b = 1/(2*AccelerometerVibrationNormLPFtau/dt + 1);
    coeff_b = 1.0F / (2.0F * AccelerometerVibrationNormLPFtau / SamplePeriod +
                      1.0F);

    //  nominator
    // 'QEKF:61' coeff_a = 1/(2*AccelerometerVibrationNormLPFtau/dt + 1) - 2/(2 + dt/AccelerometerVibrationNormLPFtau); 
    //  denominator
    // 'QEKF:62' acc_norm_filtered = coeff_b*norm(Accelerometer) + coeff_b*acc_norm_old - coeff_a*acc_norm_filtered; 
    acc_norm_filtered = (coeff_b * norm(Accelerometer) + coeff_b * acc_norm_old)
      - (1.0F / (2.0F * AccelerometerVibrationNormLPFtau / SamplePeriod + 1.0F)
         - 2.0F / (2.0F + SamplePeriod / AccelerometerVibrationNormLPFtau)) *
      acc_norm_filtered;

    // 'QEKF:63' acc_norm_old = norm(Accelerometer);
    acc_norm_old = norm(Accelerometer);

    // if (abs(acc_norm_filtered - g) > AccelerometerExaggerationAmount)
    // 'QEKF:66' VaryFactor = exp(AccelerometerVibrationCovarianceVaryFactor * (abs(acc_norm_filtered - g))); 
    coeff_b = (float)exp((double)(AccelerometerVibrationCovarianceVaryFactor *
      (float)fabs((double)(acc_norm_filtered - g))));

    // 'QEKF:67' VaryFactor = min(VaryFactor, MaxVaryFactor);
    if (!((coeff_b < MaxVaryFactor) || rtIsNaNF(MaxVaryFactor))) {
      coeff_b = MaxVaryFactor;
    }

    // cov_acc_ = cov_acc_ * VaryFactor;
    // 'QEKF:69' cov_omega = cov_omega / VaryFactor;
    for (i0 = 0; i0 < 9; i0++) {
      cov_omega[i0] /= coeff_b;
    }

    // end
    // 'QEKF:71' acc_norm_out = acc_norm_filtered;
  }

  //  Setup covariance matrices
  // 'QEKF:75' Q = [cov_q, zeros(4,3), zeros(4,3);
  // 'QEKF:76'          zeros(3,4), cov_omega, zeros(3,3);
  // 'QEKF:77'          zeros(3,4), zeros(3,3), cov_bias];
  for (i0 = 0; i0 < 4; i0++) {
    for (i = 0; i < 4; i++) {
      Q[i + 10 * i0] = 0.0F;
    }

    for (i = 0; i < 3; i++) {
      Q[(i + 10 * i0) + 4] = 0.0F;
      Q[(i + 10 * i0) + 7] = 0.0F;
    }
  }

  // 'QEKF:79' R = [GyroscopeTrustFactor*cov_acc, zeros(3,3)
  // 'QEKF:80'          zeros(3,3), cov_gyro];
  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 4; i++) {
      Q[i + 10 * (i0 + 4)] = 0.0F;
    }

    for (i = 0; i < 3; i++) {
      Q[(i + 10 * (i0 + 4)) + 4] = cov_omega[i + 3 * i0];
      Q[(i + 10 * (i0 + 4)) + 7] = 0.0F;
    }

    for (i = 0; i < 4; i++) {
      Q[i + 10 * (i0 + 7)] = 0.0F;
    }

    for (i = 0; i < 3; i++) {
      Q[(i + 10 * (i0 + 7)) + 4] = 0.0F;
      Q[(i + 10 * (i0 + 7)) + 7] = a * (float)BiasEstimationEnabledMat[i + 3 *
        i0];
      R[i + 6 * i0] = GyroscopeTrustFactor * cov_acc[i + 3 * i0];
      R[(i + 6 * i0) + 3] = 0.0F;
      R[i + 6 * (i0 + 3)] = 0.0F;
      R[(i + 6 * (i0 + 3)) + 3] = cov_gyro[i + 3 * i0];
    }
  }

  //  Measurement vector
  //  Normalize accelerometer seems to give better results
  // 'QEKF:84' if (NormalizeAccelerometer)
  if (NormalizeAccelerometer) {
    // norm_acc = norm(Accelerometer);
    // if (norm_acc > 0)% && norm_acc < 9.85)
    //     z_acc = Accelerometer / norm_acc;
    // else
    //     z_acc = 0*Accelerometer;
    // end
    // 'QEKF:91' z_acc = Accelerometer / g;
    for (i = 0; i < 3; i++) {
      z_acc[i] = Accelerometer[i] / g;
    }

    //  this is not actually a normalization, but rather a nominal normalization - but it seems to yield better results? 
  } else {
    // 'QEKF:92' else
    // 'QEKF:93' z_acc = Accelerometer;
    for (i = 0; i < 3; i++) {
      z_acc[i] = Accelerometer[i];
    }
  }

  // 'QEKF:96' z_gyro = gyro_input;
  // 'QEKF:98' z = [z_acc; z_gyro];
  for (i = 0; i < 3; i++) {
    z[i] = z_acc[i];
    z[i + 3] = gyro_input[i];
  }

  //     %% Prediction step
  // 'QEKF:101' X_apriori = single(zeros(10,1));
  for (i = 0; i < 10; i++) {
    X_apriori[i] = 0.0F;
  }

  //  Propagate quaternion
  // 'QEKF:104' if (SensorDriven)
  if (SensorDriven) {
    // 'QEKF:105' dq = 1/2 * Phi(q) * vec * (gyro_input - gyro_bias);
    fv0[0] = 0.5F * X[0];
    fv0[1] = 0.5F * -X[1];
    fv0[2] = 0.5F * -X[2];
    fv0[3] = 0.5F * -X[3];
    fv0[4] = 0.5F * X[1];
    fv0[5] = 0.5F * X[0];
    fv0[6] = 0.5F * -X[3];
    fv0[7] = 0.5F * X[2];
    fv0[8] = 0.5F * X[2];
    fv0[9] = 0.5F * X[3];
    fv0[10] = 0.5F * X[0];
    fv0[11] = 0.5F * -X[1];
    fv0[12] = 0.5F * X[3];
    fv0[13] = 0.5F * -X[2];
    fv0[14] = 0.5F * X[1];
    fv0[15] = 0.5F * X[0];
    for (i0 = 0; i0 < 3; i0++) {
      z_acc[i0] = gyro_input[i0] - gyro_bias[i0];
      for (i = 0; i < 4; i++) {
        dq[i0 + 3 * i] = 0.0F;
        for (i1 = 0; i1 < 4; i1++) {
          dq[i0 + 3 * i] += (float)iv1[i0 + 3 * i1] * fv0[i1 + (i << 2)];
        }
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      b_X[i0] = 0.0F;
      for (i = 0; i < 3; i++) {
        b_X[i0] += z_acc[i] * dq[i + 3 * i0];
      }

      b_dq[i0] = b_X[i0];
    }
  } else {
    // 'QEKF:106' else
    // 'QEKF:107' dq = 1/2 * Phi(q) * vec * omega;
    fv0[0] = 0.5F * X[0];
    fv0[1] = 0.5F * -X[1];
    fv0[2] = 0.5F * -X[2];
    fv0[3] = 0.5F * -X[3];
    fv0[4] = 0.5F * X[1];
    fv0[5] = 0.5F * X[0];
    fv0[6] = 0.5F * -X[3];
    fv0[7] = 0.5F * X[2];
    fv0[8] = 0.5F * X[2];
    fv0[9] = 0.5F * X[3];
    fv0[10] = 0.5F * X[0];
    fv0[11] = 0.5F * -X[1];
    fv0[12] = 0.5F * X[3];
    fv0[13] = 0.5F * -X[2];
    fv0[14] = 0.5F * X[1];
    fv0[15] = 0.5F * X[0];
    for (i0 = 0; i0 < 3; i0++) {
      for (i = 0; i < 4; i++) {
        dq[i0 + 3 * i] = 0.0F;
        for (i1 = 0; i1 < 4; i1++) {
          dq[i0 + 3 * i] += (float)iv1[i0 + 3 * i1] * fv0[i1 + (i << 2)];
        }
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      b_X[i0] = 0.0F;
      for (i = 0; i < 3; i++) {
        b_X[i0] += X[4 + i] * dq[i + 3 * i0];
      }

      b_dq[i0] = b_X[i0];
    }

    //  and conversely:  omeg = 2 * devec * Phi(q)' * dq;
  }

  // 'QEKF:109' q_apriori = q + dt * dq;
  for (i0 = 0; i0 < 4; i0++) {
    b_dq[i0] = q[i0] + SamplePeriod * b_dq[i0];
  }

  //  Forward Euler
  // q_apriori = Phi(q) * [1; 1/2*dt*omega]; % Delta-Quaternion integration
  //  Propagate/set angular velocity states
  // 'QEKF:113' omega_apriori = omega;
  //  Propagate gyro bias
  // 'QEKF:116' gyro_bias_apriori = gyro_bias;
  //  Determine model Jacobian (F)   -  OBS. This is not supposed to use/depend on apriori states! 
  // 'QEKF:119' F_prev = single(zeros(10,10));
  memset(&F_prev[0], 0, 100U * sizeof(float));

  // 'QEKF:120' F_prev(1:4,1:4) = eye(4) + dt * 1/2 * Gamma(vec*omega);
  coeff_b = SamplePeriod / 2.0F;
  for (i0 = 0; i0 < 4; i0++) {
    b_X[i0] = 0.0F;
    for (i = 0; i < 3; i++) {
      b_X[i0] += X[4 + i] * (float)iv1[i + 3 * i0];
    }

    b_y[i0] = b_X[i0];
  }

  b_eye(dv0);
  y[0] = coeff_b * b_y[0];
  y[1] = coeff_b * -b_y[1];
  y[2] = coeff_b * -b_y[2];
  y[3] = coeff_b * -b_y[3];
  y[4] = coeff_b * b_y[1];
  y[5] = coeff_b * b_y[0];
  y[6] = coeff_b * b_y[3];
  y[7] = coeff_b * -b_y[2];
  y[8] = coeff_b * b_y[2];
  y[9] = coeff_b * -b_y[3];
  y[10] = coeff_b * b_y[0];
  y[11] = coeff_b * b_y[1];
  y[12] = coeff_b * b_y[3];
  y[13] = coeff_b * b_y[2];
  y[14] = coeff_b * -b_y[1];
  y[15] = coeff_b * b_y[0];
  for (i0 = 0; i0 < 4; i0++) {
    for (i = 0; i < 4; i++) {
      F_prev[i + 10 * i0] = (float)dv0[i + (i0 << 2)] + y[i + (i0 << 2)];
    }
  }

  // Gamma([1, 1/2*dt*omega]); % eye(4);
  // 'QEKF:121' F_prev(1:4,5:7) = dt * 1/2 * Phi(q) * [zeros(1,3); eye(3)];
  coeff_b = SamplePeriod / 2.0F;
  y[0] = coeff_b * X[0];
  y[1] = coeff_b * -X[1];
  y[2] = coeff_b * -X[2];
  y[3] = coeff_b * -X[3];
  y[4] = coeff_b * X[1];
  y[5] = coeff_b * X[0];
  y[6] = coeff_b * -X[3];
  y[7] = coeff_b * X[2];
  y[8] = coeff_b * X[2];
  y[9] = coeff_b * X[3];
  y[10] = coeff_b * X[0];
  y[11] = coeff_b * -X[1];
  y[12] = coeff_b * X[3];
  y[13] = coeff_b * -X[2];
  y[14] = coeff_b * X[1];
  y[15] = coeff_b * X[0];
  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 4; i++) {
      F_prev[(i0 + 10 * i) + 4] = 0.0F;
      for (i1 = 0; i1 < 4; i1++) {
        F_prev[(i0 + 10 * i) + 4] += (float)iv1[i0 + 3 * i1] * y[i1 + (i << 2)];
      }
    }
  }

  // 'QEKF:122' F_prev(1:4,8:10) = zeros(4,3);
  for (i0 = 0; i0 < 4; i0++) {
    for (i = 0; i < 3; i++) {
      F_prev[(i + 10 * i0) + 7] = 0.0F;
    }
  }

  // 'QEKF:123' F_prev(5:7,1:4) = zeros(3,4);
  // 'QEKF:124' F_prev(5:7,5:7) = eye(3);
  eye(dv1);

  // 'QEKF:125' F_prev(5:7,8:10) = zeros(3,3);
  // 'QEKF:126' F_prev(8:10,1:4) = zeros(3,4);
  // 'QEKF:127' F_prev(8:10,5:7) = zeros(3,3);
  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 4; i++) {
      F_prev[i + 10 * (4 + i0)] = 0.0F;
    }

    for (i = 0; i < 3; i++) {
      F_prev[(i + 10 * (4 + i0)) + 4] = (float)dv1[i + 3 * i0];
      F_prev[(i + 10 * (4 + i0)) + 7] = 0.0F;
    }

    for (i = 0; i < 4; i++) {
      F_prev[i + 10 * (7 + i0)] = 0.0F;
    }

    for (i = 0; i < 3; i++) {
      F_prev[(i + 10 * (7 + i0)) + 4] = 0.0F;
    }
  }

  // 'QEKF:128' F_prev(8:10,8:10) = eye(3);
  eye(dv1);
  for (i0 = 0; i0 < 3; i0++) {
    for (i = 0; i < 3; i++) {
      F_prev[(i + 10 * (7 + i0)) + 7] = (float)dv1[i + 3 * i0];
    }
  }

  // 'QEKF:130' if (SensorDriven)
  if (SensorDriven) {
    //  Change covariance matrix and model Jacobian
    // 'QEKF:132' F_prev(1:4,1:4) = eye(4);
    b_eye(dv0);

    // 'QEKF:133' F_prev(1:4,5:7) = zeros(4,3);
    for (i0 = 0; i0 < 4; i0++) {
      for (i = 0; i < 4; i++) {
        F_prev[i + 10 * i0] = (float)dv0[i + (i0 << 2)];
      }

      for (i = 0; i < 3; i++) {
        F_prev[(i + 10 * i0) + 4] = 0.0F;
      }
    }

    // 'QEKF:134' F_prev(1:4,8:10) = -dt * 1/2 * Phi(q) * [zeros(1,3); BiasEstimationEnabledMat]; 
    coeff_b = -SamplePeriod / 2.0F;
    for (i0 = 0; i0 < 3; i0++) {
      dq[i0] = 0.0F;
      for (i = 0; i < 3; i++) {
        dq[i + 3 * (i0 + 1)] = BiasEstimationEnabledMat[i + 3 * i0];
      }
    }

    y[0] = coeff_b * X[0];
    y[1] = coeff_b * -X[1];
    y[2] = coeff_b * -X[2];
    y[3] = coeff_b * -X[3];
    y[4] = coeff_b * X[1];
    y[5] = coeff_b * X[0];
    y[6] = coeff_b * -X[3];
    y[7] = coeff_b * X[2];
    y[8] = coeff_b * X[2];
    y[9] = coeff_b * X[3];
    y[10] = coeff_b * X[0];
    y[11] = coeff_b * -X[1];
    y[12] = coeff_b * X[3];
    y[13] = coeff_b * -X[2];
    y[14] = coeff_b * X[1];
    y[15] = coeff_b * X[0];

    // 'QEKF:135' Q(1:4,1:4) = (dt * 1/2 * Phi(q) * [zeros(1,3); eye(3)]) * cov_gyro * (dt * 1/2 * Phi(q) * [zeros(1,3); eye(3)])'; 
    coeff_b = SamplePeriod / 2.0F;
    a = SamplePeriod / 2.0F;
    c_y[0] = a * X[0];
    c_y[1] = a * -X[1];
    c_y[2] = a * -X[2];
    c_y[3] = a * -X[3];
    c_y[4] = a * X[1];
    c_y[5] = a * X[0];
    c_y[6] = a * -X[3];
    c_y[7] = a * X[2];
    c_y[8] = a * X[2];
    c_y[9] = a * X[3];
    c_y[10] = a * X[0];
    c_y[11] = a * -X[1];
    c_y[12] = a * X[3];
    c_y[13] = a * -X[2];
    c_y[14] = a * X[1];
    c_y[15] = a * X[0];
    d_y[0] = coeff_b * X[0];
    d_y[1] = coeff_b * -X[1];
    d_y[2] = coeff_b * -X[2];
    d_y[3] = coeff_b * -X[3];
    d_y[4] = coeff_b * X[1];
    d_y[5] = coeff_b * X[0];
    d_y[6] = coeff_b * -X[3];
    d_y[7] = coeff_b * X[2];
    d_y[8] = coeff_b * X[2];
    d_y[9] = coeff_b * X[3];
    d_y[10] = coeff_b * X[0];
    d_y[11] = coeff_b * -X[1];
    d_y[12] = coeff_b * X[3];
    d_y[13] = coeff_b * -X[2];
    d_y[14] = coeff_b * X[1];
    d_y[15] = coeff_b * X[0];
    for (i0 = 0; i0 < 3; i0++) {
      for (i = 0; i < 4; i++) {
        F_prev[(i0 + 10 * i) + 7] = 0.0F;
        c_dq[i0 + 3 * i] = 0.0F;
        for (i1 = 0; i1 < 4; i1++) {
          F_prev[(i0 + 10 * i) + 7] += dq[i0 + 3 * i1] * y[i1 + (i << 2)];
          c_dq[i0 + 3 * i] += (float)iv1[i0 + 3 * i1] * d_y[i1 + (i << 2)];
        }
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      for (i = 0; i < 3; i++) {
        dq[i0 + (i << 2)] = 0.0F;
        for (i1 = 0; i1 < 4; i1++) {
          dq[i0 + (i << 2)] += (float)iv1[i + 3 * i1] * c_y[i1 + (i0 << 2)];
        }
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (i = 0; i < 4; i++) {
        b_cov_gyro[i0 + 3 * i] = 0.0F;
        for (i1 = 0; i1 < 3; i1++) {
          b_cov_gyro[i0 + 3 * i] += cov_gyro[i0 + 3 * i1] * c_dq[i1 + 3 * i];
        }
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      for (i = 0; i < 4; i++) {
        Q[i0 + 10 * i] = 0.0F;
        for (i1 = 0; i1 < 3; i1++) {
          Q[i0 + 10 * i] += dq[i0 + (i1 << 2)] * b_cov_gyro[i1 + 3 * i];
        }
      }
    }
  }

  //  Set apriori state
  // 'QEKF:139' X_apriori(1:4) = q_apriori;
  for (i = 0; i < 4; i++) {
    X_apriori[i] = b_dq[i];
  }

  // 'QEKF:140' X_apriori(5:7) = omega_apriori;
  // 'QEKF:141' X_apriori(8:10) = gyro_bias_apriori;
  for (i = 0; i < 3; i++) {
    X_apriori[i + 4] = X[i + 4];
    X_apriori[i + 7] = gyro_bias[i];
  }

  //  Calculate apriori covariance of estimate error
  // 'QEKF:144' P_apriori = F_prev * P_prev * F_prev' + Q;
  for (i0 = 0; i0 < 10; i0++) {
    for (i = 0; i < 10; i++) {
      b_P_prev[i0 + 10 * i] = 0.0F;
      for (i1 = 0; i1 < 10; i1++) {
        b_P_prev[i0 + 10 * i] += P_prev[i0 + 10 * i1] * F_prev[i1 + 10 * i];
      }
    }
  }

  for (i0 = 0; i0 < 10; i0++) {
    for (i = 0; i < 10; i++) {
      coeff_b = 0.0F;
      for (i1 = 0; i1 < 10; i1++) {
        coeff_b += F_prev[i1 + 10 * i0] * b_P_prev[i1 + 10 * i];
      }

      P_apriori[i0 + 10 * i] = coeff_b + Q[i0 + 10 * i];
    }
  }

  //     %% Update/correction step
  // 'QEKF:147' if (NormalizeAccelerometer)
  if (NormalizeAccelerometer) {
    //  Accelerometer Measurement model
    // 'QEKF:149' z_hat_acc = -devec * Phi(q_apriori)' * Gamma(q_apriori) * [0;0;0;-1]; 
    c_y[0] = b_dq[0];
    c_y[4] = -b_dq[1];
    c_y[8] = -b_dq[2];
    c_y[12] = -b_dq[3];
    c_y[1] = b_dq[1];
    c_y[5] = b_dq[0];
    c_y[9] = -b_dq[3];
    c_y[13] = b_dq[2];
    c_y[2] = b_dq[2];
    c_y[6] = b_dq[3];
    c_y[10] = b_dq[0];
    c_y[14] = -b_dq[1];
    c_y[3] = b_dq[3];
    c_y[7] = -b_dq[2];
    c_y[11] = b_dq[1];
    c_y[15] = b_dq[0];
    d_y[0] = b_dq[0];
    d_y[1] = -b_dq[1];
    d_y[2] = -b_dq[2];
    d_y[3] = -b_dq[3];
    d_y[4] = b_dq[1];
    d_y[5] = b_dq[0];
    d_y[6] = b_dq[3];
    d_y[7] = -b_dq[2];
    d_y[8] = b_dq[2];
    d_y[9] = -b_dq[3];
    d_y[10] = b_dq[0];
    d_y[11] = b_dq[1];
    d_y[12] = b_dq[3];
    d_y[13] = b_dq[2];
    d_y[14] = -b_dq[1];
    d_y[15] = b_dq[0];
    for (i0 = 0; i0 < 4; i0++) {
      for (i = 0; i < 3; i++) {
        dq[i0 + (i << 2)] = 0.0F;
        for (i1 = 0; i1 < 4; i1++) {
          dq[i0 + (i << 2)] += c_y[i0 + (i1 << 2)] * (float)iv2[i1 + (i << 2)];
        }
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      for (i = 0; i < 3; i++) {
        c_dq[i0 + (i << 2)] = 0.0F;
        for (i1 = 0; i1 < 4; i1++) {
          c_dq[i0 + (i << 2)] += d_y[i0 + (i1 << 2)] * dq[i1 + (i << 2)];
        }
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      z_acc[i0] = 0.0F;
      for (i = 0; i < 4; i++) {
        z_acc[i0] += (float)iv3[i] * c_dq[i + (i0 << 2)];
      }

      gyro_input[i0] = z_acc[i0];
    }

    //  Measurement Jacobian
    // 'QEKF:152' H_acc = [-devec * (Gamma(Gamma(q_apriori)*[0;0;0;-1])*I_conj + Phi(Phi(q_apriori)'*[0;0;0;-1])), ... 
    // 'QEKF:153'                  zeros(3,3), ...
    // 'QEKF:154'                  zeros(3,3)];
    c_y[0] = b_dq[0];
    c_y[1] = -b_dq[1];
    c_y[2] = -b_dq[2];
    c_y[3] = -b_dq[3];
    c_y[4] = b_dq[1];
    c_y[5] = b_dq[0];
    c_y[6] = b_dq[3];
    c_y[7] = -b_dq[2];
    c_y[8] = b_dq[2];
    c_y[9] = -b_dq[3];
    c_y[10] = b_dq[0];
    c_y[11] = b_dq[1];
    c_y[12] = b_dq[3];
    c_y[13] = b_dq[2];
    c_y[14] = -b_dq[1];
    c_y[15] = b_dq[0];
    d_y[0] = b_dq[0];
    d_y[4] = -b_dq[1];
    d_y[8] = -b_dq[2];
    d_y[12] = -b_dq[3];
    d_y[1] = b_dq[1];
    d_y[5] = b_dq[0];
    d_y[9] = -b_dq[3];
    d_y[13] = b_dq[2];
    d_y[2] = b_dq[2];
    d_y[6] = b_dq[3];
    d_y[10] = b_dq[0];
    d_y[14] = -b_dq[1];
    d_y[3] = b_dq[3];
    d_y[7] = -b_dq[2];
    d_y[11] = b_dq[1];
    d_y[15] = b_dq[0];
    for (i0 = 0; i0 < 4; i0++) {
      b_X[i0] = 0.0F;
      for (i = 0; i < 4; i++) {
        b_X[i0] += (float)iv3[i] * c_y[i + (i0 << 2)];
      }

      b_y[i0] = b_X[i0];
      fv2[i0] = 0.0F;
      for (i = 0; i < 4; i++) {
        fv2[i0] += (float)iv3[i] * d_y[i + (i0 << 2)];
      }

      q[i0] = fv2[i0];
    }

    y[0] = b_y[0];
    y[1] = -b_y[1];
    y[2] = -b_y[2];
    y[3] = -b_y[3];
    y[4] = b_y[1];
    y[5] = b_y[0];
    y[6] = b_y[3];
    y[7] = -b_y[2];
    y[8] = b_y[2];
    y[9] = -b_y[3];
    y[10] = b_y[0];
    y[11] = b_y[1];
    y[12] = b_y[3];
    y[13] = b_y[2];
    y[14] = -b_y[1];
    y[15] = b_y[0];
    for (i0 = 0; i0 < 4; i0++) {
      for (i = 0; i < 4; i++) {
        fv0[i0 + (i << 2)] = 0.0F;
        for (i1 = 0; i1 < 4; i1++) {
          fv0[i0 + (i << 2)] += (float)iv4[i0 + (i1 << 2)] * y[i1 + (i << 2)];
        }
      }
    }

    c_y[0] = q[0];
    c_y[1] = -q[1];
    c_y[2] = -q[2];
    c_y[3] = -q[3];
    c_y[4] = q[1];
    c_y[5] = q[0];
    c_y[6] = -q[3];
    c_y[7] = q[2];
    c_y[8] = q[2];
    c_y[9] = q[3];
    c_y[10] = q[0];
    c_y[11] = -q[1];
    c_y[12] = q[3];
    c_y[13] = -q[2];
    c_y[14] = q[1];
    c_y[15] = q[0];
    for (i0 = 0; i0 < 4; i0++) {
      for (i = 0; i < 4; i++) {
        d_y[i + (i0 << 2)] = fv0[i + (i0 << 2)] + c_y[i + (i0 << 2)];
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      for (i = 0; i < 3; i++) {
        dq[i0 + (i << 2)] = 0.0F;
        for (i1 = 0; i1 < 4; i1++) {
          dq[i0 + (i << 2)] += d_y[i0 + (i1 << 2)] * (float)iv2[i1 + (i << 2)];
        }
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (i = 0; i < 4; i++) {
        H_acc[i + 10 * i0] = dq[i + (i0 << 2)];
      }

      for (i = 0; i < 3; i++) {
        H_acc[(i + 10 * i0) + 4] = 0.0F;
        H_acc[(i + 10 * i0) + 7] = 0.0F;
      }
    }
  } else {
    // 'QEKF:155' else
    //  Accelerometer Measurement model
    // 'QEKF:157' z_hat_acc = -devec * Phi(q_apriori)' * Gamma(q_apriori) * [0;0;0;-g]; 
    b_X[0] = 0.0F;
    b_X[1] = 0.0F;
    b_X[2] = 0.0F;
    b_X[3] = -g;
    c_y[0] = b_dq[0];
    c_y[4] = -b_dq[1];
    c_y[8] = -b_dq[2];
    c_y[12] = -b_dq[3];
    c_y[1] = b_dq[1];
    c_y[5] = b_dq[0];
    c_y[9] = -b_dq[3];
    c_y[13] = b_dq[2];
    c_y[2] = b_dq[2];
    c_y[6] = b_dq[3];
    c_y[10] = b_dq[0];
    c_y[14] = -b_dq[1];
    c_y[3] = b_dq[3];
    c_y[7] = -b_dq[2];
    c_y[11] = b_dq[1];
    c_y[15] = b_dq[0];
    d_y[0] = b_dq[0];
    d_y[1] = -b_dq[1];
    d_y[2] = -b_dq[2];
    d_y[3] = -b_dq[3];
    d_y[4] = b_dq[1];
    d_y[5] = b_dq[0];
    d_y[6] = b_dq[3];
    d_y[7] = -b_dq[2];
    d_y[8] = b_dq[2];
    d_y[9] = -b_dq[3];
    d_y[10] = b_dq[0];
    d_y[11] = b_dq[1];
    d_y[12] = b_dq[3];
    d_y[13] = b_dq[2];
    d_y[14] = -b_dq[1];
    d_y[15] = b_dq[0];
    for (i0 = 0; i0 < 4; i0++) {
      for (i = 0; i < 3; i++) {
        dq[i0 + (i << 2)] = 0.0F;
        for (i1 = 0; i1 < 4; i1++) {
          dq[i0 + (i << 2)] += c_y[i0 + (i1 << 2)] * (float)iv2[i1 + (i << 2)];
        }
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      for (i = 0; i < 3; i++) {
        c_dq[i0 + (i << 2)] = 0.0F;
        for (i1 = 0; i1 < 4; i1++) {
          c_dq[i0 + (i << 2)] += d_y[i0 + (i1 << 2)] * dq[i1 + (i << 2)];
        }
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      z_acc[i0] = 0.0F;
      for (i = 0; i < 4; i++) {
        z_acc[i0] += b_X[i] * c_dq[i + (i0 << 2)];
      }

      gyro_input[i0] = z_acc[i0];
    }

    //  Measurement Jacobian
    // 'QEKF:160' H_acc = [-devec * (Gamma(Gamma(q_apriori)*[0;0;0;-g])*I_conj + Phi(Phi(q_apriori)'*[0;0;0;-g])), ... 
    // 'QEKF:161'                  zeros(3,3), ...
    // 'QEKF:162'                  zeros(3,3)];
    b_X[0] = 0.0F;
    b_X[1] = 0.0F;
    b_X[2] = 0.0F;
    b_X[3] = -g;
    c_y[0] = b_dq[0];
    c_y[1] = -b_dq[1];
    c_y[2] = -b_dq[2];
    c_y[3] = -b_dq[3];
    c_y[4] = b_dq[1];
    c_y[5] = b_dq[0];
    c_y[6] = b_dq[3];
    c_y[7] = -b_dq[2];
    c_y[8] = b_dq[2];
    c_y[9] = -b_dq[3];
    c_y[10] = b_dq[0];
    c_y[11] = b_dq[1];
    c_y[12] = b_dq[3];
    c_y[13] = b_dq[2];
    c_y[14] = -b_dq[1];
    c_y[15] = b_dq[0];
    fv1[0] = 0.0F;
    fv1[1] = 0.0F;
    fv1[2] = 0.0F;
    fv1[3] = -g;
    d_y[0] = b_dq[0];
    d_y[4] = -b_dq[1];
    d_y[8] = -b_dq[2];
    d_y[12] = -b_dq[3];
    d_y[1] = b_dq[1];
    d_y[5] = b_dq[0];
    d_y[9] = -b_dq[3];
    d_y[13] = b_dq[2];
    d_y[2] = b_dq[2];
    d_y[6] = b_dq[3];
    d_y[10] = b_dq[0];
    d_y[14] = -b_dq[1];
    d_y[3] = b_dq[3];
    d_y[7] = -b_dq[2];
    d_y[11] = b_dq[1];
    d_y[15] = b_dq[0];
    for (i0 = 0; i0 < 4; i0++) {
      fv2[i0] = 0.0F;
      for (i = 0; i < 4; i++) {
        fv2[i0] += b_X[i] * c_y[i + (i0 << 2)];
      }

      b_y[i0] = fv2[i0];
      fv3[i0] = 0.0F;
      for (i = 0; i < 4; i++) {
        fv3[i0] += fv1[i] * d_y[i + (i0 << 2)];
      }

      q[i0] = fv3[i0];
    }

    y[0] = b_y[0];
    y[1] = -b_y[1];
    y[2] = -b_y[2];
    y[3] = -b_y[3];
    y[4] = b_y[1];
    y[5] = b_y[0];
    y[6] = b_y[3];
    y[7] = -b_y[2];
    y[8] = b_y[2];
    y[9] = -b_y[3];
    y[10] = b_y[0];
    y[11] = b_y[1];
    y[12] = b_y[3];
    y[13] = b_y[2];
    y[14] = -b_y[1];
    y[15] = b_y[0];
    for (i0 = 0; i0 < 4; i0++) {
      for (i = 0; i < 4; i++) {
        fv0[i0 + (i << 2)] = 0.0F;
        for (i1 = 0; i1 < 4; i1++) {
          fv0[i0 + (i << 2)] += (float)iv4[i0 + (i1 << 2)] * y[i1 + (i << 2)];
        }
      }
    }

    c_y[0] = q[0];
    c_y[1] = -q[1];
    c_y[2] = -q[2];
    c_y[3] = -q[3];
    c_y[4] = q[1];
    c_y[5] = q[0];
    c_y[6] = -q[3];
    c_y[7] = q[2];
    c_y[8] = q[2];
    c_y[9] = q[3];
    c_y[10] = q[0];
    c_y[11] = -q[1];
    c_y[12] = q[3];
    c_y[13] = -q[2];
    c_y[14] = q[1];
    c_y[15] = q[0];
    for (i0 = 0; i0 < 4; i0++) {
      for (i = 0; i < 4; i++) {
        d_y[i + (i0 << 2)] = fv0[i + (i0 << 2)] + c_y[i + (i0 << 2)];
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      for (i = 0; i < 3; i++) {
        dq[i0 + (i << 2)] = 0.0F;
        for (i1 = 0; i1 < 4; i1++) {
          dq[i0 + (i << 2)] += d_y[i0 + (i1 << 2)] * (float)iv2[i1 + (i << 2)];
        }
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (i = 0; i < 4; i++) {
        H_acc[i + 10 * i0] = dq[i + (i0 << 2)];
      }

      for (i = 0; i < 3; i++) {
        H_acc[(i + 10 * i0) + 4] = 0.0F;
        H_acc[(i + 10 * i0) + 7] = 0.0F;
      }
    }
  }

  // 'QEKF:165' z_hat_gyro = omega_apriori + gyro_bias_apriori;
  // 'QEKF:166' H_gyro = [zeros(3,4), ...
  // 'QEKF:167'               eye(3), ...
  // 'QEKF:168'               BiasEstimationEnabledMat];
  // 'QEKF:169' z_hat = [z_hat_acc; z_hat_gyro];
  // 'QEKF:170' H = [H_acc; H_gyro];
  for (i = 0; i < 3; i++) {
    z_hat[i] = gyro_input[i];
    z_hat[i + 3] = X[i + 4] + gyro_bias[i];
    for (i0 = 0; i0 < 10; i0++) {
      H[i0 + 10 * i] = H_acc[i0 + 10 * i];
    }

    for (i0 = 0; i0 < 4; i0++) {
      H[i0 + 10 * (i + 3)] = 0.0F;
    }

    for (i0 = 0; i0 < 3; i0++) {
      H[(i0 + 10 * (i + 3)) + 4] = iv0[i0 + 3 * i];
      H[(i0 + 10 * (i + 3)) + 7] = BiasEstimationEnabledMat[i0 + 3 * i];
    }
  }

  // 'QEKF:172' if (UseHeadingForCorrection)
  if (UseHeadingForCorrection) {
    // 'QEKF:173' z_heading = Heading;
    // 'QEKF:174' heading_vector_hat = [zeros(2,1),eye(2),zeros(2,1)] * Phi(q_apriori) * Gamma(q_apriori)' * [0;1;0;0]; 
    c_y[0] = b_dq[0];
    c_y[1] = -b_dq[1];
    c_y[2] = -b_dq[2];
    c_y[3] = -b_dq[3];
    c_y[4] = b_dq[1];
    c_y[5] = b_dq[0];
    c_y[6] = -b_dq[3];
    c_y[7] = b_dq[2];
    c_y[8] = b_dq[2];
    c_y[9] = b_dq[3];
    c_y[10] = b_dq[0];
    c_y[11] = -b_dq[1];
    c_y[12] = b_dq[3];
    c_y[13] = -b_dq[2];
    c_y[14] = b_dq[1];
    c_y[15] = b_dq[0];
    d_y[0] = b_dq[0];
    d_y[4] = -b_dq[1];
    d_y[8] = -b_dq[2];
    d_y[12] = -b_dq[3];
    d_y[1] = b_dq[1];
    d_y[5] = b_dq[0];
    d_y[9] = b_dq[3];
    d_y[13] = -b_dq[2];
    d_y[2] = b_dq[2];
    d_y[6] = -b_dq[3];
    d_y[10] = b_dq[0];
    d_y[14] = b_dq[1];
    d_y[3] = b_dq[3];
    d_y[7] = b_dq[2];
    d_y[11] = -b_dq[1];
    d_y[15] = b_dq[0];
    for (i0 = 0; i0 < 4; i0++) {
      for (i = 0; i < 2; i++) {
        d_dq[i0 + (i << 2)] = 0.0F;
        for (i1 = 0; i1 < 4; i1++) {
          d_dq[i0 + (i << 2)] += c_y[i0 + (i1 << 2)] * (float)iv5[i1 + (i << 2)];
        }
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      for (i = 0; i < 2; i++) {
        e_dq[i0 + (i << 2)] = 0.0F;
        for (i1 = 0; i1 < 4; i1++) {
          e_dq[i0 + (i << 2)] += d_y[i0 + (i1 << 2)] * d_dq[i1 + (i << 2)];
        }
      }
    }

    for (i0 = 0; i0 < 2; i0++) {
      heading_vector_hat[i0] = 0.0F;
      for (i = 0; i < 4; i++) {
        heading_vector_hat[i0] += (float)iv6[i] * e_dq[i + (i0 << 2)];
      }

      b_heading_vector_hat[i0] = heading_vector_hat[i0];
    }

    //  estimated measurement
    // 'QEKF:175' z_hat_heading = atan2(heading_vector_hat(2), heading_vector_hat(1)); 
    //  d atan2(y,x) / dx = -y / (x^2+y^2)
    //  d atan2(y,x) / dy = x / (x^2+y^2)
    //  d heading_hat / d q = (d atan2(y,x) / dx)*(d heading_vector_hat(1) / dq) + (d atan2(y,x) / dy)*(d heading_vector_hat(2) / dq) 
    // 'QEKF:181' H_heading_vector = [[zeros(2,1),eye(2),zeros(2,1)] * (Phi(Phi(q_apriori)*[0;1;0;0])*I_conj + Gamma(Gamma(q_apriori)'*[0;1;0;0])), zeros(2,3), zeros(2,3)]; 
    c_y[0] = b_dq[0];
    c_y[1] = -b_dq[1];
    c_y[2] = -b_dq[2];
    c_y[3] = -b_dq[3];
    c_y[4] = b_dq[1];
    c_y[5] = b_dq[0];
    c_y[6] = -b_dq[3];
    c_y[7] = b_dq[2];
    c_y[8] = b_dq[2];
    c_y[9] = b_dq[3];
    c_y[10] = b_dq[0];
    c_y[11] = -b_dq[1];
    c_y[12] = b_dq[3];
    c_y[13] = -b_dq[2];
    c_y[14] = b_dq[1];
    c_y[15] = b_dq[0];
    d_y[0] = b_dq[0];
    d_y[4] = -b_dq[1];
    d_y[8] = -b_dq[2];
    d_y[12] = -b_dq[3];
    d_y[1] = b_dq[1];
    d_y[5] = b_dq[0];
    d_y[9] = b_dq[3];
    d_y[13] = -b_dq[2];
    d_y[2] = b_dq[2];
    d_y[6] = -b_dq[3];
    d_y[10] = b_dq[0];
    d_y[14] = b_dq[1];
    d_y[3] = b_dq[3];
    d_y[7] = b_dq[2];
    d_y[11] = -b_dq[1];
    d_y[15] = b_dq[0];
    for (i0 = 0; i0 < 4; i0++) {
      b_X[i0] = 0.0F;
      for (i = 0; i < 4; i++) {
        b_X[i0] += (float)iv6[i] * c_y[i + (i0 << 2)];
      }

      b_y[i0] = b_X[i0];
      fv2[i0] = 0.0F;
      for (i = 0; i < 4; i++) {
        fv2[i0] += (float)iv6[i] * d_y[i + (i0 << 2)];
      }

      q[i0] = fv2[i0];
    }

    // 'QEKF:182' dAtan2dxy = [-heading_vector_hat(2) / (heading_vector_hat(1)^2+heading_vector_hat(2)^2), ... 
    // 'QEKF:183'                    heading_vector_hat(1) / (heading_vector_hat(1)^2+heading_vector_hat(2)^2)]; 
    // 'QEKF:184' H_heading = dAtan2dxy * H_heading_vector;
    //  Heading measurement covariance
    // 'QEKF:187' cov_heading = sigma2_heading;
    // 'QEKF:189' z2 = [z; z_heading];
    // 'QEKF:190' z2_hat = [z_hat; z_hat_heading];
    // 'QEKF:191' H2 = [H; H_heading];
    y[0] = b_y[0];
    y[1] = -b_y[1];
    y[2] = -b_y[2];
    y[3] = -b_y[3];
    y[4] = b_y[1];
    y[5] = b_y[0];
    y[6] = -b_y[3];
    y[7] = b_y[2];
    y[8] = b_y[2];
    y[9] = b_y[3];
    y[10] = b_y[0];
    y[11] = -b_y[1];
    y[12] = b_y[3];
    y[13] = -b_y[2];
    y[14] = b_y[1];
    y[15] = b_y[0];
    for (i0 = 0; i0 < 4; i0++) {
      for (i = 0; i < 4; i++) {
        fv0[i0 + (i << 2)] = 0.0F;
        for (i1 = 0; i1 < 4; i1++) {
          fv0[i0 + (i << 2)] += (float)iv4[i0 + (i1 << 2)] * y[i1 + (i << 2)];
        }
      }
    }

    c_y[0] = q[0];
    c_y[1] = -q[1];
    c_y[2] = -q[2];
    c_y[3] = -q[3];
    c_y[4] = q[1];
    c_y[5] = q[0];
    c_y[6] = q[3];
    c_y[7] = -q[2];
    c_y[8] = q[2];
    c_y[9] = -q[3];
    c_y[10] = q[0];
    c_y[11] = q[1];
    c_y[12] = q[3];
    c_y[13] = q[2];
    c_y[14] = -q[1];
    c_y[15] = q[0];
    for (i0 = 0; i0 < 4; i0++) {
      for (i = 0; i < 4; i++) {
        d_y[i + (i0 << 2)] = fv0[i + (i0 << 2)] + c_y[i + (i0 << 2)];
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      for (i = 0; i < 2; i++) {
        d_dq[i0 + (i << 2)] = 0.0F;
        for (i1 = 0; i1 < 4; i1++) {
          d_dq[i0 + (i << 2)] += d_y[i0 + (i1 << 2)] * (float)iv5[i1 + (i << 2)];
        }
      }
    }

    for (i0 = 0; i0 < 2; i0++) {
      for (i = 0; i < 4; i++) {
        fv4[i + 10 * i0] = d_dq[i + (i0 << 2)];
      }

      for (i = 0; i < 3; i++) {
        fv4[(i + 10 * i0) + 4] = 0.0F;
        fv4[(i + 10 * i0) + 7] = 0.0F;
      }
    }

    heading_vector_hat[0] = -b_heading_vector_hat[1] / (b_heading_vector_hat[0] *
      b_heading_vector_hat[0] + b_heading_vector_hat[1] * b_heading_vector_hat[1]);
    heading_vector_hat[1] = b_heading_vector_hat[0] / (b_heading_vector_hat[0] *
      b_heading_vector_hat[0] + b_heading_vector_hat[1] * b_heading_vector_hat[1]);
    for (i0 = 0; i0 < 10; i0++) {
      c_z[i0] = 0.0F;
      for (i = 0; i < 2; i++) {
        c_z[i0] += fv4[i0 + 10 * i] * heading_vector_hat[i];
      }
    }

    for (i0 = 0; i0 < 6; i0++) {
      for (i = 0; i < 10; i++) {
        H2[i + 10 * i0] = H[i + 10 * i0];
      }
    }

    for (i0 = 0; i0 < 10; i0++) {
      H2[60 + i0] = c_z[i0];
    }

    // 'QEKF:192' R2 = [R, zeros(6,1); zeros(1,6), cov_heading];
    //  Calculate Kalman gain
    // 'QEKF:195' S2 = H2 * P_apriori * H2' + R2;
    // 'QEKF:196' K2 = P_apriori * H2' / S2;
    for (i0 = 0; i0 < 7; i0++) {
      for (i = 0; i < 10; i++) {
        b_H2[i0 + 7 * i] = 0.0F;
        for (i1 = 0; i1 < 10; i1++) {
          b_H2[i0 + 7 * i] += H2[i1 + 10 * i0] * P_apriori[i1 + 10 * i];
        }
      }
    }

    for (i0 = 0; i0 < 10; i0++) {
      for (i = 0; i < 7; i++) {
        K2[i0 + 10 * i] = 0.0F;
        for (i1 = 0; i1 < 10; i1++) {
          K2[i0 + 10 * i] += P_apriori[i0 + 10 * i1] * H2[i1 + 10 * i];
        }
      }
    }

    for (i0 = 0; i0 < 7; i0++) {
      for (i = 0; i < 7; i++) {
        c_H2[i0 + 7 * i] = 0.0F;
        for (i1 = 0; i1 < 10; i1++) {
          c_H2[i0 + 7 * i] += H2[i1 + 10 * i0] * K2[i1 + 10 * i];
        }
      }
    }

    for (i0 = 0; i0 < 6; i0++) {
      for (i = 0; i < 6; i++) {
        b_R[i + 7 * i0] = R[i + 6 * i0];
      }

      b_R[6 + 7 * i0] = 0.0F;
      b_R[42 + i0] = 0.0F;
    }

    b_R[48] = sigma2_heading;
    for (i0 = 0; i0 < 7; i0++) {
      for (i = 0; i < 7; i++) {
        d_H2[i + 7 * i0] = c_H2[i + 7 * i0] + b_R[i + 7 * i0];
      }
    }

    mrdivide(b_H2, d_H2, K2);

    //  P_apriori * H2' * inv(S2)
    // if (NormalizeAccelerometer && (norm_acc <= 0))
    //     K2 = single(zeros(size(K2))); % if there is no accelerometer measurement, then no correction will be performed 
    // end
    //  Correct using innovation
    // 'QEKF:203' X_aposteriori = X_apriori + K2 * (z2 - z2_hat);
    d_z[6] = Heading;
    for (i0 = 0; i0 < 6; i0++) {
      d_z[i0] = z[i0];
      b_z_hat[i0] = z_hat[i0];
    }

    b_z_hat[6] = rt_atan2f_snf(b_heading_vector_hat[1], b_heading_vector_hat[0]);
    for (i0 = 0; i0 < 7; i0++) {
      e_z[i0] = d_z[i0] - b_z_hat[i0];
    }

    // 'QEKF:204' P_aposteriori = (eye(10) - K2*H2) * P_apriori;
    c_eye(dv2);
    for (i0 = 0; i0 < 10; i0++) {
      c_z[i0] = 0.0F;
      for (i = 0; i < 7; i++) {
        c_z[i0] += e_z[i] * K2[i + 7 * i0];
      }

      X_out[i0] = X_apriori[i0] + c_z[i0];
      for (i = 0; i < 10; i++) {
        coeff_b = 0.0F;
        for (i1 = 0; i1 < 7; i1++) {
          coeff_b += H2[i0 + 10 * i1] * K2[i1 + 7 * i];
        }

        Q[i0 + 10 * i] = (float)dv2[i0 + 10 * i] - coeff_b;
      }
    }

    for (i0 = 0; i0 < 10; i0++) {
      for (i = 0; i < 10; i++) {
        P_out[i0 + 10 * i] = 0.0F;
        for (i1 = 0; i1 < 10; i1++) {
          P_out[i0 + 10 * i] += P_apriori[i0 + 10 * i1] * Q[i1 + 10 * i];
        }
      }
    }
  } else {
    // 'QEKF:205' else
    //  Calculate Kalman gain
    // 'QEKF:207' S = H * P_apriori * H' + R;
    // 'QEKF:208' K = P_apriori * H' / S;
    for (i0 = 0; i0 < 6; i0++) {
      for (i = 0; i < 10; i++) {
        b_H[i0 + 6 * i] = 0.0F;
        for (i1 = 0; i1 < 10; i1++) {
          b_H[i0 + 6 * i] += H[i1 + 10 * i0] * P_apriori[i1 + 10 * i];
        }
      }
    }

    for (i0 = 0; i0 < 10; i0++) {
      for (i = 0; i < 6; i++) {
        K[i0 + 10 * i] = 0.0F;
        for (i1 = 0; i1 < 10; i1++) {
          K[i0 + 10 * i] += P_apriori[i0 + 10 * i1] * H[i1 + 10 * i];
        }
      }
    }

    for (i0 = 0; i0 < 6; i0++) {
      for (i = 0; i < 6; i++) {
        coeff_b = 0.0F;
        for (i1 = 0; i1 < 10; i1++) {
          coeff_b += H[i1 + 10 * i0] * K[i1 + 10 * i];
        }

        c_H[i0 + 6 * i] = coeff_b + R[i0 + 6 * i];
      }
    }

    b_mrdivide(b_H, c_H, K);

    //  P_apriori * H' * inv(S)
    // if (NormalizeAccelerometer && (norm_acc <= 0))
    //     K = single(zeros(size(K))); % if there is no accelerometer measurement, then no correction will be performed 
    // end
    // 'QEKF:214' X_aposteriori = X_apriori + K * (z - z_hat);
    for (i0 = 0; i0 < 6; i0++) {
      b_z[i0] = z[i0] - z_hat[i0];
    }

    // 'QEKF:215' P_aposteriori = (eye(10) - K*H) * P_apriori;
    c_eye(dv2);
    for (i0 = 0; i0 < 10; i0++) {
      c_z[i0] = 0.0F;
      for (i = 0; i < 6; i++) {
        c_z[i0] += b_z[i] * K[i + 6 * i0];
      }

      X_out[i0] = X_apriori[i0] + c_z[i0];
      for (i = 0; i < 10; i++) {
        coeff_b = 0.0F;
        for (i1 = 0; i1 < 6; i1++) {
          coeff_b += H[i0 + 10 * i1] * K[i1 + 6 * i];
        }

        Q[i0 + 10 * i] = (float)dv2[i0 + 10 * i] - coeff_b;
      }
    }

    for (i0 = 0; i0 < 10; i0++) {
      for (i = 0; i < 10; i++) {
        P_out[i0 + 10 * i] = 0.0F;
        for (i1 = 0; i1 < 10; i1++) {
          P_out[i0 + 10 * i] += P_apriori[i0 + 10 * i1] * Q[i1 + 10 * i];
        }
      }
    }
  }

  //     %% Normalize quaternion
  // 'QEKF:219' X_aposteriori(1:4) = X_aposteriori(1:4) / norm(X_aposteriori(1:4)); 
  coeff_b = b_norm(*(float (*)[4])&X_out[0]);
  for (i0 = 0; i0 < 4; i0++) {
    X_out[i0] /= coeff_b;
  }

  //     %% Send output to Simulink
  // 'QEKF:222' X_out = X_aposteriori;
  // 'QEKF:223' P_out = P_aposteriori;
}

//
// Arguments    : void
// Return Type  : void
//
void acc_norm_filtered_not_empty_init()
{
  acc_norm_filtered_not_empty = false;
}

//
// Arguments    : void
// Return Type  : void
//
void acc_norm_old_not_empty_init()
{
  acc_norm_old_not_empty = false;
}

//
// File trailer for QEKF.cpp
//
// [EOF]
//
