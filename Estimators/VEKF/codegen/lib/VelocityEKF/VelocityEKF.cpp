//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: VelocityEKF.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 26-Apr-2019 13:40:20
//

// Include Files
#include <string.h>
#include "rt_nonfinite.h"
#include "VelocityEKF.h"
#include "mrdivide.h"

// Function Definitions

//
// function [X_out, P_out] = VelocityEKF(X, P_prev, EncoderDiffMeas, eta_encoder, Accelerometer, cov_acc, eta_accelerometer, eta_bias, qQEKF, cov_qQEKF, qdotQEKF, eta_acceleration, SamplePeriod, TicksPrRev, rk,rw,g)
// for q o p = Phi(q) * p
// Arguments    : const float X[7]
//                const float P_prev[49]
//                const float EncoderDiffMeas[3]
//                float eta_encoder
//                const float Accelerometer[3]
//                const float cov_acc[9]
//                float eta_accelerometer
//                float eta_bias
//                const float qQEKF[4]
//                const float cov_qQEKF[16]
//                const float qdotQEKF[4]
//                float eta_acceleration
//                float SamplePeriod
//                float TicksPrRev
//                float rk
//                float rw
//                float g
//                float X_out[7]
//                float P_out[49]
// Return Type  : void
//
void VelocityEKF(const float X[7], const float P_prev[49], const float
                 EncoderDiffMeas[3], float eta_encoder, const float
                 Accelerometer[3], const float cov_acc[9], float
                 eta_accelerometer, float eta_bias, const float qQEKF[4], const
                 float cov_qQEKF[16], const float qdotQEKF[4], float
                 eta_acceleration, float SamplePeriod, float TicksPrRev, float
                 rk, float rw, float g, float X_out[7], float P_out[49])
{
  float y;
  float b_y;
  float c_y;
  int i0;
  float fv0[3];
  int j;
  float fv1[4];
  static const float fv2[9] = { 0.0F, 1.0F, 0.0F, -0.866025388F, -0.5F, 0.0F,
    0.866025388F, -0.5F, 0.0F };

  static const signed char iv0[3] = { 1, 0, 0 };

  static const signed char iv1[12] = { 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

  float fv3[4];
  static const signed char iv2[3] = { 0, 1, 0 };

  float fv4[4];
  static const signed char iv3[3] = { 0, 0, 1 };

  float fv5[4];
  float b_qdotQEKF[4];
  float fv6[4];
  float W[12];
  static const float fv7[16] = { 0.0F, 0.353553385F, 0.612372458F, -0.707106769F,
    -0.353553385F, 0.0F, 0.707106769F, 0.612372458F, -0.612372458F,
    -0.707106769F, 0.0F, -0.353553385F, 0.707106769F, -0.612372458F,
    0.353553385F, 0.0F };

  static const float fv8[16] = { 0.0F, -0.707106769F, -0.0F, -0.707106769F,
    0.707106769F, 0.0F, 0.707106769F, -0.0F, 0.0F, -0.707106769F, 0.0F,
    0.707106769F, 0.707106769F, 0.0F, -0.707106769F, 0.0F };

  float dx_apriori;
  static const float fv9[16] = { 0.0F, 0.353553385F, -0.612372458F,
    -0.707106769F, -0.353553385F, 0.0F, 0.707106769F, -0.612372458F,
    0.612372458F, -0.707106769F, 0.0F, -0.353553385F, 0.707106769F, 0.612372458F,
    0.353553385F, 0.0F };

  float dy_apriori;
  float v[7];
  static float Q[49];
  static double F_prev[49];
  static const signed char iv4[49] = { 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    1, 0, 0, 0, 0, 0, 0, 0, 1 };

  static const signed char iv5[4] = { 1, 0, 0, 1 };

  static float b_P_prev[49];
  int i1;
  static float P_apriori[49];
  float a;
  static double H[42];
  float b_qQEKF[16];
  float c_qQEKF[16];
  float b_a[12];
  float daccelerometer_dqQEKF[12];
  float d_qQEKF[12];
  static const signed char iv6[4] = { 0, 0, 1, 0 };

  static const signed char iv7[4] = { 0, -1, 0, 0 };

  static const signed char iv8[8] = { 0, 0, 1, 0, 0, 1, 0, 0 };

  float q[4];
  static const signed char iv9[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  float p[4];
  float d_y[16];
  float fv10[16];
  float fv11[12];
  float fv12[12];
  static const signed char iv10[16] = { 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0,
    0, 0, -1 };

  static float b_H[42];
  static float K[42];
  float c_H[36];
  float b_eta_accelerometer[9];
  float d_H[36];
  float e_y[36];
  static const float fv13[9] = { 0.5F, 0.0F, 0.0F, 0.0F, 0.5F, 0.0F, 0.0F, 0.0F,
    0.5F };

  float fv14[3];
  float b_EncoderDiffMeas[6];
  float c_EncoderDiffMeas[6];
  float c_a[6];
  float b_dx_apriori[7];

  // 'VelocityEKF:3' Phi = @(q)[q(1) -q(2) -q(3) -q(4);     % for q o p = Phi(q) * p 
  // 'VelocityEKF:4'               q(2) q(1)  -q(4) q(3);
  // 'VelocityEKF:5'               q(3) q(4)  q(1)  -q(2);
  // 'VelocityEKF:6'               q(4) -q(3) q(2)  q(1)];
  //  for q o p = Gamma(p) * q
  // 'VelocityEKF:7' Gamma = @(p)[p(1) -p(2) -p(3) -p(4);   % for q o p = Gamma(p) * q 
  // 'VelocityEKF:8'                  p(2) p(1) p(4) -p(3);
  // 'VelocityEKF:9'                  p(3) -p(4) p(1) p(2);
  // 'VelocityEKF:10'                  p(4) p(3) -p(2) p(1)];
  // 'VelocityEKF:12' devec = [0,1,0,0;0,0,1,0;0,0,0,1];
  //  'v' in notes
  // 'VelocityEKF:13' vec = [0,0,0;1,0,0;0,1,0;0,0,1];
  //  '^' in notes
  // 'VelocityEKF:14' I_conj = diag([1,-1,-1,-1]);
  // 'VelocityEKF:16' dt = SamplePeriod;
  // 'VelocityEKF:17' dx = X(1);
  // 'VelocityEKF:18' dy = X(2);
  // 'VelocityEKF:19' ddx = X(3);
  // 'VelocityEKF:20' ddy = X(4);
  // 'VelocityEKF:21' acc_bias = X(5:7);
  //  motor mapping (inverse kinematics)
  // 'VelocityEKF:24' alpha = deg2rad(45);
  // 'VelocityEKF:25' gamma = deg2rad(120);
  // 'VelocityEKF:27' e1 = [1,0,0]';
  // 'VelocityEKF:28' e2 = [0,1,0]';
  // 'VelocityEKF:29' e3 = [0,0,1]';
  // 'VelocityEKF:30' R_alpha_gamma = diag([cos(alpha) cos(alpha) sin(alpha)]) * [1 cos(gamma), cos(2*gamma); 0 sin(gamma) sin(2*gamma); 1, 1, 1]; 
  // 'VelocityEKF:31' R_gamma = [0 -sin(gamma) -sin(2*gamma); 1 cos(gamma), cos(2*gamma); 0, 0, 0]; 
  // 'VelocityEKF:33' W1 = rk/rw * e1' * R_gamma' * devec * Gamma(vec * R_alpha_gamma*e1); 
  y = rk / rw;

  // 'VelocityEKF:34' W2 = rk/rw * e2' * R_gamma' * devec * Gamma(vec * R_alpha_gamma*e2); 
  b_y = rk / rw;

  // 'VelocityEKF:35' W3 = rk/rw * e3' * R_gamma' * devec * Gamma(vec * R_alpha_gamma*e3); 
  c_y = rk / rw;

  // 'VelocityEKF:36' W = [W1;W2;W3];
  for (i0 = 0; i0 < 3; i0++) {
    fv0[i0] = 0.0F;
    for (j = 0; j < 3; j++) {
      fv0[i0] += fv2[i0 + 3 * j] * (y * (float)iv0[j]);
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    fv1[i0] = 0.0F;
    for (j = 0; j < 3; j++) {
      fv1[i0] += (float)iv1[i0 + (j << 2)] * fv0[j];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    fv0[i0] = 0.0F;
    for (j = 0; j < 3; j++) {
      fv0[i0] += fv2[i0 + 3 * j] * (b_y * (float)iv2[j]);
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    fv3[i0] = 0.0F;
    for (j = 0; j < 3; j++) {
      fv3[i0] += (float)iv1[i0 + (j << 2)] * fv0[j];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    fv0[i0] = 0.0F;
    for (j = 0; j < 3; j++) {
      fv0[i0] += fv2[i0 + 3 * j] * (c_y * (float)iv3[j]);
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    fv4[i0] = 0.0F;
    for (j = 0; j < 3; j++) {
      fv4[i0] += (float)iv1[i0 + (j << 2)] * fv0[j];
    }

    b_qdotQEKF[i0] = 0.0F;
    fv6[i0] = 0.0F;
    for (j = 0; j < 4; j++) {
      b_qdotQEKF[i0] += fv8[i0 + (j << 2)] * fv1[j];
      fv6[i0] += fv9[i0 + (j << 2)] * fv3[j];
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    fv5[i0] = 0.0F;
    for (j = 0; j < 4; j++) {
      fv5[i0] += fv7[i0 + (j << 2)] * fv4[j];
    }

    W[i0] = b_qdotQEKF[i0];
    W[4 + i0] = fv6[i0];
    W[8 + i0] = fv5[i0];
  }

  //     %% Prediction step
  // 'VelocityEKF:39' X_apriori = zeros(7,1);
  //  Rotate acceleration measurement from body frame into inertial frame
  // 'VelocityEKF:42' acceleration = devec * Phi(qQEKF) * Gamma(qQEKF)' * [0;Accelerometer]; 
  //  Propagate the velocity based on acceleration
  // 'VelocityEKF:45' dx_apriori = dx + dt * ddx;
  dx_apriori = X[0] + SamplePeriod * X[2];

  // 'VelocityEKF:46' dy_apriori = dy + dt * ddy;
  dy_apriori = X[1] + SamplePeriod * X[3];

  // 'VelocityEKF:48' ddx_apriori = ddx;
  // 'VelocityEKF:49' ddy_apriori = ddy;
  // 'VelocityEKF:51' acc_bias_apriori = acc_bias;
  //  Setup process covariance
  // 'VelocityEKF:54' Q = diag([zeros(1,2), eta_acceleration*ones(1,2), eta_bias*ones(1,3)]); 
  for (i0 = 0; i0 < 2; i0++) {
    v[i0] = 0.0F;
    v[i0 + 2] = eta_acceleration;
  }

  for (i0 = 0; i0 < 3; i0++) {
    v[i0 + 4] = eta_bias;
  }

  memset(&Q[0], 0, 49U * sizeof(float));
  for (j = 0; j < 7; j++) {
    Q[j + 7 * j] = v[j];
  }

  //  Determine model Jacobian (F)
  // 'VelocityEKF:57' F_prev = eye(7);
  for (i0 = 0; i0 < 49; i0++) {
    F_prev[i0] = iv4[i0];
  }

  // 'VelocityEKF:58' F_prev(1:2,3:4) = dt * eye(2);
  for (i0 = 0; i0 < 2; i0++) {
    for (j = 0; j < 2; j++) {
      F_prev[(j + 7 * i0) + 2] = SamplePeriod * (float)iv5[j + (i0 << 1)];
    }
  }

  //  Set apriori state
  // 'VelocityEKF:61' X_apriori = [dx_apriori
  // 'VelocityEKF:62' 				 dy_apriori;
  // 'VelocityEKF:63'                  ddx_apriori;
  // 'VelocityEKF:64'                  ddy_apriori;
  // 'VelocityEKF:65'                  acc_bias_apriori];
  //  Calculate apriori covariance of estimate error
  // 'VelocityEKF:68' P_apriori = F_prev * P_prev * F_prev' + Q;
  for (i0 = 0; i0 < 7; i0++) {
    for (j = 0; j < 7; j++) {
      b_P_prev[i0 + 7 * j] = 0.0F;
      for (i1 = 0; i1 < 7; i1++) {
        b_P_prev[i0 + 7 * j] += P_prev[i0 + 7 * i1] * (float)F_prev[i1 + 7 * j];
      }
    }
  }

  for (i0 = 0; i0 < 7; i0++) {
    for (j = 0; j < 7; j++) {
      c_y = 0.0F;
      for (i1 = 0; i1 < 7; i1++) {
        c_y += (float)F_prev[i1 + 7 * i0] * b_P_prev[i1 + 7 * j];
      }

      P_apriori[i0 + 7 * j] = c_y + Q[i0 + 7 * j];
    }
  }

  //     %% Update/correction step
  // 'VelocityEKF:71' z = [EncoderDiffMeas; Accelerometer];
  //  Encoder Measurement model
  // 'VelocityEKF:74' dx_ball_apriori = dx_apriori;
  // 'VelocityEKF:75' dy_ball_apriori = dy_apriori;
  // 'VelocityEKF:77' dpsi_apriori = W * (1/rk * Phi(qQEKF)' * Gamma(qQEKF) * [0;-dy_ball_apriori;dx_ball_apriori;0] - 2*Phi(qQEKF)'*qdotQEKF); 
  y = 1.0F / rk;

  //  InverseKinematics(qdotQEKF(1),qdotQEKF(2),qdotQEKF(3),qdotQEKF(4),dx_ball_apriori,dy_ball_apriori,qQEKF(1),qQEKF(2),qQEKF(3),qQEKF(4),rk,rw); 
  // 'VelocityEKF:78' z_encoder_hat = TicksPrRev/(2*pi) * dt * dpsi_apriori;
  a = TicksPrRev / 6.28318548F * SamplePeriod;

  //  Accelerometer measurement model
  // 'VelocityEKF:81' z_accelerometer_hat = devec * Phi(qQEKF)' * Gamma(qQEKF) * [0;ddx_apriori;ddy_apriori;g] + acc_bias_apriori; 
  //  Rotate acceleration from inertial frame into body frame
  //  Assemble measurement prediction vector
  // 'VelocityEKF:84' z_hat = [z_encoder_hat; z_accelerometer_hat];
  //  Measurement Jacobian	
  // 'VelocityEKF:87' H = zeros(6,7);
  memset(&H[0], 0, 42U * sizeof(double));

  // 'VelocityEKF:88' H(1:3,1) = TicksPrRev/(2*pi) * dt * W * 1/rk * Phi(qQEKF)' * Gamma(qQEKF) * [0;0;1;0]; 
  c_y = TicksPrRev / 6.28318548F * SamplePeriod;
  b_qQEKF[0] = qQEKF[0];
  b_qQEKF[4] = -qQEKF[1];
  b_qQEKF[8] = -qQEKF[2];
  b_qQEKF[12] = -qQEKF[3];
  b_qQEKF[1] = qQEKF[1];
  b_qQEKF[5] = qQEKF[0];
  b_qQEKF[9] = -qQEKF[3];
  b_qQEKF[13] = qQEKF[2];
  b_qQEKF[2] = qQEKF[2];
  b_qQEKF[6] = qQEKF[3];
  b_qQEKF[10] = qQEKF[0];
  b_qQEKF[14] = -qQEKF[1];
  b_qQEKF[3] = qQEKF[3];
  b_qQEKF[7] = -qQEKF[2];
  b_qQEKF[11] = qQEKF[1];
  b_qQEKF[15] = qQEKF[0];
  for (i0 = 0; i0 < 3; i0++) {
    for (j = 0; j < 4; j++) {
      b_a[j + (i0 << 2)] = c_y * W[j + (i0 << 2)] / rk;
    }
  }

  c_qQEKF[0] = qQEKF[0];
  c_qQEKF[1] = -qQEKF[1];
  c_qQEKF[2] = -qQEKF[2];
  c_qQEKF[3] = -qQEKF[3];
  c_qQEKF[4] = qQEKF[1];
  c_qQEKF[5] = qQEKF[0];
  c_qQEKF[6] = qQEKF[3];
  c_qQEKF[7] = -qQEKF[2];
  c_qQEKF[8] = qQEKF[2];
  c_qQEKF[9] = -qQEKF[3];
  c_qQEKF[10] = qQEKF[0];
  c_qQEKF[11] = qQEKF[1];
  c_qQEKF[12] = qQEKF[3];
  c_qQEKF[13] = qQEKF[2];
  c_qQEKF[14] = -qQEKF[1];
  c_qQEKF[15] = qQEKF[0];
  for (i0 = 0; i0 < 4; i0++) {
    for (j = 0; j < 3; j++) {
      daccelerometer_dqQEKF[i0 + (j << 2)] = 0.0F;
      for (i1 = 0; i1 < 4; i1++) {
        daccelerometer_dqQEKF[i0 + (j << 2)] += b_qQEKF[i0 + (i1 << 2)] * b_a[i1
          + (j << 2)];
      }
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    for (j = 0; j < 3; j++) {
      d_qQEKF[i0 + (j << 2)] = 0.0F;
      for (i1 = 0; i1 < 4; i1++) {
        d_qQEKF[i0 + (j << 2)] += c_qQEKF[i0 + (i1 << 2)] *
          daccelerometer_dqQEKF[i1 + (j << 2)];
      }
    }
  }

  //  d encoder_meas  /  d dx_2L
  // 'VelocityEKF:89' H(1:3,2) = TicksPrRev/(2*pi) * dt * W * 1/rk * Phi(qQEKF)' * Gamma(qQEKF) * [0;-1;0;0]; 
  c_y = TicksPrRev / 6.28318548F * SamplePeriod;
  b_qQEKF[0] = qQEKF[0];
  b_qQEKF[4] = -qQEKF[1];
  b_qQEKF[8] = -qQEKF[2];
  b_qQEKF[12] = -qQEKF[3];
  b_qQEKF[1] = qQEKF[1];
  b_qQEKF[5] = qQEKF[0];
  b_qQEKF[9] = -qQEKF[3];
  b_qQEKF[13] = qQEKF[2];
  b_qQEKF[2] = qQEKF[2];
  b_qQEKF[6] = qQEKF[3];
  b_qQEKF[10] = qQEKF[0];
  b_qQEKF[14] = -qQEKF[1];
  b_qQEKF[3] = qQEKF[3];
  b_qQEKF[7] = -qQEKF[2];
  b_qQEKF[11] = qQEKF[1];
  b_qQEKF[15] = qQEKF[0];
  for (i0 = 0; i0 < 3; i0++) {
    fv0[i0] = 0.0F;
    for (j = 0; j < 4; j++) {
      b_a[j + (i0 << 2)] = c_y * W[j + (i0 << 2)] / rk;
      fv0[i0] += (float)iv6[j] * d_qQEKF[j + (i0 << 2)];
    }

    H[7 * i0] = fv0[i0];
  }

  c_qQEKF[0] = qQEKF[0];
  c_qQEKF[1] = -qQEKF[1];
  c_qQEKF[2] = -qQEKF[2];
  c_qQEKF[3] = -qQEKF[3];
  c_qQEKF[4] = qQEKF[1];
  c_qQEKF[5] = qQEKF[0];
  c_qQEKF[6] = qQEKF[3];
  c_qQEKF[7] = -qQEKF[2];
  c_qQEKF[8] = qQEKF[2];
  c_qQEKF[9] = -qQEKF[3];
  c_qQEKF[10] = qQEKF[0];
  c_qQEKF[11] = qQEKF[1];
  c_qQEKF[12] = qQEKF[3];
  c_qQEKF[13] = qQEKF[2];
  c_qQEKF[14] = -qQEKF[1];
  c_qQEKF[15] = qQEKF[0];
  for (i0 = 0; i0 < 4; i0++) {
    for (j = 0; j < 3; j++) {
      daccelerometer_dqQEKF[i0 + (j << 2)] = 0.0F;
      for (i1 = 0; i1 < 4; i1++) {
        daccelerometer_dqQEKF[i0 + (j << 2)] += b_qQEKF[i0 + (i1 << 2)] * b_a[i1
          + (j << 2)];
      }
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    for (j = 0; j < 3; j++) {
      d_qQEKF[i0 + (j << 2)] = 0.0F;
      for (i1 = 0; i1 < 4; i1++) {
        d_qQEKF[i0 + (j << 2)] += c_qQEKF[i0 + (i1 << 2)] *
          daccelerometer_dqQEKF[i1 + (j << 2)];
      }
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    fv0[i0] = 0.0F;
    for (j = 0; j < 4; j++) {
      fv0[i0] += (float)iv7[j] * d_qQEKF[j + (i0 << 2)];
    }

    H[1 + 7 * i0] = fv0[i0];
  }

  //  d encoder_meas  /  d dy_2L
  // 'VelocityEKF:90' H(4:6,3:4) = devec * Phi(qQEKF)' * Gamma(qQEKF) * [zeros(1,2);eye(2);zeros(1,2)]; 
  b_qQEKF[0] = qQEKF[0];
  b_qQEKF[4] = -qQEKF[1];
  b_qQEKF[8] = -qQEKF[2];
  b_qQEKF[12] = -qQEKF[3];
  b_qQEKF[1] = qQEKF[1];
  b_qQEKF[5] = qQEKF[0];
  b_qQEKF[9] = -qQEKF[3];
  b_qQEKF[13] = qQEKF[2];
  b_qQEKF[2] = qQEKF[2];
  b_qQEKF[6] = qQEKF[3];
  b_qQEKF[10] = qQEKF[0];
  b_qQEKF[14] = -qQEKF[1];
  b_qQEKF[3] = qQEKF[3];
  b_qQEKF[7] = -qQEKF[2];
  b_qQEKF[11] = qQEKF[1];
  b_qQEKF[15] = qQEKF[0];
  c_qQEKF[0] = qQEKF[0];
  c_qQEKF[1] = -qQEKF[1];
  c_qQEKF[2] = -qQEKF[2];
  c_qQEKF[3] = -qQEKF[3];
  c_qQEKF[4] = qQEKF[1];
  c_qQEKF[5] = qQEKF[0];
  c_qQEKF[6] = qQEKF[3];
  c_qQEKF[7] = -qQEKF[2];
  c_qQEKF[8] = qQEKF[2];
  c_qQEKF[9] = -qQEKF[3];
  c_qQEKF[10] = qQEKF[0];
  c_qQEKF[11] = qQEKF[1];
  c_qQEKF[12] = qQEKF[3];
  c_qQEKF[13] = qQEKF[2];
  c_qQEKF[14] = -qQEKF[1];
  c_qQEKF[15] = qQEKF[0];
  for (i0 = 0; i0 < 4; i0++) {
    for (j = 0; j < 3; j++) {
      daccelerometer_dqQEKF[i0 + (j << 2)] = 0.0F;
      for (i1 = 0; i1 < 4; i1++) {
        daccelerometer_dqQEKF[i0 + (j << 2)] += b_qQEKF[i0 + (i1 << 2)] * (float)
          iv1[i1 + (j << 2)];
      }
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    for (j = 0; j < 3; j++) {
      d_qQEKF[i0 + (j << 2)] = 0.0F;
      for (i1 = 0; i1 < 4; i1++) {
        d_qQEKF[i0 + (j << 2)] += c_qQEKF[i0 + (i1 << 2)] *
          daccelerometer_dqQEKF[i1 + (j << 2)];
      }
    }
  }

  for (i0 = 0; i0 < 2; i0++) {
    for (j = 0; j < 3; j++) {
      c_y = 0.0F;
      for (i1 = 0; i1 < 4; i1++) {
        c_y += (float)iv8[i0 + (i1 << 1)] * d_qQEKF[i1 + (j << 2)];
      }

      H[(i0 + 7 * (3 + j)) + 2] = c_y;
    }
  }

  // 'VelocityEKF:91' H(4:6,5:7) = eye(3);
  //  Measurement covariances	
  // 'VelocityEKF:94' cov_quantization = 0.5 * eye(3);
  // 'VelocityEKF:95' cov_encoder = eta_encoder * 4*cov_quantization;
  b_y = eta_encoder * 4.0F;

  // 'VelocityEKF:97' daccelerometer_dqQEKF = devec * Phi(qQEKF)' * Phi([0;Accelerometer]) + devec * Gamma(qQEKF) * Gamma([0;Accelerometer]) * I_conj; 
  for (j = 0; j < 3; j++) {
    for (i0 = 0; i0 < 3; i0++) {
      H[(i0 + 7 * (3 + j)) + 4] = iv9[i0 + 3 * j];
    }

    q[j + 1] = Accelerometer[j];
    p[j + 1] = Accelerometer[j];
  }

  b_qQEKF[0] = qQEKF[0];
  b_qQEKF[4] = -qQEKF[1];
  b_qQEKF[8] = -qQEKF[2];
  b_qQEKF[12] = -qQEKF[3];
  b_qQEKF[1] = qQEKF[1];
  b_qQEKF[5] = qQEKF[0];
  b_qQEKF[9] = -qQEKF[3];
  b_qQEKF[13] = qQEKF[2];
  b_qQEKF[2] = qQEKF[2];
  b_qQEKF[6] = qQEKF[3];
  b_qQEKF[10] = qQEKF[0];
  b_qQEKF[14] = -qQEKF[1];
  b_qQEKF[3] = qQEKF[3];
  b_qQEKF[7] = -qQEKF[2];
  b_qQEKF[11] = qQEKF[1];
  b_qQEKF[15] = qQEKF[0];
  d_y[0] = 0.0F;
  d_y[1] = -q[1];
  d_y[2] = -q[2];
  d_y[3] = -q[3];
  d_y[4] = q[1];
  d_y[5] = 0.0F;
  d_y[6] = -q[3];
  d_y[7] = q[2];
  d_y[8] = q[2];
  d_y[9] = q[3];
  d_y[10] = 0.0F;
  d_y[11] = -q[1];
  d_y[12] = q[3];
  d_y[13] = -q[2];
  d_y[14] = q[1];
  d_y[15] = 0.0F;
  c_qQEKF[0] = qQEKF[0];
  c_qQEKF[1] = -qQEKF[1];
  c_qQEKF[2] = -qQEKF[2];
  c_qQEKF[3] = -qQEKF[3];
  c_qQEKF[4] = qQEKF[1];
  c_qQEKF[5] = qQEKF[0];
  c_qQEKF[6] = qQEKF[3];
  c_qQEKF[7] = -qQEKF[2];
  c_qQEKF[8] = qQEKF[2];
  c_qQEKF[9] = -qQEKF[3];
  c_qQEKF[10] = qQEKF[0];
  c_qQEKF[11] = qQEKF[1];
  c_qQEKF[12] = qQEKF[3];
  c_qQEKF[13] = qQEKF[2];
  c_qQEKF[14] = -qQEKF[1];
  c_qQEKF[15] = qQEKF[0];
  fv10[0] = 0.0F;
  fv10[1] = -p[1];
  fv10[2] = -p[2];
  fv10[3] = -p[3];
  fv10[4] = p[1];
  fv10[5] = 0.0F;
  fv10[6] = p[3];
  fv10[7] = -p[2];
  fv10[8] = p[2];
  fv10[9] = -p[3];
  fv10[10] = 0.0F;
  fv10[11] = p[1];
  fv10[12] = p[3];
  fv10[13] = p[2];
  fv10[14] = -p[1];
  fv10[15] = 0.0F;
  for (i0 = 0; i0 < 4; i0++) {
    for (j = 0; j < 3; j++) {
      daccelerometer_dqQEKF[i0 + (j << 2)] = 0.0F;
      d_qQEKF[i0 + (j << 2)] = 0.0F;
      for (i1 = 0; i1 < 4; i1++) {
        daccelerometer_dqQEKF[i0 + (j << 2)] += b_qQEKF[i0 + (i1 << 2)] * (float)
          iv1[i1 + (j << 2)];
        d_qQEKF[i0 + (j << 2)] += c_qQEKF[i0 + (i1 << 2)] * (float)iv1[i1 + (j <<
          2)];
      }
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    for (j = 0; j < 3; j++) {
      fv11[i0 + (j << 2)] = 0.0F;
      fv12[i0 + (j << 2)] = 0.0F;
      for (i1 = 0; i1 < 4; i1++) {
        fv11[i0 + (j << 2)] += fv10[i0 + (i1 << 2)] * d_qQEKF[i1 + (j << 2)];
        fv12[i0 + (j << 2)] += d_y[i0 + (i1 << 2)] * daccelerometer_dqQEKF[i1 +
          (j << 2)];
      }
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    for (j = 0; j < 3; j++) {
      b_a[i0 + (j << 2)] = 0.0F;
      for (i1 = 0; i1 < 4; i1++) {
        b_a[i0 + (j << 2)] += (float)iv10[i0 + (i1 << 2)] * fv11[i1 + (j << 2)];
      }
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (j = 0; j < 4; j++) {
      daccelerometer_dqQEKF[j + (i0 << 2)] = fv12[j + (i0 << 2)] + b_a[j + (i0 <<
        2)];
    }
  }

  // 'VelocityEKF:98' R_accelerometer = eta_accelerometer*cov_acc + daccelerometer_dqQEKF * cov_qQEKF * daccelerometer_dqQEKF'; 
  //  Setup measurement covariance
  // 'VelocityEKF:101' R = [cov_encoder, zeros(3,3);
  // 'VelocityEKF:102'          zeros(3,3),  R_accelerometer];
  //  Calculate Kalman gain
  // 'VelocityEKF:105' S = H * P_apriori * H' + R;
  // K = P_apriori * H' * inv(S);
  // 'VelocityEKF:107' K = P_apriori * H' / S;
  for (i0 = 0; i0 < 6; i0++) {
    for (j = 0; j < 7; j++) {
      b_H[i0 + 6 * j] = 0.0F;
      for (i1 = 0; i1 < 7; i1++) {
        b_H[i0 + 6 * j] += (float)H[i1 + 7 * i0] * P_apriori[i1 + 7 * j];
      }
    }
  }

  for (i0 = 0; i0 < 7; i0++) {
    for (j = 0; j < 6; j++) {
      K[i0 + 7 * j] = 0.0F;
      for (i1 = 0; i1 < 7; i1++) {
        K[i0 + 7 * j] += P_apriori[i0 + 7 * i1] * (float)H[i1 + 7 * j];
      }
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    for (j = 0; j < 3; j++) {
      b_a[i0 + (j << 2)] = 0.0F;
      for (i1 = 0; i1 < 4; i1++) {
        b_a[i0 + (j << 2)] += cov_qQEKF[i0 + (i1 << 2)] *
          daccelerometer_dqQEKF[i1 + (j << 2)];
      }
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (j = 0; j < 3; j++) {
      c_y = 0.0F;
      for (i1 = 0; i1 < 4; i1++) {
        c_y += daccelerometer_dqQEKF[i1 + (i0 << 2)] * b_a[i1 + (j << 2)];
      }

      b_eta_accelerometer[i0 + 3 * j] = eta_accelerometer * cov_acc[i0 + 3 * j]
        + c_y;
    }
  }

  for (i0 = 0; i0 < 6; i0++) {
    for (j = 0; j < 6; j++) {
      c_H[i0 + 6 * j] = 0.0F;
      for (i1 = 0; i1 < 7; i1++) {
        c_H[i0 + 6 * j] += (float)H[i1 + 7 * i0] * K[i1 + 7 * j];
      }
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (j = 0; j < 3; j++) {
      e_y[j + 6 * i0] = b_y * fv13[j + 3 * i0];
      e_y[(j + 6 * i0) + 3] = 0.0F;
      e_y[j + 6 * (i0 + 3)] = 0.0F;
      e_y[(j + 6 * (i0 + 3)) + 3] = b_eta_accelerometer[j + 3 * i0];
    }
  }

  for (i0 = 0; i0 < 6; i0++) {
    for (j = 0; j < 6; j++) {
      d_H[j + 6 * i0] = c_H[j + 6 * i0] + e_y[j + 6 * i0];
    }
  }

  mrdivide(b_H, d_H, K);

  //  Correct using innovation
  // 'VelocityEKF:110' X_aposteriori = X_apriori + K * (z - z_hat);
  fv1[0] = 0.0F;
  fv1[1] = -dy_apriori;
  fv1[2] = dx_apriori;
  fv1[3] = 0.0F;
  b_qQEKF[0] = qQEKF[0];
  b_qQEKF[1] = -qQEKF[1];
  b_qQEKF[2] = -qQEKF[2];
  b_qQEKF[3] = -qQEKF[3];
  b_qQEKF[4] = qQEKF[1];
  b_qQEKF[5] = qQEKF[0];
  b_qQEKF[6] = qQEKF[3];
  b_qQEKF[7] = -qQEKF[2];
  b_qQEKF[8] = qQEKF[2];
  b_qQEKF[9] = -qQEKF[3];
  b_qQEKF[10] = qQEKF[0];
  b_qQEKF[11] = qQEKF[1];
  b_qQEKF[12] = qQEKF[3];
  b_qQEKF[13] = qQEKF[2];
  b_qQEKF[14] = -qQEKF[1];
  b_qQEKF[15] = qQEKF[0];
  d_y[0] = y * qQEKF[0];
  d_y[4] = y * -qQEKF[1];
  d_y[8] = y * -qQEKF[2];
  d_y[12] = y * -qQEKF[3];
  d_y[1] = y * qQEKF[1];
  d_y[5] = y * qQEKF[0];
  d_y[9] = y * -qQEKF[3];
  d_y[13] = y * qQEKF[2];
  d_y[2] = y * qQEKF[2];
  d_y[6] = y * qQEKF[3];
  d_y[10] = y * qQEKF[0];
  d_y[14] = y * -qQEKF[1];
  d_y[3] = y * qQEKF[3];
  d_y[7] = y * -qQEKF[2];
  d_y[11] = y * qQEKF[1];
  d_y[15] = y * qQEKF[0];
  for (i0 = 0; i0 < 4; i0++) {
    for (j = 0; j < 4; j++) {
      c_qQEKF[i0 + (j << 2)] = 0.0F;
      for (i1 = 0; i1 < 4; i1++) {
        c_qQEKF[i0 + (j << 2)] += b_qQEKF[i0 + (i1 << 2)] * d_y[i1 + (j << 2)];
      }
    }
  }

  d_y[0] = 2.0F * qQEKF[0];
  d_y[4] = 2.0F * -qQEKF[1];
  d_y[8] = 2.0F * -qQEKF[2];
  d_y[12] = 2.0F * -qQEKF[3];
  d_y[1] = 2.0F * qQEKF[1];
  d_y[5] = 2.0F * qQEKF[0];
  d_y[9] = 2.0F * -qQEKF[3];
  d_y[13] = 2.0F * qQEKF[2];
  d_y[2] = 2.0F * qQEKF[2];
  d_y[6] = 2.0F * qQEKF[3];
  d_y[10] = 2.0F * qQEKF[0];
  d_y[14] = 2.0F * -qQEKF[1];
  d_y[3] = 2.0F * qQEKF[3];
  d_y[7] = 2.0F * -qQEKF[2];
  d_y[11] = 2.0F * qQEKF[1];
  d_y[15] = 2.0F * qQEKF[0];
  for (i0 = 0; i0 < 4; i0++) {
    fv3[i0] = 0.0F;
    b_qdotQEKF[i0] = 0.0F;
    for (j = 0; j < 4; j++) {
      fv3[i0] += fv1[j] * c_qQEKF[j + (i0 << 2)];
      b_qdotQEKF[i0] += qdotQEKF[j] * d_y[j + (i0 << 2)];
    }

    fv4[i0] = fv3[i0] - b_qdotQEKF[i0];
  }

  for (i0 = 0; i0 < 3; i0++) {
    fv0[i0] = 0.0F;
    for (j = 0; j < 4; j++) {
      fv0[i0] += fv4[j] * W[j + (i0 << 2)];
    }
  }

  fv1[0] = 0.0F;
  fv1[1] = X[2];
  fv1[2] = X[3];
  fv1[3] = g;
  b_qQEKF[0] = qQEKF[0];
  b_qQEKF[4] = -qQEKF[1];
  b_qQEKF[8] = -qQEKF[2];
  b_qQEKF[12] = -qQEKF[3];
  b_qQEKF[1] = qQEKF[1];
  b_qQEKF[5] = qQEKF[0];
  b_qQEKF[9] = -qQEKF[3];
  b_qQEKF[13] = qQEKF[2];
  b_qQEKF[2] = qQEKF[2];
  b_qQEKF[6] = qQEKF[3];
  b_qQEKF[10] = qQEKF[0];
  b_qQEKF[14] = -qQEKF[1];
  b_qQEKF[3] = qQEKF[3];
  b_qQEKF[7] = -qQEKF[2];
  b_qQEKF[11] = qQEKF[1];
  b_qQEKF[15] = qQEKF[0];
  c_qQEKF[0] = qQEKF[0];
  c_qQEKF[1] = -qQEKF[1];
  c_qQEKF[2] = -qQEKF[2];
  c_qQEKF[3] = -qQEKF[3];
  c_qQEKF[4] = qQEKF[1];
  c_qQEKF[5] = qQEKF[0];
  c_qQEKF[6] = qQEKF[3];
  c_qQEKF[7] = -qQEKF[2];
  c_qQEKF[8] = qQEKF[2];
  c_qQEKF[9] = -qQEKF[3];
  c_qQEKF[10] = qQEKF[0];
  c_qQEKF[11] = qQEKF[1];
  c_qQEKF[12] = qQEKF[3];
  c_qQEKF[13] = qQEKF[2];
  c_qQEKF[14] = -qQEKF[1];
  c_qQEKF[15] = qQEKF[0];
  for (i0 = 0; i0 < 4; i0++) {
    for (j = 0; j < 3; j++) {
      daccelerometer_dqQEKF[i0 + (j << 2)] = 0.0F;
      for (i1 = 0; i1 < 4; i1++) {
        daccelerometer_dqQEKF[i0 + (j << 2)] += b_qQEKF[i0 + (i1 << 2)] * (float)
          iv1[i1 + (j << 2)];
      }
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    for (j = 0; j < 3; j++) {
      d_qQEKF[i0 + (j << 2)] = 0.0F;
      for (i1 = 0; i1 < 4; i1++) {
        d_qQEKF[i0 + (j << 2)] += c_qQEKF[i0 + (i1 << 2)] *
          daccelerometer_dqQEKF[i1 + (j << 2)];
      }
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    fv14[i0] = 0.0F;
    for (j = 0; j < 4; j++) {
      fv14[i0] += fv1[j] * d_qQEKF[j + (i0 << 2)];
    }

    c_EncoderDiffMeas[i0] = EncoderDiffMeas[i0];
    c_EncoderDiffMeas[i0 + 3] = Accelerometer[i0];
    c_a[i0] = a * fv0[i0];
    c_a[i0 + 3] = fv14[i0] + X[4 + i0];
  }

  for (i0 = 0; i0 < 6; i0++) {
    b_EncoderDiffMeas[i0] = c_EncoderDiffMeas[i0] - c_a[i0];
  }

  for (i0 = 0; i0 < 7; i0++) {
    v[i0] = 0.0F;
    for (j = 0; j < 6; j++) {
      v[i0] += b_EncoderDiffMeas[j] * K[j + 6 * i0];
    }
  }

  b_dx_apriori[0] = dx_apriori;
  b_dx_apriori[1] = dy_apriori;
  b_dx_apriori[2] = X[2];
  b_dx_apriori[3] = X[3];
  for (i0 = 0; i0 < 3; i0++) {
    b_dx_apriori[i0 + 4] = X[4 + i0];
  }

  for (i0 = 0; i0 < 7; i0++) {
    X_out[i0] = b_dx_apriori[i0] + v[i0];
  }

  // 'VelocityEKF:111' P_aposteriori = (eye(7) - K*H) * P_apriori;
  memset(&F_prev[0], 0, 49U * sizeof(double));
  for (j = 0; j < 7; j++) {
    F_prev[j + 7 * j] = 1.0;
  }

  for (i0 = 0; i0 < 7; i0++) {
    for (j = 0; j < 7; j++) {
      c_y = 0.0F;
      for (i1 = 0; i1 < 6; i1++) {
        c_y += (float)H[i0 + 7 * i1] * K[i1 + 6 * j];
      }

      Q[i0 + 7 * j] = (float)F_prev[i0 + 7 * j] - c_y;
    }
  }

  for (i0 = 0; i0 < 7; i0++) {
    for (j = 0; j < 7; j++) {
      P_out[i0 + 7 * j] = 0.0F;
      for (i1 = 0; i1 < 7; i1++) {
        P_out[i0 + 7 * j] += P_apriori[i0 + 7 * i1] * Q[i1 + 7 * j];
      }
    }
  }

  //     %% Send output to Simulink
  // 'VelocityEKF:114' X_out = X_aposteriori;
  // 'VelocityEKF:115' P_out = P_aposteriori;
}

//
// File trailer for VelocityEKF.cpp
//
// [EOF]
//
