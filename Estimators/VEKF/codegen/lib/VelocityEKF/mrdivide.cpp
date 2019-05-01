//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mrdivide.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 26-Apr-2019 13:40:20
//

// Include Files
#include <math.h>
#include "rt_nonfinite.h"
#include "VelocityEKF.h"
#include "mrdivide.h"

// Function Definitions

//
// Arguments    : const float A[42]
//                const float B[36]
//                float y[42]
// Return Type  : void
//
void mrdivide(const float A[42], const float B[36], float y[42])
{
  int k;
  int j;
  int jAcol;
  int c;
  float b_A[36];
  int jy;
  int ix;
  signed char ipiv[6];
  float b_B[42];
  float smax;
  int iy;
  int i;
  float s;
  for (k = 0; k < 6; k++) {
    for (jAcol = 0; jAcol < 6; jAcol++) {
      b_A[jAcol + 6 * k] = B[k + 6 * jAcol];
    }

    for (jAcol = 0; jAcol < 7; jAcol++) {
      b_B[jAcol + 7 * k] = A[k + 6 * jAcol];
    }

    ipiv[k] = (signed char)(1 + k);
  }

  for (j = 0; j < 5; j++) {
    c = j * 7;
    jAcol = 0;
    ix = c;
    smax = (float)fabs((double)b_A[c]);
    for (k = 2; k <= 6 - j; k++) {
      ix++;
      s = (float)fabs((double)b_A[ix]);
      if (s > smax) {
        jAcol = k - 1;
        smax = s;
      }
    }

    if (b_A[c + jAcol] != 0.0F) {
      if (jAcol != 0) {
        ipiv[j] = (signed char)((j + jAcol) + 1);
        ix = j;
        iy = j + jAcol;
        for (k = 0; k < 6; k++) {
          smax = b_A[ix];
          b_A[ix] = b_A[iy];
          b_A[iy] = smax;
          ix += 6;
          iy += 6;
        }
      }

      k = (c - j) + 6;
      for (i = c + 1; i < k; i++) {
        b_A[i] /= b_A[c];
      }
    }

    iy = c;
    jy = c + 6;
    for (jAcol = 1; jAcol <= 5 - j; jAcol++) {
      smax = b_A[jy];
      if (b_A[jy] != 0.0F) {
        ix = c + 1;
        k = (iy - j) + 12;
        for (i = 7 + iy; i < k; i++) {
          b_A[i] += b_A[ix] * -smax;
          ix++;
        }
      }

      jy += 6;
      iy += 6;
    }
  }

  for (j = 0; j < 6; j++) {
    jy = 7 * j;
    jAcol = 6 * j;
    for (k = 1; k <= j; k++) {
      iy = 7 * (k - 1);
      if (b_A[(k + jAcol) - 1] != 0.0F) {
        for (i = 0; i < 7; i++) {
          b_B[i + jy] -= b_A[(k + jAcol) - 1] * b_B[i + iy];
        }
      }
    }

    smax = 1.0F / b_A[j + jAcol];
    for (i = 0; i < 7; i++) {
      b_B[i + jy] *= smax;
    }
  }

  for (j = 5; j >= 0; j--) {
    jy = 7 * j;
    jAcol = 6 * j - 1;
    for (k = j + 2; k < 7; k++) {
      iy = 7 * (k - 1);
      if (b_A[k + jAcol] != 0.0F) {
        for (i = 0; i < 7; i++) {
          b_B[i + jy] -= b_A[k + jAcol] * b_B[i + iy];
        }
      }
    }
  }

  for (jAcol = 4; jAcol >= 0; jAcol--) {
    if (ipiv[jAcol] != jAcol + 1) {
      iy = ipiv[jAcol] - 1;
      for (jy = 0; jy < 7; jy++) {
        smax = b_B[jy + 7 * jAcol];
        b_B[jy + 7 * jAcol] = b_B[jy + 7 * iy];
        b_B[jy + 7 * iy] = smax;
      }
    }
  }

  for (k = 0; k < 7; k++) {
    for (jAcol = 0; jAcol < 6; jAcol++) {
      y[jAcol + 6 * k] = b_B[k + 7 * jAcol];
    }
  }
}

//
// File trailer for mrdivide.cpp
//
// [EOF]
//
