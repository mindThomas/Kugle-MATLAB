//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eye.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 26-Apr-2019 13:33:55
//

// Include Files
#include <string.h>
#include "rt_nonfinite.h"
#include "QEKF.h"
#include "eye.h"

// Function Definitions

//
// Arguments    : double I[16]
// Return Type  : void
//
void b_eye(double I[16])
{
  int k;
  memset(&I[0], 0, sizeof(double) << 4);
  for (k = 0; k < 4; k++) {
    I[k + (k << 2)] = 1.0;
  }
}

//
// Arguments    : double I[100]
// Return Type  : void
//
void c_eye(double I[100])
{
  int k;
  memset(&I[0], 0, 100U * sizeof(double));
  for (k = 0; k < 10; k++) {
    I[k + 10 * k] = 1.0;
  }
}

//
// Arguments    : double I[9]
// Return Type  : void
//
void eye(double I[9])
{
  int k;
  memset(&I[0], 0, 9U * sizeof(double));
  for (k = 0; k < 3; k++) {
    I[k + 3 * k] = 1.0;
  }
}

//
// File trailer for eye.cpp
//
// [EOF]
//
