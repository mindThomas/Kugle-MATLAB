//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: QEKF_initialize.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 26-Apr-2019 13:33:55
//

// Include Files
#include "rt_nonfinite.h"
#include "QEKF.h"
#include "QEKF_initialize.h"

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void QEKF_initialize()
{
  rt_InitInfAndNaN(8U);
  acc_norm_old_not_empty_init();
  acc_norm_filtered_not_empty_init();
}

//
// File trailer for QEKF_initialize.cpp
//
// [EOF]
//
