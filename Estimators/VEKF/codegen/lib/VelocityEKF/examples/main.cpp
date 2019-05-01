//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 26-Apr-2019 13:40:20
//

//***********************************************************************
// This automatically generated example C main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************
// Include Files
#include "rt_nonfinite.h"
#include "VelocityEKF.h"
#include "main.h"
#include "VelocityEKF_terminate.h"
#include "VelocityEKF_initialize.h"

// Function Declarations
static void argInit_3x1_real32_T(float result[3]);
static void argInit_3x3_real32_T(float result[9]);
static void argInit_4x1_real32_T(float result[4]);
static void argInit_4x4_real32_T(float result[16]);
static void argInit_7x1_real32_T(float result[7]);
static void argInit_7x7_real32_T(float result[49]);
static float argInit_real32_T();
static void main_VelocityEKF();

// Function Definitions

//
// Arguments    : float result[3]
// Return Type  : void
//
static void argInit_3x1_real32_T(float result[3])
{
  int idx0;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 3; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real32_T();
  }
}

//
// Arguments    : float result[9]
// Return Type  : void
//
static void argInit_3x3_real32_T(float result[9])
{
  int idx0;
  int idx1;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 3; idx0++) {
    for (idx1 = 0; idx1 < 3; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx1 + 3 * idx0] = argInit_real32_T();
    }
  }
}

//
// Arguments    : float result[4]
// Return Type  : void
//
static void argInit_4x1_real32_T(float result[4])
{
  int idx0;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 4; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real32_T();
  }
}

//
// Arguments    : float result[16]
// Return Type  : void
//
static void argInit_4x4_real32_T(float result[16])
{
  int idx0;
  int idx1;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 4; idx0++) {
    for (idx1 = 0; idx1 < 4; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx1 + (idx0 << 2)] = argInit_real32_T();
    }
  }
}

//
// Arguments    : float result[7]
// Return Type  : void
//
static void argInit_7x1_real32_T(float result[7])
{
  int idx0;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 7; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real32_T();
  }
}

//
// Arguments    : float result[49]
// Return Type  : void
//
static void argInit_7x7_real32_T(float result[49])
{
  int idx0;
  int idx1;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 7; idx0++) {
    for (idx1 = 0; idx1 < 7; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx1 + 7 * idx0] = argInit_real32_T();
    }
  }
}

//
// Arguments    : void
// Return Type  : float
//
static float argInit_real32_T()
{
  return 0.0F;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_VelocityEKF()
{
  float X[7];
  static float P_prev[49];
  float fv15[3];
  float fv16[3];
  float fv17[9];
  float fv18[4];
  float fv19[16];
  float fv20[4];
  float X_out[7];
  static float P_out[49];

  // Initialize function 'VelocityEKF' input arguments.
  // Initialize function input argument 'X'.
  argInit_7x1_real32_T(X);

  // Initialize function input argument 'P_prev'.
  argInit_7x7_real32_T(P_prev);

  // Initialize function input argument 'EncoderDiffMeas'.
  // Initialize function input argument 'Accelerometer'.
  // Initialize function input argument 'cov_acc'.
  // Initialize function input argument 'qQEKF'.
  // Initialize function input argument 'cov_qQEKF'.
  // Initialize function input argument 'qdotQEKF'.
  // Call the entry-point 'VelocityEKF'.
  argInit_3x1_real32_T(fv15);
  argInit_3x1_real32_T(fv16);
  argInit_3x3_real32_T(fv17);
  argInit_4x1_real32_T(fv18);
  argInit_4x4_real32_T(fv19);
  argInit_4x1_real32_T(fv20);
  VelocityEKF(X, P_prev, fv15, argInit_real32_T(), fv16, fv17, argInit_real32_T(),
              argInit_real32_T(), fv18, fv19, fv20, argInit_real32_T(),
              argInit_real32_T(), argInit_real32_T(), argInit_real32_T(),
              argInit_real32_T(), argInit_real32_T(), X_out, P_out);
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
  // Initialize the application.
  // You do not need to do this more than one time.
  VelocityEKF_initialize();

  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_VelocityEKF();

  // Terminate the application.
  // You do not need to do this more than one time.
  VelocityEKF_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
