//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 26-Apr-2019 13:33:55
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
#include "QEKF.h"
#include "main.h"
#include "QEKF_terminate.h"
#include "QEKF_initialize.h"

// Function Declarations
static void argInit_10x10_real32_T(float result[100]);
static void argInit_10x1_real32_T(float result[10]);
static void argInit_3x1_real32_T(float result[3]);
static void argInit_3x3_real32_T(float result[9]);
static boolean_T argInit_boolean_T();
static float argInit_real32_T();
static void main_QEKF();

// Function Definitions

//
// Arguments    : float result[100]
// Return Type  : void
//
static void argInit_10x10_real32_T(float result[100])
{
  int idx0;
  int idx1;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 10; idx0++) {
    for (idx1 = 0; idx1 < 10; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx1 + 10 * idx0] = argInit_real32_T();
    }
  }
}

//
// Arguments    : float result[10]
// Return Type  : void
//
static void argInit_10x1_real32_T(float result[10])
{
  int idx0;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 10; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real32_T();
  }
}

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
// Arguments    : void
// Return Type  : boolean_T
//
static boolean_T argInit_boolean_T()
{
  return false;
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
static void main_QEKF()
{
  float X[10];
  static float P_prev[100];
  float Gyroscope[3];
  float Accelerometer[3];
  float Heading;
  boolean_T UseHeadingForCorrection;
  float SamplePeriod;
  float fv5[9];
  float fv6[9];
  float X_out[10];
  static float P_out[100];

  // Initialize function 'QEKF' input arguments.
  // Initialize function input argument 'X'.
  argInit_10x1_real32_T(X);

  // Initialize function input argument 'P_prev'.
  argInit_10x10_real32_T(P_prev);

  // Initialize function input argument 'Gyroscope'.
  argInit_3x1_real32_T(Gyroscope);

  // Initialize function input argument 'Accelerometer'.
  argInit_3x1_real32_T(Accelerometer);
  Heading = argInit_real32_T();
  UseHeadingForCorrection = argInit_boolean_T();
  SamplePeriod = argInit_real32_T();

  // Initialize function input argument 'cov_gyro'.
  // Initialize function input argument 'cov_acc'.
  // Call the entry-point 'QEKF'.
  argInit_3x3_real32_T(fv5);
  argInit_3x3_real32_T(fv6);
  QEKF(X, P_prev, Gyroscope, Accelerometer, Heading, UseHeadingForCorrection,
       SamplePeriod, argInit_boolean_T(), argInit_boolean_T(), argInit_boolean_T
       (), argInit_boolean_T(), fv5, fv6, argInit_real32_T(), argInit_real32_T(),
       argInit_real32_T(), argInit_real32_T(), argInit_boolean_T(),
       argInit_real32_T(), argInit_real32_T(), argInit_real32_T(),
       argInit_real32_T(), X_out, P_out);
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
  QEKF_initialize();

  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_QEKF();

  // Terminate the application.
  // You do not need to do this more than one time.
  QEKF_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
