/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * RotateTrajectory.h
 *
 * Code generation for function 'RotateTrajectory'
 *
 */

#ifndef ROTATETRAJECTORY_H
#define ROTATETRAJECTORY_H

/* Include files */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include "rtwtypes.h"
#include "ExtractWindowTrajectory_types.h"

/* Function Declarations */
extern void RotateTrajectory(const emlrtStack *sp, const emxArray_real_T
  *trajectory, real_T rotation, emxArray_real_T *rotatedPoints);

#endif

/* End of code generation (RotateTrajectory.h) */
