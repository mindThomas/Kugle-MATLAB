/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * ExtractDistanceTrajectory.h
 *
 * Code generation for function 'ExtractDistanceTrajectory'
 *
 */

#ifndef EXTRACTDISTANCETRAJECTORY_H
#define EXTRACTDISTANCETRAJECTORY_H

/* Include files */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include "rtwtypes.h"
#include "ExtractDistanceTrajectory_types.h"

/* Function Declarations */
extern void ExtractDistanceTrajectory(const emlrtStack *sp, const
  emxArray_real_T *TrajectoryPoints, const real_T RobotPos[2], real_T RobotYaw,
  const real_T Velocity[2], real_T ExtractDist, real_T OrientationSelection,
  emxArray_real_T *WindowTrajectory, real_T *nTrajPoints, real_T
  *WindowOrientation);

#endif

/* End of code generation (ExtractDistanceTrajectory.h) */
