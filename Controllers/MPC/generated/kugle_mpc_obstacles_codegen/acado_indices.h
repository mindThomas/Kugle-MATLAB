/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details. 
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */
 
#ifndef ACADO_INDICES_H
#define ACADO_INDICES_H

#include "acado_common.h"

// ACADO Step size
#define ACADO_TS 0.100000

typedef struct x_t
{
	real_t q2;
	real_t q3;
	real_t x;
	real_t y;
	real_t dx;
	real_t dy;
	real_t s;
	real_t ds;
	real_t omega_ref_x;
	real_t omega_ref_y;
} x_t __attribute__((packed));

typedef struct u_t
{
	real_t domega_ref_x;
	real_t domega_ref_y;
	real_t dds;
	real_t velocity_slack;
	real_t angle_slack;
	real_t proximity_slack;
} u_t __attribute__((packed));

typedef struct od_t
{
	real_t desiredVelocity;
	real_t maxVelocity;
	real_t maxAngle;
	real_t maxOmegaRef;
	real_t maxdOmegaRef;
	real_t trajectoryLength;
	real_t trajectoryStart;
	real_t cx9;
	real_t cx8;
	real_t cx7;
	real_t cx6;
	real_t cx5;
	real_t cx4;
	real_t cx3;
	real_t cx2;
	real_t cx1;
	real_t cx0;
	real_t cy9;
	real_t cy8;
	real_t cy7;
	real_t cy6;
	real_t cy5;
	real_t cy4;
	real_t cy3;
	real_t cy2;
	real_t cy1;
	real_t cy0;
	real_t obs1_x;
	real_t obs1_y;
	real_t obs1_r;
	real_t obs2_x;
	real_t obs2_y;
	real_t obs2_r;
	real_t obs3_x;
	real_t obs3_y;
	real_t obs3_r;
	real_t obs4_x;
	real_t obs4_y;
	real_t obs4_r;
	real_t obs5_x;
	real_t obs5_y;
	real_t obs5_r;
	real_t proximityOffset;
	real_t proximityScale;
} od_t __attribute__((packed));

typedef struct y_t
{
	real_t lag_error;
	real_t lateral_deviation;
	real_t q2;
	real_t q3;
	real_t omega_ref_x;
	real_t omega_ref_y;
	real_t velocity_error;
	real_t away_from_end_error;
	real_t domega_ref_x;
	real_t domega_ref_y;
	real_t obstacle_proximity;
	real_t velocity_slack;
	real_t angle_slack;
	real_t proximity_slack;
} y_t __attribute__((packed));

typedef struct yN_t
{
	real_t lag_error;
	real_t lateral_deviation;
	real_t q2;
	real_t q3;
	real_t omega_ref_x;
	real_t omega_ref_y;
	real_t velocity_error;
	real_t away_from_end_error;
	real_t obstacle_proximity;
} yN_t __attribute__((packed));

typedef struct ACADO_t
{
	int dummy;
	x_t x[ACADO_N+1];
	u_t u[ACADO_N];
    od_t od[ACADO_N+1];
    y_t y[ACADO_N];
    yN_t yN;
    real_t W[ ACADO_NY*ACADO_NY ];
    real_t WN[ ACADO_NYN*ACADO_NYN ];
    x_t x0;
} ACADO_t __attribute__((packed));

#endif /* ACADO_INDICES_H */