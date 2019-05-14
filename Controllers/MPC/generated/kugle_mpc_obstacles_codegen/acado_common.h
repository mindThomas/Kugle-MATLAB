/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#ifndef ACADO_COMMON_H
#define ACADO_COMMON_H

#include <math.h>
#include <string.h>

#ifndef __MATLAB__
#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */
#endif /* __MATLAB__ */

/** \defgroup ACADO ACADO CGT generated module. */
/** @{ */

/** qpOASES QP solver indicator. */
#define ACADO_QPOASES  0
#define ACADO_QPOASES3 1
/** FORCES QP solver indicator.*/
#define ACADO_FORCES   2
/** qpDUNES QP solver indicator.*/
#define ACADO_QPDUNES  3
/** HPMPC QP solver indicator. */
#define ACADO_HPMPC    4
#define ACADO_GENERIC    5

/** Indicator for determining the QP solver used by the ACADO solver code. */
#define ACADO_QP_SOLVER ACADO_QPOASES3

#include "acado_qpoases3_interface.h"


/*
 * Common definitions
 */
/** User defined block based condensing. */
#define ACADO_BLOCK_CONDENSING 0
/** Compute covariance matrix of the last state estimate. */
#define ACADO_COMPUTE_COVARIANCE_MATRIX 0
/** Flag indicating whether constraint values are hard-coded or not. */
#define ACADO_HARDCODED_CONSTRAINT_VALUES 1
/** Indicator for fixed initial state. */
#define ACADO_INITIAL_STATE_FIXED 1
/** Number of control/estimation intervals. */
#define ACADO_N 20
/** Number of online data values. */
#define ACADO_NOD 44
/** Number of path constraints. */
#define ACADO_NPAC 20
/** Number of control variables. */
#define ACADO_NU 6
/** Number of differential variables. */
#define ACADO_NX 10
/** Number of algebraic variables. */
#define ACADO_NXA 0
/** Number of differential derivative variables. */
#define ACADO_NXD 0
/** Number of references/measurements per node on the first N nodes. */
#define ACADO_NY 14
/** Number of references/measurements on the last (N + 1)st node. */
#define ACADO_NYN 9
/** Total number of QP optimization variables. */
#define ACADO_QP_NV 120
/** Number of integration steps per shooting interval. */
#define ACADO_RK_NIS 3
/** Number of Runge-Kutta stages per integration step. */
#define ACADO_RK_NSTAGES 1
/** Single versus double precision data type representation. */
#define ACADO_SINGLE_PRECISION 0
/** Providing interface for arrival cost. */
#define ACADO_USE_ARRIVAL_COST 0
/** Indicator for usage of non-hard-coded linear terms in the objective. */
#define ACADO_USE_LINEAR_TERMS 0
/** Indicator for type of fixed weighting matrices. */
#define ACADO_WEIGHTING_MATRICES_TYPE 1


/*
 * Globally used structure definitions
 */

/** The structure containing the user data.
 * 
 *  Via this structure the user "communicates" with the solver code.
 */
typedef struct ACADOvariables_
{
int dummy;
/** Matrix of size: 21 x 10 (row major format)
 * 
 *  Matrix containing 21 differential variable vectors.
 */
real_t x[ 210 ];

/** Matrix of size: 20 x 6 (row major format)
 * 
 *  Matrix containing 20 control variable vectors.
 */
real_t u[ 120 ];

/** Matrix of size: 21 x 44 (row major format)
 * 
 *  Matrix containing 21 online data vectors.
 */
real_t od[ 924 ];

/** Column vector of size: 280
 * 
 *  Matrix containing 20 reference/measurement vectors of size 14 for first 20 nodes.
 */
real_t y[ 280 ];

/** Column vector of size: 9
 * 
 *  Reference/measurement vector for the 21. node.
 */
real_t yN[ 9 ];

/** Matrix of size: 14 x 14 (row major format) */
real_t W[ 196 ];

/** Matrix of size: 9 x 9 (row major format) */
real_t WN[ 81 ];

/** Column vector of size: 10
 * 
 *  Current state feedback vector.
 */
real_t x0[ 10 ];


} ACADOvariables;

/** Private workspace used by the auto-generated code.
 * 
 *  Data members of this structure are private to the solver.
 *  In other words, the user code should not modify values of this 
 *  structure. 
 */
typedef struct ACADOworkspace_
{
real_t rk_dim10_swap;

/** Column vector of size: 10 */
real_t rk_dim10_bPerm[ 10 ];

real_t rk_ttt;

/** Row vector of size: 60 */
real_t rk_xxx[ 60 ];

/** Column vector of size: 10 */
real_t rk_kkk[ 10 ];

/** Matrix of size: 10 x 10 (row major format) */
real_t rk_A[ 100 ];

/** Column vector of size: 10 */
real_t rk_b[ 10 ];

/** Row vector of size: 10 */
int rk_dim10_perm[ 10 ];

/** Column vector of size: 10 */
real_t rk_rhsTemp[ 10 ];

/** Row vector of size: 160 */
real_t rk_diffsTemp2[ 160 ];

/** Column vector of size: 10 */
real_t rk_diffK[ 10 ];

/** Matrix of size: 10 x 16 (row major format) */
real_t rk_diffsPrev2[ 160 ];

/** Matrix of size: 10 x 16 (row major format) */
real_t rk_diffsNew2[ 160 ];

/** Row vector of size: 220 */
real_t state[ 220 ];

/** Column vector of size: 200 */
real_t d[ 200 ];

/** Column vector of size: 280 */
real_t Dy[ 280 ];

/** Column vector of size: 9 */
real_t DyN[ 9 ];

/** Matrix of size: 200 x 10 (row major format) */
real_t evGx[ 2000 ];

/** Matrix of size: 200 x 6 (row major format) */
real_t evGu[ 1200 ];

/** Column vector of size: 259 */
real_t objAuxVar[ 259 ];

/** Row vector of size: 60 */
real_t objValueIn[ 60 ];

/** Row vector of size: 238 */
real_t objValueOut[ 238 ];

/** Matrix of size: 200 x 10 (row major format) */
real_t Q1[ 2000 ];

/** Matrix of size: 200 x 14 (row major format) */
real_t Q2[ 2800 ];

/** Matrix of size: 120 x 6 (row major format) */
real_t R1[ 720 ];

/** Matrix of size: 120 x 14 (row major format) */
real_t R2[ 1680 ];

/** Matrix of size: 10 x 10 (row major format) */
real_t QN1[ 100 ];

/** Matrix of size: 10 x 9 (row major format) */
real_t QN2[ 90 ];

/** Column vector of size: 378 */
real_t conAuxVar[ 378 ];

/** Row vector of size: 60 */
real_t conValueIn[ 60 ];

/** Row vector of size: 340 */
real_t conValueOut[ 340 ];

/** Column vector of size: 400 */
real_t evH[ 400 ];

/** Matrix of size: 400 x 10 (row major format) */
real_t evHx[ 4000 ];

/** Matrix of size: 400 x 6 (row major format) */
real_t evHu[ 2400 ];

/** Column vector of size: 20 */
real_t evHxd[ 20 ];

/** Column vector of size: 2 */
real_t pocEvH[ 2 ];

/** Matrix of size: 2 x 10 (row major format) */
real_t pocEvHx[ 20 ];

/** Matrix of size: 2 x 6 (row major format) */
real_t pocEvHu[ 12 ];

/** Column vector of size: 2 */
real_t pocEvHxd[ 2 ];

/** Column vector of size: 210 */
real_t sbar[ 210 ];

/** Column vector of size: 10 */
real_t Dx0[ 10 ];

/** Matrix of size: 200 x 10 (row major format) */
real_t C[ 2000 ];

/** Matrix of size: 10 x 6 (row major format) */
real_t W1[ 60 ];

/** Matrix of size: 10 x 6 (row major format) */
real_t W2[ 60 ];

/** Matrix of size: 2100 x 6 (row major format) */
real_t E[ 12600 ];

/** Column vector of size: 210 */
real_t QDy[ 210 ];

/** Column vector of size: 10 */
real_t w1[ 10 ];

/** Column vector of size: 10 */
real_t w2[ 10 ];

/** Matrix of size: 120 x 120 (row major format) */
real_t H[ 14400 ];

/** Matrix of size: 426 x 120 (row major format) */
real_t A[ 51120 ];

/** Column vector of size: 120 */
real_t g[ 120 ];

/** Column vector of size: 120 */
real_t lb[ 120 ];

/** Column vector of size: 120 */
real_t ub[ 120 ];

/** Column vector of size: 426 */
real_t lbA[ 426 ];

/** Column vector of size: 426 */
real_t ubA[ 426 ];

/** Column vector of size: 120 */
real_t x[ 120 ];

/** Column vector of size: 546 */
real_t y[ 546 ];


} ACADOworkspace;

/* 
 * Forward function declarations. 
 */


/** Performs the integration and sensitivity propagation for one shooting interval.
 *
 *  \param rk_eta Working array of size 60 to pass the input values and return the results.
 *  \param resetIntegrator The internal memory of the integrator can be reset.
 *
 *  \return Status code of the integrator.
 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator );

/** Export of an ACADO symbolic function.
 *
 *  \param in Input to the exported function.
 *  \param out Output of the exported function.
 */
void acado_rhs(const real_t* in, real_t* out);

/** Export of an ACADO symbolic function.
 *
 *  \param in Input to the exported function.
 *  \param out Output of the exported function.
 */
void acado_diffs(const real_t* in, real_t* out);

/** Preparation step of the RTI scheme.
 *
 *  \return Status of the integration module. =0: OK, otherwise the error code.
 */
int acado_preparationStep(  );

/** Feedback/estimation step of the RTI scheme.
 *
 *  \return Status code of the qpOASES QP solver.
 */
int acado_feedbackStep(  );

/** Solver initialization. Must be called once before any other function call.
 *
 *  \return =0: OK, otherwise an error code of a QP solver.
 */
int acado_initializeSolver(  );

/** Initialize shooting nodes by a forward simulation starting from the first node.
 */
void acado_initializeNodesByForwardSimulation(  );

/** Shift differential variables vector by one interval.
 *
 *  \param strategy Shifting strategy: 1. Initialize node 21 with xEnd. 2. Initialize node 21 by forward simulation.
 *  \param xEnd Value for the x vector on the last node. If =0 the old value is used.
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd );

/** Shift controls vector by one interval.
 *
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void acado_shiftControls( real_t* const uEnd );

/** Get the KKT tolerance of the current iterate.
 *
 *  \return The KKT tolerance value.
 */
real_t acado_getKKT(  );

/** Calculate the objective value.
 *
 *  \return Value of the objective function.
 */
real_t acado_getObjective(  );


/* 
 * Extern declarations. 
 */

extern ACADOworkspace acadoWorkspace;
extern ACADOvariables acadoVariables;

/** @} */

#ifndef __MATLAB__
#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */
#endif /* __MATLAB__ */

#endif /* ACADO_COMMON_H */
