#include <stdlib.h>
#include "mex.h"

#define RESET_WHEN_NAN 1   // no reset ( = 0 ), reset with previous traj ( = 1 )
#define KKT_MIN -1
#define KKT_MAX 1e15

#include "acado_common.h"
// #include "acado_auxiliary_functions.h"

ACADOvariables * acadoVariablesPtr = NULL;
ACADOworkspace * acadoWorkspacePtr = NULL;

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

void Acado_Initialize(double * xInit, double * uInit, double * Wmat, double * WNmat, double * refInit, double * odInit);
void Acado_Step(
	double *in_x, double *in_ref, double *in_od, double in_nIter,
	double *out_u0, double *out_xTraj, double *out_uTraj, double *out_kktTol, double *out_status, double *out_nIter, double *out_objVal
);

void Matrix_Print_RowMajor(double * matrix, int rows, int cols);
void Matrix_Print_ColMajor(double * matrix, int rows, int cols);

void exitFcn() {
	if (acadoVariablesPtr != NULL)
		mxFree(acadoVariablesPtr);
	
	if (acadoWorkspacePtr != NULL)
		mxFree(acadoWorkspacePtr);
}
 
/* mexFunction - main function called from MATLAB. */ 
void
mexFunction( int nlhs,  				// number of output arguments
			 mxArray *plhs[],			// output data containers
			 int nrhs,					// number of input arguments
			 const mxArray *prhs[]		// input data
		   )
{
	if (nrhs < 1)
		mexErrMsgTxt("ACADO MPC MEX usage:\n\tACADO_MPC_MEX(0, xInit, uInit, Wmat, WNmat, refInit, odInit) - initialization of MPC\n\t[u0, xTraj, uTraj, kktTol, status, nIter, objVal] = ACADO_MPC_MEX(1, x, ref, od, nIter) - execute one step with the MPC");
		
	if (!mxIsDouble(prhs[0]) || mxGetM(prhs[0]) != 1 || mxGetN(prhs[0]) != 1)
		mexErrMsgTxt("First argument has to be 0 (initialize) or 1 (step)");
	
	double flag = *(double *)mxGetPr((mxArray *)prhs[0]);
	
	if (acadoVariablesPtr == NULL) { 
		mexPrintf("First call to MEX-file - initializing memory\n");
        acadoVariablesPtr = (ACADOvariables *)mxCalloc(1,sizeof(ACADOvariables));
		acadoWorkspacePtr = (ACADOworkspace *)mxCalloc(1,sizeof(ACADOworkspace));
        mexMakeMemoryPersistent(acadoVariablesPtr);
		mexMakeMemoryPersistent(acadoWorkspacePtr);
        mexAtExit(exitFcn);
		
		// Zero initialize using created object
		memcpy(acadoVariablesPtr, &acadoVariables, sizeof(ACADOvariables));
		memcpy(acadoWorkspacePtr, &acadoWorkspace, sizeof(ACADOworkspace));
	}
	
	// Load data from persistent ACADO variables into temporary variables
	memcpy(&acadoVariables, acadoVariablesPtr, sizeof(ACADOvariables));
	memcpy(&acadoWorkspace, acadoWorkspacePtr, sizeof(ACADOworkspace));
		
	if (flag == 0) { // initialize
		//ACADO_MPC_MEX(0, xInit, uInit, Wmat, WNmat, refInit, odInit) - initialization of MPC		
		if (nrhs != 7 || nlhs != 0)
			mexErrMsgTxt("Incorrect number of input or output arguments");
		
		/* Parse inputs */
		if (!mxIsDouble(prhs[1]) || mxGetM(prhs[1]) != ACADO_NX || mxGetN(prhs[1]) != 1) {         // x: rows=ACADO_NX, cols=1
			mexPrintf("'xInit' needs to be a double vector of size [%d x 1]", ACADO_NX);
			mexErrMsgTxt("Incorrect input");
		}
		double *in_xInit = (double *)mxGetPr((mxArray *)prhs[1]);  
		
		if (!mxIsDouble(prhs[2]) || mxGetM(prhs[2]) != ACADO_NU || mxGetN(prhs[2]) != 1) {         
			mexPrintf("'uInit' needs to be a double vector of size [%d x 1]", ACADO_NU);
			mexErrMsgTxt("Incorrect input");
		}
		double *in_uInit = (double *)mxGetPr((mxArray *)prhs[2]);  

		if (!mxIsDouble(prhs[3]) || mxGetM(prhs[3]) != ACADO_NY || mxGetN(prhs[3]) != ACADO_NY) {         
			mexPrintf("'Wmat' needs to be a double matrix of size [%d x %d]", ACADO_NY, ACADO_NY);
			mexErrMsgTxt("Incorrect input");
		}
		double *in_Wmat = (double *)mxGetPr((mxArray *)prhs[3]);  

		if (!mxIsDouble(prhs[4]) || mxGetM(prhs[4]) != ACADO_NYN || mxGetN(prhs[4]) != ACADO_NYN) {         
			mexPrintf("'WNmat' needs to be a double matrix of size [%d x %d]", ACADO_NYN, ACADO_NYN);
			mexErrMsgTxt("Incorrect input");
		}
		double *in_WNmat = (double *)mxGetPr((mxArray *)prhs[4]);  
		
		if (!mxIsDouble(prhs[5]) || mxGetM(prhs[5]) != ACADO_N+1 || mxGetN(prhs[5]) != ACADO_NY) {         
			mexPrintf("'refInit' needs to be a double matrix of size [%d x %d]", ACADO_N+1, ACADO_NY);				
			mexErrMsgTxt("Incorrect input");
		}
		double *in_refInit = (double *)mxGetPr((mxArray *)prhs[5]);  		
	
		if (!mxIsDouble(prhs[6]) || mxGetM(prhs[6]) != ACADO_N+1 || mxGetN(prhs[6]) != ACADO_NOD) {         			
			mexPrintf("'odInit' needs to be a double matrix of size [%d x %d]", ACADO_N+1, ACADO_NOD);	
			mexErrMsgTxt("Incorrect input");
		}
		double *in_odInit = (double *)mxGetPr((mxArray *)prhs[6]);  
	
		Acado_Initialize(in_xInit, in_uInit, in_Wmat, in_WNmat, in_refInit, in_odInit);	
				
		mexPrintf("acadoVariables.x:\n"); Matrix_Print_RowMajor(acadoVariables.x, ACADO_N+1, ACADO_NX);
		mexPrintf("acadoVariables.u:\n"); Matrix_Print_RowMajor(acadoVariables.u, ACADO_N, ACADO_NU);
		
		mexPrintf("acadoVariables.y:\n"); Matrix_Print_RowMajor(acadoVariables.y, ACADO_N, ACADO_NY);
		mexPrintf("acadoVariables.yN:\n"); Matrix_Print_RowMajor(acadoVariables.yN, 1, ACADO_NYN);
		
		mexPrintf("acadoVariables.od:\n"); Matrix_Print_RowMajor(acadoVariables.od, ACADO_N+1, ACADO_NOD);
		
		mexPrintf("ACADO MPC initialized\n");
	}
	
	else if (flag == 1) { // step
		//[u0, xTraj, uTraj, kktTol, status, nIter, objVal] = ACADO_MPC_MEX(1, x, ref, od, nIter) - execute one step with the MPC		
		if (nrhs != 5 || nlhs != 7)
			mexErrMsgTxt("Incorrect number of input or output arguments");		
		
		/* Parse inputs */
		if (!mxIsDouble(prhs[1]) || mxGetM(prhs[1]) != ACADO_NX || mxGetN(prhs[1]) != 1) {         // x: rows=ACADO_NX, cols=1
			mexPrintf("'x' needs to be a double vector of size [%d x 1]", ACADO_NX);
			mexErrMsgTxt("Incorrect input");
		}
		double *in_x = (double *)mxGetPr((mxArray *)prhs[1]);  
		
		if (!mxIsDouble(prhs[2]) || mxGetM(prhs[2]) != ACADO_N+1 || mxGetN(prhs[2]) != ACADO_NY) { 	// ref: rows=ACADO_N+1, cols=ACADO_NY	
			mexPrintf("'ref' needs to be a double matrix of size [%d x %d]", ACADO_N+1, ACADO_NY);				
			mexErrMsgTxt("Incorrect input");
		}
		double *in_ref = (double *)mxGetPr((mxArray *)prhs[2]);		
		
		if (!mxIsDouble(prhs[3]) || mxGetM(prhs[3]) != ACADO_N+1 || mxGetN(prhs[3]) != ACADO_NOD) {   // od: rows=ACADO_N+1, cols=ACADO_NOD
			mexPrintf("'od' needs to be a double matrix of size [%d x %d]", ACADO_N+1, ACADO_NOD);	
			mexErrMsgTxt("Incorrect input");
		}
		double *in_od = (double *)mxGetPr((mxArray *)prhs[3]);
		
		if (!mxIsDouble(prhs[4]) || mxGetM(prhs[4]) != 1 || mxGetN(prhs[4]) != 1)
			mexPrintf("'nIter' needs to be a double", ACADO_NX);	
		double in_nIter = *(double *)mxGetPr((mxArray *)prhs[4]);

		
		/* Define outputs */
		mxArray *out_u0_ = mxCreateDoubleMatrix(ACADO_NU, 1, mxREAL);
		double *out_u0 = (double *)mxGetPr(out_u0_);
		plhs[0] = out_u0_;
		
		mxArray *out_xTraj_ = mxCreateDoubleMatrix(ACADO_N+1, ACADO_NX, mxREAL); // out_xTraj: rows=ACADO_N+1, cols=ACADO_NX
		double *out_xTraj = (double *)mxGetPr(out_xTraj_);
		plhs[1] = out_xTraj_;
		
		mxArray *out_uTraj_ = mxCreateDoubleMatrix(ACADO_N, ACADO_NU, mxREAL); // out_uTraj: rows=ACADO_N, cols=ACADO_NU
		double *out_uTraj = (double *)mxGetPr(out_uTraj_);
		plhs[2] = out_uTraj_;	

		mxArray *out_kktTol_ = mxCreateDoubleMatrix(1, 1, mxREAL);
		double *out_kktTol = (double *)mxGetPr(out_kktTol_);
		plhs[3] = out_kktTol_;				
		
		mxArray *out_status_ = mxCreateDoubleMatrix(1, 1, mxREAL);
		double *out_status = (double *)mxGetPr(out_status_);
		plhs[4] = out_status_;			

		mxArray *out_nIter_ = mxCreateDoubleMatrix(1, 1, mxREAL);
		double *out_nIter = (double *)mxGetPr(out_nIter_);
		plhs[5] = out_nIter_;			

		mxArray *out_objVal_ = mxCreateDoubleMatrix(1, 1, mxREAL);
		double *out_objVal = (double *)mxGetPr(out_objVal_);
		plhs[6] = out_objVal_;	

		Acado_Step(
			in_x, in_ref, in_od, in_nIter,
			out_u0, out_xTraj, out_uTraj, out_kktTol, out_status, out_nIter, out_objVal
		);
	}			
	
	// After doing modifications to the data (running the solver etc.) copy the data back into persistent memory
	memcpy(acadoVariablesPtr, &acadoVariables, sizeof(ACADOvariables));
	memcpy(acadoWorkspacePtr, &acadoWorkspace, sizeof(ACADOworkspace));	
}


void Acado_Initialize(double * xInit, double * uInit, double * Wmat, double * WNmat, double * refInit, double * odInit)
{
	int i, j;
	
	/* Initialize the solver. */
	acado_initializeSolver();	
	
	for( i=0; i < ACADO_N+1; ++i ) {
		for( j=0; j < ACADO_NX; ++j ) acadoVariables.x[i*ACADO_NX+j] = xInit[j];
    }
    /*#if ACADO_NXA > 0   // algebraic variables
    for( i=0; i < ACADO_N; ++i ) {
        for( j=0; j < ACADO_NXA; ++j ) acadoVariables.z[i*ACADO_NXA+j] = zInit[j];
    }
    #endif*/
    for( i=0; i < ACADO_N; ++i ) {
        for( j=0; j < ACADO_NU; ++j ) acadoVariables.u[i*ACADO_NU+j] = uInit[j];
    }
	
	for( i=0; i < ACADO_N; ++i ) {
        for( j=0; j < ACADO_NY; ++j ) acadoVariables.y[i*ACADO_NY+j] = refInit[j*(ACADO_N+1)+i];
    }
    for( j=0; j < ACADO_NYN; ++j ) acadoVariables.yN[j] = refInit[j*(ACADO_N+1) + ACADO_N];

    #if ACADO_NOD
    for( i=0; i < ACADO_N+1; ++i ) {
        for( j=0; j < ACADO_NOD; ++j ) acadoVariables.od[i*ACADO_NOD+j] = odInit[j*(ACADO_N+1)+i];
    }
    #endif

	/* ACADO Matrices are stored in row-major order 
	uint i,j;
 	for( i=0; i < getNumRows(); i++ ){
        for( j=0; j < getNumCols(); j++ ){
            if( types[i][j] != SBMT_ZERO ){
                types   [i][j]  = SBMT_DENSE;
                elements[i][j] *= scalar    ;
            }
        }
    }
	*/
	
    for( i = 0; i < (ACADO_NYN); ++i )  {
        for( j = 0; j < ACADO_NYN; ++j ) {
            acadoVariables.WN[i*ACADO_NYN+j] = WNmat[i*ACADO_NYN+j];
        }
    }
    for( i = 0; i < (ACADO_NY); ++i )  {
        for( j = 0; j < ACADO_NY; ++j ) {
            acadoVariables.W[i*ACADO_NY+j] = Wmat[i*ACADO_NY+j];
        }
    }
    
    /*#if VARYING_BOUNDS
    for( i = 0; i < QPOASES_NVMAX; ++i )  {
        acadoVariables.lbValues[i] = bValues[i];
        acadoVariables.ubValues[i] = bValues[QPOASES_NVMAX+QPOASES_NCMAX+i];
    }
    
    for( i = 0; i < QPOASES_NCMAX; ++i )  {
        acadoVariables.lbAValues[i] = bValues[QPOASES_NVMAX+i];
        acadoVariables.ubAValues[i] = bValues[QPOASES_NVMAX+QPOASES_NCMAX+QPOASES_NVMAX+i];
    }
    #endif*/

	/* Prepare first step */
	acado_preparationStep();
}

void Acado_Step(
	double *in_x, double *in_ref, double *in_od, double in_nIter,
	double *out_u0, double *out_xTraj, double *out_uTraj, double *out_kktTol, double *out_status, double *out_nIter, double *out_objVal
)
{
    int i, j, status;
	double sumIter;
    real_t kkt;

    //real_t *in_x, *in_ref, *in_W, *in_WN, *in_bValues, *in_od, *in_nIter;
    //real_t *out_u0, *out_xTraj, *out_uTraj, *out_kktTol, *out_cpuTime, *out_status, *out_nIter, *out_objVal, *xInit, *zInit, *uInit;
    
    #if RESET_WHEN_NAN == 1
        real_t x_prev[ (ACADO_N+1)*ACADO_NX ];
        #if ACADO_NXA > 0
        real_t z_prev[ ACADO_N*ACADO_NXA ];
        #endif
        real_t u_prev[ ACADO_N*ACADO_NU ];
    #endif    
    
    for( i=0; i < ACADO_NX; ++i ) acadoVariables.x0[i] = in_x[i];   
	
    for( i=0; i < ACADO_N; ++i ) {
        for( j=0; j < ACADO_NY; ++j ) acadoVariables.y[i*ACADO_NY+j] = in_ref[j*(ACADO_N+1)+i];
    }
    for( j=0; j < ACADO_NYN; ++j ) acadoVariables.yN[j] = in_ref[j*(ACADO_N+1) + ACADO_N];

    #if ACADO_NOD
    for( i=0; i < ACADO_N+1; ++i ) {
        for( j=0; j < ACADO_NOD; ++j ) acadoVariables.od[i*ACADO_NOD+j] = in_od[j*(ACADO_N+1)+i];
    }
    #endif
    
	/* Do not update weight matrices */
    /*for( i=0; i < ACADO_NY; ++i ) {
        for( j=0; j < ACADO_NY; ++j ) acadoVariables.W[i*ACADO_NY+j] = (double)(*in_W[i*ACADO_NY+j]);
    }
    for( i=0; i < ACADO_NYN; ++i ) {
        for( j=0; j < ACADO_NYN; ++j ) acadoVariables.WN[i*ACADO_NYN+j] = (double)(*in_WN[i*ACADO_NYN+j]);
    }*/
    
    /*#if VARYING_BOUNDS
    for( i = 0; i < QPOASES_NVMAX; ++i )  {
        acadoVariables.lbValues[i] = (double)(*in_bValues[i]);
        acadoVariables.ubValues[i] = (double)(*in_bValues[QPOASES_NVMAX+QPOASES_NCMAX+i]);
    }
    
    for( i = 0; i < QPOASES_NCMAX; ++i )  {
        acadoVariables.lbAValues[i] = (double)(*in_bValues[QPOASES_NVMAX+i]);
        acadoVariables.ubAValues[i] = (double)(*in_bValues[QPOASES_NVMAX+QPOASES_NCMAX+QPOASES_NVMAX+i]);
    }
    #endif*/
    
    #if RESET_WHEN_NAN == 1
        for( i = 0; i < (ACADO_N+1)*ACADO_NX; ++i ) x_prev[ i ] = acadoVariables.x[ i ];
        #if ACADO_NXA > 0
        for( i = 0; i < ACADO_N*ACADO_NXA; ++i ) z_prev[ i ] = acadoVariables.z[ i ];
        #endif
        for( i = 0; i < ACADO_N*ACADO_NU; ++i ) u_prev[ i ] = acadoVariables.u[ i ];
    #endif
        
    status = acado_feedbackStep( );
    sumIter = acado_getNWSR();     
    
    for( i = 1; i < in_nIter; i++ ) {
        acado_preparationStep( );
        status = acado_feedbackStep( );
        sumIter += acado_getNWSR();     
    }

    #if RESET_WHEN_NAN == 1
        kkt = acado_getKKT();
        if( kkt < KKT_MIN || kkt > KKT_MAX || kkt != kkt ) {
            for( i = 0; i < (ACADO_N+1)*ACADO_NX; ++i ) acadoVariables.x[ i ] = x_prev[ i ];
            #if ACADO_NXA > 0
            for( i = 0; i < ACADO_N*ACADO_NXA; ++i ) acadoVariables.z[ i ] = z_prev[ i ];
            #endif
            for( i = 0; i < ACADO_N*ACADO_NU; ++i ) acadoVariables.u[ i ] = u_prev[ i ];
            
            status = -30; // PERFORMED RESET TO PREVIOUS TRAJECTORIES BECAUSE OF NAN
        }
    #endif   
	
    for( i=0; i < ACADO_N+1; ++i ) {
        for( j = 0; j < ACADO_NX; ++j ) {
            out_xTraj[j*(ACADO_N+1)+i] = acadoVariables.x[i*ACADO_NX+j];
        }
    }
    for( i=0; i < ACADO_N; ++i ) {
        for( j = 0; j < ACADO_NU; ++j ) {
            out_uTraj[j*(ACADO_N)+i] = acadoVariables.u[i*ACADO_NU+j];
        }
    }   

    /* return outputs and prepare next iteration */
    for( i=0; i < ACADO_NU; ++i ) out_u0[i] = acadoVariables.u[i];
    *out_kktTol = acado_getKKT( );
    *out_status = status;    
    *out_nIter = (double) sumIter;
    *out_objVal = acado_getObjective( );
	
	/* Shift the initialization (look at acado_common.h). */
    acado_shiftStates(1, 0, 0); // Shifting strategy: 1. Initialize node 51 with xEnd. 2. Initialize node 51 by forward simulation.
	acado_shiftControls( 0 );
    
	/* Prepare for the next step. */
	acado_preparationStep();
}	
	
void Matrix_Print_RowMajor(double * matrix, int rows, int cols)
{
  for (int m = 0; m < rows; m++) {
    mexPrintf(" ");
    for (int n = 0; n < cols; n++) {
        mexPrintf("%8.4f ", matrix[cols*m + n]);        
    }
    mexPrintf("\n");
  }
}	

void Matrix_Print_ColMajor(double * matrix, int rows, int cols)
{
  for (int m = 0; m < rows; m++) {
    mexPrintf(" ");
    for (int n = 0; n < cols; n++) {
        mexPrintf("%8.4f ", matrix[rows*n + m]);        
    }
    mexPrintf("\n");
  }
}	