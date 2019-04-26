#include <math.h>
#include "mex.h"

void SquareTest(mxArray *array_in, mxArray *outputArrays[]);
void AddTest(mxArray *array_in1, mxArray *array_in2, mxArray *outputArrays[]);

/* mexFunction - main function called from MATLAB. */
void
mexFunction( int nlhs,  				// number of output arguments
			 mxArray *plhs[],			// output data containers
			 int nrhs,					// number of input arguments
			 const mxArray *prhs[]		// input data
		   )
{
    mxArray *array_in0;
    mxArray *array_in1;
    double *pr, s0, s1;
    int ind;
    
    if (nrhs < 1) {
        mexErrMsgTxt("Hello world requires at least one input argument.");
    } else if (nlhs > 1) {
        mexErrMsgTxt("Hello world requires only one output argument.");
    }
    
    if (mxIsComplex(prhs[0]) || !mxIsDouble(prhs[0])) {
        mexErrMsgTxt("Hello world requires DOUBLE ARRAY as first input argument.");
    }
    
    switch(nrhs) {
        case 1: { // only one input argument
			s0 = mxGetM(prhs[0]) * mxGetN(prhs[0]); // size of first input
			if (s0 != 1) {
				mexErrMsgTxt("Hello World requires a scalar input.");
			}
			array_in0 = (mxArray *)prhs[0];
            SquareTest(array_in0, plhs);
            break;
        }
        case 2: { // two input arguments
            if (mxIsComplex(prhs[1]) || !mxIsDouble(prhs[1])) {
                mexErrMsgTxt("Hello World requires two DOUBLE ARRAY input arguments.");
            }
            s0 = mxGetM(prhs[0]) * mxGetN(prhs[0]); // size of first input
            s1 = mxGetM(prhs[1]) * mxGetN(prhs[1]); // size of second input
			if (s0 != 1 || s1 != 1) {
				mexErrMsgTxt("Hello World requires a scalar input.");
			}
			array_in0 = (mxArray *)prhs[0];
			array_in1 = (mxArray *)prhs[1];
			AddTest(array_in0, array_in1, plhs);
            break;
        }
        default: {
            mexErrMsgTxt("Hello World: To many input arguments.");
            break;
        }
    }
}

void SquareTest(mxArray *array_in, mxArray *outputArrays[])
{	
    double *in = (double *)mxGetPr(array_in);    
    
    mxArray *array_out = mxCreateDoubleMatrix(1, 1, mxREAL);
    double *out = (double *)mxGetPr(array_out);
	
	out[0] = in[0] * in[0]; // Perform the action on the data
	
	// Set output dimensions if changed since the array was created
	//mxSetM(array_out, 1);
	//mxSetN(array_out, 1);
	
	// Set outputs
    outputArrays[0] = array_out;
}

void AddTest(mxArray *array_in1, mxArray *array_in2, mxArray *outputArrays[])
{
	double *in1 = (double *)mxGetPr(array_in1);    
	double *in2 = (double *)mxGetPr(array_in2);    
    
    mxArray *array_out = mxCreateDoubleMatrix(1, 1, mxREAL);
    double *out = (double *)mxGetPr(array_out);
	
	out[0] = in1[0] + in2[0]; // Perform the action on the data
	
	// Set output dimensions if changed since the array was created
	//mxSetM(array_out, 1);
	//mxSetN(array_out, 1);
	
	// Set outputs
    outputArrays[0] = array_out;
}
