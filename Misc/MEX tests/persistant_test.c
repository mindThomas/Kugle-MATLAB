// test.c
#include <stdlib.h>
#include "mex.h"

static double *myarray = NULL;
double *pr;

void exitFcn() {
	if (myarray != NULL)
		mxFree(myarray);
}
 
void
mexFunction( int nlhs,  				// number of output arguments
			 mxArray *plhs[],			// output data containers
			 int nrhs,					// number of input arguments
			 const mxArray *prhs[]		// input data
		   )
{
	if (nrhs < 1)
		mexErrMsgTxt("Must have one non-string input");
	if (mxIsChar(prhs[0]))
		mexErrMsgTxt("Must have one non-string input");
   
   if (myarray==NULL) {  
 /* since myarray is initialized to NULL, we know
       this is the first call of the MEX-function 
   after it was loaded.  Therefore, we should
   set up myarray and the exit function. */
 /* Allocate array. Use mexMackMemoryPersistent to make the allocated memory persistent in subsequent calls*/
		 mexPrintf("First call to MEX-file\n");
         myarray = mxCalloc(1,8);
         mexMakeMemoryPersistent(myarray);
         mexAtExit(exitFcn);
   }
   mexPrintf("Old string was '%f'.\n",myarray[0]);
   pr = mxGetPr(prhs[0]);
   mexPrintf("New string is '%f'.\n",pr[0]);
   memcpy((char*)myarray,(char*)mxGetPr(prhs[0]), sizeof(double));
 }