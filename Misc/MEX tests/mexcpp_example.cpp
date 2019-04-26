/*==========================================================
 * mexcpp.cpp - example in MATLAB External Interfaces
 *
 * Illustrates how to use some C++ language features in a MEX-file.
 *
 * The routine simply defines a class, constructs a simple object,
 * and displays the initial values of the internal variables. It
 * then sets the data members of the object based on the input given
 * to the MEX-file and displays the changed values.
 *
 * This file uses the extension .cpp. Other common C++ extensions such
 * as .C, .cc, and .cxx are also supported.
 *
 * The calling syntax is:
 *
 *              mexcpp( num1, num2 )
 *
 * This is a MEX-file for MATLAB.
 * Copyright 1984-2016 The MathWorks, Inc.
 *
 *========================================================*/

#include <iostream>
#include "mex.h"

/* Test class */
class MyData {
  public:
    MyData(double v1 = 0, double v2 = 0);
    void setData(double v1, double v2);
    void display();
  private:
    double val1;
    double val2;
};

MyData::MyData(double v1, double v2) : val1(v1), val2(v2) {}

void MyData::setData(double v1, double v2) {
    val1 = v1;
    val2 = v2;
}

void MyData::display() {
    mexPrintf("Value1 = %g\n", val1);
    mexPrintf("Value2 = %g\n\n", val2);
}

/***********************************/

/* Creates, modifies and displays a MyData object */
void mexcpp(double num1, double num2) {
    mexPrintf("\nThe initialized data in object:\n");

    MyData d;                                // Create a MyData object
    d.display();                            // By default, the values should be initialized to zeros
    d.setData(num1,num2);                  // Set data members to incoming values

    mexPrintf("After setting the object's data to your input:\n");
    d.display();
}

/* The gateway function. */ 
void mexFunction(int nlhs, mxArray* plhs[],
                 int nrhs, const mxArray* prhs[]) {

    /* Check for proper number of arguments */
    if(nrhs != 2) {
        mexErrMsgIdAndTxt("MATLAB:mexcpp:nargin",
                          "MEXCPP requires two input arguments.");
    }
    if(nlhs != 0) {
        mexErrMsgIdAndTxt("MATLAB:mexcpp:nargout",
                          "MEXCPP requires no output argument.");
    }

    /* Check if the input is of proper type */
    if(!mxIsDouble(prhs[0]) ||                                    // not double
       mxIsComplex(prhs[0]) ||                                   // or complex
       !mxIsScalar(prhs[0])) {                                  // or not scalar
        mexErrMsgIdAndTxt("MATLAB:mexcpp:typeargin",
                          "First argument has to be double scalar.");
    }
    if(!mxIsDouble(prhs[1]) ||                                    // not double
       mxIsComplex(prhs[1]) ||                                   // or complex
       !mxIsScalar(prhs[1])) {                                  // or not scalar
        mexErrMsgIdAndTxt("MATLAB:mexcpp:typeargin",
                          "Second argument has to be double scalar.");
    }

    /* Acquire pointers to the input data */
    double* vin1 = mxGetPr(prhs[0]);
    double* vin2 = mxGetPr(prhs[1]);

    mexcpp(*vin1, *vin2);
}
