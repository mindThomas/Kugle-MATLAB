/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState q2;
    DifferentialState q3;
    DifferentialState x;
    DifferentialState y;
    DifferentialState dx;
    DifferentialState dy;
    DifferentialState s;
    DifferentialState ds;
    DifferentialState omega_ref_x;
    DifferentialState omega_ref_y;
    Control domega_ref_x;
    Control domega_ref_y;
    Control dds;
    Control velocity_slack;
    Control proximity_slack;
    OnlineData desiredVelocity; 
    OnlineData maxVelocity; 
    OnlineData maxAngle; 
    OnlineData maxOmegaRef; 
    OnlineData maxdOmegaRef; 
    OnlineData trajectoryLength; 
    OnlineData trajectoryStart; 
    OnlineData cx9; 
    OnlineData cx8; 
    OnlineData cx7; 
    OnlineData cx6; 
    OnlineData cx5; 
    OnlineData cx4; 
    OnlineData cx3; 
    OnlineData cx2; 
    OnlineData cx1; 
    OnlineData cx0; 
    OnlineData cy9; 
    OnlineData cy8; 
    OnlineData cy7; 
    OnlineData cy6; 
    OnlineData cy5; 
    OnlineData cy4; 
    OnlineData cy3; 
    OnlineData cy2; 
    OnlineData cy1; 
    OnlineData cy0; 
    OnlineData obs1_x; 
    OnlineData obs1_y; 
    OnlineData obs1_r; 
    OnlineData obs2_x; 
    OnlineData obs2_y; 
    OnlineData obs2_r; 
    OnlineData obs3_x; 
    OnlineData obs3_y; 
    OnlineData obs3_r; 
    OnlineData obs4_x; 
    OnlineData obs4_y; 
    OnlineData obs4_r; 
    OnlineData obs5_x; 
    OnlineData obs5_y; 
    OnlineData obs5_r; 
    IntermediateState intS1 = (s+trajectoryStart);
    IntermediateState intS2 = (cx0+cx1*intS1+cx2*pow(intS1,2.00000000000000000000e+00)+cx3*pow(intS1,3.00000000000000000000e+00)+cx4*pow(intS1,4.00000000000000000000e+00)+cx5*pow(intS1,5.00000000000000000000e+00)+cx6*pow(intS1,6.00000000000000000000e+00)+cx7*pow(intS1,7.00000000000000000000e+00)+cx8*pow(intS1,8.00000000000000000000e+00)+cx9*pow(intS1,9.00000000000000000000e+00));
    IntermediateState intS3 = (cy0+cy1*intS1+cy2*pow(intS1,2.00000000000000000000e+00)+cy3*pow(intS1,3.00000000000000000000e+00)+cy4*pow(intS1,4.00000000000000000000e+00)+cy5*pow(intS1,5.00000000000000000000e+00)+cy6*pow(intS1,6.00000000000000000000e+00)+cy7*pow(intS1,7.00000000000000000000e+00)+cy8*pow(intS1,8.00000000000000000000e+00)+cy9*pow(intS1,9.00000000000000000000e+00));
    IntermediateState intS4 = sqrt((dx*dx+dy*dy));
    IntermediateState intS5 = (2.00000000000000000000e+00*cx2*intS1+3.00000000000000000000e+00*cx3*pow(intS1,2.00000000000000000000e+00)+4.00000000000000000000e+00*cx4*pow(intS1,3.00000000000000000000e+00)+5.00000000000000000000e+00*cx5*pow(intS1,4.00000000000000000000e+00)+6.00000000000000000000e+00*cx6*pow(intS1,5.00000000000000000000e+00)+7.00000000000000000000e+00*cx7*pow(intS1,6.00000000000000000000e+00)+8.00000000000000000000e+00*cx8*pow(intS1,7.00000000000000000000e+00)+9.00000000000000000000e+00*cx9*pow(intS1,8.00000000000000000000e+00)+cx1);
    IntermediateState intS6 = (2.00000000000000000000e+00*cy2*intS1+3.00000000000000000000e+00*cy3*pow(intS1,2.00000000000000000000e+00)+4.00000000000000000000e+00*cy4*pow(intS1,3.00000000000000000000e+00)+5.00000000000000000000e+00*cy5*pow(intS1,4.00000000000000000000e+00)+6.00000000000000000000e+00*cy6*pow(intS1,5.00000000000000000000e+00)+7.00000000000000000000e+00*cy7*pow(intS1,6.00000000000000000000e+00)+8.00000000000000000000e+00*cy8*pow(intS1,7.00000000000000000000e+00)+9.00000000000000000000e+00*cy9*pow(intS1,8.00000000000000000000e+00)+cy1);
    IntermediateState intS7 = intS5/sqrt((pow(intS5,2.00000000000000000000e+00)+pow(intS6,2.00000000000000000000e+00)));
    IntermediateState intS8 = intS6/sqrt((pow(intS5,2.00000000000000000000e+00)+pow(intS6,2.00000000000000000000e+00)));
    IntermediateState intS9 = (-intS2+x);
    IntermediateState intS10 = (-intS3+y);
    IntermediateState intS11 = (-intS10*intS7+intS8*intS9);
    IntermediateState intS12 = (dx*intS7+dy*intS8);
    IntermediateState intS13 = (-ds+intS12);
    IntermediateState intS14 = ((-intS7)*intS9-intS10*intS8);
    IntermediateState intS15 = (intS12-maxVelocity);
    IntermediateState intS16 = (-obs1_r+sqrt(((-obs1_x+x)*(-obs1_x+x)+(-obs1_y+y)*(-obs1_y+y))));
    IntermediateState intS17 = (-obs2_r+sqrt(((-obs2_x+x)*(-obs2_x+x)+(-obs2_y+y)*(-obs2_y+y))));
    IntermediateState intS18 = (-obs3_r+sqrt(((-obs3_x+x)*(-obs3_x+x)+(-obs3_y+y)*(-obs3_y+y))));
    IntermediateState intS19 = (-obs4_r+sqrt(((-obs4_x+x)*(-obs4_x+x)+(-obs4_y+y)*(-obs4_y+y))));
    IntermediateState intS20 = (-obs5_r+sqrt(((-obs5_x+x)*(-obs5_x+x)+(-obs5_y+y)*(-obs5_y+y))));
    BMatrix acadodata_M1;
    acadodata_M1.read( "kugle_mpc_obstacles_codegen_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "kugle_mpc_obstacles_codegen_data_acadodata_M2.txt" );
    Function acadodata_f2;
    acadodata_f2 << intS9;
    acadodata_f2 << intS10;
    acadodata_f2 << q2;
    acadodata_f2 << q3;
    acadodata_f2 << omega_ref_x;
    acadodata_f2 << omega_ref_y;
    acadodata_f2 << intS13;
    acadodata_f2 << intS15;
    acadodata_f2 << domega_ref_x;
    acadodata_f2 << domega_ref_y;
    acadodata_f2 << velocity_slack;
    acadodata_f2 << proximity_slack;
    Function acadodata_f3;
    acadodata_f3 << intS9;
    acadodata_f3 << intS10;
    acadodata_f3 << q2;
    acadodata_f3 << q3;
    acadodata_f3 << omega_ref_x;
    acadodata_f3 << omega_ref_y;
    acadodata_f3 << intS13;
    acadodata_f3 << intS15;
    IntermediateState intS21 = sin(5.00000000000000000000e-01*maxAngle);
    DifferentialEquation acadodata_f1;
    acadodata_f1 << dot(q2) == 5.00000000000000000000e-01*omega_ref_x;
    acadodata_f1 << dot(q3) == 5.00000000000000000000e-01*omega_ref_y;
    acadodata_f1 << dot(x) == dx;
    acadodata_f1 << dot(y) == dy;
    acadodata_f1 << dot(dx) == 1.45288999999999930424e+01*q3;
    acadodata_f1 << dot(dy) == (-1.45288999999999930424e+01)*q2;
    acadodata_f1 << dot(s) == ds;
    acadodata_f1 << dot(ds) == dds;
    acadodata_f1 << dot(omega_ref_x) == domega_ref_x;
    acadodata_f1 << dot(omega_ref_y) == domega_ref_y;

    OCP ocp1(0, 2, 20);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f2);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f3);
    ocp1.subjectTo(AT_END, q2 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_END, q3 == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_END, omega_ref_x == 0.00000000000000000000e+00);
    ocp1.subjectTo(AT_END, omega_ref_y == 0.00000000000000000000e+00);
    ocp1.subjectTo((-intS21+q2) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((-intS21-q2) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((-intS21+q3) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((-intS21-q3) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((-maxOmegaRef+omega_ref_x) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((-maxOmegaRef-omega_ref_x) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((-maxOmegaRef+omega_ref_y) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((-maxOmegaRef-omega_ref_y) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((domega_ref_x-maxdOmegaRef) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((-domega_ref_x-maxdOmegaRef) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((domega_ref_y-maxdOmegaRef) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((-domega_ref_y-maxdOmegaRef) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((-desiredVelocity+intS4-velocity_slack) <= 0.00000000000000000000e+00);
    ocp1.subjectTo((intS1-trajectoryLength) <= 0.00000000000000000000e+00);
    ocp1.subjectTo(intS1 >= 0.00000000000000000000e+00);
    ocp1.subjectTo((intS16+proximity_slack) >= 0.00000000000000000000e+00);
    ocp1.subjectTo((intS17+proximity_slack) >= 0.00000000000000000000e+00);
    ocp1.subjectTo((intS18+proximity_slack) >= 0.00000000000000000000e+00);
    ocp1.subjectTo((intS19+proximity_slack) >= 0.00000000000000000000e+00);
    ocp1.subjectTo((intS20+proximity_slack) >= 0.00000000000000000000e+00);
    ocp1.subjectTo(proximity_slack >= 0.00000000000000000000e+00);
    ocp1.setModel( acadodata_f1 );


    ocp1.setNU( 5 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 42 );
    OCPexport ExportModule1( ocp1 );
    ExportModule1.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule1.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HESSIAN_APPROXIMATION");
    options_flag = ExportModule1.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: DISCRETIZATION_TYPE");
    options_flag = ExportModule1.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: SPARSE_QP_SOLUTION");
    options_flag = ExportModule1.set( INTEGRATOR_TYPE, INT_IRK_GL2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: INTEGRATOR_TYPE");
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 60 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    options_flag = ExportModule1.set( QP_SOLVER, QP_QPOASES3 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: QP_SOLVER");
    options_flag = ExportModule1.set( LEVENBERG_MARQUARDT, 1.000000E-10 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: LEVENBERG_MARQUARDT");
    options_flag = ExportModule1.set( HOTSTART_QP, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HOTSTART_QP");
    options_flag = ExportModule1.set( FIX_INITIAL_STATE, YES );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: FIX_INITIAL_STATE");
    uint export_flag;
    export_flag = ExportModule1.exportCode( "kugle_mpc_obstacles_codegen" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

