clear all;
scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir, 'functions'));
addpath(fullfile(scriptDir, '../../Parameters'));
addpath(fullfile(scriptDir, '../../Misc'));
load(fullfile(scriptDir, '../../Linearization/generated/SteadyStateAccelerationConstants.mat'));
cd(fullfile(scriptDir, 'generated')); % move to generated folder if we are not already there for export    
Parameters_MPC; % load MPC parameters such as sample rate and horizon length

    problemName = 'kugle_mpc_obstacles';
    enableVerboseMPC = true;
    acadoSet('problemname', [problemName '_codegen']); 

    %% Define sample rate and horizon length
    %Ts_MPC = 1/10; % Sample rate, Ts_MPC, is defined in 'Parameters_MPC.m'
    %N = 20;        % Horizon length, N, is defined in 'Parameters_MPC.m'
    
    %% Define variables (both internal and external/inputs)
    % OBS. The order of construction defines the order in the chi vector
    DifferentialState q2; % OBS. These are the x- and y-axis vector elements of the quaternion.
    DifferentialState q3; % In the thesis these are defined as q1 and q2
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
    Control angle_slack;
    Control proximity_slack;     
    
    %OnlineData represents data that can be passed to the solver online (real-time)
    OnlineData desiredVelocity;
    OnlineData maxVelocity;    
    OnlineData maxAngle;
    OnlineData maxOmegaRef; 
    OnlineData maxdOmegaRef; 
    OnlineData trajectoryLength;
    OnlineData trajectoryStart;    
    
    % Reference polynomial coefficients (for up to 9th order polynomial)
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
    
    OnlineData proximityOffset;
    OnlineData proximityScale;

    % Evaluate polynomial based on s variable
    % Intermediate states helps to speed up the Automatic Differentiation of ACADO Symbolic
    s_ = acado.IntermediateState(s + trajectoryStart);
    x_ref = acado.IntermediateState(cx9*s_^9 + cx8*s_^8 + cx7*s_^7 + cx6*s_^6 + cx5*s_^5 + cx4*s_^4 + cx3*s_^3 + cx2*s_^2 + cx1*s_ + cx0);
    y_ref = acado.IntermediateState(cy9*s_^9 + cy8*s_^8 + cy7*s_^7 + cy6*s_^6 + cy5*s_^5 + cy4*s_^4 + cy3*s_^3 + cy2*s_^2 + cy1*s_ + cy0);
    
    %TIME t; % the TIME
    %Parameter l; % parameters are time varying variables
    %Variable
    %Vector  (use BVector instead)
    %Matrix  (use BMatrix instead)
    %ExportVariable
        
    %input1 = acado.MexInput; % inputs into the MEX function in the order of construction
    %input2 = acado.MexInputVector; % myexample RUN(10, [0 1 2 3], eye(3,3)),
    %input3 = acado.MexInputMatrix;
    
    % Quaternion to Acceleration coefficient can either be hardcoded or taken as input through OnlineData (since I have not been able to find any other/better way?)
    % In this case we load it from the Linearized model
    load(fullfile(scriptDir, '../../Linearization/generated/SteadyStateAccelerationConstants.mat'));   
    %OnlineData QuaternionToAccelerationCoefficient;

    %% Define differential equation (model of plant) - (see page ?? in ACADO MATLAB manual)
    f = acado.DifferentialEquation();  
    % possibility 1: link a Matlab ODE
    % f.linkMatlabODE('LinearMPCmodel_acado'); % however this method will slow down the generated code
    % possibility 2: write down the ODE directly in ACADO syntax    
    f.add(dot(q2) == 1/2 * omega_ref_x);
    f.add(dot(q3) == 1/2 * omega_ref_y);
    f.add(dot(x) == dx);
    f.add(dot(y) == dy);
    f.add(dot(dx) == AccelerationConstant_q3_to_ddx*q3);
    f.add(dot(dy) == AccelerationConstant_q2_to_ddy*q2);
    f.add(dot(s) == ds);  
    f.add(dot(ds) == dds);
    f.add(dot(omega_ref_x) == domega_ref_x);
    f.add(dot(omega_ref_y) == domega_ref_y);
    % possibility 4: write down the discretized ODE directly in ACADO syntax
    % Note that the start time, end time, step size (ts), and the number N of control intervals should be chosen in such a way that the relation
    % (t_end - t_start) / ts = N*i    (should hold for some integer, i)
    % f = acado.DiscretizedDifferentialEquation(ts); 
    % f.add(next(q2) == q2 + ts * 1/2 * omega_ref_x);           
    
    %% Define optimal control problem (see page 29 in ACADO MATLAB manual)
    %ocp = acado.OCP(0.0, tEnd, N); % note that if the time is optimized, the output time will be normalized between [0:1]
    ocp = acado.OCP(0.0, N*Ts_MPC, N);   
    
    % Define intermediate states and errors used in cost
    velocity = acado.IntermediateState(sqrt(dx*dx + dy*dy));
    
    % Heading definitions
    dx_ref = acado.IntermediateState(9*cx9*s_^8 + 8*cx8*s_^7 + 7*cx7*s_^6 + 6*cx6*s_^5 + 5*cx5*s_^4 + 4*cx4*s_^3 + 3*cx3*s_^2 + 2*cx2*s_ + cx1);
    dy_ref = acado.IntermediateState(9*cy9*s_^8 + 8*cy8*s_^7 + 7*cy7*s_^6 + 6*cy6*s_^5 + 5*cy5*s_^4 + 4*cy4*s_^3 + 3*cy3*s_^2 + 2*cy2*s_ + cy1);    
    % yaw_ref = atan2(dy_ref, dx_ref); % instead of using this we replace using the relationships
    %   cos(atan2(y,x)) = x / sqrt(x^2+y^2)
    %   sin(atan2(y,x)) = y / sqrt(x^2+y^2)
    % In our case this eg. becomes
    %   cos(yaw_ref) = cos(atan2(dy_ref, dx_ref)) = dx_ref / sqrt(dx_ref^2 + dy_ref^2)
    %   sin(yaw_ref) = sin(atan2(dy_ref, dx_ref)) = dy_ref / sqrt(dx_ref^2 + dy_ref^2)    
    cos_yaw_ref = acado.IntermediateState(dx_ref / sqrt(dx_ref^2 + dy_ref^2));
    sin_yaw_ref = acado.IntermediateState(dy_ref / sqrt(dx_ref^2 + dy_ref^2));
    
    % Tracking error
    x_err = acado.IntermediateState(x - x_ref);
    y_err = acado.IntermediateState(y - y_ref);  
    
    % Lateral and longitudinal error
    lateral_deviation = acado.IntermediateState(sin_yaw_ref * x_err - cos_yaw_ref * y_err);  % positive towards right
    longitudinal_velocity = acado.IntermediateState(cos_yaw_ref * dx + sin_yaw_ref * dy);  
    velocity_matching = acado.IntermediateState(longitudinal_velocity - ds);
    lag_error = acado.IntermediateState(-cos_yaw_ref * x_err - sin_yaw_ref * y_err);
    velocity_error = acado.IntermediateState(longitudinal_velocity - desiredVelocity);
    away_from_end_error = acado.IntermediateState(trajectoryLength - s);
    
    % Compute proximity to the nearest 5 obstacles
    proximityObstacle1 = acado.IntermediateState( sqrt( (x - obs1_x)*(x - obs1_x) + (y - obs1_y)*(y - obs1_y) ) - obs1_r );
    proximityObstacle2 = acado.IntermediateState( sqrt( (x - obs2_x)*(x - obs2_x) + (y - obs2_y)*(y - obs2_y) ) - obs2_r );
    proximityObstacle3 = acado.IntermediateState( sqrt( (x - obs3_x)*(x - obs3_x) + (y - obs3_y)*(y - obs3_y) ) - obs3_r );
    proximityObstacle4 = acado.IntermediateState( sqrt( (x - obs4_x)*(x - obs4_x) + (y - obs4_y)*(y - obs4_y) ) - obs4_r );
    proximityObstacle5 = acado.IntermediateState( sqrt( (x - obs5_x)*(x - obs5_x) + (y - obs5_y)*(y - obs5_y) ) - obs5_r );    

    obstacle_proximity = acado.IntermediateState( exp(proximityScale*(-proximityObstacle1 + proximityOffset)) + exp(proximityScale*(-proximityObstacle2 + proximityOffset)) + exp(proximityScale*(-proximityObstacle3 + proximityOffset)) + exp(proximityScale*(-proximityObstacle4 + proximityOffset)) + exp(proximityScale*(-proximityObstacle5 + proximityOffset)) );
    
    %h = [diffStates; controls]; % 'diffStates' and 'controls' are automatically defined by ACADO
    %hN = [diffStates]; 
    h = DefineWeightedStates('lag_error; lateral_deviation;  q2;q3;  omega_ref_x;omega_ref_y;  velocity_error; away_from_end_error;   domega_ref_x;domega_ref_y;   obstacle_proximity;   velocity_slack;angle_slack;proximity_slack');
    hN = DefineWeightedTerminalStates('lag_error; lateral_deviation;  q2;q3;  omega_ref_x;omega_ref_y;  velocity_error; away_from_end_error;   obstacle_proximity');
    W = acado.BMatrix(eye(length(h))); % Cost-function weighting matrix
    WN = acado.BMatrix(eye(length(hN)));
    
    %Slx = acado.BVector(eye(10,1));  % [0,0,0,0,0,0,0,-gamma]  :  the ratio between gamma and q_c controls the trade off between maximum progress (large gamma) and tight path following (large q_c)  -- q_l should be chosen high
    %Slu = acado.BVector(eye(6, 1));    
    
    ocp.minimizeLSQ(W, h);   % W = diag([q_c, q_l, Ru_x, Ru_y, Rv])
    ocp.minimizeLSQEndTerm(WN, hN);
    %ocp.minimizeLSQLinearTerms(Slx, Slu);
    %ocp.minimizeLSQ({q2,q3}); % min(q2^2 + q3^2)
    %ocp.minimizeLagrangeTerm( omeg_ref_x*omeg_ref_x + omeg_ref_y*omeg_ref_y ); % Lagrange terms are on the whole sequence    
    %ocp.minimizeMayerTerm( x ); % Mayer terms are only the final state, control input etc.
    
    %ocp.subjectTo( f );
    ocp.setModel( f );  
    
    %% Define final-state requirements    
    ocp.subjectTo( 'AT_END', q2 == 0 );
    ocp.subjectTo( 'AT_END', q3 == 0 );    
     
    ocp.subjectTo( 'AT_END', omega_ref_x == 0 );
    ocp.subjectTo( 'AT_END', omega_ref_y == 0 );      
 
    %ocp.subjectTo( 'AT_END', dx == 0 );
    %ocp.subjectTo( 'AT_END', dy == 0 );

    ocp.subjectTo( 'AT_END', velocity - maxVelocity <= 0 );
    ocp.subjectTo( 'AT_END', s_ - trajectoryLength <= 0 );
    
    %% Define constraints
    quaternion_max = acado.IntermediateState( sin(1/2*(maxAngle)) + angle_slack );
    ocp.subjectTo( q2 - quaternion_max <= 0 );  % q2 <= sin(1/2*maxAngle)
    ocp.subjectTo( -q2 - quaternion_max <= 0 ); % -q2 <= sin(1/2*maxAngle)  --->  q2 >= -sin(1/2*maxAngle)
    ocp.subjectTo( q3 - quaternion_max <= 0 );  % q3 <= sin(1/2*maxAngle)
    ocp.subjectTo( -q3 - quaternion_max <= 0 ); % -q3 <= sin(1/2*maxAngle)  --->  q2 >= -sin(1/2*maxAngle)
    ocp.subjectTo( angle_slack >= 0 );
    %ocp.subjectTo( angle_slack - pi/2 + maxAngle <= 0 );
    
    ocp.subjectTo( omega_ref_x - maxOmegaRef <= 0 ); % omega_ref_x <= maxOmegaRef
    ocp.subjectTo( -omega_ref_x - maxOmegaRef <= 0 ); % omega_ref_x >= -maxOmegaRef
    ocp.subjectTo( omega_ref_y - maxOmegaRef <= 0 ); % omega_ref_x <= maxOmegaRef
    ocp.subjectTo( -omega_ref_y - maxOmegaRef <= 0 ); % omega_ref_x >= -maxOmegaRef    

    ocp.subjectTo( domega_ref_x - maxdOmegaRef <= 0 ); % domega_ref_x <= maxdOmegaRef
    ocp.subjectTo( -domega_ref_x - maxdOmegaRef <= 0 ); % domega_ref_x >= -maxdOmegaRef
    ocp.subjectTo( domega_ref_y - maxdOmegaRef <= 0 ); % domega_ref_x <= maxdOmegaRef
    ocp.subjectTo( -domega_ref_y - maxdOmegaRef <= 0 ); % domega_ref_x >= -maxdOmegaRef    
        
    ocp.subjectTo( ds >= 0 );       
    %ocp.subjectTo( ds - desiredVelocity <= 0 );
    %ocp.subjectTo( velocity >= 0 );    
    ocp.subjectTo( velocity - maxVelocity - velocity_slack <= 0 );
    %ocp.subjectTo( velocity_slack >= 0 );
    
    ocp.subjectTo( s_ - trajectoryLength <= 0 ); % s_ <= trajectoryLength
    ocp.subjectTo( s_ >= 0 ); % s_ >= 0         
    
    ocp.subjectTo( proximityObstacle1 + proximity_slack >= 0 );
    ocp.subjectTo( proximityObstacle2 + proximity_slack >= 0 );
    ocp.subjectTo( proximityObstacle3 + proximity_slack >= 0 );
    ocp.subjectTo( proximityObstacle4 + proximity_slack >= 0 );
    ocp.subjectTo( proximityObstacle5 + proximity_slack >= 0 );
    ocp.subjectTo( proximity_slack >= 0 );   

    %% Create and configure ACADO optimization algorithm and export C++ code
    %algo = acado.OptimizationAlgorithm(ocp);   
    %algo.set('KKT_TOLERANCE', 1e-10); % Set a custom KKT tolerance
    
    mpc = acado.OCPexport( ocp );
    % All possible parameters are defined here: http://acado.sourceforge.net/matlab/doc/html/matlab/acado/packages/+acado/@OptimizationAlgorithmBase/set.html
    mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
    mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
    mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');  % FULL_CONDENSING, FULL_CONDENSING_N2
    mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL2'       );  % INT_RK45, INT_IRK_GL2, INT_IRK_GL4
    mpc.set( 'NUM_INTEGRATOR_STEPS',        3*N                 );
    mpc.set( 'QP_SOLVER',                   'QP_QPOASES3'    	);
    mpc.set( 'LEVENBERG_MARQUARDT',         1e-10                );    
    mpc.set( 'HOTSTART_QP',                 'YES'             	);   
    %mpc.set( 'CG_HARDCODE_CONSTRAINT_VALUES','YES'             	);    % Specifies whether to hard-code the constraint values.  Works only for box constraints on control and differential state variables.
    mpc.set( 'FIX_INITIAL_STATE',            'YES'             	);     % should be set to YES for MPC
    %mpc.set( 'GENERATE_MATLAB_INTERFACE', 'YES'               );     
    %mpc.set( 'GENERATE_SIMULINK_INTERFACE', 'YES'               );     
    %mpc.set( 'CG_USE_VARIABLE_WEIGHTING_MATRIX', 'YES'       );   % allow different weighting matrices for each stage in the horizon (1:(N+1))    
    
    %mpc.set('USE_SINGLE_PRECISION', 'BT_TRUE');
    
    %mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
    %mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
    %mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2'); % FULL_CONDENSING, FULL_CONDENSING_N2
    %mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'       ); % INT_RK45, INT_IRK_GL4
    %mpc.set( 'NUM_INTEGRATOR_STEPS',         3*N                );
    %mpc.set( 'QP_SOLVER',                   'QP_QPOASES3'    	);
    %mpc.set( 'HOTSTART_QP',                 'YES'             	);
    %mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-10				);
    %mpc.set( 'CG_HARDCODE_CONSTRAINT_VALUES','YES'             	);
    %mpc.set( 'FIX_INITIAL_STATE',            'YES'             	);
    %mpc.set('KKT_TOLERANCE',1e-10)
    %mpc.set('MAX_NUM_ITERATIONS ',100)    

    %mpc.set ( 'HESSIAN_APPROXIMATION' , 'GAUSS_NEWTON' ); % solving algorithm
    %mpc.set ( 'DISCRETIZATION_TYPE' , 'MULTIPLE_SHOOTING' ); %  Discretization algorithm
    %mpc.set ( 'INTEGRATOR_TYPE' , 'INT_RK4' ) ; % Intergation algorithm
    %mpc.set ( 'NUM_INTEGRATOR_STEPS' , 250) ; % Number of integration steps
    %mpc.set ( 'SPARSE_QP_SOLUTION' , 'FULL_CONDENSING_N2' );
    %mpc.set ( 'FIX_INITIAL_STATE' , 'YES' );
    %mpc.set ( 'HOTSTART_QP' , 'YES' );
    %mpc.set ( 'GENERATE_TEST_FILE' , 'YES' );          

    global ACADO_;
    ACADO_.helper
    mpc.exportCode( ACADO_.problemname );
    mpc.printDimensionsQP();                
    copyfile([ACADO_.pwd '/../../external_packages/qpoases3'], [ACADO_.problemname '/qpoases3'])
    cd(ACADO_.problemname);    
    if (enableVerboseMPC)
        ReplaceStringInFile('acado_qpoases3_interface.h', 'PL_NONE', 'PL_DEBUG_ITER'); % enable debugging in qpOASES
    end
    make_acado_solver
    copyfile(['acado_solver.' mexext], ['../' problemName '.' mexext]);
    WriteArrayIndexFile(Ts_MPC);
    cd('../..');   