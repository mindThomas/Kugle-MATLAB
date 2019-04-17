% Let the nominal parameters be the same as the model parameters except for
% the parameters changed below

% No friction in nominal model
Bvk_nominal = 0;
Bvm_nominal = 0;
Bvb_nominal = 0;

Mb_nominal = Mb;
Jbx_nominal = Jbx;
Jby_nominal = Jby;
Jbz_nominal = Jbz;

% Center of mass is assumed to be right above center of ball
%COM_nominal = [0, 0, l]';
COM_nominal = COM;

%% Assemble constant vector used in simulation
constants_nominal = [Jk, Mk, rk, Mb_nominal, Jbx_nominal, Jby_nominal, Jbz_nominal, Jw, rw, Bvk_nominal, Bvm_nominal, Bvb_nominal, g]';