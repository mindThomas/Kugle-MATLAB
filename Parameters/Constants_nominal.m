% Let the nominal parameters be the same as the model parameters

Bvk = 0*0.001;
Bvm = 0*0.001;
Bvb = 0*0.001;

beta = 0; % no quaternion regularization term in the nominal model

constants_nominal = [Jk, Mk, rk, Mb, Jbx, Jby, Jbz, Jw, rw, Bvk, Bvm, Bvb, l, g]';

COM_nominal = [0, 0, l]';
%COM_nominal = COM;