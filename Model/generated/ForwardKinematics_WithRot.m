function dxdydyaw = ForwardKinematics_WithRot(dpsi1,dpsi2,dpsi3,dq1,dq2,dq3,dq4,q1,q2,q3,q4,rk,rw)
%FORWARDKINEMATICS_WITHROT
%    DXDYDYAW = FORWARDKINEMATICS_WITHROT(DPSI1,DPSI2,DPSI3,DQ1,DQ2,DQ3,DQ4,Q1,Q2,Q3,Q4,RK,RW)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    13-Feb-2019 11:03:00

t2 = sqrt(2.0);
t3 = 1.0./rk;
t4 = dpsi1.*-2.0+dpsi2+dpsi3;
t5 = q1.^2;
t6 = q2.^2;
t7 = q3.^2;
t8 = q4.^2;
t9 = sqrt(6.0);
t10 = dpsi2-dpsi3;
t11 = q1.*q4.*2.0;
t12 = q2.*q3.*2.0;
t13 = dpsi1+dpsi2+dpsi3;
dxdydyaw = [-rk.*(dq1.*q3.*2.0-dq3.*q1.*2.0-dq2.*q4.*2.0+dq4.*q2.*2.0+rw.*t2.*t3.*t13.*(q1.*q2.*2.0-q3.*q4.*2.0).*(1.0./3.0)+rw.*t3.*t9.*t10.*(t5-t6+t7-t8).*(1.0./3.0)-rw.*t2.*t3.*t4.*(t11+t12).*(1.0./3.0));-rk.*(dq1.*q2.*-2.0+dq2.*q1.*2.0-dq3.*q4.*2.0+dq4.*q3.*2.0+rw.*t2.*t3.*t13.*(q1.*q3.*2.0+q2.*q4.*2.0).*(1.0./3.0)+rw.*t2.*t3.*t4.*(t5+t6-t7-t8).*(1.0./3.0)+rw.*t3.*t9.*t10.*(t11-t12).*(1.0./3.0));dpsi1.*rw.*t2.*t3.*(1.0./3.0)+dpsi2.*rw.*t2.*t3.*(1.0./3.0)+dpsi3.*rw.*t2.*t3.*(1.0./3.0)];