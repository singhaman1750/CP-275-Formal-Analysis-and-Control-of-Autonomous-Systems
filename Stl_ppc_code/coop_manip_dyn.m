function [M,S,g,v_o,dv_o] = coop_manip_dyn(x_o,v_o,u)
%COOP_MANIP_DYN Summary of this function goes here
%   Detailed explanation goes here
m_o = 0.5;
d_o = 0.5;

m1 = 0.5;
m2 = 0.5;
m3 = 0.05;
l1 = 1.5;
l2 = 1.5;
l3 = 0.1;
d1 = 0.5;
d2 = 0.5;
d3 = 0.05;
I1zz = (m1*l1^2)/12;
I2zz = (m2*l2^2)/12;
I3zz = (m3*l3^2)/12;


u1_ts = [0;0;0];
u2_ts = [0;0;0];
lambda_o = [0;0;0];
yaw = x_o(3);

%% Object dynamics
[M_o,c_o,S_o,g_o,~,~] = obj_dyn(x_o,v_o,lambda_o,d_o,m_o);
[J_o1,J_o2,dJ_o1,dJ_o2] = coup_dyn(x_o,v_o,d_o);

x_ee1_wrt_grd = [x_o(1);x_o(2);x_o(3)] - [(d_o/2)*cos(yaw);(d_o/2)*sin(yaw);0];
x_ee2_wrt_grd = [x_o(1);x_o(2);pi+x_o(3)] + [(d_o/2)*cos(yaw);(d_o/2)*sin(yaw);0];

x_ee1 = x_ee1_wrt_grd + [1;0;0];
x_ee2 = x_ee2_wrt_grd - [1;0;0];

v_ee1 = J_o1*v_o;
v_ee2 = J_o2*v_o;

% Manipulator Dynamics in Task space
[M1_ts,c1_ts,S1_ts,g1_ts,~,~] = Manip_dyn_ts(x_ee1,v_ee1,u1_ts,m1,m2,m3,l1,l2,l3,d1,d2,d3,I1zz,I2zz,I3zz);
[M2_ts,c2_ts,S2_ts,g2_ts,~,~] = Manip_dyn_ts(x_ee2,v_ee2,u2_ts,m1,m2,m3,l1,l2,l3,d1,d2,d3,I1zz,I2zz,I3zz);

M_ts = blkdiag(M1_ts,M2_ts);
S_ts = blkdiag(S1_ts,S2_ts);
g_ts = [g1_ts;g2_ts];

G = [J_o1;J_o2];
dG = [dJ_o1;dJ_o2];
G_T = transpose(G);
% dG_T = transpose(dG);

%% Final Dynamics
M = M_o + G_T*M_ts*G;
S = S_o + G_T*M_ts*dG + G_T*S_ts*G;
g = g_o + G_T*g_ts;
%M_inv = inv(M);

dv_o = M\(G_T*u - g - S*v_o);
end

