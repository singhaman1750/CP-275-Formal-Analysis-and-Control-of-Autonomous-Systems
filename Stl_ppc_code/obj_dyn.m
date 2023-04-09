function [M_o,c_o,S_o,g_o,v,dv] = obj_dyn(x,v,u,d_o,m_o)
% OBJ_DYN Summary of this function goes here
%   Detailed explanation goes here
g0 = 9.81;
I_o_cm = (m_o * (d_o^2))/6;

M_o = [m_o,   0,      0;
         0, m_o,      0;
         0,   0, I_o_cm];

c_o = [0;0;0];
S_o = [0,0,0;
       0,0,0;
       0,0,0];

g_o = [0;m_o*g0;0];

dv = inv(M_o)*(u - g_o);

end