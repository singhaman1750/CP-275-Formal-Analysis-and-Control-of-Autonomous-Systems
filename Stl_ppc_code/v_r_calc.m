function [v_r] = v_r_calc(t,x_o,x_obj_i,h_bar,h,stl)
%V_R_CALC Summary of this function goes here
%   Detailed explanation goes here
g_s = diag([10,10,10]);
[dh_bar_dx_o] = dh_bar_d_x_o_calc(t,x_o,h_bar,h,stl);
[gamma_h,gamma_h_inv] = gamma_h_calc(stl,t,x_obj_i);

Zi_h = gamma_h_inv * h_bar;

sz_stl_array = size(stl);
sz_stl = sz_stl_array(2);

%% epsilon_h and r_h
r_h_vec = zeros(1,sz_stl);
epsilon_h = zeros(sz_stl,1);

for i = 1:sz_stl
   r_h_vec(i) = 2/(1-Zi_h(i)^2);
   epsilon_h(i) = log( (1+Zi_h(i)) / (1-Zi_h(i)) );
end

r_h = diag(r_h_vec);

v_r = -1 * g_s * transpose(dh_bar_dx_o)*gamma_h_inv*r_h*epsilon_h;
%r_h
%epsilon_h
%gamma_h_inv
%dh_bar_dx_o
end

