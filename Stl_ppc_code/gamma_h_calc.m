function [gamma_h,gamma_h_inv] = gamma_h_calc(stl,t,x_obj_i)
sz_stl_array = size(stl);
sz_stl = sz_stl_array(2);

gamma_h_0 = zeros(1,sz_stl);
gamma_h_inf = zeros(1,sz_stl);
gamma_h_vec = zeros(1,sz_stl);

l = 0.5;
alpha = 50;

for i = 1:sz_stl
if (i == 1)
    gamma_h_0(i)  = alpha*(norm(x_obj_i-stl(i).des_config));
    gamma_h_inf(i) = stl(i).tolr_err;
    gamma_h_vec(i) = (gamma_h_0(i) - gamma_h_inf(i)) * exp(-l*t) + gamma_h_inf(i);
else
    gamma_h_0(i)  = alpha*(norm(stl(i-1).des_config-stl(i).des_config));
    gamma_h_inf(i) = stl(i).tolr_err;
    gamma_h_vec(i) = (gamma_h_0(i) - gamma_h_inf(i)) * exp(-l*(t-stl(i).time_range(1))) + gamma_h_inf(i);
end
%gamma_h_vec(i) = (gamma_h_0(i) - gamma_h_inf(i)) * exp(-l*t) + gamma_h_inf(i);
end

gamma_h = diag(gamma_h_vec);
gamma_h_inv = diag(1./gamma_h_vec);

end