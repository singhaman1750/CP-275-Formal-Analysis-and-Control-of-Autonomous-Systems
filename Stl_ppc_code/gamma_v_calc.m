function [gamma_v,gamma_v_inv] = gamma_v_calc(t,e_v_i)

gamma_v_0 = zeros(1,3);
gamma_v_inf = zeros(1,3);
gamma_v_vec = zeros(1,3);

l = 0.2;
alpha_1 = 10;
alpha_2 = 1;

for i = 1:3
gamma_v_0(i)  = alpha_1*(e_v_i(i));
gamma_v_inf(i) = alpha_2*(e_v_i(i));
gamma_v_vec(i) = (gamma_v_0(i) - gamma_v_inf(i)) * exp(-l*t);% + gamma_v_inf(i);
end

gamma_v = diag(gamma_v_vec);
gamma_v_inv = diag(1./gamma_v_vec);

end
