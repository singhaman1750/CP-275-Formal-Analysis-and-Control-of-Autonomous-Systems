function [h,h_bar] = stl2ppc(stl,t,x_o)

%% Calling switching function
beta = switching_func(stl,t);

sz_stl_array = size(stl);
sz_stl = sz_stl_array(2);
h_T = zeros(1,sz_stl);

for i=1:sz_stl
    h_T(i) = norm(x_o - stl(i).des_config);
end
h_bar_T = beta .* h_T;

h = transpose(h_T);
h_bar = transpose(h_bar_T);

end
