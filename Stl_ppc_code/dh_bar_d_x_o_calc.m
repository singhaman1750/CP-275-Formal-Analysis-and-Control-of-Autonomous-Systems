function [dh_bar_dx_o] = dh_bar_d_x_o_calc(t,x_o,h_bar,h,stl)
sz_stl_array = size(stl);
sz_stl = sz_stl_array(2);

beta = switching_func(stl,t);

dh_bar_dx_o = zeros(sz_stl,3);

for i=1:sz_stl
   for j = 1:3
      dh_bar_dx_o(i,j) = (beta(i)*(x_o(j) - stl(i).des_config(j)))/h(i);
      %disp('stl(i).des_config(j):')
      %disp(stl(i).des_config(j));
      %x_o
      %disp('dh_bar_dx_o(i,j):');
      %disp(dh_bar_dx_o(i,j));
   end
end
%dh_bar_dx_o
end