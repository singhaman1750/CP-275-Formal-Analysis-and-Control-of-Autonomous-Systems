function dydt = ode_stl_ppc_control(t,y,stl,y0)
%% Initial Value
x_o = [y(1); y(2); y(3)]; 
v_o = [y(4); y(5); y(6)];

x_obj_i = [y0(1); y0(2); y0(3)]; 
v_obj_i = [y0(4); y0(5); y0(6)];

sz_stl_array = size(stl);
sz_stl = sz_stl_array(2);

[h,h_bar] = stl2ppc(stl,t,x_o);
[h_i,h_bar_i] = stl2ppc(stl,2.5,x_obj_i);

%% reference velocity
v_r = v_r_calc(t,x_o,x_obj_i,h_bar,h,stl);
v_r_i = v_r_calc(0,x_obj_i,x_obj_i,h_bar_i,h_i,stl);
e_v = v_o - v_r;
e_v_i = abs(v_obj_i - v_r_i);

%v_obj_i
%v_r_i
%v_r
%e_v_i
%e_v

[gamma_v,gamma_v_inv] = gamma_v_calc(t,e_v_i);

Zi_v = gamma_v_inv * e_v;

for i=1:3
    if Zi_v(i) >= 1
        Zi_v(i) = 0.99;
    elseif Zi_v(i) <= -1
        Zi_v(i) = -0.99;
    end
end

%% epsilon_v and r_v
r_v_vec = zeros(1,3);
epsilon_v = zeros(3,1);

for i = 1:3
   r_v_vec(i) = 2/(1-Zi_v(i)^2);
   epsilon_v(i) = log( (1+Zi_v(i)) / (1-Zi_v(i)) );
end

r_v = diag(r_v_vec);

d_o = 0.2;
r = d_o/2;
yaw = x_o(3);
w = v_o(3);

J_o1 = [1, 0,    r*sin(x_o(3));
        0, 1, -1*r*cos(x_o(3));
        0, 0,                1];

J_o2 = [1, 0, -1*r*sin(x_o(3));
        0, 1,    r*cos(x_o(3));
        0, 0,                1];


%% Gains
c_i = 0.5;
C_g = c_i*diag([10,10,10,10,10,10]);
    
%% G_star matrix
G_star = [J_o1;J_o2];

%% Control Input
u = -C_g * G_star * gamma_v_inv * r_v * epsilon_v;
%u
%x_o
%v_o


t
if(imag(u(1)) ~= 0)
G_star
gamma_v_inv
r_v
epsilon_v
Zi_v
e_v
gamma_v
t
disp('yikes');
end
[~,~,~,v_o,dv_o] = coop_manip_dyn(x_o,v_o,u);
dydt = [v_o;dv_o];
%dydt = transpose(dydt_T);
end