function [J_o1,J_o2,dJ_o1,dJ_o2] = coup_dyn(x_o,v_o,d_o)
%COUP_DYN Summary of this function goes here
%   Detailed explanation goes here
r = d_o/2;
yaw = x_o(3);
w = v_o(3);

J_o1 = [1, 0,    r*sin(yaw);
        0, 1, -1*r*cos(yaw);
        0, 0,             1];

J_o2 = [1, 0, -1*r*sin(yaw);
        0, 1,    r*cos(yaw);
        0, 0,             1];

dJ_o1 = [0, 0, r*cos(yaw)*w;
         0, 0, r*sin(yaw)*w;
         0, 0,            0];

dJ_o2 = [0, 0, -1*r*cos(yaw)*w;
         0, 0, -1*r*sin(yaw)*w;
         0, 0,               0];
end

