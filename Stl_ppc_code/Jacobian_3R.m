function [J,J_inv,J_T,J_T_inv,dJ] = Jacobian_3R(q1,q2,q3,l1,l2,l3)
%JACOBIAN3R Summary of this function goes here

%% Jacobian
J = ...
[- l2*sin(q1 + q2) - l1*sin(q1) - l3*sin(q1 + q2 + q3), ...
 - l2*sin(q1 + q2) - l3*sin(q1 + q2 + q3), ...
 -l3*sin(q1 + q2 + q3);
 
 l2*cos(q1 + q2) + l1*cos(q1) + l3*cos(q1 + q2 + q3), ...
 l2*cos(q1 + q2) + l3*cos(q1 + q2 + q3), ...
 l3*cos(q1 + q2 + q3);
 
 1,1,1];

%% J^-1: Inverse of jacobian
J_inv = ...
[-cos(q1 + q2)/(l1*cos(q1 + q2)*sin(q1) - l1*sin(q1 + q2)*cos(q1)), ...
 -sin(q1 + q2)/(l1*cos(q1 + q2)*sin(q1) - l1*sin(q1 + q2)*cos(q1)), ...
 (l3*cos(q1 + q2 + q3)*sin(q1 + q2) - l3*sin(q1 + q2 + q3)*cos(q1 + q2))/(l1*cos(q1 + q2)*sin(q1) - l1*sin(q1 + q2)*cos(q1));
 
 (l2*cos(q1 + q2) + l1*cos(q1))/(l1*l2*cos(q1 + q2)*sin(q1) - l1*l2*sin(q1 + q2)*cos(q1)), ...
 (l2*sin(q1 + q2) + l1*sin(q1))/(l1*l2*cos(q1 + q2)*sin(q1) - l1*l2*sin(q1 + q2)*cos(q1)), ...
 -(l2*l3*cos(q1 + q2 + q3)*sin(q1 + q2) - l2*l3*sin(q1 + q2 + q3)*cos(q1 + q2) + l1*l3*cos(q1 + q2 + q3)*sin(q1) - l1*l3*sin(q1 + q2 + q3)*cos(q1))/(l1*l2*cos(q1 + q2)*sin(q1) - l1*l2*sin(q1 + q2)*cos(q1));
 
 -cos(q1)/(l2*cos(q1 + q2)*sin(q1) - l2*sin(q1 + q2)*cos(q1)), ...
 -sin(q1)/(l2*cos(q1 + q2)*sin(q1) - l2*sin(q1 + q2)*cos(q1)), ...
 (l2*cos(q1 + q2)*sin(q1) - l2*sin(q1 + q2)*cos(q1) + l3*cos(q1 + q2 + q3)*sin(q1) - l3*sin(q1 + q2 + q3)*cos(q1))/(l2*cos(q1 + q2)*sin(q1) - l2*sin(q1 + q2)*cos(q1))];

%% Jacobian Transpose and inverse of jacobian transpose
J_T = transpose(J);
J_T_inv = transpose(J_inv);

end
