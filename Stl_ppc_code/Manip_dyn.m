function [M,c,S,g,dq,ddq] = Manip_dyn(q,dq,u,m1,m2,m3,l1,l2,l3,d1,d2,d3,I1zz,I2zz,I3zz)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
g0 = 9.81;

q1 = q(1);
q2 = q(2);
q3 = q(3);

dq1 = dq(1);
dq2 = dq(2);
dq3 = dq(3);

u1 = u(1);
u2 = u(2);
u3 = u(3);

% Mass Matrix
M = ...
[I1zz + I2zz + I3zz + d1^2*m1 + d2^2*m2 + d3^2*m3 + l1^2*m2 + l1^2*m3 + l2^2*m3 + 2*d3*l1*m3*cos(q2 + q3) + 2*d2*l1*m2*cos(q2) + 2*d3*l2*m3*cos(q3) + 2*l1*l2*m3*cos(q2), ...
 m2*d2^2 + l1*m2*cos(q2)*d2 + m3*d3^2 + 2*m3*cos(q3)*d3*l2 + l1*m3*cos(q2 + q3)*d3 + m3*l2^2 + l1*m3*cos(q2)*l2 + I2zz + I3zz, ...
 I3zz + d3^2*m3 + d3*l1*m3*cos(q2 + q3) + d3*l2*m3*cos(q3);
 
 m2*d2^2 + l1*m2*cos(q2)*d2 + m3*d3^2 + 2*m3*cos(q3)*d3*l2 + l1*m3*cos(q2 + q3)*d3 + m3*l2^2 + l1*m3*cos(q2)*l2 + I2zz + I3zz, ...
 m2*d2^2 + m3*d3^2 + 2*m3*cos(q3)*d3*l2 + m3*l2^2 + I2zz + I3zz, ...
 m3*d3^2 + l2*m3*cos(q3)*d3 + I3zz;
 
 I3zz + d3^2*m3 + d3*l1*m3*cos(q2 + q3) + d3*l2*m3*cos(q3), ...
 m3*d3^2 + l2*m3*cos(q3)*d3 + I3zz, ...
 m3*d3^2 + I3zz];

% Corriollis Vector
c = ... 
[- d3*dq2^2*l1*m3*sin(q2 + q3) - d3*dq3^2*l1*m3*sin(q2 + q3) - d2*dq2^2*l1*m2*sin(q2) - d3*dq3^2*l2*m3*sin(q3) - dq2^2*l1*l2*m3*sin(q2) - 2*d3*dq1*dq2*l1*m3*sin(q2 + q3) - 2*d3*dq1*dq3*l1*m3*sin(q2 + q3) - 2*d3*dq2*dq3*l1*m3*sin(q2 + q3) - 2*d2*dq1*dq2*l1*m2*sin(q2) - 2*d3*dq1*dq3*l2*m3*sin(q3) - 2*d3*dq2*dq3*l2*m3*sin(q3) - 2*dq1*dq2*l1*l2*m3*sin(q2);
                                                                                                                                                                                                d3*dq1^2*l1*m3*sin(q2 + q3) + d2*dq1^2*l1*m2*sin(q2) - d3*dq3^2*l2*m3*sin(q3) + dq1^2*l1*l2*m3*sin(q2) - 2*d3*dq1*dq3*l2*m3*sin(q3) - 2*d3*dq2*dq3*l2*m3*sin(q3);
                                                                                                                                                                                                                                                                      d3*m3*(dq1^2*l1*sin(q2 + q3) + dq1^2*l2*sin(q3) + dq2^2*l2*sin(q3) + 2*dq1*dq2*l2*sin(q3))];
S = ... 
[- dq2*l1*(d2*m2*sin(q2) + l2*m3*sin(q2) + d3*m3*sin(q2 + q3)) - d3*dq3*m3*(l1*sin(q2 + q3) + l2*sin(q3)), ...
 - dq1*l1*(d2*m2*sin(q2) + l2*m3*sin(q2) + d3*m3*sin(q2 + q3)) - dq2*l1*(d2*m2*sin(q2) + l2*m3*sin(q2) + d3*m3*sin(q2 + q3)) - d3*dq3*m3*(l1*sin(q2 + q3) + l2*sin(q3)), ...
 -d3*m3*(l1*sin(q2 + q3) + l2*sin(q3))*(dq1 + dq2 + dq3);
 
 dq1*l1*(d2*m2*sin(q2) + l2*m3*sin(q2) + d3*m3*sin(q2 + q3)) - d3*dq3*l2*m3*sin(q3), ...
 -d3*dq3*l2*m3*sin(q3), ...
 -d3*l2*m3*sin(q3)*(dq1 + dq2 + dq3);
                                          
 d3*dq1*m3*(l1*sin(q2 + q3) + l2*sin(q3)) + d3*dq2*l2*m3*sin(q3), ...
 d3*l2*m3*sin(q3)*(dq1 + dq2), ...
 0];
 
% Gravity 
g = ...
[g0*m3*(l2*cos(q1 + q2) + l1*cos(q1) + d3*cos(q1 + q2 + q3)) + g0*m2*(d2*cos(q1 + q2) + l1*cos(q1)) + d1*g0*m1*cos(q1);
                                               g0*m3*(l2*cos(q1 + q2) + d3*cos(q1 + q2 + q3)) + d2*g0*m2*cos(q1 + q2);
                                                                                           d3*g0*m3*cos(q1 + q2 + q3)];

%% Equation solution
temp_vec = u - g - c;
ddq = M\temp_vec; % ddq = inv(M) * temp_vec;

end

