% Input: desired end-effector position (x,y) and yaw angle
% Output: joint angles (q1,q2,q3)
function [q1, q2, q3] = inverseKinematics3R(x, y, yaw,l1,l2,l3,branch)

% Compute the joint angles q1 and q2 using the 2R inverse kinematics function
[~, q] = inverseKinematics2R(x - l3*cos(yaw), y - l3*sin(yaw), l1, l2,branch);

% Compute the joint angle q3 using the law of cosines
%q3_1 = acos((x*sin(yaw) + y*cos(yaw) - l1*cos(q1_1) - l2*cos(q1_1 + q2_1))/l3);
q3 = yaw - q(1) - q(2);

% returning angles in radians
q1 = q(1);
q2 = q(2);

end
