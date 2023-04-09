% Input: desired end-effector position (x,y)
% Output: joint angles (q1,q2)

function [q1, q2] = inverseKinematics2R(x, y,l1,l2)

% Define manipulator parameters
%l1 = 1; % length of link 1
%l2 = 1; % length of link 2

% Compute the distance from the origin to the end-effector
d = sqrt(x^2 + y^2);

% Check if the desired position is reachable
if d > l1 + l2
    error('Position is not reachable');
end

% Compute the joint angle q2 using the atan2 function
q2 = atan2(sqrt(1 - ((x^2 + y^2 - l1^2 - l2^2)/(2*l1*l2))^2), ...
           (x^2 + y^2 - l1^2 - l2^2)/(2*l1*l2));

% Compute the joint angle q1 using the atan2 function
q1 = atan2(y, x) - atan2(l2*sin(q2), l1 + l2*cos(q2));

% Convert the angles to degrees
q1 = q1 * 180/pi;
q2 = q2 * 180/pi;

end
