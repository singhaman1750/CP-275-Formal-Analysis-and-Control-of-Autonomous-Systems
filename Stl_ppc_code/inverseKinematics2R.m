function [valid, angles] = inverseKinematics2R(x, y, l1, l2, branch)
    % Inverse kinematics modules for the serial-2R manipulator.
    % The base of the manipulator is placed at [0,0].
    % Axis of rotation is the Z+ axis.
    % Inputs:
    %   - x: X co-ordinate of the end-effector
    %   - y: Y co-ordinate of the end-effector
    %   - l1: Length of link l1
    %   - l2: Length of link l2
    %   - branch: Branch of the inverse kinematics solution.
    % Outputs:
    %   - valid: Binary variable indicating if the solution is valid or not
    %   - angles: Angles made by link l1 w.r.t X+ axis and the relative
    %             angle between links l1 and l2 respectively.
    
    a = 2*x*l2;
    b = 2*y*l2;
    c =  l1^2 - x^2 - y^2 - l2^2;
    psi = atan2(b, a);
    d = -c/sqrt(a^2 + b^2);
    
    if (d < -1) || (d > 1)
        fprintf('Position out of workspace.\n');
        valid = false;
        angles = [0,0];
        return;
    end
    
    if branch == 1
        theta12 = psi + acos(-c/sqrt(a^2 + b^2));
    else
        theta12 = psi - acos(-c/sqrt(a^2 + b^2));
    end
    
    theta1 = atan2((y - l2*sin(theta12))/l1, (x - l2*cos(theta12))/l1);
    valid = true;
    angles = [theta1, theta12-theta1];
end
