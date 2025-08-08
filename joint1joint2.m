function [theta1, theta2] = joint1joint2(p, e3, goal, axis1, axis2)
    % e3: 3x3 matrix
    % p: 3x1 a point on axis 4
    endpoint = goal * [p; 1];
    endpoint(4) = [];
    
    startpoint = e3 * [p; 1];
    startpoint(4) = [];
    [theta1, theta2, ~] = subproblem.sp_2(startpoint, endpoint, axis1, axis2);
end