function theta3 = joint3(p, q, axis, goal)
    % p: a point on axis 4
    % q: a point on axis 1 and axis 2
    
    goalxp = goal * [p; 1];
    goalxp(4) = [];
    [theta3, ~] = subproblem.sp_3(p, q, axis, norm(goalxp - q));
end