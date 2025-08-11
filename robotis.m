% robot setup

axis1 = [0,0,1];
axis2 = [0,1,0];
axis3 = [0,1,0];
axis4 = [0,1,0];

p_01 = [0.012, 0, 0.017];
p_12 = [0, 0, 0.0595];
p_23 = [0.024, 0, 0.128];
p_34 = [0.124, 0, 0];
p_4T = [0.1467, 0, 0];

% forward kinematics [specify]
joint_1_rot = rand * pi;
joint_2_rot = rand * pi;
joint_3_rot = rand * pi;
joint_4_rot = rand * pi;

final_pos = p_01' + rot_z(joint_1_rot)*p_12' ...
+ rot_z(joint_1_rot) * rot_y(joint_2_rot) * p_23' ...
+ rot_z(joint_1_rot) * rot_y(joint_2_rot+joint_3_rot) * p_34' ...
+ rot_z(joint_1_rot) * rot_y(joint_2_rot+joint_3_rot+joint_4_rot) * p_4T'; 

final_rot = rot_z(joint_1_rot) * rot_y(joint_2_rot+joint_3_rot+joint_4_rot);

disp("FK Joint rotations");
disp([joint_1_rot, joint_2_rot, joint_3_rot, joint_4_rot]);
disp("Expected Position");
disp(final_pos');

% inverse kinematics

% Goal rotation/position
% x_angle = 0;
% y_angle = 0;
% z_angle = 0;

goal_rotation = final_rot;%rot_z(z_angle) * rot_y(y_angle) * rot_x(x_angle);
goal_position = final_pos;%[0.3067; 0; 0.2045];

goal = [goal_rotation, goal_position]; % goal pos/rot as 4x4
goal = [goal; 0, 0, 0, 1];

base_rotation = rot_z(0) * rot_y(0) * rot_x(0); % home pos/rot as 4x4
base_position = p_01' + p_12' + p_23' + p_34'+ p_4T';
base = [base_rotation, base_position];
base = [base; 0, 0, 0, 1];

% Matrix multiply: Base^(-1)*Goal^(-1)
g = goal*inv(base);

% Find a point on axis 4
p_04 = p_01 + p_12 + p_23 + p_34;
% Find a point on axis 1 and 2
p_02 = p_01 + p_12;

p_03 = p_01 + p_12 + p_23;

theta3_list = joint3(p_04', p_02', axis3', g, p_03'); % subproblem 3

potential_solutions = []; % table to store all the possible solution combinations

p_03 = p_01 + p_12 + p_23;

for theta3 = theta3_list
    y_matrix = [rot_y(theta3), [0;0;0]];
    y_matrix = [y_matrix ; 0 0 0 1];
    e_3 = [1 0 0 p_03(1); 0 1 0 p_03(2); 0 0 1 p_03(3); 0 0 0 1] * y_matrix * [1 0 0 -p_03(1); 0 1 0 -p_03(2); 0 0 1 -p_03(3); 0 0 0 1];
    
    [theta1_list, theta2_list] = joint1joint2(p_04', e_3, g, axis1', axis2', p_02');
    
    for theta1 = theta1_list
        for theta2 = theta2_list
            theta4 = joint4(joint_2_rot+joint_3_rot+joint_4_rot, theta2, theta3);
            potential_solutions = [potential_solutions, [-theta1; theta2; theta3; theta4;]];
        end
    end
end

final_solutions = [];

% forward kinematics
for i = 1:size(potential_solutions, 2)
    joint_1_rot = potential_solutions(1,i);
    joint_2_rot = potential_solutions(2,i);
    joint_3_rot = potential_solutions(3,i);
    joint_4_rot = potential_solutions(4,i);
    
    pos = p_01' + rot_z(joint_1_rot)*p_12' ...
    + rot_z(joint_1_rot) * rot_y(joint_2_rot) * p_23' ...
    + rot_z(joint_1_rot) * rot_y(joint_2_rot+joint_3_rot) * p_34' ...
    + rot_z(joint_1_rot) * rot_y(joint_2_rot+joint_3_rot+joint_4_rot) * p_4T'; 
    
    rot = rot_z(joint_1_rot) * rot_y(joint_2_rot+joint_3_rot+joint_4_rot);
    
    % disp(final_rot);
    if(norm(pos - final_pos) <= 0.001)
        final_solutions = [final_solutions; potential_solutions(1:end,i:i)'];
    end
end

disp(final_solutions);