function angle = rot_z(radians)
angle = [cos(radians), -sin(radians), 0;
       sin(radians),  cos(radians), 0;
       0, 0, 1];
end