function angle = rot_x(radians)
    angle = [1, 0, 0;
       0, cos(radians), -sin(radians);
       0, sin(radians),  cos(radians)];
end