% successive rotations
syms theta phi Rx Ry Rz

Rx = [1 0 0;
    0 cos(phi) -sin(phi);
    0 sin(phi) cos(phi)];

Ry = [cos(theta) 0 sin(theta);
    0 1 0;
    -sin(theta) 0 cos(theta)];

Rz = [cos(phi) -sin(phi) 0;
    sin(phi) cos(phi) 0;
    0 0 1];

%% Ry then Rx

R2 = Rx*Ry;