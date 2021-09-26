function extrinsics = generalExtrinsics()
%same as simpleExtrinsics()

%% camera extrinsics
syms R C theta d
% We consider the case where the second camera is the same distance d away 
% from the object though rotated by an angle theta in radians

% R is the rotation matrix, here we consider a 1D rotation about the y axis
R = [cos(theta) 0 sin(theta);
    0 1 0;
    -sin(theta) 0 cos(theta)];

% C is the translation vector, here we consider a 2D translation in the
% xz plane
C = [d*sin(theta);
    0;
    d-d*cos(theta)];

% extrinsics contains the extrinsic camera parameters for these two
% positions
extrinsics = [R, -R*C;
                0 0 0 1];

end