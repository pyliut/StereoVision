%find errorMatrix given 3x1 translation vector, 3x3
% rotation and camera matrices and the error object

% initialise camera parameters for testing
theta = [60 60 60;
    60 60 60;
    60 60 60];
r = cosd(theta);
t = [500 500 0];
kl = [700 0 500;
    0 700 500;
    0 0 1];
kr = [700 0 500;
    0 700 500;
    0 0 1];
err.T = 1;
err.R = 0.01;
err.F = 1;
err.C = 1;

[Fundamental,Error] = fMatrixError(t,r,kl,kr,err)

