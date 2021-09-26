% Effect of focal length variation on error in general model

%% create cam & camErr structures

%-------------------sensor parameters-----------------------
pixelWidth = 10e-3;                            % mm, taking 3500x2000 pixels across a 35mm film

%-------------------uncertainties------------------------------------
u.f = 0.5/100;                      %percent
u.d = 1;                            %mm
u.theta = pi/180;                   %radians
u.u = 1;                            %number of pixels - uncertainty in u1,v1,pU1 etc
u.alpha = 1/100;                    %percent
u.s = 1/100;                        %percent
%--------------------------camera params--------------------------------
% simple
cam.f = 25;                         %mm not the same as the camera intrinsics f
cam.d = 1000;                       %mm
cam.theta = pi/2;                   %radians
cam.u1 = 0*pixelWidth;              %mm 
cam.u2 = 0*pixelWidth;
cam.v1 = 0*pixelWidth;
cam.v2 = 0*pixelWidth;
% general
cam.alphaU = 1;                     %1:1 ratio of alphas is square
cam.alphaV = 1;
cam.s = 0;                          
cam.pU1 = 0*pixelWidth;             %mm 
cam.pU2 = 0*pixelWidth;
cam.pV1 = 0*pixelWidth;
cam.pV2 = 0*pixelWidth;

%--------------------------initialise errors--------------------------
%errors are the VARIANCE = sd squared
% simple
camErr.f = (cam.f*u.f)^2;                           %mm
camErr.d = (u.d)^2;                                 %mm
camErr.theta = (u.theta)^2;                         %radians
camErr.u1 = (u.u*pixelWidth)^2;                     %mm 
camErr.u2 = (u.u*pixelWidth)^2;                     %mm 
camErr.v1 = (u.u*pixelWidth)^2;                     %mm 
camErr.v2 = (u.u*pixelWidth)^2;                     %mm 
% general
camErr.alphaU = (cam.alphaU*u.alpha)^2;             %mm
camErr.alphaV = (cam.alphaV*u.alpha)^2;
camErr.s = (cam.s*u.s)^2;                           %no skew
camErr.pU1 = (u.u*pixelWidth)^2;                    %mm 
camErr.pU2 = (u.u*pixelWidth)^2;                    %mm 
camErr.pV1 = (u.u*pixelWidth)^2;                    %mm 
camErr.pV2 = (u.u*pixelWidth)^2;                    %mm

%% define parameter variation & store results

h = 35;                                             %sensor size (width in mm)
HFOV = 1400;                                        % horizontal field of view in mm

f_start = 1;                                        % focal length (mm) start value
f_end = 100;                                        % end value
f_step = 1;                                           % difference between values

theta_start = pi/180;                                        % focal length (mm) start value
theta_end = pi;                                        % end value
theta_step = 179*pi/180;                                          % difference between values

genVector_f = [f_start:f_step:f_end];                   % store dependent variable f
genVector_theta = [theta_start:theta_step:theta_end];                 % store dependent variable theta
n_f = length(genVector_f);                            % number of iterations in f
n_theta = length(genVector_theta);                     % number of iterations in theta
genErrorVector = zeros(n_f,n_theta);                      % store results (independent variable) 

%% create model

% initialise camera parameters in symbolic
genK = generalPinholeIntrinsics();
extrinsics = generalExtrinsics();
% Find solutions for the world points in symbolic
% uses basic triangulation methods
[genSolX,genSolY,genSolZ] = generalWorldPoints(genK,extrinsics);
% find error in symbolic
genError = generalError(genSolX,genSolY,genSolZ);

%% evaluate error whilst varying the parameter
% loop through all values
for i = 1:n_f
    for j = 1:n_theta
        cam.f = genVector_f(i);
        cam.theta = genVector_theta(j);
        cam.d = cam.f*HFOV/h;
        genNumErr = generalErrorEvaluation(genError,cam,camErr);     %evaluate error
        genErrorVector(i,j) = genNumErr;                             %update result vector
    end
end

%% plot error
figure;mesh(genVector_f,genVector_theta,genErrorVector);                       %plot
xlabel('Focal length, f (mm)');ylabel('Camera rotation angle, theta (rad)');zlabel('Error (mm)'); 
title('General Model - Variation of f & theta, h = 35mm');    %label
savefig('generalModelVar_f_theta_hequals35')                                     %save