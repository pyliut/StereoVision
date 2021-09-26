%test
%% create cam & camErr structures

%-------------------sensor parameters-----------------------
%pixelWidth = randi([14 100])/10000;                            % mm, taking 3500x2000 pixels across a 35mm film
pixelWidth = 1.35e-3;
HFOV = 100;                 %mm horizontal field of view
h = 10;                      %mm sensor width
combination = [2 4 5];

%-------------------uncertainties------------------------------------
%u.f = randi([1 100])/100;                      %mm
u.f = 0.01;
%u.d = randi([1 100])/100;                            %mm
u.d = 0.01;
u.theta = pi/180;                   %radians
u.phi = pi/180;                     %radians
u.u = 0.5;                            %number of pixels - uncertainty in u1,v1,pU1 etc
u.alpha = 1/100;                    %percent
u.s = 1/100;                        %percent
%--------------------------camera params--------------------------------
% simple
cam.f = 50;                         %mm not the same as the camera intrinsics f
cam.d = 1000;                       %mm
cam.theta = pi/2;                   %radians already determined (see notes at top)
cam.phi = 80*pi/180;

cam.u1 = 0*pixelWidth;              %mm 
cam.v1 = 0*pixelWidth;
cam.u2 = 0*pixelWidth;
cam.v2 = 0*pixelWidth;
cam.u3 = 0*pixelWidth;
cam.v3 = 0*pixelWidth;

%random u,v
%cam.u1 = randi([-100 100])*pixelWidth;              %mm 
%cam.v1 = randi([-100 100])*pixelWidth;
%cam.u2 = randi([-100 100])*pixelWidth;
%cam.v2 = randi([-100 100])*pixelWidth;
%cam.u3 = randi([-100 100])*pixelWidth;
%cam.v3 = randi([-100 100])*pixelWidth;

% general
cam.alphaU = 1;                     %1:1 ratio of alphas is square
cam.alphaV = 1;
cam.s = 0; 

cam.pU1 = 0*pixelWidth;             %mm 
cam.pV1 = 0*pixelWidth;
cam.pU2 = 0*pixelWidth;
cam.pV2 = 0*pixelWidth;
cam.pU3 = 0*pixelWidth;
cam.pV3 = 0*pixelWidth;

%--------------------------initialise errors--------------------------
%errors are the VARIANCE = sd squared
% simple
camErr.f = (u.f)^2;                           %mm
camErr.d = (u.d)^2;                                 %mm
camErr.theta = (u.theta)^2;                         %radians
camErr.phi = (u.phi)^2;                             %rad
camErr.u1 = (u.u*pixelWidth)^2;                     %mm 
camErr.v1 = (u.u*pixelWidth)^2;                     %mm 
camErr.u2 = (u.u*pixelWidth)^2;                     %mm 
camErr.v2 = (u.u*pixelWidth)^2;                     %mm 
camErr.u3 = (u.u*pixelWidth)^2;                     %mm 
camErr.v3 = (u.u*pixelWidth)^2;                     %mm 
% general
camErr.alphaU = (cam.alphaU*u.alpha)^2;             %mm
camErr.alphaV = (cam.alphaV*u.alpha)^2;
camErr.s = (cam.s*u.s)^2;                           %no skew
camErr.pU1 = 0*(u.u*pixelWidth)^2;                    %mm 
camErr.pV1 = 0*(u.u*pixelWidth)^2;                    %mm 
camErr.pU2 = 0*(u.u*pixelWidth)^2;                    %mm 
camErr.pV2 = 0*(u.u*pixelWidth)^2;                    %mm
camErr.pU3 = 0*(u.u*pixelWidth)^2;                    %mm 
camErr.pV3 = 0*(u.u*pixelWidth)^2;                    %mm

%% create intrinsics, extrinsics
% same for all solution combinations

[genK1,genK2,genK3] = generalPinholeIntrinsics();
[extrinsics1,extrinsics2,extrinsics3] = generalExtrinsics();
[genSolX,genSolY,genSolZ] = generalWorldPointsTest(genK1,genK2,genK3,extrinsics1,extrinsics2,extrinsics3);
% find error in symbolic
genError = generalError(genSolX,genSolY,genSolZ);
genNumErr = generalErrorEvaluation(genError,cam,camErr)    %evaluate error