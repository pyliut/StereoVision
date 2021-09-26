% make some matrix calculations based on Schreve's 'How accurate can a
% Stereovision Measurement be?', 2014

%% pinhole camera model (just a brief intro)
syms U V S f x y z t
% For image coordinates u,v, focal length f, world points x,y,z
% u = U/S and v = V/S
m = [U;V;S];                    %image coordinates
K = [-f 0 0 0;                  %simplified camera intrinsics
    0 -f 0 0;
    0 0 1 0];
M = [x;y;z;t];                  %world points

m = K*M;                        

%% camera intrinsics
syms alphaU alphaV pU pV s
% alphaU & alphaV are scaling factors if CCD pixels are not square
% s is skewness parameter
% pU & PV are offsets of the principal point from the image centre
% f is focal length: the perpendicular distance between the focal and image
% planes

% SIMPLE case: simK is the simplified camera intrinsics model, with no 
% skew, square pixels and zero offset of the principle coordinates
simK = K;

% GENERAL case: genK is a general use camera intrinsics model 
genK = [-alphaU*f s pU 0;
        0 -alphaV*f pV 0;
        0 0 1 0];

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

% positions
extrinsics = [R, -R*C;
                0 0 0 1];


%% overall camera matrix

% extrinsics contains the extrinsic camera parameters for these two

% camera matrix with simplified intrinsics
simP = simK*extrinsics;

% camera matrix with general intrinsics
genP = genK*extrinsics;

%% Calculating the 3D coordinates from u, v & P 

% We need to consider image points and camera matrices of both cameras

% In the SIMPLIFIED case, the camera matrices are the same
syms u1 u2 v1 v2

% Splitting the camera matrices into their rows, see Hartley Part 1 p.159
% simP(i)_(j)T refers to the nth row of the simplified camera matrix simP
% for the ith camera
simP1_1T = simK(1,:);
simP1_2T = simK(2,:);
simP1_3T = simK(3,:);

simP2_1T = simP(1,:);
simP2_2T = simP(2,:);
simP2_3T = simP(3,:);

% AX = 0, see Hartley Part 2, page 312
simA = [u1*simP1_3T - simP1_1T;
    v1*simP1_3T - simP1_2T;
    u2*simP2_3T - simP2_1T;
    v2*simP2_3T - simP2_2T];

% we can solve for world points with simA*M = 0

%--------------------------------------------------------------------

% GENERAL case, different camera matrices, but we assume the camera
% properties (e.g. focal length) are identical

% camera intrinsics for 2 identical cameras
% alphaU, alphaV and s are the same though that is not necessarily true 
% for the camera centres pU & pV
syms pU pV s pU1 pU2 pV1 pV2

% We create general case intrinsic matrices for each camera

genK1 = [-alphaU*f s pU1 0;
        0 -alphaV*f pV1 0;
        0 0 1 0];
genK2 = [-alphaU*f s pU2 0;
        0 -alphaV*f pV2 0;
        0 0 1 0];

% Convert to camera matrices as camera 2 is translated & rotated
genP1 = genK1;
genP2 = genK2*extrinsics;

% genP(i)_(j)T refers to the jth row of the general case camera matrix 
% genP for the ith camera 
genP1_1T = genP1(1,:);
genP1_2T = genP1(2,:);
genP1_3T = genP1(3,:);

genP2_1T = genP2(1,:);
genP2_2T = genP2(2,:);
genP2_3T = genP2(3,:);

% again we use AX = 0 from Hartley (see simA above)
genA = [u1*genP1_3T - genP1_1T;
    v1*genP1_3T - genP1_2T;
    u2*genP2_3T - genP2_1T;
    v2*genP2_3T - genP2_2T];

% we need to solve genA*M = 0

%% Solve A*X = 0 equations

% SIMPLE case

% Initialise world coordinates
syms simX simY simZ
simM = [simX; simY; simZ; 1];

% Separate the equations out of simA*simM 
simEqns = simA*simM;
simEqn1 = simEqns(1) == 0;
simEqn2 = simEqns(2) == 0;
simEqn3 = simEqns(3) == 0;
simEqn4 = simEqns(4) == 0;

%Solve
[simSolX,simSolY,simSolZ] = solve(simEqn1,simEqn2,simEqn3,simX,simY,simZ);
%note that using eqns 1,2,3 or 1,3,4 grant results similar to those
%obtained in the paper except with u1 & u2 being the wrong way around in
%simSolX and simSolZ

%-----------------------------------------------------------------------

% GENERAL case

% Initialise world coordinates
syms genX genY genZ
genM = [genX; genY; genZ; 1];

% Separate the equations out of simA*simM 
genEqns = genA*genM;
genEqn1 = genEqns(1) == 0;
genEqn2 = genEqns(2) == 0;
genEqn3 = genEqns(3) == 0;
genEqn4 = genEqns(4) == 0;

% Solve
[genSolX,genSolY,genSolZ] = solve(genEqn1,genEqn2,genEqn3,genX,genY,genZ);


%% camera errors

% Err_x is covariance matrix of the independent variables
% Err_F is covariance matrix of the dependent variable
% J is jacobian of X,Y,Z wrt f,d,theta,u1,u2,v1,v2
syms worldPoints X Y Z
syms err_f err_d err_theta err_u1 err_u2 err_v1 err_v2
syms err_alphaU err_alphaV err_s err_pU1 err_pU2 err_pV1 err_pV2
%err_f etc are variances (sd squared)
% SIMPLE case
simErr_x = [err_f 0 0 0 0 0 0;
        0 err_d 0 0 0 0 0;
        0 0 err_theta 0 0 0 0;
        0 0 0 err_u1 0 0 0;
        0 0 0 0 err_u2 0 0;
        0 0 0 0 0 err_v1 0;
        0 0 0 0 0 0 err_v2];

% simple case Jacobian: diff X,Y,Z wrt f,d,theta,u1,u2,v1,v2
simJ = [diff(simSolX,f) diff(simSolX,d) diff(simSolX,theta) diff(simSolX,u1) diff(simSolX,u2) diff(simSolX,v1) diff(simSolX,v2);
        diff(simSolY,f) diff(simSolY,d) diff(simSolY,theta) diff(simSolY,u1) diff(simSolY,u2) diff(simSolY,v1) diff(simSolY,v2);
        diff(simSolZ,f) diff(simSolZ,d) diff(simSolZ,theta) diff(simSolZ,u1) diff(simSolZ,u2) diff(simSolZ,v1) diff(simSolZ,v2)];
simJTranspose = transpose(simJ);

% calculate covariance matrix of dependent variable
% values along the main diagonal correspond to sigma_X, sigma_Y, sigma_Z
% measurement error is (sigma_X^2 + sigma_Y^2 + sigma_Z^2)^(1/2)
simErr_F = simJ*simErr_x*simJTranspose;
simSigma_X = simErr_F(1,1);             %variance of X
simSigma_Y = simErr_F(2,2);
simSigma_Z = simErr_F(3,3);
simError = sqrt(simSigma_X + simSigma_Y + simSigma_Z);

%-----------------------------------------------------------------------

% GENERAL case
genErr_x = [err_f 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 err_d 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 err_theta 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 err_u1 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 err_u2 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 err_v1 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 err_v2 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 err_alphaU 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 err_alphaV 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 err_s 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 err_pU1 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 err_pU2 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 err_pV1 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 err_pV2];
% general case Jacobian: diff X,Y,Z wrt f,d,theta,u1,u2,v1,v2 + also wrt
% alphaU,alphaV,s,pU1,pU2,pV1,pV2
genJ = [diff(genSolX,f) diff(genSolX,d) diff(genSolX,theta) diff(genSolX,u1) diff(genSolX,u2) diff(genSolX,v1) diff(genSolX,v2) diff(genSolX,alphaU) diff(genSolX,alphaV) diff(genSolX,s) diff(genSolX,pU1) diff(genSolX,pU2) diff(genSolX,pV1) diff(genSolX,pV2);
        diff(genSolY,f) diff(genSolY,d) diff(genSolY,theta) diff(genSolY,u1) diff(genSolY,u2) diff(genSolY,v1) diff(genSolY,v2) diff(genSolY,alphaU) diff(genSolY,alphaV) diff(genSolY,s) diff(genSolY,pU1) diff(genSolY,pU2) diff(genSolY,pV1) diff(genSolY,pV2);
        diff(genSolZ,f) diff(genSolZ,d) diff(genSolZ,theta) diff(genSolZ,u1) diff(genSolZ,u2) diff(genSolZ,v1) diff(genSolZ,v2) diff(genSolZ,alphaU) diff(genSolZ,alphaV) diff(genSolZ,s) diff(genSolZ,pU1) diff(genSolZ,pU2) diff(genSolZ,pV1) diff(genSolZ,pV2)];
genJTranspose = transpose(genJ);

% calculate covariance matrix of dependent variable
% values along the main diagonal correspond to sigma_X, sigma_Y, sigma_Z
% measurement error is (sigma_X^2 + sigma_Y^2 + sigma_Z^2)^(1/2)
genErr_F = genJ*genErr_x*genJTranspose;
genSigma_X = genErr_F(1,1);                 %variance of X
genSigma_Y = genErr_F(2,2);
genSigma_Z = genErr_F(3,3);
genError = sqrt(genSigma_X + genSigma_Y + genSigma_Z);

%% Initialise camera parameters & errors

%--------------------------camera params--------------------------------
% simple
cam.f = 25;                         %mm not the same as the camera intrinsics f
cam.d = 400;
cam.theta = pi/2;
cam.u1 = -12.5;                %mm assuming each pixel is 2um
cam.u2 = 12.5;
cam.v1 = -12.5;
cam.v2 = 12.5;
% general
cam.alphaU = 1;                     %1:1 is square
cam.alphaV = 1;
cam.s = 0;                          
cam.pU1 = 0*0.002;                %mm assuming each pixel is 2um
cam.pU2 = 0*0.002;
cam.pV1 = 0*0.002;
cam.pV2 = 0*0.002;

%--------------------------initialise errors--------------------------
%errors are the variance = sd squared
% simple
camErr.f = (24e-6)^2;                     %mm
camErr.d = (24e-6)^2;                     %mm
camErr.theta = (0.005*pi/(180*3600))^2;             %radians
camErr.u1 = (0.68e-3)^2;                  %mm min res is 1.38um
camErr.u2 = (0.68e-3)^2;                  %mm min res is 1.38um
camErr.v1 = (0.68e-3)^2;                  %mm min res is 1.38um
camErr.v2 = (0.68e-3)^2;                  %mm min res is 1.38um
% general
camErr.alphaU = (24e-6)^2;                  %mm
camErr.alphaV = (24e-6)^2;
camErr.s = 0;                               %no skew
camErr.pU1 = 1*(0.68e-3)^2;                 %mm min resolution is 1.38um
camErr.pU2 = 1*(0.68e-3)^2;                 %mm min res is 1.38um
camErr.pV1 = 1*(0.68e-3)^2;                 %mm min res is 1.38um
camErr.pV2 = 1*(0.68e-3)^2;                 %mm min res is 1.38um

%% Evaluate error

% SIMPLE case
simSub = subs(simError,{f,d,theta,u1,u2,v1,v2,err_f,err_d,err_theta,err_u1,err_u2,err_v1,err_v2}...
    ,{cam.f,cam.d,cam.theta,cam.u1,cam.u2,cam.v1,cam.v2,camErr.f,camErr.d,camErr.theta,camErr.u1,camErr.u2,camErr.v1,camErr.v2});
simNumErr = vpa(simSub)                             % numerical error for simple model

%----------------------------------------------------------------------

%GENERAL case
genSub = subs(genError,{f,d,theta,u1,u2,v1,v2,alphaU,alphaV,s,pU1,pU2,pV1,pV2...
            ,err_f,err_d,err_theta,err_u1,err_u2,err_v1,err_v2,err_alphaU,err_alphaV,err_s,err_pU1,err_pU2,err_pV1,err_pV2}...
                ,{cam.f,cam.d,cam.theta,cam.u1,cam.u2,cam.v1,cam.v2,cam.alphaU,cam.alphaV,cam.s,cam.pU1,cam.pU2,cam.pV1,cam.pV2...
            ,camErr.f,camErr.d,camErr.theta,camErr.u1,camErr.u2,camErr.v1,camErr.v2,camErr.alphaU,camErr.alphaV,camErr.s,camErr.pU1,camErr.pU2,camErr.pV1,camErr.pV2});
genNumErr = vpa(genSub)                             % numerical error for general model



