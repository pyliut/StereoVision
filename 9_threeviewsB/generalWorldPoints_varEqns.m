function [genSolX,genSolY,genSolZ] = generalWorldPoints_varEqns(genK1,genK2,genK3,extrinsics1,extrinsics2,extrinsics3,combination)
%% Calculating the 3D coordinates from u, v & P 

% We need to consider image points and camera matrices of both cameras

% GENERAL case, different camera matrices, but we assume the camera
% properties (e.g. focal length) are identical

% camera intrinsics for 2 identical cameras
% alphaU, alphaV and s are the same though that is not necessarily true 
% for the camera centres pU & pV
    syms alphaU alphaV f d theta phi s pU1 pU2 pU3 pV1 pV2 pV3 u1 u2 u3 v1 v2 v3

    %---------------camera matrices P------------------------
    genP1 = genK1*extrinsics1;
    genP2 = genK2*extrinsics2;
    genP3 = genK3*extrinsics3;

    % genP(i)_(j)T refers to the jth row of the general case camera matrix 
    % genP for the ith camera 
    genP1_1T = genP1(1,:);
    genP1_2T = genP1(2,:);
    genP1_3T = genP1(3,:);

    genP2_1T = genP2(1,:);
    genP2_2T = genP2(2,:);
    genP2_3T = genP2(3,:);

    genP3_1T = genP3(1,:);
    genP3_2T = genP3(2,:);
    genP3_3T = genP3(3,:);

    % again we use AX = 0 from Hartley (see simA above)
    genA = [u1*genP1_3T - genP1_1T;
        v1*genP1_3T - genP1_2T;
        u2*genP2_3T - genP2_1T;
        v2*genP2_3T - genP2_2T;
        u3*genP3_3T - genP3_1T;
        v3*genP3_3T - genP3_2T];

    % we need to solve genA*M = 0

%% Solve A*X = 0 equations

% GENERAL case

% Initialise world coordinates
    syms genX genY genZ
    genM = [genX; genY; genZ; 1];

    % Separate the equations out of simA*simM 
    genEqns = genA*genM;
    usedEqn1 = genEqns(combination(1)) == 0;
    usedEqn2 = genEqns(combination(2)) == 0;
    usedEqn3 = genEqns(combination(3)) == 0;
    
    

% Solve
    [genSolX,genSolY,genSolZ] = solve(usedEqn1,usedEqn2,usedEqn3,genX,genY,genZ);

%simplify to save time on later steps
    genSolX = simplify(genSolX);
    genSolY = simplify(genSolY);
    genSolZ = simplify(genSolZ);
end
