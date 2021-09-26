function [simSolX,simSolY,simSolZ] = simpleWorldPoints(simK,extrinsics)
%% overall camera matrix

% camera matrix with simplified intrinsics
simP = simK*extrinsics;

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
end
