function simError = simpleError(simSolX,simSolY,simSolZ)
%% camera errors

% Err_x is covariance matrix of the independent variables
% Err_F is covariance matrix of the dependent variable
% J is jacobian of X,Y,Z wrt f,d,theta,u1,u2,v1,v2
syms worldPoints X Y Z
syms f d theta u1 u2 v1 v2
syms err_f err_d err_theta err_u1 err_u2 err_v1 err_v2
syms err_alphaU err_alphaV err_s err_pU1 err_pU2 err_pV1 err_pV2
%err_f etc are variances

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
simSigma_X = simErr_F(1,1);                 %this is variance (sd squared)
simSigma_Y = simErr_F(2,2);
simSigma_Z = simErr_F(3,3);
simError = sqrt(simSigma_X + simSigma_Y + simSigma_Z);
end