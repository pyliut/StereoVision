function genError = generalError(genSolX,genSolY,genSolZ)

%% camera errors

% Err_x is covariance matrix of the independent variables
% Err_F is covariance matrix of the dependent variable
% J is jacobian of X,Y,Z wrt f,d,theta,u1,u2,v1,v2
syms worldPoints X Y Z
syms f d theta phi u1 v1 u2 v2 u3 v3 alphaU alphaV s pU1 pV1 pU2 pV2 pU3 pV3
syms err_f err_d err_theta err_phi err_u1 err_v1 err_u2 err_v2 err_u3 err_v3 err_alphaU err_alphaV err_s err_pU1 err_pV1 err_pU2 err_pV2 err_pU3 err_pV3
%err_f etc are variances

% GENERAL case
    genErr_x = [err_f 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
                0 err_d 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
                0 0 err_theta 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
                0 0 0 err_phi 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
                0 0 0 0 err_u1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
                0 0 0 0 0 err_v1 0 0 0 0 0 0 0 0 0 0 0 0 0;
                0 0 0 0 0 0 err_u2 0 0 0 0 0 0 0 0 0 0 0 0;
                0 0 0 0 0 0 0 err_v2 0 0 0 0 0 0 0 0 0 0 0;
                0 0 0 0 0 0 0 0 err_u3 0 0 0 0 0 0 0 0 0 0;
                0 0 0 0 0 0 0 0 0 err_v3 0 0 0 0 0 0 0 0 0;
                0 0 0 0 0 0 0 0 0 0 err_alphaU 0 0 0 0 0 0 0 0;
                0 0 0 0 0 0 0 0 0 0 0 err_alphaV 0 0 0 0 0 0 0;
                0 0 0 0 0 0 0 0 0 0 0 0 err_s 0 0 0 0 0 0;
                0 0 0 0 0 0 0 0 0 0 0 0 0 err_pU1 0 0 0 0 0;
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 err_pV1 0 0 0 0;
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 err_pU2 0 0 0;
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 err_pV2 0 0;
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 err_pU3 0;
                0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 err_pV3];
% general case Jacobian: diff X,Y,Z wrt f,d,theta,u1,u2,v1,v2 + also wrt
% alphaU,alphaV,s,pU1,pU2,pV1,pV2
    genJ = [diff(genSolX,f) diff(genSolX,d) diff(genSolX,theta) diff(genSolX,phi) diff(genSolX,u1) diff(genSolX,v1) diff(genSolX,u2) diff(genSolX,v2) diff(genSolX,u3) diff(genSolX,v3) diff(genSolX,alphaU) diff(genSolX,alphaV) diff(genSolX,s) diff(genSolX,pU1) diff(genSolX,pV2) diff(genSolX,pU2) diff(genSolX,pV2) diff(genSolX,pU3) diff(genSolX,pV3);
            diff(genSolY,f) diff(genSolY,d) diff(genSolY,theta) diff(genSolY,phi) diff(genSolY,u1) diff(genSolY,v1) diff(genSolY,u2) diff(genSolY,v2) diff(genSolY,u3) diff(genSolY,v3) diff(genSolY,alphaU) diff(genSolY,alphaV) diff(genSolY,s) diff(genSolY,pU1) diff(genSolY,pV2) diff(genSolY,pU2) diff(genSolY,pV2) diff(genSolY,pU3) diff(genSolY,pV3);
            diff(genSolZ,f) diff(genSolZ,d) diff(genSolZ,theta) diff(genSolZ,phi) diff(genSolZ,u1) diff(genSolZ,v1) diff(genSolZ,u2) diff(genSolZ,v2) diff(genSolZ,u3) diff(genSolZ,v3) diff(genSolZ,alphaU) diff(genSolZ,alphaV) diff(genSolZ,s) diff(genSolZ,pU1) diff(genSolZ,pV2) diff(genSolZ,pU2) diff(genSolZ,pV2) diff(genSolZ,pU3) diff(genSolZ,pV3)];
    genJTranspose = transpose(genJ);

% calculate covariance matrix of dependent variable
% values along the main diagonal correspond to sigma_X, sigma_Y, sigma_Z
% measurement error is (sigma_X^2 + sigma_Y^2 + sigma_Z^2)^(1/2)
    genErr_F = genJ*genErr_x*genJTranspose;
    genSigma_X = genErr_F(1,1);                     % this is variance (sd squared)
    genSigma_Y = genErr_F(2,2);
    genSigma_Z = genErr_F(3,3);
    genError = sqrt(genSigma_X + genSigma_Y + genSigma_Z);
%genError = simplify(genError);
end