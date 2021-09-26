% Compare the errors from each of the possible combinations of error

%% initiate camera parameters & uncertainties


%-------------------sensor parameters-----------------------
pixelWidth = 1.4e-3;                            % mm, taking 3500x2000 pixels across a 35mm film

HFOV = 100;                 %mm horizontal field of view
h = 10;                      %mm sensor width

%-------------------uncertainties------------------------------------
%u.f = randi([25 60])/100;                      %mm
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
cam.f = randi([25 60]);                         %mm not the same as the camera intrinsics f
cam.f = 50;
cam.d = randi([1 100])*100;                       %mm
cam.d = 1000;
cam.theta = randi([25 170])*pi/180;                   %radians already determined (see notes at top)
cam.theta = 90*pi/180;
cam.phi = 89*pi/180;

cam.u1 = 0*pixelWidth;              %mm 
cam.v1 = 0*pixelWidth;
cam.u2 = 0*pixelWidth;
cam.v2 = 0*pixelWidth;
cam.u3 = 0*pixelWidth;
cam.v3 = 0*pixelWidth;
% general
cam.alphaU = 1;                     %1:1 ratio of alphas is square
cam.alphaV = 1;
cam.s = 0; 

cam.pU1 = 1*pixelWidth;             %mm 
cam.pV1 = 1*pixelWidth;
cam.pU2 = 1*pixelWidth;
cam.pV2 = 1*pixelWidth;
cam.pU3 = 1*pixelWidth;
cam.pV3 = 1*pixelWidth;

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
camErr.pU1 = (u.u*pixelWidth)^2;                    %mm 
camErr.pV1 = (u.u*pixelWidth)^2;                    %mm 
camErr.pU2 = (u.u*pixelWidth)^2;                    %mm 
camErr.pV2 = (u.u*pixelWidth)^2;                    %mm
camErr.pU3 = (u.u*pixelWidth)^2;                    %mm 
camErr.pV3 = (u.u*pixelWidth)^2;                    %mm

%% create intrinsics, extrinsics
% same for all solution combinations

[genK1,genK2,genK3] = generalPinholeIntrinsics();
[extrinsics1,extrinsics2,extrinsics3] = generalExtrinsics();

%% possible combinations & initialise matrix to store error
combinations = [1 3 5;
                1 3 6;
                1 4 5;
                1 4 6;
                2 3 5;
                2 3 6;
                2 4 5;
                2 4 6];
%combinations = nchoosek([1 2 3 4 5 6],3);
combinationNum = length(combinations);
            
combinationErr = zeros(combinationNum,1);

%% generate & store error for each combination
for i = 1:combinationNum
    combination = combinations(i,:);
    % find worldpoints in symbolic
    [genSolX,genSolY,genSolZ] = generalWorldPoints_varEqns(genK1,genK2,genK3,extrinsics1,extrinsics2,extrinsics3,combination);
    % find error in symbolic
    genError = generalError(genSolX,genSolY,genSolZ);
    % evaluate 
    genNumErr = generalErrorEvaluation(genError,cam,camErr);
    % store
    combinationErr(i) = genNumErr;
end

%% print results in table
variableText = 'f = ' + string(cam.f) + 'mm, d = ' + string(cam.d) + 'mm, theta = ' + string(cam.theta/pi*180) + 'deg, phi = ' + string(cam.phi/pi*180) + 'deg \n';
fprintf(variableText)
%%
fprintf('Combination    Error(mm)\n')
fprintf('------------------------\n')
for i = 1:combinationNum
    fprintf('%-2.0f %-2.0f %-2.0f %-2.5f\n',combinations(i,1),combinations(i,2),combinations(i,3),combinationErr(i))
end

    



