% Effect of focal length variation on error in simple model

%% create cam & camErr structures

%--------------------------camera params--------------------------------
% simple
cam.f = 25;                         %mm not the same as the camera intrinsics f
cam.d = 400;
cam.theta = pi/2;
cam.u1 = 0*0.002;                %mm assuming each pixel is 2um
cam.u2 = 0*0.002;
cam.v1 = 0*0.002;
cam.v2 = 0*0.002;

%--------------------------initialise errors--------------------------
%errors are the VARIANCE = sd squared
% simple
camErr.f = (24e-6)^2;                     %mm
camErr.d = (24e-6)^2;                     %mm
camErr.theta = (0.005*pi/(180*3600))^2;             %radians
camErr.u1 = (0.68e-3)^2;                  %mm min res is 1.38um
camErr.u2 = (0.68e-3)^2;                  %mm min res is 1.38um
camErr.v1 = (0.68e-3)^2;                  %mm min res is 1.38um
camErr.v2 = (0.68e-3)^2;                  %mm min res is 1.38um


%% define parameter variation & store results

f_start = 1;                                    % start value
f_end = 100;                                   % end value
step = 1;                                       % difference between values

simVector_f = [f_start:step:f_end];                 % store dependent variable
n = length(simVector_f);                           % number of iterations tested
simErrorVector_f = zeros(1,n);                        % store results (independent variable) in a 1xn row vector

%% create model

% initialise camera parameters in symbolic
simK = simplePinholeIntrinsics();
extrinsics = simpleExtrinsics();
% Find solutions for the world points in symbolic
% uses basic triangulation methods
[simSolX,simSolY,simSolZ] = simpleWorldPoints(simK,extrinsics);
% find error in symbolic
simError = simpleError(simSolX,simSolY,simSolZ);

%% evaluate error whilst varying the parameter
% loop through all values
for i = 1:n
    cam.f = simVector_f(i);
    simNumErr = simpleErrorEvaluation(simError,cam,camErr);         %evaluate error
    simErrorVector_f(i) = simNumErr;                                   %update result vector
end

%% plot error
figure;plot(simVector_f,simErrorVector_f);                                %plot
xlabel('Focal length, f (mm)');ylabel('Error (mm)');title('Simple Model - Variation of f'); %labels
savefig('simpleModelVar_f')                                         %save
    