% Effect of focal length d a& object distance d variation on error in general model
% min error camera rotation angle theta of pi/2, determined from
% theta_General_v3_allrandomvariables.m
%vary f between 1-100 mm, d between 100-10000 mm
% All other variables & uncertainties are random within a reasonable range

%% create cam & camErr structures

%-------------------sensor parameters-----------------------
%pixelWidth = randi([14 100])/10000;                            % mm, taking 3500x2000 pixels across a 35mm film
pixelWidth = 1.4e-3;

%-------------------uncertainties------------------------------------
%u.f = randi([1 100])/100;                      %mm
u.f = 0.1;
%u.d = randi([1 100])/100;                            %mm
u.d = 0.1;
u.theta = pi/180;                   %radians
u.u = 1;                            %number of pixels - uncertainty in u1,v1,pU1 etc
u.alpha = 1/100;                    %percent
u.s = 1/100;                        %percent
%--------------------------camera params--------------------------------
% simple
cam.f = randi([1 100]);                         %mm not the same as the camera intrinsics f
cam.d = randi([1 100])*100;                       %mm
cam.theta = pi/2;                   %radians already determined (see notes at top)
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

f_start = 1;                                        % focal length (mm) start value
f_end = 101;                                        % end value
f_step = 10;                                          % difference between values

d_start = 200;                                        % object distance (mm) start value
d_end = 2000;                                        % end value
d_step = 200;                                          % difference between values

genVector_f = [f_start:f_step:f_end];                 % store dependent variable f
genVector_d = [d_start:d_step:d_end];                 % store dependent variable d
n_f = length(genVector_f);                     % number of iterations in f
n_d = length(genVector_d);                     % number of iterations in d
genErrorVector = zeros(n_f,n_d);                      % store results (independent variable) 

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
for i = 1:n_f           %loop through values of f
    for j = 1:n_d       %loop through values of d
%update values
        cam.f = genVector_f(i);
        cam.d = genVector_d(j);
        genNumErr = generalErrorEvaluation(genError,cam,camErr);     %evaluate error
        genErrorVector(i,j) = genNumErr;                             %update result vector
    end
end

%% analysis of values

% find minimum error
%[minError,minIndex] = min(genErrorVector);
%sprintf('Minimum error = %e',minError)
%sprintf('Angle = %g',genVector_theta(minIndex))

% find range in which the error is within 5% of minimum error
%[rowBound,colBound] = find(genErrorVector<(1.05*minError));        %colBound is the column indices of the values within the specified boundaries
%lowerBound = genErrorVector(colBound(1));                       %lowerbound is the maximum error within the specified bounds
%upperBound = genErrorVector(colBound(end));

    
%% plot error
font = 20;                                                          % change size of text on graph
lineWidth = 1;                                                      % change line width of graph
figure;mesh(genVector_d,genVector_f,genErrorVector,'LineWidth',lineWidth);                       %plot
zlabel('Error (mm)','FontSize',font);
ylabel('Focal length, f (mm)','FontSize',font); xlabel('Object distance, d (mm)','FontSize',font); 
title('General Model - Variation of f & d, theta = ' + string(cam.theta) + ' rad '+ ', pixelWidth = '+ string(pixelWidth) + ' mm ' ... 
    + ', u.f = ' + string(u.f) + ' mm '+ ', u.d = ' + string(u.d)+ ' mm ','FontSize',font-4);    %label
hold on
%plot(genVector_f(minIndex),minError,'rx','MarkerSize',font,'LineWidth',lineWidth)                                                       %mark minimum
%plot([genVector_f(colBound(1)),genVector_f(colBound(end))],[lowerBound,upperBound],'gx','MarkerSize',font,'LineWidth',lineWidth)    %mark bounds
%text(genVector_f(minIndex),minError*2,'Minimum error = ' + string(minError) + ', Angle = ' + string(genVector_f(minIndex)*180/pi) + ' degrees','FontSize',font) % add minimum label to graph
%text(genVector_f(colBound(1)),lowerBound*3.5,'Lower bound = ' + string(lowerBound) + ', Angle = ' + string(genVector_f(colBound(1))*180/pi) + ' degrees','FontSize',font) % add lower bound label to graph
%text(genVector_f(colBound(end)),upperBound*0.5,'Upper bound = ' + string(upperBound) + ', Angle = ' + string(genVector_f(colBound(end))*180/pi) + ' degrees','FontSize',font) % add upper bound label to graph

%% plot a plane to find values with the correct HFOV
HFOV = 1400;                 %mm horizontal field of view
h = 35;                      %mm sensor width

dPoints = [genVector_d(1),genVector_d(end),genVector_d(end),genVector_d(1)];    %find points in plane in terms of d
fPoints = dPoints .*(h/HFOV);                                    %find corresponding f using f = d*h/HFOV
maxError = max(max(genErrorVector));
zPoints = [0 0 maxError maxError];            %complete the plane (the height is the same as the max error)
fill3(dPoints,fPoints,zPoints,'r')
alpha(0.3)

savefig('generalModelVar_f_d_1a')                                     %save