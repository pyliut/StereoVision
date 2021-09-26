% Effect of camera rotation variation on error in general model
% All other variables & uncertainties are random within a reasonable range

%% create cam & camErr structures

%-------------------sensor parameters-----------------------
pixelWidth = randi([14 100])/10000;                            % mm, taking 3500x2000 pixels across a 35mm film

%-------------------uncertainties------------------------------------
u.f = randi([1 100])/100;                      %mm
u.d = randi([1 100])/100;                            %mm
u.theta = pi/180;                   %radians
u.u = 1;                            %number of pixels - uncertainty in u1,v1,pU1 etc
u.alpha = 1/100;                    %percent
u.s = 1/100;                        %percent
%--------------------------camera params--------------------------------
% simple
cam.f = randi([1 100]);                         %mm not the same as the camera intrinsics f
cam.d = randi([1 100])*100;                       %mm
cam.theta = randi([1 179])*pi/180;                   %radians
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

theta_start = pi/180;                                        % focal length (mm) start value
theta_end = 179*pi/180;                                        % end value
theta_step = pi/180;                                          % difference between values

genVector_theta = [theta_start:theta_step:theta_end];                 % store dependent variable theta
n_theta = length(genVector_theta);                     % number of iterations in theta
genErrorVector = zeros(1,n_theta);                      % store results (independent variable) 

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
for i = 1:n_theta
%update values
    cam.theta = genVector_theta(i);                        
    genNumErr = generalErrorEvaluation(genError,cam,camErr);     %evaluate error
    genErrorVector(1,i) = genNumErr;                             %update result vector
end

%% analysis of values
% find minimum error
[minError,minIndex] = min(genErrorVector);
%sprintf('Minimum error = %e',minError)
%sprintf('Angle = %g',genVector_theta(minIndex))

% find range in which the error is within 5% of minimum error
[rowBound,colBound] = find(genErrorVector<(1.05*minError));        %colBound is the column indices of the values within the specified boundaries
lowerBound = genErrorVector(colBound(1));                       %lowerbound is the maximum error within the specified bounds
upperBound = genErrorVector(colBound(end));

    
%% plot error
font = 20;                                                          % change size of text on graph
lineWidth = 1;                                                      % change line width of graph
figure;plot(genVector_theta,genErrorVector,'LineWidth',lineWidth);                       %plot
ylabel('Error (mm)','FontSize',font);xlabel('Camera rotation angle, theta (rad)','FontSize',font); 
title('General Model - Variation of theta, f = ' + string(cam.f) + ' mm '+ ', d = ' + string(cam.d) ...
    + ' mm '+ ', pixelWidth = '+ string(pixelWidth) + ' mm '+ ', u.f = ' + string(u.f) + ' mm '...
    + ', u.d = ' + string(u.d)+ ' mm ','FontSize',font-4);    %label
hold on
plot(genVector_theta(minIndex),minError,'rx','MarkerSize',font,'LineWidth',lineWidth)                                                       %mark minimum
plot([genVector_theta(colBound(1)),genVector_theta(colBound(end))],[lowerBound,upperBound],'gx','MarkerSize',font,'LineWidth',lineWidth)    %mark bounds
text(genVector_theta(minIndex),minError*2,'Minimum error = ' + string(minError) + ', Angle = ' + string(genVector_theta(minIndex)*180/pi) + ' degrees','FontSize',font) % add minimum label to graph
text(genVector_theta(colBound(1)),lowerBound*3.5,'5% bound = ' + string(lowerBound) + ', Angle = ' + string(genVector_theta(colBound(1))*180/pi) + ' degrees','FontSize',font) % add lower bound label to graph
text(genVector_theta(colBound(end)),upperBound*0.5,'5% bound = ' + string(upperBound) + ', Angle = ' + string(genVector_theta(colBound(end))*180/pi) + ' degrees','FontSize',font) % add upper bound label to graph

savefig('fig1d_theta')                                     %save