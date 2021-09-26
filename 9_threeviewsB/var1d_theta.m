% How does error vary with theta?

%% initiate camera parameters & uncertainties


%-------------------sensor parameters-----------------------
%pixelWidth = randi([14 100])/10000;                            % mm, taking 3500x2000 pixels across a 35mm film
pixelWidth = 1.4e-3;
HFOV = 200;                 %mm horizontal field of view
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

cam.u1 = 1*pixelWidth;              %mm 
cam.v1 = 2*pixelWidth;
cam.u2 = 3*pixelWidth;
cam.v2 = 4*pixelWidth;
cam.u3 = 5*pixelWidth;
cam.v3 = 6*pixelWidth;

%random u,v 
cam.u1 = randi([-2000 2000])*pixelWidth;              %mm 
cam.v1 = randi([-2000 2000])*pixelWidth;
cam.u2 = randi([-2000 2000])*pixelWidth;
cam.v2 = randi([-2000 2000])*pixelWidth;
cam.u3 = randi([-2000 2000])*pixelWidth;
cam.v3 = randi([-2000 2000])*pixelWidth;

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

%random pU,pV 
%cam.pU1 = randi([-20 20])*pixelWidth;              %mm 
%cam.pV1 = randi([-20 20])*pixelWidth;
%cam.pU2 = randi([-20 20])*pixelWidth;
%cam.pV2 = randi([-20 20])*pixelWidth;
%cam.pU3 = randi([-20 20])*pixelWidth;
%cam.pV3 = randi([-20 20])*pixelWidth;

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
[genSolX,genSolY,genSolZ] = generalWorldPoints_varEqns(genK1,genK2,genK3,extrinsics1,extrinsics2,extrinsics3,combination);
% find error in symbolic
genError = generalError(genSolX,genSolY,genSolZ);

%% range of independent variable

theta_start = pi/180;                                        % focal length (mm) start value
theta_end = 179*pi/180;                                        % end value
theta_step = 1*pi/180;                                          % difference between values

gen_theta = [theta_start:theta_step:theta_end];                 % store dependent variable theta
n_theta = length(gen_theta);                     % number of iterations in theta
genErr_theta = zeros(1,n_theta);                      % store results (independent variable) 

%% evaluate error whilst varying the parameter
% loop through all values
for i = 1:n_theta
%update values
    cam.theta = gen_theta(i);
    genNumErr = generalErrorEvaluation(genError,cam,camErr);     %evaluate error
    genErr_theta(1,i) = genNumErr;                             %update result vector
end

%% analysis of values
% find minimum error
[minError,minIndex] = min(genErr_theta);

% find maximum error
[maxError,maxIndex] = max(genErr_theta);

%% plot
font = 20;                                                          % change size of text on graph
lineWidth = 1;                                                      % change line width of graph
figure;plot(gen_theta*180/pi,genErr_theta,'LineWidth',lineWidth);                       %plot
ylabel('Error (mm)','FontSize',font);xlabel('Camera rotation angle 1, theta (deg)','FontSize',font); 
title('Variation of theta, f = ' + string(cam.f) + 'mm'+ ', d = ' + string(cam.d) + 'mm' + ', phi = ' + string(cam.phi*180/pi)...
    + 'deg'+ ', pixelWidth = '+ string(pixelWidth*1000) + 'um, ' + '(u1,v1,u2,v2,u3,v3) = (' ...
    + string(cam.u1/pixelWidth)+','+ string(cam.v1/pixelWidth)+','+ string(cam.u2/pixelWidth)+','+ string(cam.v2/pixelWidth)...
    +','+ string(cam.u3/pixelWidth)+','+ string(cam.v3/pixelWidth)+')','FontSize',font-4);    %label
hold on
plot(gen_theta(minIndex)*180/pi,minError,'rx','MarkerSize',font,'LineWidth',lineWidth)                                                       %mark minimum
plot(gen_theta(maxIndex)*180/pi,maxError,'rx','MarkerSize',font,'LineWidth',lineWidth)                                                       %mark maximum
text(gen_theta(ceil(3*minIndex/4))*180/pi,minError,'Minimum error = ' + string(minError*1000) + 'um, Angle = ' + string(gen_theta(minIndex)*180/pi) + 'deg','FontSize',font-8) % add minimum label to graph
text(gen_theta(ceil(3*maxIndex/4))*180/pi,maxError,'Maximum error = ' + string(maxError*1000) + 'um, Angle = ' + string(gen_theta(maxIndex)*180/pi) + 'deg','FontSize',font-8) % add minimum label to graph

savefig('fig1d_theta')                                     %save
