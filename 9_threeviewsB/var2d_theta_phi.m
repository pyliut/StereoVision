% How does error vary with theta?

%% initiate camera parameters & uncertainties


%-------------------sensor parameters-----------------------
%pixelWidth = randi([14 100])/10000;                            % mm, taking 3500x2000 pixels across a 35mm film
pixelWidth = 1.4e-3;
HFOV = 100;                 %mm horizontal field of view
h = 10;                      %mm sensor width
combination = [2 4 5];

%-------------------uncertainties------------------------------------
%u.f = randi([1 100])/100;                      %mm
u.f = 0.01;
%u.d = randi([1 100])/100;                            %mm
u.d = 0.01;
u.theta = pi/180;                   %radians
u.phi = pi/180;                     %radians
u.u = 1;                            %number of pixels - uncertainty in u1,v1,pU1 etc
u.alpha = 1/100;                    %percent
u.s = 1/100;                        %percent
%--------------------------camera params--------------------------------
% simple
cam.f = 25;                         %mm not the same as the camera intrinsics f
cam.d = 250;                       %mm
cam.theta = pi/2;                   %radians already determined (see notes at top)
cam.phi = pi/3;

cam.u1 = 0*pixelWidth;              %mm 
cam.v1 = 0*pixelWidth;
cam.u2 = 0*pixelWidth;
cam.v2 = 0*pixelWidth;
cam.u3 = 0*pixelWidth;
cam.v3 = 0*pixelWidth;

%random u,v
%cam.u1 = randi([-100 100])*pixelWidth;              %mm 
%cam.v1 = randi([-100 100])*pixelWidth;
%cam.u2 = randi([-100 100])*pixelWidth;
%cam.v2 = randi([-100 100])*pixelWidth;
%cam.u3 = randi([-100 100])*pixelWidth;
%cam.v3 = randi([-100 100])*pixelWidth;

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
[genSolX,genSolY,genSolZ] = generalWorldPoints(genK1,genK2,genK3,extrinsics1,extrinsics2,extrinsics3);
% find error in symbolic
genError = generalError(genSolX,genSolY,genSolZ);

%% range of independent variable

theta_start = 10*pi/180;                                        % focal length (mm) start value
theta_end = 179*pi/180;                                        % end value
theta_step = 10*pi/180;                                          % difference between values

phi_start = 5*pi/180;                                        % focal length (mm) start value
phi_end = 89*pi/180;                                        % end value
phi_step = 5*pi/180;                                          % difference between values

gen_theta = [theta_start:theta_step:theta_end];                 % store dependent variable theta
n_theta = length(gen_theta);                     % number of iterations in theta

gen_phi = [phi_start:phi_step:phi_end];                 % store dependent variable theta
n_phi = length(gen_phi);                     % number of iterations in theta

gen_err = zeros(n_phi,n_theta);                      % store results (independent variable) 

%% evaluate error whilst varying the parameter
% loop through all values
for i = 1:n_theta
    for j = 1:n_phi
%update values
        cam.theta = gen_theta(i);
        cam.phi = gen_phi(j);
        genNumErr = generalErrorEvaluation(genError,cam,camErr);     %evaluate error
        gen_err(j,i) = genNumErr;                             %update result vector
    end
end

%% analysis 0.1% error range

% find minimum error
minError_phi= min(gen_err);
[minError,minIndex_theta] = min(minError_phi);
minIndex_phi = find(gen_err(:,minIndex_theta) == minError);

%% plot

font = 20;                                                          % change size of text on graph
lineWidth = 1;                                                      % change line width of graph
figure;

mesh(gen_theta*180/pi,gen_phi*180/pi,gen_err,'FaceAlpha','0.5','LineWidth',lineWidth);                       %plot

%label graph
zlabel('Error (mm)','FontSize',font);
xlabel('Cam. Angle 1, theta (deg)','FontSize',font); ylabel('Cam. Angle 2, phi (deg)','FontSize',font); 
title('Variation of theta & phi, f = ' + string(cam.f) + ' mm '+ ', d = ' + string(cam.d) + 'mm' + ', phi = ' + string(cam.phi*180/pi)...
    + ' deg '+ ', pixelWidth = '+ string(pixelWidth) + ' mm, ' + '(u1,v1,u2,v2,u3,v3) = (' ...
    + string(cam.u1/pixelWidth)+','+ string(cam.v1/pixelWidth)+','+ string(cam.u2/pixelWidth)+','+ string(cam.v2/pixelWidth)...
    +','+ string(cam.u3/pixelWidth)+','+ string(cam.v3/pixelWidth)+')','FontSize',font-4);    %label
hold on

% plot analysis
plot3(gen_theta(minIndex_theta)*180/pi,gen_phi(minIndex_phi)*180/pi,minError,'rx','MarkerSize',font,'LineWidth',lineWidth)                                                       %mark minimum
text(gen_theta(minIndex_theta)*180/pi,gen_phi(minIndex_phi)*180/pi,minError*2,'Min Error = ' + string(minError*1000) + 'um')
savefig('fig2d_theta_phi')                                     %save
