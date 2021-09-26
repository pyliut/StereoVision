% How does error vary with the uncertainty in pixelWidth?

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
cam.phi = pi/4;

cam.u1 = 0*pixelWidth;              %mm 
cam.v1 = 0*pixelWidth;
cam.u2 = 0*pixelWidth;
cam.v2 = 0*pixelWidth;
cam.u3 = 0*pixelWidth;
cam.v3 = 0*pixelWidth;

%random u,v
cam.u1 = randi([-20 20])*pixelWidth;              %mm 
cam.v1 = randi([-20 20])*pixelWidth;
cam.u2 = randi([-20 20])*pixelWidth;
cam.v2 = randi([-20 20])*pixelWidth;
cam.u3 = randi([-20 20])*pixelWidth;
cam.v3 = randi([-20 20])*pixelWidth;

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

%random u,v
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

pixelWidthErr_start = 1e-3;                                        % focal length (mm) start value
pixelWidthErr_end = 10e-3;                                        % end value
pixelWidthErr_step = 0.1e-3;                                          % difference between values

gen_pixelWidthErr = [pixelWidthErr_start:pixelWidthErr_step:pixelWidthErr_end];                 % store dependent variable theta
n_pixelWidthErr = length(gen_pixelWidthErr);                     % number of iterations in theta
genErr_pixelWidthErr = zeros(1,n_pixelWidthErr);                      % store results (independent variable) 

%% evaluate error whilst varying the parameter
% loop through all values
    %scale coords back to pixels
    cam.u1 = cam.u1/pixelWidth;              %mm 
    cam.v1 = cam.v1/pixelWidth;
    cam.u2 = cam.u2/pixelWidth;
    cam.v2 = cam.v2/pixelWidth;
    cam.u3 = cam.u3/pixelWidth;
    cam.v3 = cam.v3/pixelWidth;
    
    cam.pU1 = cam.pU1/pixelWidth;             %mm 
    cam.pV1 = cam.pV1/pixelWidth;
    cam.pU2 = cam.pU2/pixelWidth;
    cam.pV2 = cam.pV3/pixelWidth;
    cam.pU3 = cam.pU3/pixelWidth;
    cam.pV3 = cam.pV3/pixelWidth;
for i = 1:n_pixelWidthErr
    %update coords in mm
    cam.u1 = cam.u1*gen_pixelWidthErr(i);              %mm 
    cam.v1 = cam.v1*gen_pixelWidthErr(i);
    cam.u2 = cam.u2*gen_pixelWidthErr(i);
    cam.v2 = cam.v2*gen_pixelWidthErr(i);
    cam.u3 = cam.u3*gen_pixelWidthErr(i);
    cam.v3 = cam.v3*gen_pixelWidthErr(i);
    
    cam.pU1 = cam.pU1*gen_pixelWidthErr(i);              %mm 
    cam.pV1 = cam.pV1*gen_pixelWidthErr(i);
    cam.pU2 = cam.pU2*gen_pixelWidthErr(i);
    cam.pV2 = cam.pV2*gen_pixelWidthErr(i);
    cam.pU3 = cam.pU3*gen_pixelWidthErr(i);
    cam.pV3 = cam.pV3*gen_pixelWidthErr(i);
    
    %update errors
    camErr.u1 = (u.u*gen_pixelWidthErr(i))^2;                     %mm 
    camErr.v1 = (u.u*gen_pixelWidthErr(i))^2;                     %mm 
    camErr.u2 = (u.u*gen_pixelWidthErr(i))^2;                     %mm 
    camErr.v2 = (u.u*gen_pixelWidthErr(i))^2;                     %mm 
    camErr.u3 = (u.u*gen_pixelWidthErr(i))^2;                     %mm 
    camErr.v3 = (u.u*gen_pixelWidthErr(i))^2;
    
    camErr.pU1 = (u.u*gen_pixelWidthErr(i))^2;                     %mm 
    camErr.pV1 = (u.u*gen_pixelWidthErr(i))^2;                     %mm 
    camErr.pU2 = (u.u*gen_pixelWidthErr(i))^2;                     %mm 
    camErr.pV2 = (u.u*gen_pixelWidthErr(i))^2;                     %mm 
    camErr.pU3 = (u.u*gen_pixelWidthErr(i))^2;                     %mm 
    camErr.pV3 = (u.u*gen_pixelWidthErr(i))^2;
    
    %evaluate & store error
    genNumErr = generalErrorEvaluation(genError,cam,camErr);     %evaluate error
    genErr_pixelWidthErr(1,i) = genNumErr;                             %update result vector
    
    %reset coords back to pixels
    cam.u1 = cam.u1/gen_pixelWidthErr(i);              %mm 
    cam.v1 = cam.v1/gen_pixelWidthErr(i);
    cam.u2 = cam.u2/gen_pixelWidthErr(i);
    cam.v2 = cam.v2/gen_pixelWidthErr(i);
    cam.u3 = cam.u3/gen_pixelWidthErr(i);
    cam.v3 = cam.v3/gen_pixelWidthErr(i);
    
    cam.pU1 = cam.pU1/gen_pixelWidthErr(i);              %mm 
    cam.pV1 = cam.pV1/gen_pixelWidthErr(i);
    cam.pU2 = cam.pU2/gen_pixelWidthErr(i);
    cam.pV2 = cam.pV2/gen_pixelWidthErr(i);
    cam.pU3 = cam.pU3/gen_pixelWidthErr(i);
    cam.pV3 = cam.pV3/gen_pixelWidthErr(i);
end

    %scale coords back to mm
    cam.u1 = cam.u1*pixelWidth;              %mm 
    cam.v1 = cam.v1*pixelWidth;
    cam.u2 = cam.u2*pixelWidth;
    cam.v2 = cam.v2*pixelWidth;
    cam.u3 = cam.u3*pixelWidth;
    cam.v3 = cam.v3*pixelWidth;
    
    cam.pU1 = cam.pU1*pixelWidth;             %mm 
    cam.pV1 = cam.pV1*pixelWidth;
    cam.pU2 = cam.pU2*pixelWidth;
    cam.pV2 = cam.pV3*pixelWidth;
    cam.pU3 = cam.pU3*pixelWidth;
    cam.pV3 = cam.pV3*pixelWidth;
%% analysis of values
% find minimum error
[minError,minIndex] = min(genErr_pixelWidthErr);

% find minimum error
%[maxError,maxIndex] = max(genErr_pixelWidthErr);


%% plot
font = 20;                                                          % change size of text on graph
lineWidth = 1;                                                      % change line width of graph
figure;plot(gen_pixelWidthErr,genErr_pixelWidthErr,'LineWidth',lineWidth);                       %plot
ylabel('Error (mm)','FontSize',font);xlabel('pixelWidth (mm)','FontSize',font); 
title('Variation of pixelWidth, f = ' + string(cam.f) + 'mm'+ ', d = ' + string(cam.d) + 'mm' + ', theta = ' + string(cam.theta*180/pi)...
    + 'deg'+ ', phi = ' + string(cam.phi*180/pi) + 'deg'+ ', pixelWidth = '+ string(pixelWidth*1000) + 'um, '+ '(u1,v1,u2,v2,u3,v3) = (' ...
    + string(cam.u1/pixelWidth)+','+ string(cam.v1/pixelWidth)+','+ string(cam.u2/pixelWidth)+','+ string(cam.v2/pixelWidth)...
    +','+ string(cam.u3/pixelWidth)+','+ string(cam.v3/pixelWidth)+')','FontSize',font-4);    %label
hold on
plot(gen_pixelWidthErr(minIndex),minError,'rx','MarkerSize',font,'LineWidth',lineWidth)                                          %mark minimum
%plot(gen_pixelWidthErr(maxIndex),maxError,'rx','MarkerSize',font,'LineWidth',lineWidth)                                          %mark maximum

text(gen_pixelWidthErr(ceil(3*minIndex/4)),minError,'Minimum error = ' + string(minError*1000) + 'um @ pixelWidth = ' + string(gen_pixelWidthErr(minIndex)) + 'mm','FontSize',font-12) % add minimum label to graph
%text(gen_pixelWidthErr(ceil(3*maxIndex/4)),maxError,'Maximum error = ' + string(maxError*1000) + 'um @ u.s = ' + string(gen_pixelWidthErr(maxIndex)) + '','FontSize',font-12) % add maximum label to graph

savefig('figErr_pixelWidth')                                     %save
