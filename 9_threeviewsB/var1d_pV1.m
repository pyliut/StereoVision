% How does error vary with pU1?

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
%cam.u1 = randi([-2000 2000])*pixelWidth;              %mm 
%cam.v1 = randi([-2000 2000])*pixelWidth;
%cam.u2 = randi([-2000 2000])*pixelWidth;
%cam.v2 = randi([-2000 2000])*pixelWidth;
%cam.u3 = randi([-2000 2000])*pixelWidth;
%cam.v3 = randi([-2000 2000])*pixelWidth;

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
[genSolX,genSolY,genSolZ] = generalWorldPoints_varEqns(genK1,genK2,genK3,extrinsics1,extrinsics2,extrinsics3,combination);
% find error in symbolic
genError = generalError(genSolX,genSolY,genSolZ);

%% range of independent variable

p_start = -2000*pixelWidth;                                        % focal length (mm) start value
p_end = 2000*pixelWidth;                                        % end value
p_step = 20*pixelWidth;                                          % difference between values

gen_p = [p_start:p_step:p_end];                 % store dependent variable theta
n_p = length(gen_p);                     % number of iterations in theta
genErr_p = zeros(1,n_p);                      % store results (independent variable) 

%% evaluate error whilst varying the parameter
% loop through all values
for i = 1:n_p
%update values
    cam.pV1 = gen_p(i);                        
    genNumErr = generalErrorEvaluation(genError,cam,camErr);     %evaluate error
    genErr_p(1,i) = genNumErr;                             %update result vector
end

%% analysis of values
% find minimum error
[minError,minIndex] = min(genErr_p);

% find range in which the error is within 5% of minimum error
%[rowBound,colBound] = find(genErr_phi<(1.05*minError));        %colBound is the column indices of the values within the specified boundaries
%lowerBound = genErr_phi(colBound(1));                       %lowerbound is the maximum error within the specified bounds


%% plot
font = 20;                                                          % change size of text on graph
lineWidth = 1;                                                      % change line width of graph
figure;plot(gen_p/pixelWidth,genErr_p,'LineWidth',lineWidth);                       %plot
ylabel('Error (mm)','FontSize',font);xlabel('Principle coord, pV1 (pixels)','FontSize',font); 
title('Variation of pV1, f = ' + string(cam.f) + 'mm'+ ', d = ' + string(cam.d) + 'mm' + ', theta = ' + string(cam.theta*180/pi)...
    + 'deg'+ ', phi = ' + string(cam.phi*180/pi) + 'deg'+ ', pixelWidth = '+ string(pixelWidth*1000) + 'um','FontSize',font-4);    %label
hold on
plot(gen_p(minIndex),minError,'rx','MarkerSize',font,'LineWidth',lineWidth)                                                       %mark minimum
%plot([gen_phi(colBound(1)),gen_phi(colBound(end))],[lowerBound,upperBound],'gx','MarkerSize',font,'LineWidth',lineWidth)    %mark both bounds
%plot(gen_phi(colBound(1))*180/pi,lowerBound,'gx','MarkerSize',font,'LineWidth',lineWidth)    %mark only the lower bound
text(gen_p(ceil(3*minIndex/4)),minError,'Minimum error = ' + string(minError*1000) + 'um, Angle = ' + string(gen_p(minIndex)) + ' pixels','FontSize',font-8) % add minimum label to graph
%text(gen_phi(colBound(1))*180/pi,lowerBound,'5% bound = ' + string(lowerBound*1000) + 'um, Angle = ' + string(gen_phi(colBound(1))*180/pi) + ' degrees','FontSize',font-10) % add lower bound label to graph

savefig('fig1d_pV1')                                     %save
