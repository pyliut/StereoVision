% Effect of pixel location u2 on variation of error in general model

% min error camera rotation angle theta of pi/2, determined from
% theta_General_v3_allrandomvariables.m



%% create cam & camErr structures

%-------------------sensor parameters-----------------------
pixelWidth = randi([14 100])/10000;                            % mm, taking 3500x2000 pixels across a 35mm film

HFOV = 100;                 %mm horizontal field of view
h = randi([10 50]);                      %mm sensor width


%-------------------uncertainties------------------------------------
%u.f = randi([1 100])/100;                      %mm
u.f = 0.01;
%u.d = randi([1 100])/100;                            %mm
u.d = 0.01;
u.theta = pi/180;                   %radians
u.u = 1;                            %number of pixels - uncertainty in u1,v1,pU1 etc
u.alpha = 1/100;                    %percent
u.s = 1/100;                        %percent
%--------------------------camera params--------------------------------
% simple
cam.f = randi([10 150]);                         %mm not the same as the camera intrinsics f
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
camErr.f = (u.f)^2;                           %mm
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

    %++++++++++++CHANGE VARIABLE NAMES+++++++++++++
u2_start = -2000;                                        % focal length (mm) start value
u2_end = 2000;                                        % end value
u2_step = 10;                                          % difference between values

gen_u2 = [u2_start:u2_step:u2_end];                 % store dependent variable pixelWidth
n_u2 = length(gen_u2);                     % number of iterations in f
gen_err = zeros(1,n_u2);                      % store results (independent variable) 

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
for i = 1:n_u2           %loop through values of d
    
        %++++++++++++++++CHANGE PARAMETER++++++++++++
        cam.u2 = gen_u2(i)*pixelWidth;
        genNumErr = generalErrorEvaluation(genError,cam,camErr);     %evaluate error
        gen_err(1,i) = genNumErr;                             %update result vector
end

%% find central error
desiredCoord = 0;                                       %the required resolution of the sensor
[closestDif,closestIndex] = min(abs(gen_u2 - desiredCoord));    %find the point closest to the desired accuracy
closestError = gen_err(closestIndex);
closestCoord = gen_u2(closestIndex);


%% fit & plot
font = 20;                                                          % change size of text on graph
lineWidth = 2;                                                      % change line width of graph
figure;
hold on

%----------fit------------

zeroIndex = find(gen_u2==0); % find zero in the gen_u2 array. This is used to define the correct range of gen_err to find rmse
[fitdata,gof] = fit(transpose(gen_u2(zeroIndex+1:end)),transpose(gen_err(zeroIndex+1:end)),'power2'); % creates fit & goodness-of-fit structures
fitlabel = sprintf('Fit: %g*u2^{%g} + %g, RMSE = %g',fitdata.a,fitdata.b, fitdata.c,gof.rmse); % create a label for the fit text
%plot(fitdata,transpose(gen_u2(zeroIndex+1:end)),transpose(gen_err(zeroIndex+1:end)));                       %plot
fitError = fitdata.a*gen_u2(zeroIndex+1:end).^fitdata.b + fitdata.c;
plot(gen_u2(zeroIndex+1:end),fitError, 'r', 'LineWidth', lineWidth); %plot with variable linewidth

text(gen_u2(ceil(end/4)),gen_err(ceil(end/4))*1.5,fitlabel,'FontSize',font-8); %create a label on the graph describing the fitting curve
plot(gen_u2,gen_err,'b.');                       % plot error values from the model
legend('fitted curve', 'actual error')

    %+++++++++++CHANGE TITLE+++++++++++++++
xlabel('Pixel position, u2 (mm)','FontSize',font); ylabel('Error (mm)','FontSize',font); 
title('General Model - Variation of u2 , theta = ' + string(cam.theta) + ' rad '+ ', f = '+ string(cam.f) + ' mm '...
    + ', d = '+ string(cam.d) + ' mm '+ ', pixelWidth = '+ string(pixelWidth*1000) + ' um ','FontSize',font-4);    %label
hold on

%plot(gen_u1(closestIndex),closestError,'rx','MarkerSize',font,'LineWidth',lineWidth)
%text(gen_u1(closestIndex)*0.9,closestError*0.9,closestText,'FontSize',font-8)
%text(gen_u2(closestIndex)*0.9,closestError*0.5,HFOVText,'FontSize',font-8)

    %++++++++++++CHANGE FIG NAME++++++++++++++++++++
savefig('fig1d_u2_Gen')     %save    

    