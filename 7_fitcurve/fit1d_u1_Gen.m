% Effect of pixel location u1 on variation of error in general model

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
u1_start = -2000;                                        % focal length (mm) start value
u1_end = 2000;                                        % end value
u1_step = 10;                                          % difference between values

gen_u1 = [u1_start:u1_step:u1_end];                 % store dependent variable pixelWidth
n_u1 = length(gen_u1);                     % number of iterations in f
gen_err = zeros(1,n_u1);                      % store results (independent variable) 

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
for i = 1:n_u1           %loop through values of d
    
        %++++++++++++++++CHANGE PARAMETER++++++++++++
        cam.u1 = gen_u1(i)* pixelWidth;
        genNumErr = generalErrorEvaluation(genError,cam,camErr);     %evaluate error
        gen_err(1,i) = genNumErr;                             %update result vector
end

%% find central error
desiredCoord = 0;                                       %the required resolution of the sensor
[closestDif,closestIndex] = min(abs(gen_u1 - desiredCoord));    %find the point closest to the desired accuracy
closestError = gen_err(closestIndex);
closestCoord = gen_u1(closestIndex);


%% fit & plot
font = 20;                                                          % change size of text on graph
lineWidth = 2;                                                      % change line width of graph
figure;
hold on

%----------fit-------------
gen_u1Positive = gen_u1(gen_u1 >= 0);   % separates out the positive half of the array
zeroIndex = find(gen_u1==0); % find zero in the gen_u1 array. This is used to define the correct range of gen_err to find rmse
[fitdata,gof] = fit(transpose(gen_u1(zeroIndex+ceil(length(gen_u1))/8:end)),transpose(gen_err(zeroIndex+ceil(length(gen_u1))/8:end)),'poly1'); % creates fit & goodness-of-fit structures
expFitFactor = 5;  %an initial estimate
RMSE = 10; % initial RMSE (this would be a very bad fit)

%loop through values of exp fit factor to find smallest rmse
for trialExpFitFactor = 0.1:0.1:20
    fitError = (closestError-fitdata.p2)*exp(-pixelWidth*gen_u1Positive*trialExpFitFactor) + fitdata.p1*gen_u1Positive+fitdata.p2; % fitted error
    trialRMSE = sqrt(mean((fitError-gen_err(zeroIndex:end)).^2)); % find goodness of fit with rmse
    if trialRMSE < RMSE
        RMSE = trialRMSE;
        expFitFactor = trialExpFitFactor;
    end  
end
fitError = (closestError-fitdata.p2)*exp(-pixelWidth*gen_u1Positive*expFitFactor) + fitdata.p1*gen_u1Positive+fitdata.p2; % recreate best fit line


%--------plot-------------------
%test plot
%plot(gen_u1Positive,(closestError-fitdata.p2)*exp(-pixelWidth*gen_u1Positive));
%plot(gen_u1Positive,fitdata.p1*gen_u1Positive+fitdata.p2);

plot(gen_u1Positive,fitError,'r', 'LineWidth',lineWidth);
fitlabel = sprintf('Error = (%g-%g)e^{-u1*%g} + %g*u1 + %g, rmse = %g', closestError,fitdata.p2,expFitFactor,fitdata.p1,fitdata.p2,RMSE); % make a label for the fitting curve
text(gen_u1(ceil(end/4)),gen_err(ceil(end/4))*1.5,fitlabel,'FontSize',font-8); %create a label on the graph describing the fitting curve
plot(gen_u1,gen_err,'b.');                       % plot error values from the model
legend('fitted curve', 'actual error')

    %+++++++++++CHANGE TITLE+++++++++++++++
xlabel('Pixel position, u1 (mm)','FontSize',font); ylabel('Error (mm)','FontSize',font); 
title('General Model - Variation of u1 , theta = ' + string(cam.theta) + ' rad '+ ', f = '+ string(cam.f) + ' mm '...
    + ', d = '+ string(cam.d) + ' mm '+ ', pixelWidth = '+ string(pixelWidth*1000) + ' um ','FontSize',font-4);    %label
hold on

%plot(gen_u1(closestIndex),closestError,'rx','MarkerSize',font,'LineWidth',lineWidth)
%text(gen_u1(closestIndex)*0.9,closestError*0.9,closestText,'FontSize',font-8)
%text(gen_u2(closestIndex)*0.9,closestError*0.5,HFOVText,'FontSize',font-8)

    %++++++++++++CHANGE FIG NAME++++++++++++++++++++
savefig('fig1d_u1_Gen')     %save    

    