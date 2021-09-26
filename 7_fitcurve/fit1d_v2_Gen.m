% Effect of pixel location v2 on variation of error in general model

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
cam.u1 = randi([-100 100])*pixelWidth;              %mm 
cam.u2 = randi([-100 100])*pixelWidth;
cam.v1 = randi([-100 100])*pixelWidth;
cam.v2 = randi([-100 100])*pixelWidth;
% general
cam.alphaU = 1;                     %1:1 ratio of alphas is square
cam.alphaV = 1;
cam.s = 0;                          
cam.pU1 = randi([-100 100])*pixelWidth;             %mm 
cam.pU2 = randi([-100 100])*pixelWidth;
cam.pV1 = randi([-100 100])*pixelWidth;
cam.pV2 = randi([-100 100])*pixelWidth;

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
v2_start = -2000;                                        % focal length (mm) start value
v2_end = 2000;                                        % end value
v2_step = 10;                                          % difference between values

gen_v2 = [v2_start:v2_step:v2_end];                 % store dependent variable pixelWidth
n_v2 = length(gen_v2);                     % number of iterations in f
gen_err = zeros(1,n_v2);                      % store results (independent variable) 

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
for i = 1:n_v2           %loop through values of d
    
        %++++++++++++++++CHANGE PARAMETER++++++++++++
        cam.v2 = gen_v2(i)* pixelWidth;
        genNumErr = generalErrorEvaluation(genError,cam,camErr);     %evaluate error
        gen_err(1,i) = genNumErr;                             %update result vector
end

%% find error closest to desired
%desiredError = 50e-3;                                       %the required resolution of the sensor
%[closestDif,closestIndex] = min(abs(gen_err - desiredError));    %find the point closest to the desired accuracy
%closestError = gen_err(closestIndex);

    %++++++++++++++CHANGE MARKER++++++++++++++
%closestText = 'Min error = ' + string(closestError*1000) + 'um @ v2 = ' + string(gen_v2(closestIndex)) + 'pixels';
%HFOVText = 'HFOV w/ ' + string(h) + 'mm sensor = ' + string(gen_u2(closestIndex)*h/cam.f) + 'mm';


%minError = min(min(gen_err));
%[minIndex_f,minIndex_d] = find(gen_err == minError);

%% plot 
font = 20;                                                          % change size of text on graph
lineWidth = 2;                                                      % change line width of graph
figure;

[fitdata,gof] = fit(transpose(gen_v2),transpose(gen_err),'poly1'); % creates fit & goodness-of-fit structures
fitlabel = sprintf('Fit: %g*v2 + %g, standard error = %g',fitdata.p1,fitdata.p2,gof.rmse); % create a label for the fit text

plot(fitdata,transpose(gen_v2),transpose(gen_err));                       %plot

%plot(gen_v2,gen_err,'LineWidth',lineWidth);                       %plot

coordText = '(' + string(cam.u1/pixelWidth) + ',' +string(cam.u2/pixelWidth) + ',' +string(cam.v1/pixelWidth) + ',' +string(cam.v2/pixelWidth) ...
    + ',' +string(cam.pU1/pixelWidth) + ',' +string(cam.pU2/pixelWidth) + ',' +string(cam.pV1/pixelWidth) + ',' +string(cam.pV2/pixelWidth) + ')';

    %+++++++++++CHANGE TITLE+++++++++++++++
xlabel('Pixel position, v2 (mm)','FontSize',font); ylabel('Error (mm)','FontSize',font); 
title('General Model - Variation of v2 , theta = ' + string(cam.theta) + ' rad '+ ', f = '+ string(cam.f) + ' mm '...
    + ', d = '+ string(cam.d) + ' mm '+ ', pixelWidth = '+ string(pixelWidth*1000) + ' um '+ ', (u1,u2,v1,v2,pU1,pU2,pV1,pV2) = ' + coordText,'FontSize',font-8);    %label
hold on
%plot(gen_v2(closestIndex),closestError,'rx','MarkerSize',font,'LineWidth',lineWidth)
%text(gen_v2(closestIndex)*0.9,closestError*0.9,closestText,'FontSize',font-8)
%text(gen_u2(closestIndex)*0.9,closestError*0.5,HFOVText,'FontSize',font-8)

text(gen_v2(ceil(end/4)),gen_err(ceil(end/4)),fitlabel,'FontSize',font-8); %create a label on the graph describing the fitting curve

    %++++++++++++CHANGE FIG NAME++++++++++++++++++++
savefig('fig1d_v2_Gen')     %save    

    