% Effect of object distance d on variation of error in general model

% min error camera rotation angle theta of pi/2, determined from
% theta_General_v3_allrandomvariables.m

%vary d between 100-10000 mm


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

d_start = 100;                                        % focal length (mm) start value
d_end = 10000;                                        % end value
d_step = 100;                                          % difference between values

gen_d = [d_start:d_step:d_end];                 % store dependent variable f
n_d = length(gen_d);                     % number of iterations in f
gen_err = zeros(1,n_d);                      % store results (independent variable) 

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
for i = 1:n_d           %loop through values of d
%update values
        cam.d = gen_d(i);
        genNumErr = generalErrorEvaluation(genError,cam,camErr);     %evaluate error
        gen_err(1,i) = genNumErr;                             %update result vector
end

%% find error closest to desired
%desiredError = 50e-3;                                       %the required resolution of the sensor
%[closestDif,closestIndex] = min(abs(gen_err - desiredError));    %find the point closest to the desired accuracy
%closestError = gen_err(closestIndex);
%closestText = 'Min error = ' + string(closestError*1000) + 'um @ d = ' + string(gen_d(closestIndex)) + 'mm';
%HFOVText = 'HFOV w/ ' + string(h) + 'mm sensor = ' + string(gen_d(closestIndex)*h/cam.f) + 'mm';


%minError = min(min(gen_err));
%[minIndex_f,minIndex_d] = find(gen_err == minError);

%% fit & plot
font = 20;                                                          % change size of text on graph
lineWidth = 2;                                                      % change line width of graph
figure;

[fitdata,gof] = fit(transpose(gen_d),transpose(gen_err),'poly1'); % creates fit & goodness-of-fit structures
fitlabel = sprintf('Fit: %g*d + %g, standard error = %g',fitdata.p1,fitdata.p2,gof.rmse); % create a label for the fit text

plot(fitdata,transpose(gen_d),transpose(gen_err));                       %plot
xlabel('Object distance, d (mm)','FontSize',font); ylabel('Error (mm)','FontSize',font); 
title('General Model - Variation of d , theta = ' + string(cam.theta) + ' rad '+ ', f = '+ string(cam.f) + ' mm '+ ', pixelWidth = '+ string(pixelWidth*1000) + ' um ' ... 
    + ', u.f = ' + string(u.f) + ' mm '+ ', u.d = ' + string(u.d)+ ' mm '+ ', u.theta = ' + string(u.theta*180/pi)+ ' deg ','FontSize',font-4);    %label
hold on

text(gen_f(ceil(end/4)),gen_err(ceil(end/4))*1.5,fitlabel,'FontSize',font-8); %create a label on the graph describing the fitting curve

%plot(gen_d(closestIndex),closestError,'rx','MarkerSize',font,'LineWidth',lineWidth)
%text(gen_d(closestIndex)*0.9,closestError*0.9,closestText,'FontSize',font-8)
%text(gen_d(closestIndex)*0.9,closestError*0.5,HFOVText,'FontSize',font-8)

savefig('fig1d_d_Gen')     %save    

    