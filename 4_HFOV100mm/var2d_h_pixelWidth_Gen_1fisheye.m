% Effect of sensor size h & pixelWidth variation on error in general model

% uses min error camera rotation angle theta of pi/2, determined from theta_General_v3_allrandomvariables.m
% vary h between 10-50 mm, pixelWidth between 1.4-8.4 um


%% create cam & camErr structures

%-------------------sensor parameters-----------------------
%pixelWidth = randi([14 100])/10000;                            % mm, taking 3500x2000 pixels across a 35mm film
pixelWidth = 1.4e-3;
HFOV = 100;                 %mm horizontal field of view
h = 10;                      %mm sensor width

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
cam.f = 5;                         %mm not the same as the camera intrinsics f from matlab calibrator
cam.d = 100;                       %mm
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

h_start = 10;                                        % sensor size (mm) start value
h_end = 50;                                        % end value
h_step = 5;                                          % difference between values

pixelWidth_start = 1.4e-3;                                        % pixel width (mm) start value
pixelWidth_end = 8.4e-3;                                        % end value
pixelWidth_step = 0.7e-3;                                          % difference between values

gen_h = [h_start:h_step:h_end];                 % store dependent variable h
gen_pixelWidth = [pixelWidth_start:pixelWidth_step:pixelWidth_end];                 % store dependent variable pixelWidth
n_h = length(gen_h);                     % number of iterations in h
n_pixelWidth = length(gen_pixelWidth);                     % number of iterations in pixelWidth
gen_err = zeros(n_h,n_pixelWidth);                      % store results (independent variable) 

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
for i = 1:n_h           %loop through values of h
    for j = 1:n_pixelWidth       %loop through values of pixelWidth
%update values
        cam.u1 = 0*gen_pixelWidth(j);              %mm 
        cam.u2 = 0*gen_pixelWidth(j);
        cam.v1 = 0*gen_pixelWidth(j);
        cam.v2 = 0*gen_pixelWidth(j);
        cam.pU1 = 0*gen_pixelWidth(j);             %mm 
        cam.pU2 = 0*gen_pixelWidth(j);
        cam.pV1 = 0*gen_pixelWidth(j);
        cam.pV2 = 0*gen_pixelWidth(j);
        camErr.u1 = (u.u*gen_pixelWidth(j))^2;                     %mm 
        camErr.u2 = (u.u*gen_pixelWidth(j))^2;                     %mm 
        camErr.v1 = (u.u*gen_pixelWidth(j))^2;                     %mm 
        camErr.v2 = (u.u*gen_pixelWidth(j))^2;                     %mm 
        camErr.pU1 = (u.u*gen_pixelWidth(j))^2;                    %mm 
        camErr.pU2 = (u.u*gen_pixelWidth(j))^2;                    %mm 
        camErr.pV1 = (u.u*gen_pixelWidth(j))^2;                    %mm 
        camErr.pV2 = (u.u*gen_pixelWidth(j))^2;                    %mm

        if gen_h(i) < HFOV*cam.f/cam.d
            gen_err(i,j) = NaN;
        else
            genNumErr = generalErrorEvaluation(genError,cam,camErr);     %evaluate error
            gen_err(i,j) = genNumErr;                             %update result vector
        end
    end
end

%% plot a plane to find values with the correct HFOV
font = 20;                                                          % change size of text on graph
lineWidth = 2;                                                      % change line width of graph

%dPoints = [gen_pixelWidth(1),gen_pixelWidth(end),gen_pixelWidth(end),gen_pixelWidth(1)];    %find points in plane in terms of d
%fPoints = dPoints .*(h/HFOV);                                    %find corresponding f using f = d*h/HFOV
%maxError = max(max(gen_err));
%zPoints = [0 0 maxError maxError];            %complete the plane (the height is the same as the max error)

%% find minimum error
minError = min(min(gen_err));
[minIndex_f,minIndex_d] = find(gen_err == minError);
minLength = length(minIndex_f);     %find number of minima of same value
minErrorVector = zeros(1,minLength); % create a zero vector for each minima
minErrorVector = minErrorVector + minError; % update minimum vector to have the value of the minima


%% plot with points removed & minimum

figure;
mesh(gen_pixelWidth,gen_h,gen_err,'LineWidth',lineWidth);                       %plot
zlabel('Error (mm)','FontSize',font);
ylabel('Sensor size, h (mm)','FontSize',font); xlabel('Pixel size, pixelWidth (mm)','FontSize',font); 
title('General Model - fisheye lens, f = ' + string(cam.f) + ' mm '+ ', d = '+ string(cam.d) + ' mm ' ... 
    + ', u.f = ' + string(u.f) + ' mm '+ ', u.d = ' + string(u.d)+ ' mm '+ ', u.theta = ' + string(u.theta*180/pi)+ ' deg ','FontSize',font-4);    %label
hold on
%fill3(dPoints,fPoints,zPoints,'r')
%alpha(0.3)
plot3(gen_pixelWidth(minIndex_d),gen_h(minIndex_f),minErrorVector,'ko','MarkerSize',2,'LineWidth',lineWidth) %add markers for minima
plot3(gen_pixelWidth(minIndex_d),gen_h(minIndex_f),minErrorVector,'k','MarkerSize',font,'LineWidth',lineWidth)%plot line through minima markers
%plot([gen_f(colBound(1)),gen_f(colBound(end))],[lowerBound,upperBound],'gx','MarkerSize',font,'LineWidth',lineWidth)    %mark bounds
text(gen_pixelWidth(minIndex_d(1)),gen_h(minIndex_f(1)),minErrorVector(1),'Minimum error = ' + string(minErrorVector(1)*1000) + ' um '...
    + ', f = d/' + string(gen_pixelWidth(minIndex_d(1))/gen_h(minIndex_f(1))) + ' mm ','FontSize',font-4) % add minimum label to graph
%text(gen_f(colBound(1)),lowerBound*3.5,'Lower bound = ' + string(lowerBound) + ', Angle = ' + string(gen_f(colBound(1))*180/pi) + ' degrees','FontSize',font) % add lower bound label to graph
%text(gen_f(colBound(end)),upperBound*0.5,'Upper bound = ' + string(upperBound) + ', Angle = ' + string(gen_f(colBound(end))*180/pi) + ' degrees','FontSize',font) % add upper bound label to graph

savefig('fig_h_pixelWidth_Gen_1fisheye')     %save    

    