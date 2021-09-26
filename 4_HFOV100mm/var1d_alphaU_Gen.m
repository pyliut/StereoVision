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
cam.u1 = 1*pixelWidth;              %mm 
cam.u2 = 2*pixelWidth;
cam.v1 = 3*pixelWidth;
cam.v2 = 4*pixelWidth;
% general
cam.alphaU = 1;                     %1:1 ratio of alphas is square
cam.alphaV = 1;
cam.s = 0;                          
cam.pU1 = -1*pixelWidth;             %mm 
cam.pU2 = -2*pixelWidth;
cam.pV1 = -3*pixelWidth;
cam.pV2 = -4*pixelWidth;

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
alpha_start = 1;                                        % focal length (mm) start value
alpha_end = 20;                                        % end value
alpha_step = 0.05;                                          % difference between values

gen_alpha = [alpha_start:alpha_step:alpha_end];                 % store dependent variable pixelWidth
n_alpha = length(gen_alpha);                     % number of iterations in f
gen_err = zeros(1,n_alpha);                      % store results (independent variable) 

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
cam.u1 = cam.u1/pixelWidth;           %mm 
cam.u2 = cam.u2/pixelWidth; 

cam.pU1 = cam.pU1/pixelWidth;              %mm 
cam.pU2 = cam.pU2/pixelWidth; 

for i = 1:n_alpha           %loop through values of d
    
        %++++++++++++++++CHANGE PARAMETER++++++++++++
        cam.alphaU = gen_alpha(i);
        adjustedPixelWidth = cam.alphaU*pixelWidth;
        
        cam.u1 = cam.u1*adjustedPixelWidth;              %mm 
        cam.u2 = cam.u2*adjustedPixelWidth;

        cam.pU1 = cam.pU1*adjustedPixelWidth;             %mm 
        cam.pU2 = cam.pU2*adjustedPixelWidth;

        camErr.u1 = (u.u*adjustedPixelWidth)^2;                     %mm 
        camErr.u2 = (u.u*adjustedPixelWidth)^2;                     %mm 

        camErr.pU1 = (u.u*adjustedPixelWidth)^2;                    %mm 
        camErr.pU2 = (u.u*adjustedPixelWidth)^2;                    %mm 

        %calculate error and write into matrix
        genNumErr = generalErrorEvaluation(genError,cam,camErr);     %evaluate error
        gen_err(1,i) = genNumErr;                             %update result vector
        %reset cam.u/v
        cam.u1 = cam.u1/adjustedPixelWidth;              %mm 
        cam.u2 = cam.u2/adjustedPixelWidth;

        cam.pU1 = cam.pU1/adjustedPixelWidth;             %mm 
        cam.pU2 = cam.pU2/adjustedPixelWidth;

end

%% find difference in error
minError = gen_err(end);
maxError = gen_err(1);
difError = abs(maxError-minError);
percentError = difError/maxError * 100;

minText = 'Min error = ' + string(minError) + 'mm';
maxText = 'Max error = ' + string(maxError) + 'mm';
difText = 'Error difference = ' + string(difError) + 'mm, %change = ' + string(percentError) + '%';



%% plot 
font = 20;                                                          % change size of text on graph
lineWidth = 2;                                                      % change line width of graph
figure;

plot(gen_alpha,gen_err,'LineWidth',lineWidth);                       %plot

    %+++++++++++CHANGE TITLE+++++++++++++++
xlabel('Pixel scaling factor, alphaU ','FontSize',font); ylabel('Error (mm)','FontSize',font); 
title('General Model - Variation of alphaU , theta = ' + string(cam.theta) + ' rad '+ ', f = '+ string(cam.f) + ' mm '...
    + ', d = '+ string(cam.d) + ' mm '+ ', pixelWidth = '+ string(pixelWidth*1000) + ' um ','FontSize',font-4);    %label
hold on
plot(gen_alpha(1),maxError,'rx','MarkerSize',font,'LineWidth',lineWidth)
plot(gen_alpha(end),minError,'rx','MarkerSize',font,'LineWidth',lineWidth)
text(gen_alpha(1),maxError,maxText,'FontSize',font-8)
text(gen_alpha(ceil(4*end/5)),minError,minText,'FontSize',font-8)
text(gen_alpha(ceil(end/2)),maxError,difText,'FontSize',font-8)

    %++++++++++++CHANGE FIG NAME++++++++++++++++++++
savefig('fig1d_alphaU_Gen')     %save    

    