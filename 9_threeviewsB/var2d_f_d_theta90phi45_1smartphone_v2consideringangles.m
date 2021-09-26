% Effect of focal length d a& object distance d variation on error in general model
% min error camera rotation angle theta of pi/2, determined from
% theta_General_v3_allrandomvariables.m
%vary f between 1-100 mm, d between 100-10000 mm
% All other variables & uncertainties are random within a reasonable range

%% create cam & camErr structures

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
u.theta = pi/1800;                   %radians
u.phi = pi/1800;                     %radians
u.u = 1;                            %number of pixels - uncertainty in u1,v1,pU1 etc
u.alpha = 1/100;                    %percent
u.s = 1/100;                        %percent
%--------------------------camera params--------------------------------
% simple
cam.f = 25;                         %mm not the same as the camera intrinsics f
cam.d = 250;                       %mm
cam.theta = pi/2;                   %radians already determined (see notes at top)
cam.phi = 80*pi/180;

HFOV = HFOV*sin(cam.theta/2);
pixelWidth = pixelWidth*sin(cam.theta/2);

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

%% define parameter variation & store results

f_start = 10;                                        % focal length (mm) start value
f_end = 100;                                        % end value
f_step = 10;                                          % difference between values

d_start = 200;                                        % object distance (mm) start value
d_end = 2000;                                        % end value
d_step = 200;                                          % difference between values

gen_f = [f_start:f_step:f_end];                 % store dependent variable f
gen_d = [d_start:d_step:d_end];                 % store dependent variable d
n_f = length(gen_f);                     % number of iterations in f
n_d = length(gen_d);                     % number of iterations in d
gen_err = zeros(n_f,n_d);                      % store results (independent variable) 


%% create intrinsics, extrinsics
% same for all solution combinations

[genK1,genK2,genK3] = generalPinholeIntrinsics();
[extrinsics1,extrinsics2,extrinsics3] = generalExtrinsics();
[genSolX,genSolY,genSolZ] = generalWorldPoints(genK1,genK2,genK3,extrinsics1,extrinsics2,extrinsics3);
% find error in symbolic
genError = generalError(genSolX,genSolY,genSolZ);

%% evaluate error whilst varying the parameter
% loop through all values
for i = 1:n_f           %loop through values of f
    for j = 1:n_d       %loop through values of d
%update values
        cam.f = gen_f(i);
        cam.d = gen_d(j);
        if gen_d(j)/gen_f(i) < HFOV/h
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
figure;


dPoints = [gen_d(1),gen_d(end),gen_d(end),gen_d(1)];    %find points in plane in terms of d
fPoints = dPoints .*(h/HFOV);                                    %find corresponding f using f = d*h/HFOV
maxError = max(max(gen_err));
zPoints = [0 0 maxError maxError];            %complete the plane (the height is the same as the max error)
fill3(dPoints,fPoints,zPoints,'r')
alpha(0.3)

%% find minimum error
minError = min(min(gen_err));
[minIndex_f,minIndex_d] = find(gen_err == minError);
minLength = length(minIndex_f);     %find number of minima of same value
minErrorVector = zeros(1,minLength); % create a zero vector for each minima
minErrorVector = minErrorVector + minError; % update minimum vector to have the value of the minima


%% plot with points removed & minimum

mesh(gen_d,gen_f,gen_err,'LineWidth',lineWidth);                       %plot
zlabel('Error (mm)','FontSize',font);
ylabel('Focal length, f (mm)','FontSize',font); xlabel('Object distance, d (mm)','FontSize',font); 
title('Variation of f & d with cutoff, theta = ' + string(cam.theta*180/pi) + 'deg'+ ', phi = ' + string(cam.phi*180/pi) + 'deg'...
    + ', pixelWidth = '+ string(pixelWidth*1000) + 'um, ' + '(u1,v1,u2,v2,u3,v3) = (' ...
    + string(cam.u1/pixelWidth)+','+ string(cam.v1/pixelWidth)+','+ string(cam.u2/pixelWidth)+','+ string(cam.v2/pixelWidth)...
    +','+ string(cam.u3/pixelWidth)+','+ string(cam.v3/pixelWidth)+')','FontSize',font-4);    %label
hold on
fill3(dPoints,fPoints,zPoints,'r')
alpha(0.3)
plot3(gen_d(minIndex_d),gen_f(minIndex_f),minErrorVector,'ko','MarkerSize',2,'LineWidth',lineWidth) %add markers for minima
plot3(gen_d(minIndex_d),gen_f(minIndex_f),minErrorVector,'k','MarkerSize',font,'LineWidth',lineWidth)%plot line through minima markers
%plot([gen_f(colBound(1)),gen_f(colBound(end))],[lowerBound,upperBound],'gx','MarkerSize',font,'LineWidth',lineWidth)    %mark bounds
text(gen_d(minIndex_d(1)),gen_f(minIndex_f(1)),minErrorVector(1),'Minimum error = ' + string(minErrorVector(1)*1000) + ' um '...
    + ', f = d/' + string(gen_d(minIndex_d(1))/gen_f(minIndex_f(1))) + ' mm ','FontSize',font-4) % add minimum label to graph
%text(gen_f(colBound(1)),lowerBound*3.5,'Lower bound = ' + string(lowerBound) + ', Angle = ' + string(gen_f(colBound(1))*180/pi) + ' degrees','FontSize',font) % add lower bound label to graph
%text(gen_f(colBound(end)),upperBound*0.5,'Upper bound = ' + string(upperBound) + ', Angle = ' + string(gen_f(colBound(end))*180/pi) + ' degrees','FontSize',font) % add upper bound label to graph

savefig('fig2d_f_d_1smartphone')     %save    

    