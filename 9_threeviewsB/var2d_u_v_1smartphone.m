% Effect of focal length d a& object distance d variation on error in general model
% min error camera rotation angle theta of pi/2, determined from
% theta_General_v3_allrandomvariables.m
%vary f between 1-100 mm, d between 100-10000 mm
% All other variables & uncertainties are random within a reasonable range

%v2 remove points below cutoff plane
%v3 vary the errors
%v4 no calculations for points below cutoff (saves compute time)

%% create cam & camErr structures
%-------------------sensor parameters-----------------------
%pixelWidth = randi([14 100])/10000;                            % mm, taking 3500x2000 pixels across a 35mm film
pixelWidth = 0.07e-3;
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
cam.phi = 80*pi/180;

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
%% create an object plane 

%object rotations
xR = [1 0 0;
    0 cos(cam.theta/2) -sin(cam.theta/2);
    0 sin(cam.theta/2) cos(cam.theta/2)]; % about x axis
%----------object plane-----------

% at origin

xUntransformed = -50:5:50;
yUntransformed = -50:5:50;
zUntransformed = zeros(length(xUntransformed),length(yUntransformed));

xTransformed = zeros(1,length(xUntransformed));
yTransformed = zeros(1,length(yUntransformed));
zTransformed = zeros(length(xUntransformed),length(yUntransformed));

% rotate object plane

for i = 1:length(xUntransformed)
    for j = 1:length(yUntransformed)
        x0 = xUntransformed(i);
        y0 = yUntransformed(j);
        z0 = zUntransformed(i,j);
        rotated = xR*[x0;y0;z0];
        
        xTransformed(i) = rotated(1);
        yTransformed(j) = rotated(2);
        zTransformed(i,j) = rotated(3)+cam.d;
    end
end

%% create x,y,z & u1,v1,u2,v2 & error vectors
gen_x = xTransformed;                   %object coords
gen_y = yTransformed;                 
gen_z = zTransformed;

gen_u1 = zeros(length(gen_x),length(gen_y));        %image coords
gen_u2 = zeros(length(gen_x),length(gen_y));
gen_u3 = zeros(length(gen_x),length(gen_y));
gen_v1 = zeros(length(gen_x),length(gen_y));
gen_v2 = zeros(length(gen_x),length(gen_y));
gen_v3 = zeros(length(gen_x),length(gen_y));

gen_err = zeros(length(gen_x),length(gen_y));                      % store results (independent variable) 

%% create intrinsics, extrinsics
% same for all solution combinations

[genK1,genK2,genK3] = generalPinholeIntrinsics();
[extrinsics1,extrinsics2,extrinsics3] = generalExtrinsics();
[genSolX,genSolY,genSolZ] = generalWorldPoints(genK1,genK2,genK3,extrinsics1,extrinsics2,extrinsics3);
% find error in symbolic
genError = generalError(genSolX,genSolY,genSolZ);

%% find the u,v coordinates
%--------transformation
numRx = [1 0 0;
        0 cos(cam.phi) -sin(cam.phi);
        0 sin(cam.phi) cos(cam.phi)];

    numRxMinus = [1 0 0;
        0 cos(cam.phi) sin(cam.phi);
        0 -sin(cam.phi) cos(cam.phi)];

    numRy = [cos(cam.theta) 0 sin(cam.theta);
        0 1 0;
        -sin(cam.theta) 0 cos(cam.theta)];

%------------R is the rotation matrix-------------------
    numR1 = [1 0 0;
        0 1 0;
        0 0 1];

    numR2 = numRxMinus*numRy;
    numR3 = numRx*numRy;

%---------------C is the translation vector-------------------
    numC1 = [0;0;0];

    numC2 = [cam.d*sin(cam.theta)*cos(cam.phi);
        cam.d*sin(cam.phi);
        cam.d*(1-cos(cam.phi)*cos(cam.theta))];

    numC3 = [cam.d*sin(cam.theta)*cos(cam.phi);
        -cam.d*sin(cam.phi);
        cam.d*(1-cos(cam.phi)*cos(cam.theta))];

%------camera matrix
% intrinsics
numK1 = [-cam.alphaU*cam.f cam.s cam.pU1 0;
        0 -cam.alphaV*cam.f cam.pV1 0;
        0 0 1 0];
numK2 = [-cam.alphaU*cam.f cam.s cam.pU2 0;
        0 -cam.alphaV*cam.f cam.pV2 0;
        0 0 1 0];
numK3 = [-cam.alphaU*cam.f cam.s cam.pU3 0;
        0 -cam.alphaV*cam.f cam.pV3 0;
        0 0 1 0];
    
%extrinsics
numExtrinsics1 = [numR1, -numR1*numC1;
                0 0 0 1];
numExtrinsics2 = [numR2, -numR2*numC2;
                0 0 0 1];
numExtrinsics3 = [numR3, -numR3*numC3;
                0 0 0 1];

%matrices
numP1 = numK1*numExtrinsics1;
numP2 = numK2*numExtrinsics2;
numP3 = numK3*numExtrinsics3;




for i = 1:length(gen_x)
    for j = 1:length(gen_y)
        worldVector = [gen_x(i);gen_y(j);gen_z(i,j);1];
        imageVector1 = numP1*worldVector;
        imageVector2 = numP2*worldVector;
        imageVector3 = numP3*worldVector;
        gen_u1(i,j) = imageVector1(1)/imageVector1(3);
        gen_v1(i,j) = imageVector1(2)/imageVector1(3);
        gen_u2(i,j) = imageVector2(1)/imageVector2(3);
        gen_v2(i,j) = imageVector2(2)/imageVector2(3);
        gen_u3(i,j) = imageVector3(1)/imageVector3(3);
        gen_v3(i,j) = imageVector3(2)/imageVector3(3);
    end
end
        
        
%% evaluate error whilst varying the parameter
% loop through all values

for i = 1:length(gen_x)
    for j = 1:length(gen_y)      
%update values
        cam.u1 = gen_u1(i,j);              %mm 
        cam.v1 = gen_v1(i,j);
        cam.u2 = gen_u1(i,j);
        cam.v2 = gen_v2(i,j);
        cam.u3 = gen_u3(i,j);
        cam.v3 = gen_v3(i,j);
        genNumErr = generalErrorEvaluation(genError,cam,camErr);     %evaluate error
        gen_err(i,j) = genNumErr;                             %update result vector
    end
end


%% find minimum error
minError = min(min(gen_err));
[minIndex_u,minIndex_v] = find(gen_err == minError);
%minLength = length(minIndex_u);     %find number of minima of same value
%minErrorVector = zeros(1,minLength); % create a zero vector for each minima
%minErrorVector = minErrorVector + minError; % update minimum vector to have the value of the minima


%% plot with points removed & minimum
font = 20;                                                          % change size of text on graph
lineWidth = 2;                                                      % change line width of graph
figure;

% plot error distribution
mesh(gen_y,gen_x,gen_err,'LineWidth',lineWidth);                       %plot
zlabel('Error (mm)','FontSize',font);
ylabel('Object coordinates a (mm)','FontSize',font); xlabel('Object coordinates b (mm)','FontSize',font); 
title('Variation over the object, theta = ' + string(cam.theta) + ' rad '+ ', pixelWidth = '+ string(pixelWidth) + ' mm ' ... 
    + ', f = ' + string(cam.f) + ' mm '+ ', d = ' + string(cam.d)+ ' mm ','FontSize',font-4);    %label
hold on

% plot a maximum error plane at error = 50um
uPoints = [gen_x(1),gen_x(end),gen_x(end),gen_x(1)];    %find points in plane in terms of d
vPoints = [gen_y(1),gen_y(1),gen_y(end),gen_y(end)];                                    %find corresponding f using f = d*h/HFOV
errPoints = [0.05 0.05 0.05 0.05];            %complete the plane (the height is the same as the max error)
fill3(vPoints,uPoints,errPoints,'r')
alpha(0.3)

%plot minimum error & label
plot3(gen_y(minIndex_v),gen_x(minIndex_u),minError,'ko','MarkerSize',2,'LineWidth',lineWidth) %add markers for minima
plot3(gen_y(minIndex_v),gen_x(minIndex_u),minError,'k','MarkerSize',font,'LineWidth',lineWidth)%plot line through minima markers
text(gen_y(minIndex_v),gen_x(minIndex_u),minError,'Minimum error = ' + string(minError*1000) + ' um '...
    + ', (a,b) = (' + string(gen_x(minIndex_u)) + ',' + string(gen_y(minIndex_v)) + ') pixels','FontSize',font-4) % add minimum label to graph
savefig('fig2d_u_v_1smartphone')     %save    

    