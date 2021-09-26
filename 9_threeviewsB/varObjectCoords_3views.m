% create a 3d model of the setup

%% initial
%------------init------------
% some key camera parameters
cam.theta = 90*pi/180;
cam.phi = 60*pi/180;
cam.d = 80;
cam.f = 30;


%% cameras

% 1D rotations
    Rx = [1 0 0;
        0 cos(cam.phi) -sin(cam.phi);
        0 sin(cam.phi) cos(cam.phi)];

    RxMinus = [1 0 0;
        0 cos(cam.phi) sin(cam.phi);
        0 -sin(cam.phi) cos(cam.phi)];

    Ry = [cos(cam.theta) 0 sin(cam.theta);
        0 1 0;
        -sin(cam.theta) 0 cos(cam.theta)];

%------------R is the rotation matrix-------------------
    R1 = [1 0 0;
        0 1 0;
        0 0 1];

    R2 = RxMinus*Ry;
    R3 = Rx*Ry;

%---------------C is the translation vector-------------------
    C1 = [0;0;0];

    C2 = [cam.d*sin(cam.theta)*cos(cam.phi);
        cam.d*sin(cam.phi);
        cam.d*(1-cos(cam.phi)*cos(cam.theta))];

    C3 = [cam.d*sin(cam.theta)*cos(cam.phi);
        -cam.d*sin(cam.phi);
        cam.d*(1-cos(cam.phi)*cos(cam.theta))];

%% object rotations
xR = [1 0 0;
    0 cos(cam.theta/2) -sin(cam.theta/2);
    0 sin(cam.theta/2) cos(cam.theta/2)]; % about x axis
%yR = [cos(cam.theta/2) 0 sin(cam.theta/2);
%    0 1 0;
%    -sin(cam.theta/2) 0 cos(cam.theta/2)]; % about y axis
%zR = [cos(cam.theta/2) -sin(cam.theta/2) 0;
%    sin(cam.theta/2) cos(cam.theta/2) 0;
%    0 0 1]; % about y axis
%---------cam locations------------
cam1Coords = C1;
cam2Coords = C2;
cam3Coords = C3;

%% Object
%----------object plane-----------

% at origin

xUntransformed = -50:10:50;
yUntransformed = -50:10:50;
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


%% plot for testing
figure;
hold on
%plane
surf(xTransformed,yTransformed,zTransformed)
colormap([0 1 0])
%mesh(xUntransformed,yUntransformed,zUntransformed) %original plane

%labels
xlabel('x (mm)')
ylabel('y (mm)')
zlabel('z (mm)')
title('3-view setup')
axis equal


%camera points
plot3(cam1Coords(1)+10,cam1Coords(2),cam1Coords(3),'rx')
text(cam1Coords(1)+10,cam1Coords(2),cam1Coords(3),'1','FontSize',12)
plot3(cam2Coords(1)+10,cam2Coords(2),cam2Coords(3),'bx')
text(cam2Coords(1)+10,cam2Coords(2),cam2Coords(3),'2','FontSize',12)
plot3(cam3Coords(1)+10,cam3Coords(2),cam3Coords(3),'kx')
text(cam3Coords(1)+10,cam3Coords(2),cam3Coords(3),'3','FontSize',12)

%plot cameras
plotCamera('Size',3,'Orientation',R1,'Location',C1);
plotCamera('Size',3,'Orientation',R2,'Location',C2);
plotCamera('Size',3,'Orientation',R3,'Location',C3);

%plot lines from camera to centre
plot3([cam1Coords(1),xTransformed(ceil(end/2))],[cam1Coords(2),yTransformed(ceil(end/2))],[cam1Coords(3),zTransformed(ceil(end/2),ceil(end/2))],'r')
plot3([cam2Coords(1),xTransformed(ceil(end/2))],[cam2Coords(2),yTransformed(ceil(end/2))],[cam2Coords(3),zTransformed(ceil(end/2),ceil(end/2))],'b')
plot3([cam3Coords(1),xTransformed(ceil(end/2))],[cam3Coords(2),yTransformed(ceil(end/2))],[cam3Coords(3),zTransformed(ceil(end/2),ceil(end/2))],'k')


