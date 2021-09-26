function [extrinsics1,extrinsics2,extrinsics3] = generalExtrinsics()
%same as simpleExtrinsics()

%% camera extrinsics
syms R1 R2 R3 C1 C2 C3 theta phi d
% We consider the case where the second camera is the same distance d away 
% from the object though rotated by an angle theta in radians

% 1D rotations
    Rx = [1 0 0;
        0 cos(phi) -sin(phi);
        0 sin(phi) cos(phi)];

    RxMinus = [1 0 0;
        0 cos(phi) sin(phi);
        0 -sin(phi) cos(phi)];

    Ry = [cos(theta) 0 sin(theta);
        0 1 0;
        -sin(theta) 0 cos(theta)];

%------------R is the rotation matrix-------------------
    R1 = [1 0 0;
        0 1 0;
        0 0 1];

    R2 = Rx*Ry;
    R3 = RxMinus*Ry;

%---------------C is the translation vector-------------------
    C1 = [0;0;0];

    C2 = [d*sin(theta)*cos(phi);
        d*sin(phi);
        d*(1-cos(phi)*cos(theta))];

    C3 = [d*sin(theta)*cos(phi);
        -d*sin(phi);
        d*(1-cos(phi)*cos(theta))];

%---------------------extrinsics-----------------------------
    extrinsics1 = [R1, -R1*C1;
                    0 0 0 1];
    extrinsics2 = [R2, -R2*C2;
                    0 0 0 1];
    extrinsics3 = [R3, -R3*C3;
                    0 0 0 1];

end