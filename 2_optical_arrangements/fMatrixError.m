% function calculates the fundamental matrix error given 3x1 translation vector, 3x3
% rotation and camera matrices and the error object

function [fundamentalMatrix,errorMatrix] = fMatrixError(t,r,kl,kr,err)

    %% Essential matrix

    % rotation matrix R
    syms r11 r12 r13 r21 r22 r23 r31 r32 r33
    R = [r11 r12 r13;
        r21 r22 r23;
        r31 r32 r33];

    % translation matrix
    syms Tx Ty Tz
    T = [Tx; Ty; Tz];
    Tcross = [0 -Tz Ty;
                Tz 0 -Tx;
                -Ty Tx 0];

    % Essential matrix
    E = Tcross*R;

    %% Fundamental matrix

    % camera matrices
    syms fx fy cxl cyl cxr cyr
    Kl = [fx 0 cxl;
        0 fy cyl;
        0 0 1];
    Kr = [fx 0 cxr;
        0 fy cyr;
        0 0 1];

    % Fundamental matrix
    F = transpose(inv(Kl))*E*inv(Kr);

    %% error calculations

    %initialise errors
    syms errT errR errF errC

    %separate F matrix elements for ease of typing
    f11 = F(1,1);f12 = F(1,2);f13 = F(1,3);
    f21 = F(2,1);f22 = F(2,2);f23 = F(2,3);
    f31 = F(3,1);f32 = F(3,2);f33 = F(3,3);

    %calculate error matrix elements
    err11 = errT^2*(diff(f11,Ty))^2 + errT^2*(diff(f11,Tz))^2 ...
        + errR^2*(diff(f11,r21))^2 + errR^2*(diff(f11,r31))^2 ...
        + errF^2*(diff(f11,fx))^2;
    err12 = errT^2*(diff(f12,Ty))^2 + errT^2*(diff(f12,Tz))^2 ...
        + errR^2*(diff(f12,r22))^2 + errR^2*(diff(f12,r32))^2 ...
        + errF^2*(diff(f12,fx))^2 + errF^2*(diff(f12,fy))^2;
    err13 = errT^2*(diff(f13,Ty))^2 + errT^2*(diff(f13,Tz))^2 ...
        + errR^2*(diff(f13,r21))^2 + errR^2*(diff(f13,r22))^2 + errR^2*(diff(f13,r23))^2 ...
                + errR^2*(diff(f13,r31))^2 + errR^2*(diff(f13,r32))^2 + errR^2*(diff(f13,r33))^2 ...
        + errF^2*(diff(f13,fx))^2 + errF^2*(diff(f13,fy))^2 ... 
        + errC^2*(diff(f13,cxr))^2 + errC^2*(diff(f13,cyr))^2;

    err21 = errT^2*(diff(f21,Tx))^2 + errT^2*(diff(f21,Tz))^2 ...
        + errR^2*(diff(f21,r11))^2 + errR^2*(diff(f21,r31))^2 ...
        + errF^2*(diff(f21,fx))^2 + errF^2*(diff(f21,fy))^2;
    err22 = errT^2*(diff(f22,Tx))^2 + errT^2*(diff(f22,Tz))^2 ...
        + errR^2*(diff(f22,r12))^2 + errR^2*(diff(f22,r32))^2 ...
        + errF^2*(diff(f22,fy))^2;
    err23 = errT^2*(diff(f23,Tx))^2 + errT^2*(diff(f23,Tz))^2 ...
        + errR^2*(diff(f23,r11))^2 + errR^2*(diff(f23,r12))^2 + errR^2*(diff(f23,r13))^2 ...
                + errR^2*(diff(f23,r31))^2 + errR^2*(diff(f23,r32))^2 + errR^2*(diff(f23,r33))^2 ...
        + errF^2*(diff(f23,fx))^2 + errF^2*(diff(f23,fy))^2 ... 
        + errC^2*(diff(f23,cxr))^2 + errC^2*(diff(f23,cyr))^2;

    err31 = errT^2*(diff(f31,Tx))^2 + errT^2*(diff(f31,Ty))^2 + errT^2*(diff(f31,Tz))^2 ...
        + errR^2*(diff(f31,r11))^2 + errR^2*(diff(f31,r21))^2 + errR^2*(diff(f31,r31))^2 ...
        + errF^2*(diff(f31,fx))^2 + errF^2*(diff(f31,fy))^2 ... 
        + errC^2*(diff(f31,cxl))^2 + errC^2*(diff(f31,cyl))^2;
    err32 = errT^2*(diff(f32,Tx))^2 + errT^2*(diff(f32,Ty))^2 + errT^2*(diff(f32,Tz))^2 ...
        + errR^2*(diff(f32,r12))^2 + errR^2*(diff(f32,r22))^2 + errR^2*(diff(f32,r33))^2 ...
        + errF^2*(diff(f32,fx))^2 + errF^2*(diff(f32,fy))^2 ... 
        + errC^2*(diff(f32,cxl))^2 + errC^2*(diff(f32,cyl))^2;
    err33 = errT^2*(diff(f33,Tx))^2 + errT^2*(diff(f33,Ty))^2 + errT^2*(diff(f33,Tz))^2 ...
        + errR^2*(diff(f33,r11))^2 + errR^2*(diff(f33,r12))^2 + errR^2*(diff(f33,r13))^2 ...
                + errR^2*(diff(f33,r21))^2 + errR^2*(diff(f33,r22))^2 + errR^2*(diff(f33,r23))^2 ...
                + errR^2*(diff(f33,r31))^2 + errR^2*(diff(f33,r32))^2 + errR^2*(diff(f33,r33))^2 ...
        + errF^2*(diff(f33,fx))^2 + errF^2*(diff(f33,fy))^2 ... 
        + errC^2*(diff(f33,cxl))^2 + errC^2*(diff(f33,cyl))^2 + errC^2*(diff(f33,cxr))^2 + errC^2*(diff(f33,cyr))^2;

    Error = [err11 err12 err13;
            err21 err22 err23;
            err31 err32 err33];


%substitute in variables
    fundamentalMatrix = subs(F,{errT,errR,errF,errC,fx,fy,cxl,cyl,cxr,cyr,r11,r12,r13,r21,r22,r23,r31,r32,r33,Tx,Ty,Tz},{err.T,err.R,err.F,err.C,kl(1,1),kl(2,2),kl(1,3),kl(2,3),kr(1,3),kr(2,3),r(1,1),r(1,2),r(1,3),r(2,1),r(2,2),r(2,3),r(3,1),r(3,2),r(3,3),t(1),t(2),t(3)})
    errorMatrix = subs(Error,{errT,errR,errF,errC,fx,fy,cxl,cyl,cxr,cyr,r11,r12,r13,r21,r22,r23,r31,r32,r33,Tx,Ty,Tz},{err.T,err.R,err.F,err.C,kl(1,1),kl(2,2),kl(1,3),kl(2,3),kr(1,3),kr(2,3),r(1,1),r(1,2),r(1,3),r(2,1),r(2,2),r(2,3),r(3,1),r(3,2),r(3,3),t(1),t(2),t(3)})
end
