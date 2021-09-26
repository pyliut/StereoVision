function [genK1,genK2,genK3] = generalPinholeIntrinsics()

%% pinhole camera model (just a brief intro)
% For image coordinates u,v, focal length f, world points x,y,z
% u = U/S and v = V/S   

%% camera intrinsics
syms alphaU alphaV pU1 pV1 pU2 pV2 pU3 pV3 s f
% alphaU & alphaV are scaling factors if CCD pixels are not square
% s is skewness parameter
% pU & PV are offsets of the principal point from the image centre
% f is focal length: the perpendicular distance between the focal and image
% planes

% GENERAL case: genK is a general use camera intrinsics model 
    genK1 = [-alphaU*f s pU1 0;
            0 -alphaV*f pV1 0;
            0 0 1 0];
    genK2 = [-alphaU*f s pU2 0;
            0 -alphaV*f pV2 0;
            0 0 1 0];
    genK3 = [-alphaU*f s pU3 0;
            0 -alphaV*f pV3 0;
            0 0 1 0];
end