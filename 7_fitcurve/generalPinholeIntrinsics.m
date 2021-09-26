function genK = generalPinholeIntrinsics()

%% pinhole camera model (just a brief intro)
% For image coordinates u,v, focal length f, world points x,y,z
% u = U/S and v = V/S   

%% camera intrinsics
syms alphaU alphaV pU pV s f
% alphaU & alphaV are scaling factors if CCD pixels are not square
% s is skewness parameter
% pU & PV are offsets of the principal point from the image centre
% f is focal length: the perpendicular distance between the focal and image
% planes

% GENERAL case: genK is a general use camera intrinsics model 
genK = [-alphaU*f s pU 0;
        0 -alphaV*f pV 0;
        0 0 1 0];
end