function simK = simplePinholeIntrinsics()

%% pinhole camera model (just a brief intro)
syms f
% For image coordinates u,v, focal length f, world points x,y,z
% u = U/S and v = V/S

simK = [-f 0 0 0;                  %simplified camera intrinsics
    0 -f 0 0;
    0 0 1 0];                      

end