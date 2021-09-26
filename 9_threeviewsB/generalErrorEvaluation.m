function genNumErr = generalErrorEvaluation(genError,cam,camErr)
% required creation of cam & camErr objects in the main program
% the arguments are:
% cam.f,cam.d,cam.theta,cam.u1,cam.u2,cam.v1,cam.v2,cam.alphaU,cam.alphaV,cam.s,cam.pU1,cam.pU2,cam.pV1,cam.pV2
% camErr.f,camErr.d,camErr.theta,camErr.u1,camErr.u2,camErr.v1,camErr.v2,camErr.alphaU,camErr.alphaV,camErr.s,camErr.pU1,camErr.pU2,camErr.pV1,camErr.pV2
% to replace
% f,d,theta,u1,u2,v1,v2,alphaU,alphaV,s,pU1,pU2,pV1,pV2
% err_f,err_d,err_theta,err_u1,err_u2,err_v1,err_v2,err_alphaU,err_alphaV,err_s,err_pU1,err_pU2,err_pV1,err_pV2
%------------------------------------------------------
%GENERAL case
    syms f d theta phi u1 v1 u2 v2 u3 v3 alphaU alphaV s pU1 pV1 pU2 pV2 pU3 pV3
    syms err_f err_d err_theta err_phi err_u1 err_v1 err_u2 err_v2 err_u3 err_v3 err_alphaU err_alphaV err_s err_pU1 err_pV1 err_pU2 err_pV2 err_pU3 err_pV3

    genSub = subs(genError,{f,d,theta,phi,u1,v1,u2,v2,u3,v3,alphaU,alphaV,s,pU1,pV1,pU2,pV2,pU3,pV3,err_f,err_d,err_theta,err_phi,err_u1,err_v1,err_u2,err_v2,err_u3,err_v3,err_alphaU,err_alphaV,err_s,err_pU1,err_pV1,err_pU2,err_pV2,err_pU3,err_pV3}...
                    ,{cam.f,cam.d,cam.theta,cam.phi,cam.u1,cam.v1,cam.u2,cam.v2,cam.u3,cam.v3,cam.alphaU,cam.alphaV,cam.s,cam.pU1,cam.pV1,cam.pU2,cam.pV2,cam.pU3,cam.pV3,camErr.f,camErr.d,camErr.theta,camErr.phi,camErr.u1,camErr.v1,camErr.u2,camErr.v2,camErr.u3,camErr.v3,camErr.alphaU,camErr.alphaV,camErr.s,camErr.pU1,camErr.pV1,camErr.pU2,camErr.pV2,camErr.pU3,camErr.pV3});
    genNumErr = vpa(genSub);
end