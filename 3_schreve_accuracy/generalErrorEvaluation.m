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
syms f d theta u1 u2 v1 v2 alphaU alphaV s pU1 pU2 pV1 pV2
syms err_f err_d err_theta err_u1 err_u2 err_v1 err_v2 err_alphaU err_alphaV err_s err_pU1 err_pU2 err_pV1 err_pV2
genSub = subs(genError,{f,d,theta,u1,u2,v1,v2,alphaU,alphaV,s,pU1,pU2,pV1,pV2,err_f,err_d,err_theta,err_u1,err_u2,err_v1,err_v2,err_alphaU,err_alphaV,err_s,err_pU1,err_pU2,err_pV1,err_pV2}...
                ,{cam.f,cam.d,cam.theta,cam.u1,cam.u2,cam.v1,cam.v2,cam.alphaU,cam.alphaV,cam.s,cam.pU1,cam.pU2,cam.pV1,cam.pV2,camErr.f,camErr.d,camErr.theta,camErr.u1,camErr.u2,camErr.v1,camErr.v2,camErr.alphaU,camErr.alphaV,camErr.s,camErr.pU1,camErr.pU2,camErr.pV1,camErr.pV2});
genNumErr = vpa(genSub);
end