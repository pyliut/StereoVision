function simNumErr = simpleErrorEvaluation(simError,cam,camErr)
% required creation of cam & camErr objects in the main program
% the arguments are:
% cam.f,cam.d,cam.theta,cam.u1,cam.u2,cam.v1,cam.v2
% camErr.f,camErr.d,camErr.theta,camErr.u1,camErr.u2,camErr.v1,camErr.v2
% to replace
% f,d,theta,u1,u2,v1,v2
% err_f,err_d,err_theta,err_u1,err_u2,err_v1,err_v2
%-------------Evaluate error--------------------
%SIMPLE
syms f d theta u1 u2 v1 v2
syms err_f err_d err_theta err_u1 err_u2 err_v1 err_v2
simSub = subs(simError,{f,d,theta,u1,u2,v1,v2,err_f,err_d,err_theta,err_u1,err_u2,err_v1,err_v2}...
    ,{cam.f,cam.d,cam.theta,cam.u1,cam.u2,cam.v1,cam.v2,camErr.f,camErr.d,camErr.theta,camErr.u1,camErr.u2,camErr.v1,camErr.v2});
simNumErr = vpa(simSub);
end