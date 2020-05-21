function [DOMEGAe,DOMEGAt] = RotatoryEoMsolver(i,p,c,M,OMEGAe,OMEGAt)
global nnzeros
DOMEGAe = nnzeros;
DOMEGAt = nnzeros;
p.M_clutch_engine = nnzeros;
p.M_clutch_tyre = nnzeros;

% engine == "Upstream of the clutch"; tyre == "Downstream of the clutch"

% Compute angular accelerations
% 1. If in neutral (N)perc_blending = 0.15; % % above rpm_idle at which engine brake starts to decline
perc_blending = 0.15; % % above rpm_idle at which engine brake starts to decline

case1a = p.gear==0 & c.delta_t~=0; % (N) and throttle applied
case1b = p.gear==0 & c.delta_t==0 & (OMEGAe > (1+perc_blending)*p.omega_idle); % (N), no throttle and rpm > idle_rpm*(1+X%)
case1c = p.gear==0 & c.delta_t==0 & (OMEGAe <= (1+perc_blending)*p.omega_idle); % (N), no throttle and rpm < idle_rpm*(1+X%): "blending zone"

DOMEGAe(case1a) = M.engine(case1a)/i.Iengine;
DOMEGAe(case1b) = p.M_engine_brake/i.Iengine;
DOMEGAe(case1c) = p.M_engine_brake*(OMEGAe(case1c)-p.omega_idle)/(perc_blending*p.omega_idle)/i.Iengine;
DOMEGAt(p.gear==0) = 0;

% % 2. If clutch partially engaged
% f = 1 - c.delta_c;
% 
% case2 = (c.delta_c > 0) & (c.delta_c < 1);
% 
% p.M_clutch_engine(case2) = f*;
% p.M_clutch_tyre(case2) = f.*M.engine(case2);
% 
% DOMEGAe(case2) = M.engine(case2) - 

% 2. If clutch totally engaged
DOMEGAe(p.gear~=0) = M.engine(p.gear~=0)./(i.Iengine+i.Idrivetrain./p.ratios(p.gear(p.gear~=0)).');
% DOMEGAe = M.engine/(i.Iengine); %+i.Idrivetrain);
DOMEGAt(p.gear~=0) = DOMEGAe(p.gear~=0)./p.ratios(p.gear(p.gear~=0)).';

end