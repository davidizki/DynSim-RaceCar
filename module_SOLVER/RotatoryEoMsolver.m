function [DOMEGAe,DOMEGAt] = RotatoryEoMsolver(i,p,c,M,OMEGAe,OMEGAt)
global nnzeros
DOMEGAe = nnzeros;
DOMEGAt = nnzeros;
p.M_clutch_engine = nnzeros;
p.M_clutch_tyre = nnzeros;

% engine == "Upstream of the clutch"; tyre == "Downstream of the clutch"

% Compute angular accelerations
% 1. If in neutral (N) or clutch fully disengaged
f_idle_blend = 0.15; % above rpm_idle, at which engine brake starts to decline (in % of rpm_idle)

case1a = (p.gear==0 | c.delta_c==1) & c.delta_t~=0; % (N) and throttle applied
case1b = (p.gear==0 | c.delta_c==1) & c.delta_t==0 & (OMEGAe > (1+f_idle_blend)*p.omega_idle); % (N), no throttle and rpm > idle_rpm*(1+X%)
case1c = (p.gear==0 | c.delta_c==1) & c.delta_t==0 & (OMEGAe <= (1+f_idle_blend)*p.omega_idle); % (N), no throttle and rpm < idle_rpm*(1+X%): "blending zone"

DOMEGAe(case1a) = M.engine(case1a)/i.Iengine;
DOMEGAe(case1b) = p.M_engine_brake/i.Iengine;
DOMEGAe(case1c) = p.M_engine_brake*(OMEGAe(case1c)-p.omega_idle)/(f_idle_blend*p.omega_idle)/i.Iengine;
DOMEGAt(p.gear==0) = 0;

% 2. If clutch partially engaged
f_clutch_blend = 1 - c.delta_c; % engagement factor

case2 = (c.delta_c > 0) & (c.delta_c < 1) & (p.gear ~= 0);

p.M_clutch_engine(case2) = f_clutch_blend(case2).*M.tyres(case2);
p.M_clutch_tyre(case2) = f_clutch_blend(case2).*M.engine(case2);

% REVIEW BEFORE UNCOMMENTING
DOMEGAe(case2) = 0;
DOMEGAt(case2) = 0;
% DOMEGAe(case2) = M.engine(case2) - p.M_clutch_engine(case2);
% DOMEGAt(case2) = M.tyres(case2) - p.M_clutch_tyres(case2);

% 2. If clutch totally engaged
case3 = c.delta_c==0 & p.gear~=0;
if any(case3 ~= 0)
    DOMEGAe(case3) = M.engine(case3)./(i.Iengine+i.Idrivetrain./p.ratios(p.gear(case3)).');
    DOMEGAt(case3) = DOMEGAe(case3)./p.ratios(p.gear(case3)).';
end

end