function [Fengine Mengine] = engineMap(p,c,speed,rho)
global nnzeros

% % Define idle vs. engaged indices
% idle = nnzeros;
% idle(delta_c==1) = 1;
% engaged = ~idle;
% idle = idle==1;
% engaged = engaged==1;
% 
% % p.rpm_idle;

% delta_t_ECU(idle) = delta_t;
% delta_t_ECU(engaged) = delta_t(engaged);


P = interp1(p.curves(:,1),p.curves(:,3),p.rpm).*c.delta_t; % Assuming linear with throttle
Mengine = interp1(p.curves(:,1),p.curves(:,2),p.rpm).*c.delta_t; % Assuming linear with throttle

Fengine(p.gear==0)=0;
Fengine(p.gear~=0) = P(p.gear~=0)./speed(p.gear~=0);
Fengine = Fengine.';

% % Case A. Idling
% p.rpm(c.mode=="idling") = p.rpm_idle;
% Fengine(c.mode=="idling") = 0;
% 
% % Case B. Releasing the clutch
% asdf
% 
% % Case C. Clutch fully engaged
% asdf


end