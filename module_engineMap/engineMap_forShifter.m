function Fengine = engineMap_forShifter(p,speed,delta_t,delta_c,rho)
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


P = interp1(p.curves(:,1),p.curves(:,3),p.rpm).*delta_t; % Assuming linear with throttle

Fengine = P./speed;

% Fengine(idle)=0;
% Fengine(engaged) = P(engaged)./speed(engaged);
% Fengine = Fengine.';

end