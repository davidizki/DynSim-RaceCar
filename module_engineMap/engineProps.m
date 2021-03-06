function dyn = engineProps(dyn)

p = dyn.p; i = dyn.i;

% Empirical data from engine log - rpm drop with no throttle and in neutral (clutch disengaged)
t_i = 0;
t_f = 1.7;
rpm_i = 10599;
rpm_f = 3386;

rpm_avg = mean([rpm_i rpm_f]);
omegaAvg = convangvel(rpm_avg,'rpm','rad/s');
alphaAvg = convangvel(rpm_f-rpm_i,'rpm','rad/s')/(t_f-t_i);

p.M_engine_brake = i.Iengine*alphaAvg; % engine brake torque (when delta_t=0)

% The objective of this function was to compute the torque "consumption" by engine and ancillaries and the real throttle position at idle
% For that purpose, data regarding how fast does the engine deccelerate after a sudden throttle input in idle has been used
% However, it has been discovered that the behaviour of the engine under those circumstances is dominated by engine brake effect (due to work
% "wasted" against vacuum when the throttle is closed), so the torque "consumption" cannot be estimated by the method below.

% Tprod_max_omegaAvg = interp1(p.curves(:,1),p.curves(:,2),rpm_avg);
% Tprod_max_omegaIdle = interp1(p.curves(:,1),p.curves(:,2),p.rpm_idle);
% omegaIdle = convangvel(p.rpm_idle,'rpm','rad/s');
% 
% A = [Tprod_max_omegaIdle omegaAvg; Tprod_max_omegaIdle omegaIdle];
% b = [i.Iengine*alphaAvg; 0];
% x = A\b;
% 
% p.delta_t_idle = x(1);
% p.C_cons = x(2);

dyn.p = p;

end