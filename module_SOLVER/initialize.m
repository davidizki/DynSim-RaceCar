function [X0,dyn] = initialize(dyn)

% Unfold parameters
e = dyn.e; c = dyn.c; g = dyn.g; i = dyn.i; s = dyn.s; d = dyn.d; p = dyn.p; t = dyn.t; a = dyn.a; n = dyn.n; o = dyn.o;

Xe0 = 0; Ye0 = 0; Ze0 = 0; % typically not used in the equations
PSI0 = 0; THETA0 = 0; PHI0 = 0;
Ue0 = 5; Ve0 = 0; We0 = 0;
P0 = 0; Q0 = 0; R0 = 0;
OMEGAe = convangvel(p.rpm_idle,'rpm','rad/s'); OMEGAt = Ue0/g.R;

% g.hgc provides the position of the GC in the CAD, without taking into account static compression
% That one is the reference position of the car. However, the static weight will alter the initial position of the car

zF0 = -g.hgc + i.m*g.wDistr*e.g/(2*s.krF);
zR0 = -g.hgc + i.m*(1-g.wDistr)*e.g/(2*s.krR);

% Ze0 = mean([zF0 zR0]);
% THETA0 = atan2(zR0-zF0,g.wb);

t(1).Fstatic = -[0 0 i.m*e.g*g.wDistr/2]; % "-" because they go in negative z direction
t(2).Fstatic = -[0 0 i.m*e.g*g.wDistr/2];
t(3).Fstatic = -[0 0 i.m*e.g*(1-g.wDistr)/2];
t(4).Fstatic = -[0 0 i.m*e.g*(1-g.wDistr)/2];

X0 = [Xe0 Ye0 Ze0 PSI0 THETA0 PHI0 Ue0 Ve0 We0 P0 Q0 R0 OMEGAe OMEGAt];

% p.gear = 1;
% p.rpm = p.rpm_idle;

% Fold parameters
dyn.e = e; dyn.c = c; dyn.g = g; dyn.i = i; dyn.s = s; dyn.d = d; dyn.p = p; dyn.t = t; dyn.a = a; dyn.n = n; dyn.o = o;

end