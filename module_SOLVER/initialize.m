function [X0,dyn] = initialize(dyn)

% Unfold parameters
e = dyn.e; c = dyn.c; g = dyn.g; i = dyn.i; s = dyn.s; d = dyn.d; p = dyn.p; a = dyn.a; n = dyn.n; o = dyn.o;

Xe0 = 0; Ye0 = 0; Ze0 = 0; % typically not used in the equations
PSI0 = 0; THETA0 = 0; PHI0 = 0;
Ue0 = 5; Ve0 = 0; We0 = 0;
P0 = 0; Q0 = 0; R0 = 0;

X0 = [Xe0 Ye0 Ze0 PSI0 THETA0 PHI0 Ue0 Ve0 We0 P0 Q0 R0];

p.gear = 1;
% p.rpm = 6200; % it will be necessary when grip-limited conditions are taken into account

% Fold parameters
dyn.e = e; dyn.c = c; dyn.g = g; dyn.i = i; dyn.s = s; dyn.d = d; dyn.p = p; dyn.a = a; dyn.n = n; dyn.o = o;

end