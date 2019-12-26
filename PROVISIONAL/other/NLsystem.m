function xp = NLsystem(~,x)

global m g Ix Iy Iz Ixz theta0 u0 st X0 Y0 Z0 L0 M0 N0 

% Unfold the input
xE = x(1); yE = x(2); zE = x(3); % not used in the equations
psi = x(4); theta = x(5); phi = x(6);
u = x(7); v = x(8); w = x(9);
p = x(10); q = x(11); r = x(12);

% Get the forces and the moments from the derivates and the state variables
Deltau = u - u0;

DeltaX = st.Xu*Deltau + st.Xw*w; X = X0 + DeltaX;
DeltaY = st.Yv*v + st.Yp*p + st.Yr*r; Y = Y0 + DeltaY;
DeltaZ_nodw = st.Zu*Deltau + st.Zw*w + st.Zq*q; Z_nodw = Z0 + DeltaZ_nodw; % st.Zdw*dw included directly in eq3

DeltaL = st.Lv*v + st.Lp*p + st.Lr*r; L = L0 + DeltaL;
DeltaM_nodw = st.Mu*Deltau + st.Mw*w + st.Mq*q; M_nodw = M0 + DeltaM_nodw; % st.Mdw*dw "same" as for z: dw computed in eq3, then used in eq5
DeltaN = st.Nv*v + st.Np*p + st.Nr*r; N = N0 + DeltaN;

% Use fzero to solve the decoupled equations -just 1 variable-
ansatz = 0; % initial guess for fzero / fsolve: at time zero perturbations are 0, so 0 may be a good guess

eq1 = @(du) m*(du + q*w - r*v) - (X - m*g*sin(theta));
du = fzero(eq1,ansatz);

eq2 = @(dv) m*(dv + r*u - p*w) - (Y + m*g*cos(theta)*sin(phi));
dv = fzero(eq2,ansatz);

eq3 = @(dw) m*(dw + p*v - q*u) - (Z_nodw + st.Zdw*dw + m*g*cos(theta)*cos(phi));
dw = fzero(eq3,ansatz);

eq5 = @(dq) Iy*dq + r*p*(Ix-Iz) + Ixz*(p^2-r^2) - (M_nodw + st.Mdw*dw);
dq = fzero(eq5,ansatz);

eq7 = @(dphi) p + (q*sin(phi) + r*cos(phi))*tan(theta) - (dphi);
dphi = fzero(eq7,ansatz);

eq8 = @(dtheta) q*cos(phi) - r*sin(phi) - (dtheta);
dtheta = fzero(eq8,ansatz);

eq9 = @(dpsi) (q*sin(phi) + r*cos(phi))*sec(theta) - (dpsi); % sec=1/cos
dpsi = fzero(eq9,ansatz);

eq10 = @(dxE) u*cos(theta)*cos(psi) + v*(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) + w*(cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi)) - (dxE);
dxE = fzero(eq10,ansatz);

eq11 = @(dyE) u*cos(theta)*sin(psi) + v*(sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) + w*(cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi)) - (dyE);
dyE = fzero(eq11,ansatz);

eq12 = @(dzE) -u*sin(theta) + v*sin(phi)*cos(theta) + w*cos(phi)*cos(theta) - (dzE);
dzE = fzero(eq12,ansatz);

% Use fsolve to solve the equations that are coupled -several variables-

function F = eq46(x0_F) % L, M, p, q and r are global inside the scope of NLsystem.m, it is not needed to pass them as input
    dp_F = x0_F(1);
    dr_F = x0_F(2);
    eq4 = Ix*dp_F - Ixz*dr_F + q*r*(Iz-Iy) - Ixz*p*q - (L);
    eq6 = Iz*dr_F - Ixz*dp_F + p*q*(Iy-Ix) + Ixz*q*r - (N);
    F(1) = eq4;
    F(2) = eq6;
end

output_F = fsolve(@eq46,[ansatz ansatz]);
dp = output_F(1);
dr = output_F(2);

% Fold the output
xp = zeros(12,1); % column vector
xp(1) = dxE; xp(2) = dyE; xp(3) = dzE;
xp(4) = dpsi; xp(5) = dtheta; xp(6) = dphi;
xp(7) = du; xp(8) = dv; xp(9) = dw;
xp(10) = dp; xp(11) = dq; xp(12) = dr;

end