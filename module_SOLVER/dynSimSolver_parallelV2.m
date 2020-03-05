function [DX, dyn] = dynSimSolver_parallelV2(T,X,dyn,integration_flag)

% INPUT
% Unfold parameters
e = dyn.e; c = dyn.c; g = dyn.g; i = dyn.i; s = dyn.s; d = dyn.d; p = dyn.p; a = dyn.a; n = dyn.n; o = dyn.o;

% Unfold the input
X = reshape(X,[],12);
Xe = X(:,1); Ye = X(:,2); Ze = X(:,3); % typically not used in the equations
PSI = X(:,4); THETA = X(:,5); PHI = X(:,6);
Ue = X(:,7); Ve = X(:,8); We = X(:,9); % velocity with respect to to earth IN BODY AXES
P = X(:,10); Q = X(:,11); R = X(:,12);


% 1. CONTROL VECTOR UPDATE (prescribed driver models)
mode = 'trial';
c = controlMap(c,T,mode);


% 2. KINEMATICS
% FRAMES:
%     -EARTH FIXED
%     -BODY FIXED (for the tensor of inertia not to change with body motion...)
%     -AERO (wind)
%     -TRACK FIXED: for the forces at the tyres contact patches
%     -FRAMES FOR THE SUSPENSION POINTS AND TYRES

VE = [Ue Ve We]; % velocities in earth-fixed frame
speed = vecnorm(VE,2,2); % velocity 2-norm
[U,V,W] = E2B([Ue,Ve,We],VE); % velocities in body-fixed frame


% 3. DYNAMICS
% 3.1 FORCES & MOMENTS
% 3.1.A AERODYNAMIC FORCES
[a, FX.aero, FZ.aero] = aeroMap(a,speed,e.w.rho);


% 3.1.B POWERTRAIN FORCES
p.gear = interp1(p.shift_speeds,[1 2 3 4 5 6],speed,'previous','extrap');
p.rpm = s2rpm(p,speed,p.gear,g.R);
FX.engine = engineMap(p,speed,c.delta_t,e.w.rho);


% 3.1.C TYRES FORCES
theta_t = steeringMap(c.theta_s); % front wheels steering position

% PROVISIONAL
FZ.tyre = (i.m*e.g + FZ.aero)./4;
mu = 1.7; %*
FY.tyre = mu*(i.m*e.g + FZ.aero).*theta_t; % just provisional

% FINAL
% % Compute normal loads at each tyre
% FZ = loadTransfer(FZ,U,V,W...)
% 
% % Compute surface loads at each tyre
% fields = {'tyreFL','tyreFR','tyreRL','tyreRR'};
% compute slip angle from theta_t and v direction
% for ii = 1:4
%     [FX.(fields(ii)), FY.(fields(ii))] = tyresMap(FZ.(fields(ii)),slip(ii),speed(ii),PRESSURE(ii),IA(ii),mu/grip?);
% end


% 3.1.D GROUP ALL FORCES & PROJECT ALL COMPUTED FORCES INTO THE BODY-FIXED FRAME
FX.total = FX.aero + FX.engine;

FY.total = FY.tyre;

FZ.total = zeros(size(T));
MX.total = zeros(size(T));
MY.total = zeros(size(T));
MZ.total = zeros(size(T));

% 3.2 POSE & SOLVE EoM
% POSE THE SET OF EQUATIONS FOR THE BODY (including wheel rotation effects; wind) -> Coupled with the suspension and tyres motion?
% POSE THE SUSPENSION POINTS (and tyres) EQUATIONS OF MOTION
[DXe,DYe,DZe,DPSI,DTHETA,DPHI,DU,DV,DW,DP,DQ,DR] = EoMsolver(i,e,Xe,Ye,Ze,PSI,THETA,PHI,U,V,W,P,Q,R,VE,FX.total,FY.total,FZ.total,MX.total,MY.total,MZ.total);


% OUTPUT
[DUe,DVe,DWe] = B2E([DU DV DW],VE); % derivatives of the linear velocities back to earth-fixed frame

DX = zeros(size(X));
DX(:,1) = DXe; DX(:,2) = DYe; DX(:,3) = DZe;
DX(:,4) = DPSI; DX(:,5) = DTHETA; DX(:,6) = DPHI;
DX(:,7) = DUe; DX(:,8) = DVe; DX(:,9) = DWe;
DX(:,10) = DP; DX(:,11) = DQ; DX(:,12) = DR;

if integration_flag % make it column for ode45 integration
    DX = reshape(DX,[],1);
end

o.G = [DU DV DW]./e.g;


% Fold parameters
dyn.e = e; dyn.c = c; dyn.g = g; dyn.i = i; dyn.s = s; dyn.d = d; dyn.p = p; dyn.a = a; dyn.n = n; dyn.o = o;

end