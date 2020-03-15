function [DX, dyn] = dynSimSolver_parallelV3(T,X,dyn,integration_flag)

% INPUT
% Unfold parameters
e = dyn.e; c = dyn.c; g = dyn.g; i = dyn.i; s = dyn.s; d = dyn.d; p = dyn.p; t = dyn.t; a = dyn.a; n = dyn.n; o = dyn.o;

% Unfold the input
X = reshape(X,[],12);
Xe = X(:,1); Ye = X(:,2); Ze = X(:,3); % typically not used in the equations
PSI = X(:,4); THETA = X(:,5); PHI = X(:,6);
U = X(:,7); V = X(:,8); W = X(:,9); % velocity with respect to earth IN BODY AXES
P = X(:,10); Q = X(:,11); R = X(:,12);

if T > 0.1
    1;
end

% 1. CONTROL VECTOR UPDATE (prescribed driver models)
mode = 'trial';
c = controlMap(c,T,mode);
theta_t = steeringMap(c.theta_s); % front wheels steering position

% 2. KINEMATICS
% FRAMES:
%     -EARTH FIXED
%     -BODY FIXED (for the tensor of inertia not to change with body motion...)
%     -AERO (wind)
%     -TRACK FIXED: for the forces at the tyres contact patches
%     -FRAMES FOR THE SUSPENSION POINTS AND TYRES

% VE = [Ue Ve We]; % velocities in earth-fixed frame
speed = vecnorm([U V W],2,2); % velocity 2-norm
% [U,V,W] = E2B([Ue,Ve,We],VE); % velocities in body-fixed frame
beta = atan2(V,U);
t(1).alpha = -beta + theta_t;
t(2).alpha = beta - theta_t;
t(3).alpha = -beta;
t(4).alpha = beta;

% 3. DYNAMICS
% 3.1 FORCES & MOMENTS
% 3.1.A POWERTRAIN FORCES
p.gear = interp1(p.shift_speeds,[1 2 3 4 5 6],speed,'previous','extrap');
p.rpm = s2rpm(p,speed,p.gear,g.R);
FX.engine = engineMap(p,speed,c.delta_t,e.w.rho);

% 3.1.B AERODYNAMIC FORCES
[a, FX.aero, FZ.aero] = aeroMap(a,speed,e.w.rho);

% 3.1.C TYRES FORCES
[t(1:4).FZ] = deal((i.m*e.g + FZ.aero)./4);
lambda_muy = 0.65; % [-] grip_track/grip_test

sigma = 0;

for ii = 1:4
    [t(ii).FY, t(ii).FX, t(ii).SA, t(ii).SR] = tyresMap(t(ii).FZ,V,lambda_muy,t(ii),t(end),t(ii).alpha,sigma);
end
t(2).FY = -t(2).FY;
t(4).FY = -t(4).FY;

% 3.1.D GROUP ALL FORCES & PROJECT ALL COMPUTED FORCES INTO THE BODY-FIXED FRAME
FX.total = FX.aero + FX.engine;

FY.total = sum([t.FY],2);

FZ.total = zeros(size(T));
MX.total = zeros(size(T));
MY.total = zeros(size(T));
MZ.total = zeros(size(T));
MZ.total = +(1-g.wDistr)*g.wb*(t(1).FY+t(2).FY) -g.wDistr*g.wb*(t(3).FY+t(4).FY);

% 3.2 POSE & SOLVE EoM
% POSE THE SET OF EQUATIONS FOR THE BODY (including wheel rotation effects; wind) -> Coupled with the suspension and tyres motion?
% POSE THE SUSPENSION POINTS (and tyres) EQUATIONS OF MOTION
[DXe,DYe,DZe,DPSI,DTHETA,DPHI,DU,DV,DW,DP,DQ,DR] = EoMsolverV2(i,e,Xe,Ye,Ze,PSI,THETA,PHI,U,V,W,P,Q,R,FX.total,FY.total,FZ.total,MX.total,MY.total,MZ.total);

% Cut-off implementation: limit velocity and acceleration
if integration_flag
    speed_nocutoff = vecnorm([DXe DYe DZe],2,2); % velocity 2-norm
    p.gear = interp1(p.shift_speeds,[1 2 3 4 5 6],speed_nocutoff,'previous','extrap');
    p.rpm = s2rpm(p,speed_nocutoff,p.gear,g.R);
    if p.rpm > p.rpm_limit_top % if rpm over cut-off, set it to cut-off
        p.rpm = p.rpm_limit_top;
        speed = rpm2s(p,p.rpm,p.gear,g.R);
        DRe = [DXe DYe DZe].*(speed./speed_nocutoff); % correct absolute value of velocity (speed)
        DXe = DRe(:,1); DYe = DRe(:,2); DZe = DRe(:,3);
        if DU > 0
            DU = 0; % limit longitudinal acceleration
        end
    end
end


% OUTPUT
% [DUe,DVe,DWe] = B2E([DU DV DW],VE); % derivatives of the linear velocities back to earth-fixed frame

DX = zeros(size(X));
DX(:,1) = DXe; DX(:,2) = DYe; DX(:,3) = DZe;
DX(:,4) = DPSI; DX(:,5) = DTHETA; DX(:,6) = DPHI;
DX(:,7) = DU; DX(:,8) = DV; DX(:,9) = DW;
DX(:,10) = DP; DX(:,11) = DQ; DX(:,12) = DR;

if integration_flag % make it column for ode45 integration
    DX = reshape(DX,[],1);
end

o.G = [DU DV DW]./e.g;
o.beta = beta;
o.MZ = MZ.total;

% Fold parameters
dyn.e = e; dyn.c = c; dyn.g = g; dyn.i = i; dyn.s = s; dyn.d = d; dyn.p = p; dyn.t = t; dyn.a = a; dyn.n = n; dyn.o = o;

end