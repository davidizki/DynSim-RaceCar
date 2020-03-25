function [DX, dyn] = dynSimSolver_parallelV3(T,X,dyn,integration_flag)

% INPUT
% Unfold parameters
e = dyn.e; c = dyn.c; g = dyn.g; i = dyn.i; s = dyn.s; d = dyn.d; p = dyn.p; t = dyn.t; a = dyn.a; n = dyn.n; o = dyn.o;

% Unfold the input
STEPS = numel(T);
X = reshape(X,[],12);
Xe = X(:,1); Ye = X(:,2); Ze = X(:,3); % typically not used in the equations
PSI = X(:,4); THETA = X(:,5); PHI = X(:,6);
U = X(:,7); V = X(:,8); W = X(:,9); % velocity with respect to earth IN BODY AXES
P = X(:,10); Q = X(:,11); R = X(:,12);

if T > 0.1 % for debudding purposes
    1;
end

% 1. CONTROL VECTOR UPDATE (prescribed driver models)
mode = 'trial';
c = controlMap(c,T,mode);
o.theta_t = steeringMap(c.theta_s); % front wheels steering position

% 2. KINEMATICS
% FRAMES:
%     -EARTH FIXED
%     -BODY FIXED (for the tensor of inertia not to change with body motion...)
%     -AERO (wind)
%     -TRACK FIXED: for the forces at the tyres contact patches
%     -FRAMES FOR THE SUSPENSION POINTS AND TYRES

o.speed = vecnorm([U V W],2,2); % velocity 2-norm
o.beta = atan2(V,U);

t(1).beta = atan2(V+R*(1-g.wDistr)*g.wb,U);
t(2).beta = atan2(V+R*(1-g.wDistr)*g.wb,U);
t(3).beta = atan2(V-R*g.wDistr*g.wb,U);
t(4).beta = atan2(V-R*g.wDistr*g.wb,U);

t(1).alpha = -t(1).beta + o.theta_t;
t(2).alpha = t(2).beta - o.theta_t;
t(3).alpha = -t(3).beta;
t(4).alpha = t(4).beta;

% 3. DYNAMICS
% 3.1 FORCES & MOMENTS
% 3.1.A AERODYNAMIC FORCES
[a, FX.aero, FZ.aeroF, FZ.aeroR, MY.aero] = aeroMap(g,a,o.speed,e.w.rho); % assuming aerodynamic loads are applied at the centre of gravity

% 3.1.B LOAD TRANSFERS
% Assuming that the corner stiffness is located at the centre of each wheel
% 1st compute the displacement at each wheel centre
o.corner.staticFL = [(1-g.wDistr)*g.wb -g.trackF/2 0];
o.corner.staticFR = [(1-g.wDistr)*g.wb g.trackF/2 0];
o.corner.staticRL = [g.wDistr*g.wb -g.trackR/2 0];
o.corner.staticRR = [g.wDistr*g.wb g.trackR/2 0];

% preallocate
o.corner.zFL = zeros(STEPS,1);
o.corner.zFR = zeros(STEPS,1);
o.corner.zRL = zeros(STEPS,1);
o.corner.zRR = zeros(STEPS,1);
for ii = 1:STEPS
    rotm = eul2rotm([0 THETA(ii) PHI(ii)],"ZYX"); % rotation around the centre of gravity following Euler convention from Earth to body
    tempFL = o.corner.staticFL*rotm;
    tempFR = o.corner.staticFR*rotm;
    tempRL = o.corner.staticRL*rotm;
    tempRR = o.corner.staticRR*rotm;
    o.corner.zFR(ii,1) = tempFR(3);
    o.corner.zFL(ii,1) = tempFL(3);
    o.corner.zRL(ii,1) = tempRL(3);
    o.corner.zRR(ii,1) = tempRR(3);
end

o.corner.zFL = o.corner.zFL + Ze;
o.corner.zFR = o.corner.zFR + Ze;
o.corner.zRL = o.corner.zRL + Ze;
o.corner.zRR = o.corner.zRR + Ze;

% 2nd compute the load at each spring
FZ.springFL = -s.krF*o.corner.zFL;
FZ.springFR = -s.krF*o.corner.zFR;
FZ.springRL = -s.krR*o.corner.zRL;
FZ.springRR = -s.krR*o.corner.zRR;

% 3rd compute the velocity at each damper

% 4th compute the load at each damper

% 5th Compute the total reaction at each wheel
t(1).FZ = i.m*e.g*(1-g.wDistr)/2 + FZ.aeroF/2 + FZ.springFL;
t(2).FZ = i.m*e.g*(1-g.wDistr)/2 + FZ.aeroF/2 + FZ.springFR;
t(3).FZ = i.m*e.g*g.wDistr/2 + FZ.aeroR/2 + FZ.springRL;
t(4).FZ = i.m*e.g*g.wDistr/2 + FZ.aeroR/2 + FZ.springRR;
t(1).percFZ = [i.m*e.g*(1-g.wDistr)/2*ones(STEPS,1) FZ.aeroF/2 FZ.springFL]./t(1).FZ;
t(2).percFZ = [i.m*e.g*(1-g.wDistr)/2*ones(STEPS,1) FZ.aeroF/2 FZ.springFR]./t(1).FZ;
t(3).percFZ = [i.m*e.g*g.wDistr/2*ones(STEPS,1) FZ.aeroR/2 FZ.springRL]./t(1).FZ;
t(4).percFZ = [i.m*e.g*g.wDistr/2*ones(STEPS,1) FZ.aeroR/2 FZ.springRR]./t(1).FZ;

% 3.1.C POWERTRAIN FORCES
p.gear = interp1(p.shift_speeds,[1 2 3 4 5 6],o.speed,'previous','extrap');
p.rpm = s2rpm(p,o.speed,p.gear,g.R);
FX.engine = engineMap(p,o.speed,c.delta_t,e.w.rho);

% 3.1.D TYRES FORCES
% [t(1:4).FZ] = deal((i.m*e.g + FZ.aeroF + FZ.aeroR)./4);
lambda_muy = 0.65; % [-] grip_track/grip_test

sigma = 0;

for ii = 1:4
    [t(ii).FY, t(ii).FX, t(ii).SA, t(ii).SR] = tyresMap(t(ii).FZ,V,lambda_muy,t(ii),t(end),t(ii).alpha,sigma);
end
t(2).FY = -t(2).FY;
t(4).FY = -t(4).FY;

% 3.1.E GROUP ALL FORCES & PROJECT ALL COMPUTED FORCES INTO THE BODY-FIXED FRAME
FX.total = FX.aero + FX.engine;
FY.total = sum([t.FY],2);
FZ.total = sum([t.FZ],2);

MX.total = -g.hgc*((t(1).FY+t(2).FY).*cos(o.theta_t) + t(3).FY+t(4).FY) - g.trackF/2*t(1).FZ + g.trackF/2*t(2).FZ - g.trackR/2*t(3).FZ + g.trackR/2*t(4).FZ; % FYtyres + FZtyres
MY.total = g.hgc*((t(1).FX+t(2).FX).*cos(o.theta_t) + t(3).FX+t(4).FX) - (1-g.wDistr)*g.wb*(t(1).FZ+t(2).FZ) + g.wDistr*g.wb*(t(3).FZ+t(4).FZ) + MY.aero; % FXtyres + FZtyres + Aero
MZ.total = +(1-g.wDistr)*g.wb*(t(1).FY+t(2).FY).*cos(o.theta_t) - g.wDistr*g.wb*(t(3).FY+t(4).FY)...
    + (g.trackF/2*t(1).FX - g.trackF/2*t(2).FX).*cos(o.theta_t) + g.trackR/2*t(3).FX - g.trackR/2*t(4).FX; % FXtyres + FYtyres

% 3.2 POSE & SOLVE EoM
% POSE THE SET OF EQUATIONS FOR THE BODY (including wheel rotation effects; wind) -> Coupled with the suspension and tyres motion?
% POSE THE SUSPENSION POINTS (and tyres) EQUATIONS OF MOTION
[DXe,DYe,DZe,DPSI,DTHETA,DPHI,DU,DV,DW,DP,DQ,DR] = EoMsolverV3(i,e,Xe,Ye,Ze,PSI,THETA,PHI,U,V,W,P,Q,R,FX.total,FY.total,FZ.total,MX.total,MY.total,MZ.total);

% Cut-off implementation: limit velocity and acceleration
if integration_flag
    speed_nocutoff = vecnorm([DXe DYe DZe],2,2); % velocity 2-norm
    p.gear = interp1(p.shift_speeds,[1 2 3 4 5 6],speed_nocutoff,'previous','extrap');
    p.rpm = s2rpm(p,speed_nocutoff,p.gear,g.R);
    if p.rpm > p.rpm_limit_top % if rpm over cut-off, set it to cut-off
        p.rpm = p.rpm_limit_top;
        o.speed = rpm2s(p,p.rpm,p.gear,g.R);
        DRe = [DXe DYe DZe].*(o.speed./speed_nocutoff); % correct absolute value of velocity (speed)
        DXe = DRe(:,1); DYe = DRe(:,2); DZe = DRe(:,3);
        if DU > 0
            DU = 0; % limit longitudinal acceleration.
        end
    end
end


% OUTPUT
DX = zeros(size(X));
DX(:,1) = DXe; DX(:,2) = DYe; DX(:,3) = DZe;
DX(:,4) = DPSI; DX(:,5) = DTHETA; DX(:,6) = DPHI;
DX(:,7) = DU; DX(:,8) = DV; DX(:,9) = DW;
DX(:,10) = DP; DX(:,11) = DQ; DX(:,12) = DR;

if integration_flag % make it column for ode45 integration
    DX = reshape(DX,[],1);
end

o.theta_t;
o.speed;
o.beta;
% o.G = [DU DV DW]./e.g; % WRONG!!!
o.G(:,1) = FX.total/i.m/e.g;
o.G(:,2) = U.*R./e.g; % by definition of centripetal acceleration
o.G(:,2) = FY.total/i.m/e.g; % easier
o.FX = FX;
o.FY = FY;
o.FZ = FZ;
o.MX = MX;
o.MY = MY;
o.MZ = MZ;

% Fold parameters
dyn.e = e; dyn.c = c; dyn.g = g; dyn.i = i; dyn.s = s; dyn.d = d; dyn.p = p; dyn.t = t; dyn.a = a; dyn.n = n; dyn.o = o;

end