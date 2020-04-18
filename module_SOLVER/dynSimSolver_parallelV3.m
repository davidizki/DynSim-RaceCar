function [DX, dyn] = dynSimSolver_parallelV3(T,X,dyn,integration_flag)

% INPUT
% Unfold parameters
e = dyn.e; c = dyn.c; g = dyn.g; i = dyn.i; s = dyn.s; d = dyn.d; p = dyn.p; t = dyn.t; a = dyn.a; n = dyn.n; o = dyn.o;

% Unfold the input
global nn nnzeros
nn = numel(T);
nnzeros = zeros(nn,1);
X = reshape(X,[],12);
Xe = X(:,1); Ye = X(:,2); Ze = X(:,3); % typically not used in the equations
PSI = X(:,4); THETA = X(:,5); PHI = X(:,6);
U = X(:,7); V = X(:,8); W = X(:,9); % velocity with respect to earth IN BODY AXES
P = X(:,10); Q = X(:,11); R = X(:,12);

if T > 10.3 % for debudding purposes
    1;
end

%% 1. CONTROL VECTOR UPDATE (prescribed driver models)
mode = 'trial';
c = controlMap(c,T,mode);
o.theta_t = steeringMap(c.theta_s); % front wheels steering position

%% 2. KINEMATICS
% 2.1 Frames
%     -EARTH FIXED
%     -POSITIONAL (sames as body fixed, but without roll and pitch rotations: easier to express tyre forces)
%     -BODY FIXED (for the tensor of inertia not to change with body motion)
%     -To be defined:
%       -AERO (wind)
%       -TRACK FIXED: for the forces at the tyres contact patches
%       -FRAMES FOR THE SUSPENSION POINTS AND TYRES

% 2.1.1 Rotation matrices
rot.E2P = quat2rotm(eul2quat([PSI nnzeros nnzeros],"ZYX"));
rot.P2E = quat2rotm(eul2quat(-[nnzeros nnzeros PHI],"XYZ"));
rot.E2B = quat2rotm(eul2quat([PSI THETA PHI],"ZYX"));
rot.B2E = quat2rotm(eul2quat(-[PHI THETA PSI],"XYZ"));
rot.P2B = quat2rotm(eul2quat([nnzeros THETA PHI],"ZYX"));
rot.B2P = quat2rotm(eul2quat(-[PHI THETA nnzeros],"XYZ"));

% 2.2 Basic kinematics
o.speed = vecnorm([U V W],2,2); % speed: velocity 2-norm
o.beta = atan2(V,U); % car side-slip angle

% 2.2.1 Slip angle at the tyres
% Local sideslilp angle
t(1).beta = atan2(V+R*(1-g.wDistr)*g.wb,U);
t(2).beta = atan2(V+R*(1-g.wDistr)*g.wb,U);
t(3).beta = atan2(V-R*g.wDistr*g.wb,U);
t(4).beta = atan2(V-R*g.wDistr*g.wb,U);

% Slip angle
t(1).alpha = -t(1).beta + o.theta_t;
t(2).alpha = t(2).beta - o.theta_t;
t(3).alpha = -t(3).beta;
t(4).alpha = t(4).beta;

%% 3. DYNAMICS
% 3.1 FORCES & MOMENTS
% 3.1.A AERODYNAMIC FORCES
[a, Faero, Maero] = aeroMap(g,a,o.speed,e.w.rho); % torsor defined with aerodynamic loads applied at the centre of gravity
FX.aero = Faero(:,1);
FZ.aero = Faero(:,3);
MY.aero = Maero(:,2);

% 3.1.B SPRING & DAMPER LOADS / "LOAD TRANSFERS"
% Assuming that the corner stiffness is located at the centre of each wheel
% 1st compute the displacement at each wheel centre with respect to initial position (0, defined with static forces)
o.corner.staticFL = [(1-g.wDistr)*g.wb -g.trackF/2 g.hgc];
o.corner.staticFR = [(1-g.wDistr)*g.wb g.trackF/2 g.hgc];
o.corner.staticRL = [-g.wDistr*g.wb -g.trackR/2 g.hgc];
o.corner.staticRR = [-g.wDistr*g.wb g.trackR/2 g.hgc];

% preallocate
o.corner.zFL = nnzeros;
o.corner.zFR = nnzeros;
o.corner.zRL = nnzeros;
o.corner.zRR = nnzeros;
% rotm = eul2rotm(-[PHI THETA nnzeros],"XYZ"); % rotation around the centre of gravity following Euler convention from Earth to body
for ii = 1:nn
    tempFL = o.corner.staticFL*rot.B2P(:,:,ii);
    tempFR = o.corner.staticFR*rot.B2P(:,:,ii);
    tempRL = o.corner.staticRL*rot.B2P(:,:,ii);
    tempRR = o.corner.staticRR*rot.B2P(:,:,ii);
    o.corner.zFL(ii,1) = tempFL(3);
    o.corner.zFR(ii,1) = tempFR(3);
    o.corner.zRL(ii,1) = tempRL(3);
    o.corner.zRR(ii,1) = tempRR(3);
end

o.corner.zFL = o.corner.zFL + Ze - g.hgc; % deltaZ with respect to static
o.corner.zFR = o.corner.zFR + Ze - g.hgc;
o.corner.zRL = o.corner.zRL + Ze - g.hgc;
o.corner.zRR = o.corner.zRR + Ze - g.hgc;

% 2nd compute the load at each spring
FZ.springFL = -s.krF*o.corner.zFL; % if z>0 (car "sinking"), F<0 (springs push upwards)
FZ.springFR = -s.krF*o.corner.zFR;
FZ.springRL = -s.krR*o.corner.zRL;
FZ.springRR = -s.krR*o.corner.zRR;

% 3rd compute the velocity at each damper
% omega cross r
rotm = quat2rotm(eul2quat(-[PHI THETA nnzeros],"XYZ")); % express omega in "body-fixed NON ROTATING" frame
V_rot = zeros(nn,3);
OMEGA_rot = zeros(nn,3);
for ii = 1:nn
    V_rot(ii,:) = [U(ii) V(ii) W(ii)]*rotm(:,:,ii);
    OMEGA_rot(ii,:) = [P(ii) Q(ii) R(ii)]*rotm(:,:,ii);
end

g.hgc_corr = g.hgc + Ze;
t(1).arm = [(1-g.wDistr)*g.wb*ones(nn,1), -g.trackF/2*ones(nn,1), g.hgc_corr];
t(2).arm = [(1-g.wDistr)*g.wb*ones(nn,1), +g.trackF/2*ones(nn,1), g.hgc_corr];
t(3).arm = [-g.wDistr*g.wb*ones(nn,1), -g.trackR/2*ones(nn,1), g.hgc_corr];
t(4).arm = [-g.wDistr*g.wb*ones(nn,1), +g.trackR/2*ones(nn,1), g.hgc_corr];

o.corner.dzFL = cross(OMEGA_rot,t(1).arm);
o.corner.dzFR = cross(OMEGA_rot,t(2).arm);
o.corner.dzRL = cross(OMEGA_rot,t(3).arm);
o.corner.dzRR = cross(OMEGA_rot,t(4).arm);

o.corner.dzFL = o.corner.dzFL(3) + V_rot(:,3);
o.corner.dzFR = o.corner.dzFR(3) + V_rot(:,3);
o.corner.dzRL = o.corner.dzRL(3) + V_rot(:,3);
o.corner.dzRR = o.corner.dzRR(3) + V_rot(:,3);

% 4th compute the load at each damper
FZ.damperFL = -o.corner.dzFL*dyn.d.c.bsF;
FZ.damperFR = -o.corner.dzFR*dyn.d.c.bsF;
FZ.damperRL = -o.corner.dzRL*dyn.d.c.bsR;
FZ.damperRR = -o.corner.dzRR*dyn.d.c.bsR;

FZ.springFL = FZ.springFL + FZ.damperFL;
FZ.springFR = FZ.springFR + FZ.damperFR;
FZ.springRL = FZ.springRL + FZ.damperRL;
FZ.springRR = FZ.springRR + FZ.damperRR;

% 5th Compute the total reaction at each wheel
t(1).FZ = i.m*e.g*(1-g.wDistr)/2 - FZ.springFL; % In the car system ref. the spring force is <0, but for the tyres they are >0!
t(2).FZ = i.m*e.g*(1-g.wDistr)/2 - FZ.springFR;
t(3).FZ = i.m*e.g*g.wDistr/2 - FZ.springRL;
t(4).FZ = i.m*e.g*g.wDistr/2 - FZ.springRR;
% t(1).FZ = i.m*e.g*(1-g.wDistr)/2 + FZ.aeroF/2 + FZ.springFL; % static forces only enter here!
% t(2).FZ = i.m*e.g*(1-g.wDistr)/2 + FZ.aeroF/2 + FZ.springFR;
% t(3).FZ = i.m*e.g*g.wDistr/2 + FZ.aeroR/2 + FZ.springRL;
% t(4).FZ = i.m*e.g*g.wDistr/2 + FZ.aeroR/2 + FZ.springRR;
% t(1).percFZ = [i.m*e.g*(1-g.wDistr)/2*ones(STEPS,1) FZ.aeroF/2 FZ.springFL]./t(1).FZ;
% t(2).percFZ = [i.m*e.g*(1-g.wDistr)/2*ones(STEPS,1) FZ.aeroF/2 FZ.springFR]./t(1).FZ;
% t(3).percFZ = [i.m*e.g*g.wDistr/2*ones(STEPS,1) FZ.aeroR/2 FZ.springRL]./t(1).FZ;
% t(4).percFZ = [i.m*e.g*g.wDistr/2*ones(STEPS,1) FZ.aeroR/2 FZ.springRR]./t(1).FZ;

% 3.1.C POWERTRAIN FORCES
p.gear = interp1(p.shift_speeds,[1 2 3 4 5 6],o.speed,'previous','extrap');
p.rpm = s2rpm(p,o.speed,p.gear,g.R);
% p.rpm(p.gear==6 & p.rpm>p.rpm_limit_top) = p.rpm_limit_top;
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

t(1).FX = nnzeros;
t(2).FX = nnzeros;
t(3).FX = FX.engine/2;
t(4).FX = FX.engine/2;

% 3.1.E GROUP ALL FORCES (IN BODY-Z-ALIGNED FRAME; NOT BODY-FIXED!)
FX.total = FX.aero + sum([t.FX],2); % last term == FX.engine
FY.total = sum([t.FY],2);
FZ.total = (FZ.aero) + (FZ.springFL + FZ.springFR + FZ.springRL + FZ.springRR); % static weight is cancelled out by static reactions, and therefore is only relevant in terms of total normal loads at the tyres

% MX.total = -g.hgc_corr.*((t(1).FY+t(2).FY).*cos(o.theta_t) + t(3).FY+t(4).FY) - g.trackF/2*t(1).FZ + g.trackF/2*t(2).FZ - g.trackR/2*t(3).FZ + g.trackR/2*t(4).FZ; % FYtyres + FZtyres
% MY.total = g.hgc_corr.*((t(1).FX+t(2).FX).*cos(o.theta_t) + t(3).FX+t(4).FX) - (1-g.wDistr)*g.wb*(t(1).FZ+t(2).FZ) + g.wDistr*g.wb*(t(3).FZ+t(4).FZ) + MY.aero; % FXtyres + FZtyres + Aero
% MZ.total = +(1-g.wDistr)*g.wb*(t(1).FY+t(2).FY).*cos(o.theta_t) - g.wDistr*g.wb*(t(3).FY+t(4).FY)...
%     + (g.trackF/2*t(1).FX - g.trackF/2*t(2).FX).*cos(o.theta_t) + g.trackR/2*t(3).FX - g.trackR/2*t(4).FX; % FXtyres + FYtyres

t(1).Fvec = [t(1).FX.*cos(o.theta_t)-t(1).FY.*sin(o.theta_t) t(1).FX.*sin(o.theta_t)+t(1).FY.*cos(o.theta_t) FZ.springFL];
t(2).Fvec = [t(2).FX.*cos(o.theta_t)-t(2).FY.*sin(o.theta_t) t(2).FX.*sin(o.theta_t)+t(2).FY.*cos(o.theta_t) FZ.springFR];
t(3).Fvec = [t(3).FX t(3).FY FZ.springRL];
t(4).Fvec = [t(4).FX t(4).FY FZ.springRR];

for ii = 1:4
    t(ii).Mvec = cross(t(ii).arm,t(ii).Fvec);
    t(ii).Mx = t(ii).Mvec(:,1);
    t(ii).My = t(ii).Mvec(:,2);
    t(ii).Mz = t(ii).Mvec(:,3);
end

MX.total = sum([t.Mx],2);
MY.total = sum([t.My],2) + MY.aero;
MZ.total = sum([t.Mz],2);

%% 4. POSE & SOLVE EoM
% POSE THE SET OF EQUATIONS FOR THE BODY (including wheel rotation effects; wind) -> Coupled with the suspension and tyres motion?
% POSE THE SUSPENSION POINTS (and tyres) EQUATIONS OF MOTION
[DXe,DYe,DZe,DPSI,DTHETA,DPHI,DU,DV,DW,DP,DQ,DR] = EoMsolverV3(i,e,Xe,Ye,Ze,PSI,THETA,PHI,U,V,W,P,Q,R,[FX.total,FY.total,FZ.total],[MX.total,MY.total,MZ.total]);

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


%% 5. OUTPUTS
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