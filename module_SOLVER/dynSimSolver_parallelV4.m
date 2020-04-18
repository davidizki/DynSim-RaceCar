function [DX, dyn] = dynSimSolver_parallelV4(T,X,dyn)
global integration_flag

% INPUT
% Unfold parameters
e = dyn.e; c = dyn.c; g = dyn.g; i = dyn.i; s = dyn.s; d = dyn.d; p = dyn.p; t = dyn.t; a = dyn.a; n = dyn.n; o = dyn.o;

% Unfold the input
global nn nnzeros nn3zeros
nn = numel(T);
nnzeros = zeros(nn,1);
nn3zeros = zeros(nn,3);
X = reshape(X,[],12);
Xe = X(:,1); Ye = X(:,2); Ze = X(:,3); % typically not used in the equations
PSI = X(:,4); THETA = X(:,5); PHI = X(:,6);
U = X(:,7); V = X(:,8); W = X(:,9); % velocity with respect to earth IN BODY AXES
P = X(:,10); Q = X(:,11); R = X(:,12);

if T > 14 % for debudding purposes
    1;
end

%% 1. CONTROL VECTOR UPDATE (prescribed driver models)
mode = 'trial';
c = controlMap(c,T,mode);
o.theta_t = steeringMap(c.theta_s); % front wheels steering position
o.theta_t(:,2) = o.theta_t(:,1);
o.theta_t(:,3) = nnzeros;
o.theta_t(:,4) = nnzeros;

%% 2. KINEMATICS
% 2.1 Frames
%     -EARTH FIXED
%     -POSITIONAL (same as body fixed, but without roll and pitch rotations: easier to express tyre forces)
%     -TYRE (same as positional but for the steering rotation of the front tyres)
%     -BODY FIXED (for the tensor of inertia not to change with body motion)
%     -To be defined:
%       -AERO (wind)
%       -TRACK FIXED: for the forces at the tyres contact patches
%       -FRAMES FOR THE SUSPENSION POINTS AND TYRES

% 2.1.1 Rotation matrices
rot.E2P = quat2rotm(eul2quat([PSI nnzeros nnzeros],"ZYX"));
rot.P2E = quat2rotm(eul2quat(-[nnzeros nnzeros PSI],"XYZ"));
rot.E2B = quat2rotm(eul2quat([PSI THETA PHI],"ZYX"));
rot.B2E = quat2rotm(eul2quat(-[PHI THETA PSI],"XYZ"));
rot.P2B = quat2rotm(eul2quat([nnzeros THETA PHI],"ZYX"));
rot.B2P = quat2rotm(eul2quat(-[PHI THETA nnzeros],"XYZ"));
for ii = 1:4
    rot.P2T{ii} = quat2rotm(eul2quat([o.theta_t(:,ii) nnzeros nnzeros],"ZYX"));
    rot.T2P{ii} = quat2rotm(eul2quat(-[nnzeros nnzeros o.theta_t(:,ii)],"XYZ"));
end

% 2.2 Basic kinematics
% Express the linear and angular velocities in S_P
Vp = zeros(nn,3);
OMEGAp = zeros(nn,3);
for ii = 1:nn
    Vp(ii,:) = [U(ii) V(ii) W(ii)]*rot.B2P(:,:,ii); 
    OMEGAp(ii,:) = [P(ii) Q(ii) R(ii)]*rot.B2P(:,:,ii);
end

o.speed = vecnorm([Vp(:,1) Vp(:,2)],2,2); % speed: velocity 2-norm of the component of the velocity parallel to the ground
o.beta = atan2(V,U); % car side-slip angle

% 2.2.1 Slip angle at the tyres
% Local sideslip angle
t(1).beta = atan2(V+R*(1-g.wDistr)*g.wb,U);
t(2).beta = atan2(V+R*(1-g.wDistr)*g.wb,U);
t(3).beta = atan2(V-R*g.wDistr*g.wb,U);
t(4).beta = atan2(V-R*g.wDistr*g.wb,U);

% Slip angle
t(1).alpha = -t(1).beta + o.theta_t(:,1);
t(2).alpha = t(2).beta - o.theta_t(:,2);
t(3).alpha = -t(3).beta;
t(4).alpha = t(4).beta;

%% 3. DYNAMICS
% 3.1 FORCES & MOMENTS
% 3.1.A AERODYNAMIC FORCES
[a, F.aero, M.aero] = aeroMap(g,a,o.speed,e.w.rho); % torsor defined with aerodynamic loads applied at the centre of gravity

% 3.1.B SPRING & DAMPER LOADS / "LOAD TRANSFERS"
% Assuming that the corner stiffness is located at the centre of each wheel
% 1st compute the displacement at each wheel centre with respect to initial position (0, defined with static forces)
o.corner(1).r0 = [(1-g.wDistr)*g.wb -g.trackF/2 g.hgc];
o.corner(2).r0 = [(1-g.wDistr)*g.wb g.trackF/2 g.hgc];
o.corner(3).r0 = [-g.wDistr*g.wb -g.trackR/2 g.hgc];
o.corner(4).r0 = [-g.wDistr*g.wb g.trackR/2 g.hgc];

o.corner(1).r = [nnzeros nnzeros nnzeros];
o.corner(2).r = [nnzeros nnzeros nnzeros];
o.corner(3).r = [nnzeros nnzeros nnzeros];
o.corner(4).r = [nnzeros nnzeros nnzeros];

for ii = 1:nn
    for jj = 1:4
        o.corner(jj).r(ii,:) = o.corner(jj).r0*rot.B2P(:,:,ii);
    end
end

for ii = 1:4
    o.corner(ii).r(:,3) = o.corner(ii).r(:,3) + Ze - g.hgc; % deltaZ with respect to static
end

% 2nd compute the load at each spring
for ii = 1:2
    t(ii).Fspring = [nnzeros nnzeros -s.krF*o.corner(ii).r(:,3)]; % if z>0 (car "sinking"), F<0 (springs push upwards)
end
for ii = 3:4
    t(ii).Fspring = [nnzeros nnzeros -s.krR*o.corner(ii).r(:,3)]; % if z>0 (car "sinking"), F<0 (springs push upwards)
end

% 3rd compute the velocity at each damper
% omega cross r
% in the previous section, the Z position is in Earth frame -no rotation needed: z aligned to P frame-. But the velocity is in body frame! Vp, OMEGAp needed

g.hgc_corr = g.hgc - Ze; % if the car sinks, the distance to the ground is reduced
t(1).arm = [(1-g.wDistr)*g.wb*ones(nn,1), -g.trackF/2*ones(nn,1), g.hgc_corr];
t(2).arm = [(1-g.wDistr)*g.wb*ones(nn,1), +g.trackF/2*ones(nn,1), g.hgc_corr];
t(3).arm = [-g.wDistr*g.wb*ones(nn,1), -g.trackR/2*ones(nn,1), g.hgc_corr];
t(4).arm = [-g.wDistr*g.wb*ones(nn,1), +g.trackR/2*ones(nn,1), g.hgc_corr];

for ii = 1:4
    o.corner(ii).dr = Vp + cross(OMEGAp,t(ii).arm);
end

% 4th compute the load at each damper
for ii = 1:2
    t(ii).Fdamper = [nnzeros nnzeros -d.c.bsF*o.corner(ii).dr(:,3)]; % if z>0 (car "sinking"), F<0 (springs push upwards)
end
for ii = 3:4
    t(ii).Fdamper = [nnzeros nnzeros -d.c.bsR*o.corner(ii).dr(:,3)]; % if z>0 (car "sinking"), F<0 (springs push upwards)
end

% 5th Compute the total reaction at each wheel
for ii = 1:4
    t(ii).Fnormal = t(ii).Fstatic + t(ii).Fspring + t(ii).Fdamper;
end

% 3.1.C POWERTRAIN FORCES
p.gear = interp1(p.shift_speeds,[1 2 3 4 5 6],o.speed,'previous','extrap');
p.rpm = s2rpm(p,o.speed,p.gear,g.R);
% p.rpm(p.gear==6 & p.rpm>p.rpm_limit_top) = p.rpm_limit_top;
F.engine = engineMap(p,o.speed,c.delta_t,e.w.rho);
F.engine = [F.engine nnzeros nnzeros];

% 3.1.D TYRES FORCES
% [t(1:4).FZ] = deal((i.m*e.g + FZ.aeroF + FZ.aeroR)./4);
lambda_muy = 0.65; % [-] grip_track/grip_test

sigma = 0;

for ii = 1:4
    [t(ii).Flat, t(ii).Flon, t(ii).SA, t(ii).SR] = tyresMap(-t(ii).Fnormal(:,3),V,lambda_muy,t(ii),t(end),t(ii).alpha,sigma);
end
t(1).Flat = [nnzeros t(1).Flat nnzeros]; % change of sign in 2 and 4 due to all Fy defined in the same sense inside the function
t(2).Flat = [nnzeros -t(2).Flat nnzeros];
t(3).Flat = [nnzeros t(3).Flat nnzeros];
t(4).Flat = [nnzeros -t(4).Flat nnzeros];

t(1).Flon = [nnzeros nnzeros nnzeros];
t(2).Flon = [nnzeros nnzeros nnzeros];
t(3).Flon = F.engine/2;
t(4).Flon = F.engine/2;

for ii = 1:nn
    for jj = 1:4
        t(jj).Fext(ii,:) = (t(jj).Fspring(ii,:)+t(jj).Fdamper(ii,:) + t(jj).Flon(ii,:) + t(jj).Flat(ii,:))*rot.T2P{jj}(:,:,ii);
    end
end

% t(1).Fext = (t(1).Fspring+t(1).Fdamper) + (t(1).Flon.*cos(o.theta_t)-t(1).Flat.*sin(o.theta_t)) + (t(1).Flon.*sin(o.theta_t)+t(1).Flat.*cos(o.theta_t));
% t(2).Fext = (t(2).Fspring+t(2).Fdamper) + (t(2).Flon.*cos(o.theta_t)-t(2).Flat.*sin(o.theta_t)) + (t(2).Flon.*sin(o.theta_t)+t(2).Flat.*cos(o.theta_t));
% t(3).Fext = (t(3).Fspring+t(3).Fdamper) + t(3).Flon + t(3).Flat;
% t(4).Fext = (t(4).Fspring+t(4).Fdamper) + t(4).Flon + t(4).Flat;

% F.tyres = sum([t.Fext],2);
F.tyres = [t(1).Fext(:,1)+t(2).Fext(:,1)+t(3).Fext(:,1)+t(4).Fext(:,1) t(1).Fext(:,2)+t(2).Fext(:,2)+t(3).Fext(:,2)+t(4).Fext(:,2) t(1).Fext(:,3)+t(2).Fext(:,3)+t(3).Fext(:,3)+t(4).Fext(:,3) ];

for ii = 1:4
    t(ii).Mext = cross(t(ii).arm,t(ii).Fext);
end

% M.tyres = sum([t.Mext],2);
M.tyres = [t(1).Mext(:,1)+t(2).Mext(:,1)+t(3).Mext(:,1)+t(4).Mext(:,1) t(1).Mext(:,2)+t(2).Mext(:,2)+t(3).Mext(:,2)+t(4).Mext(:,2) t(1).Mext(:,3)+t(2).Mext(:,3)+t(3).Mext(:,3)+t(4).Mext(:,3) ];

% 3.1.E GROUP ALL FORCES (IN BODY-Z-ALIGNED FRAME; NOT BODY-FIXED!)
% FX.total = FX.aero + sum([t.FX],2); % last term == FX.engine
% FY.total = sum([t.FY],2);
% FZ.total = (FZ.aeroF + FZ.aeroR) + sum([t.Fspring],2) + sum([t.Fspring],2); % static weight is cancelled out by static reactions, and therefore is only relevant in terms of total normal loads at the tyres

F.total = F.tyres + F.aero;

% MX.total = sum([t.Mx],2);
% MY.total = sum([t.My],2) + MY.aero;
% MZ.total = sum([t.Mz],2);

M.total = M.tyres + M.aero;

%% 4. POSE & SOLVE EoM
% POSE THE SET OF EQUATIONS FOR THE BODY (including wheel rotation effects; wind) -> Coupled with the suspension and tyres motion?
% POSE THE SUSPENSION POINTS (and tyres) EQUATIONS OF MOTION
[DXe,DYe,DZe,DPSI,DTHETA,DPHI,DU,DV,DW,DP,DQ,DR] = EoMsolverV3(i,e,Xe,Ye,Ze,PSI,THETA,PHI,U,V,W,P,Q,R,F.total,M.total);

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

o.theta_t = mean([o.theta_t(1) o.theta_t(2)],2);
o.speed;
o.beta;
% o.G = [DU DV DW]./e.g; % WRONG!!!
o.G(:,1) = F.total(:,1)/i.m/e.g;
o.G(:,2) = U.*R./e.g; % by definition of centripetal acceleration
o.G(:,2) = F.total(:,2)/i.m/e.g; % easier
o.F = F;
o.M = M;

% Fold parameters
dyn.e = e; dyn.c = c; dyn.g = g; dyn.i = i; dyn.s = s; dyn.d = d; dyn.p = p; dyn.t = t; dyn.a = a; dyn.n = n; dyn.o = o;

end