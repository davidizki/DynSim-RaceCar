function [DX] = dynSimSolver_parallelV1(T,X,integration_flag)

global dyn

% Unfold parameters
e = dyn.e; c = dyn.c; g = dyn.g; i = dyn.i; s = dyn.s; d = dyn.d; p = dyn.p; a = dyn.a; n = dyn.n; o = dyn.o;

% Unfold the input
X = reshape(X,[],12);
% r = X(:,1:3);
% v = X(:,4:6);

Xe = X(:,1); Ye = X(:,2); Ze = X(:,3); % typically not used in the equations
PSI = X(:,4); THETA = X(:,5); PHI = X(:,6);
U = X(:,7); V = X(:,8); W = X(:,9); % velocity with respect to to earth IN BODY AXES
P = X(:,10); Q = X(:,11); R = X(:,12);


% 1. APPLY DRIVER INPUTS (prescribed driver models)
mode = 'trial';
if integration_flag
    c = controlMap(c,T,mode);
else
    c = controlMap_eval(c,T,mode);
end


% GET DATA FROM MAPS, that have been loaded out of the function (using data from ICs / previous step)

% PROJECT ALL COMPUTED FORCES INTO THE BODY-FIXED FRAME
% Use: https://es.mathworks.com/help/robotics/ref/eul2rotm.html
% FRAMES:
%     -EARTH FIXED
%     -BODY FIXED (for the tensor of inertia not to change with body motion...)
%     -AERO (wind)
%     -TRACK FIXED: for the forces at the tyres contact patches
%     -FRAMES FOR THE SUSPENSION POINTS AND TYRES

% POSE THE SET OF EQUATIONS FOR THE BODY (including wheel rotation effects; wind) -> Coupled with the suspension and tyres motion?

% POSE THE SUSPENSION POINTS (and tyres) EQUATIONS OF MOTION


% GENERAL
mu = 1.7; %*

% AERO
v = vecnorm([U V W],2,2);
[a, FXaero, FZaero] = aeroMap(a,v,e.w.rho);

% POWERTRAIN
radius = 0.1955;

if integration_flag
    % rpm
    p.rpm = v/radius*p.ratios(p.gear)*(60/2/pi);
    if p.rpm > 13500
        p.rpm = 13500;
    end

    % Gear selection
    if p.rpm > p.rpm_gearshift_top && p.gear < 6
        p.gear = p.gear+1;
    elseif p.rpm < p.rpm_gearshift_bottom && p.gear > 1
        p.gear = p.gear-1;
    end

    FXengine = engineMap(p,v,c.delta_t,e.w.rho);
else
    FXengine = zeros(size(FXaero));
    
    for ii = 1:length(v) % this section is not vectorizable in the post-integration evaluation: looping is unavoidable
        % rpm
        p.rpm = v(ii)/radius*p.ratios(p.gear)*(60/2/pi);
        if p.rpm > 13500
            p.rpm = 13500;
        end
        
        % Gear selection
        if p.rpm > p.rpm_gearshift_top && p.gear < 6
            p.gear = p.gear+1;
        elseif p.rpm < p.rpm_gearshift_bottom && p.gear > 1
            p.gear = p.gear-1;
        end
        
        FXengine(ii) = engineMap(p,v(ii),c.delta_t(ii),e.w.rho);
        p.rpm_history(ii) = p.rpm;
        p.gear_history(ii) = p.gear;
    end
end

% DYNAMICS
theta_t = steeringMap(c.theta_s); % front wheels steering position



% EOM
FX = FXaero + FXengine;

FZtyre = (i.m*e.g + FZaero)./4;
% compute slip angle from theta_t and v direction
% FY=f(alpha,FZtyre,IA,p,V,mu?) from tyreMap
FY = mu*(i.m*e.g + FZaero).*theta_t; % just provisional

FZ = zeros(size(FZtyre));

a_loc = [FX FY FZ]/i.m;

if integration_flag
    a_abs = rotAccVector(a_loc,[U V W]);
    v_abs = rotAccVector([U V W],[U V W]);
else
    a_abs = zeros(size(a_loc));
    for ii = 1:length(v) % this section is not vectorizable in the post-integration evaluation: looping is unavoidable (*CHECK!)
        a_abs(ii,:) = rotAccVector(a_loc(ii,:),[U(ii) V(ii) W(ii)]);
    end
    v_abs = zeros(size([U V W]));
    for ii = 1:length(v) % this section is not vectorizable in the post-integration evaluation: looping is unavoidable (*CHECK!)
        v_abs(ii,:) = rotAccVector([U(ii) V(ii) W(ii)],[U(ii) V(ii) W(ii)]);
    end
end






% % Get the forces and the moments from the derivates and the state variables
% Deltau = u - u0;
% 
% DeltaX = st.Xu*Deltau + st.Xw*w; X = X0 + DeltaX;
% DeltaY = st.Yv*v + st.Yp*p + st.Yr*r; Y = Y0 + DeltaY;
% DeltaZ_nodw = st.Zu*Deltau + st.Zw*w + st.Zq*q; Z_nodw = Z0 + DeltaZ_nodw; % st.Zdw*dw included directly in eq3
% 
% DeltaL = st.Lv*v + st.Lp*p + st.Lr*r; L = L0 + DeltaL;
% DeltaM_nodw = st.Mu*Deltau + st.Mw*w + st.Mq*q; M_nodw = M0 + DeltaM_nodw; % st.Mdw*dw "same" as for z: dw computed in eq3, then used in eq5
% DeltaN = st.Nv*v + st.Np*p + st.Nr*r; N = N0 + DeltaN;

VE = [U,V,W];
if integration_flag
    [U,V,W] = E2B([U,V,W],VE);
    [DXe,DYe,DZe,DPSI,DTHETA,DPHI,DU,DV,DW,DP,DQ,DR] = EoMsolver(i,e,Xe,Ye,Ze,PSI,THETA,PHI,U,V,W,P,Q,R,FX,FY,FZ,VE);
    [U,V,W] = B2E([U,V,W],VE);
    [DU,DV,DW] = B2E([DU,DV,DW],VE);
else
    DXe = zeros(size(T));DYe = zeros(size(T));DZe = zeros(size(T));
    DPSI = zeros(size(T));DTHETA = zeros(size(T));DPHI = zeros(size(T));
    DU = zeros(size(T));DV = zeros(size(T));DW = zeros(size(T));
    DP = zeros(size(T));DQ = zeros(size(T));DR = zeros(size(T));
    for ii = 1:numel(T)
        [DXe(ii),DYe(ii),DZe(ii),DPSI(ii),DTHETA(ii),DPHI(ii),DU(ii),DV(ii),DW(ii),DP(ii),DQ(ii),DR(ii)]...
            = EoMsolver(i,e,Xe(ii),Ye(ii),Ze(ii),PSI(ii),THETA(ii),PHI(ii),U(ii),V(ii),W(ii),P(ii),Q(ii),R(ii),FX(ii),FY(ii),FZ(ii),VE(ii,:));
    end
end

% DXe = U;
% DYe = V;
% DZe = W;
% 
% DU = a_abs(1);
% DV = a_abs(2);
% DW = a_abs(3);

% v_abs = v_abs.';
% DXe = v_abs(:,1);
% DYe = v_abs(:,2);
% DZe = v_abs(:,3);
% 
% DU = a_loc(:,1);
% DV = a_loc(:,2);
% DW = a_loc(:,3);


% Collect the output
if integration_flag
    DX = zeros(12,1); % column vector
    DX(1) = DXe; DX(2) = DYe; DX(3) = DZe;
    DX(4) = DPSI; DX(5) = DTHETA; DX(6) = DPHI;
    DX(7) = DU; DX(8) = DV; DX(9) = DW;
    DX(10) = DP; DX(11) = DQ; DX(12) = DR;
else
    DX = zeros(size(X));
    DX(:,1) = DXe; DX(:,2) = DYe; DX(:,3) = DZe;
    DX(:,4) = DPSI; DX(:,5) = DTHETA; DX(:,6) = DPHI;
    DX(:,7) = DU; DX(:,8) = DV; DX(:,9) = DW;
    DX(:,10) = DP; DX(:,11) = DQ; DX(:,12) = DR;
%     DX(:,1:3) = [U V W];
%     DX(:,4:6) = a_abs;
    
    p.rpm_history;
    p.gear_history;
    o.G = a_loc./e.g;
end


% Fold parameters
dyn.e = e; dyn.c = c; dyn.g = g; dyn.i = i; dyn.s = s; dyn.d = d; dyn.p = p; dyn.a = a; dyn.n = n; dyn.o = o;

end