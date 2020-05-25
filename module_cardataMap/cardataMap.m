function dyn = cardataMap(dyn)

% I. FUNCTIONALITY
% "cardataMap" is used to load the design parameters of the car
% 
% II. INPUTS
%     dyn: core struct
% 
% III. OUTPUTS
%     dyn: core struct updated


% Unfold dyn variable
e = dyn.e; c = dyn.c; g = dyn.g; i = dyn.i; s = dyn.s; d = dyn.d; p = dyn.p; t = dyn.t; a = dyn.a; n = dyn.n; o = dyn.o;

% 1. NUMERICS
n.Ibin2Nm = 0.175126835*1000; % Conversion factor from Ib/in to N/m

% 2. GEOMETRY
g.trackF = 1.23; % [m] Front track
g.trackR = 1.17; % [m] Rear track
g.wb = 1.58; % [m] Wheelbase

g.hgc = 0.300; % [m] Centre of gravity height
g.hrcF = 0.010; % [m] Front roll centre height
g.hrcR = 0.040; % [m] Rear roll centre height

g.wDistr = 0.42; % [% front / 100]

g.hraGC = g.hrcF*g.wDistr + g.hrcR*(1-g.wDistr); % [m] Roll axis height at centre of gravity
g.anglera = atan(g.hrcR-g.hrcF)/g.wb; % [m] Roll axis pitch angle

g.draGC = (g.hgc - g.hraGC)*cos(g.anglera); % [m] Perpendicular distance from ra to GC

g.mrF = 1/0.93; % [-] Front motion ratio
g.mrR = 1/1.09; % [-] Rear motion ratio

g.R = t(3).Rstatic; % [m] Wheel radius (static) - Using rear wheel

% 3. INERTIA
i.mDriver = 70;
i.mCar = 243;
i.m = i.mDriver + i.mCar; % [kg] Total mass estimation

% MIXED: masses that are partially sprung, partially unsprung
% All masses given as total axle mass (adding contributions from both left and right sides)
i.mixed.mArmsF = (0.435+0.440+0.150)*2; % [kg] Mass of front arms (both sides)
i.mixed.mArmsR = (0.646+0.567+0.100)*0.75*2; % [kg] " rear "
i.mixed.mWheelsF = 7.6*2; % [kg] Mass of front wheels (both sides)
i.mixed.mWheelsR = 8.3*2; % [kg] " rear "
i.mixed.mBarF = 0.200*2; % [kg] Mass of front bars (i.e. push/pull) (both sides)
i.mixed.mBarR = 0.300*2; % [kg] " rear "
i.mixed.mRockerF = 0.090*2; % [kg] Mass of front arms (both sides)
i.mixed.mRockerR = 0.110*2; % [kg] " rear "
i.mixed.driveshaft = 0.400*2; % [kg] Mass of driveshaft (both sides)

i.muF = i.mixed.mWheelsF + 0.5*i.mixed.mArmsF + 0.5/g.mrF * (i.mixed.mBarF+i.mixed.mRockerF); % [kg] Front unsprung mass (2 wheels contribution) estimation
i.muR = i.mixed.mWheelsR + 0.5*(i.mixed.mArmsR+i.mixed.driveshaft) + 0.5/g.mrR * (i.mixed.mBarR+i.mixed.mRockerR); % [kg] Rear "

i.mu = i.muF + i.muR; % [kg] Unsprung mass of the whole car with driver
i.ms = i.m - i.mu; % [kg] Sprung mass of the whole car with driver

i.msF = i.ms.*(g.wDistr); % [kg] Front sprung mass (with driver). Lower - Upper bounds
i.msR = i.ms.*(1 - g.wDistr); % [kg] Rear sprung mass (with driver). Lower - Upper bounds

i.Ixx = 25.289; % [kg*m^2]
i.Iyy = 135.560; % [kg*m^2]
i.Izz = 135.243; % [kg*m^2]
i.Ixy = -1.1496; % [kg*m^2] (sign change due to different ref. system wrt VI GRADE)
i.Izx = 3.7654; % [kg*m^2] (sign change due to different ref. system wrt VI GRADE)
i.Iyz = -0.15195; % [kg*m^2] (sign change cancels out (-*- = +)

i.Iengine = 0.076; % [kg*m^2] CBR 600? http://www.fsae.com/forums/showthread.php?3101-ENGINE-ROTATIONAL-INERTIA
i.IWheelsRear = 1/2*i.mixed.mWheelsR*g.R; % [kg*m^2] Assuming uniform distribution of mass and perfect cylinder
i.Idrivetrain = 1.5*i.IWheelsRear; % [kg*m^2] Factor that takes into account inertia of driveshaft, differential, gears+gearshafts, etc.

% 4. STIFFNESS
% s.ktF = 530; % [Ibs/in] Front stiffness. Using data from Hoosier R25B
% s.ktR = 530; % [Ibs/in] Rear "
s.ktF = t(1).Kt/n.Ibin2Nm; % [Ibs/in] Front stiffness. Using data from Hoosier R25B
s.ktR = t(3).Kt/n.Ibin2Nm; % [Ibs/in] Rear "

s.ktF = s.ktF*n.Ibin2Nm; s.ktR = s.ktR*n.Ibin2Nm; % [N/m] Tyre stiffnesses

s.ksF = 250; % [Ibs/in] Front spring stiffness
s.ksR = 200; % [Ibs/in] Rear "
s.ksF = s.ksF*n.Ibin2Nm; s.ksR = s.ksR*n.Ibin2Nm; % [N/m] Springs stiffnesses

% Compliance correction

% s.jounceMeasuredF = 0.02; % [m] Experimental front jounce
% s.jounceMeasuredR = 0.025; % [m] " rear "

if isfield(s, 'jounceMeasuredF') % if there is data about TOTAL jounce
    s = complianceCalculator(e, i, s); % return kc, kw and kr
else
    s.kcF = inf; % [N/m] (front compliance)^-1 ("front compliance stiffness")
    s.kcR = inf; % [N/m] (rear compliance)^-1 ("rear compliance stiffness")
    
    s.kwF = (g.mrF^2./s.ksF + 1./s.kcF).^-1; % [N/m] Front wheel centre stiffness
    s.kwR = (g.mrF^2./s.ksR + 1./s.kcR).^-1; % [N/m] Front wheel centre stiffness
    
    s.krF = (1./s.kwF + 1/s.ktF).^-1; % [N/m] Front ride stiffness
    s.krR = (1./s.kwR + 1/s.ktR).^-1; % [N/m] Front ride stiffness
end

s.fF = 1/(2*pi).*sqrt(s.krF./(i.msF/2)); % [s^-1] Front ride frequency
s.fR = 1/(2*pi).*sqrt(s.krR./(i.msR/2)); % [s^-1] Front ride frequency

s.omegaF = 2*pi*s.fF; % [s^-1] Front natural frequency
s.omegaR = 2*pi*s.fR; % [s^-1] Rear "



% 5. DAMPING
% Dimensional coeffs
d.c.bsF = 4130; % [N/(m/s)] Bump - Slow - Front damping setting
d.c.bfF = 4130;
d.c.rsF = 4130; % [N/(m/s)] Rebound - Slow - Front damping setting
d.c.rfF = 4130;
d.c.bsR = 4130;
d.c.bfR = 4130;
d.c.rsR = 4130;
d.c.rfR = 4130;

% Non-dimensional coeffs
d.ccF = 2*(i.msF/2).*s.omegaF;
d.ccR = 2*(i.msR/2).*s.omegaR;

fieldNames = fieldnames(d.c);
for ii = 1:length(fieldnames(d.c))/2
    d.z.(fieldNames{ii}) = d.c.(fieldNames{ii})./d.ccF;
end
for ii = (length(fieldnames(d.c))/2 + 1):length(fieldnames(d.c))
    d.z.(fieldNames{ii}) = d.c.(fieldNames{ii})./d.ccR;
end

% Fold dyn variable
dyn.g = g; dyn.i = i; dyn.s = s; dyn.d = d; dyn.n = n;


end

