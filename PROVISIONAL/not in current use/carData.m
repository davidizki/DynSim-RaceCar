
% 0.1 Reference
ref.Ibin2Nm = 0.175126835*1000; % Conversion factor from Ib/in to N/m
ref.g = 9.80665; % [m/s^2]

% 0.2 Geometric
g.trackF = 1.23; % [m] Front track
g.trackR = 1.17; % [m] Rear track
g.wb = 1.58; % [m] Wheelbase

g.hgc = 0.300; % [m] Centre of gravity height
g.hrcF = 0.010; % [m] Front roll centre height
g.hrcR = 0.040; % [m] Rear roll centre height

g.wDistr = 0.42; % [% front / 100]

g.hraGC = g.hrcF*g.wDistr + g.hrcR*(1-g.wDistr); % [m] Roll axis height at centre of gravity
g.hraANGLE = (g.hrcR-g.hrcF)/g.wb;

g.ha = (g.hgc - g.hraGC)*cos(g.hraANGLE); % [m] 

g.mrF = 1.0; % [-] Front motion ratio
g.mrR = 1.0; % [-] Rear motion ratio

% 0.3 Inertia
i.m_est = 313; % [kg] Total mass estimation
i.m_tol(2) = 15; % ±[kg] Total mass tolerance

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

i.muF_est = i.mixed.mWheelsF + 0.5*i.mixed.mArmsF + 0.5/g.mrF * (i.mixed.mBarF+i.mixed.mRockerF); % [kg] Front unsprung mass (2 wheels contribution) estimation
i.muR_est = i.mixed.mWheelsR + 0.5*(i.mixed.mArmsR+i.mixed.driveshaft) + 0.5/g.mrR * (i.mixed.mBarR+i.mixed.mRockerR); % [kg] Rear "
i.muF_tol(2) = 0.5; % ±[kg] Front unsprung mass tolerance
i.muR_tol(2) = 0.5; % [kg] Rear "
i.m_tol(1) = -i.m_tol(2); i.muF_tol(1) = -i.muF_tol(2); i.muR_tol(1) = -i.muR_tol(2); % create the negative tolerances

% 0.4 Stiffness
s.ktF = 530; % [Ibs/in] Front stiffness. Using data from Hoosier R25B. It is a funciton of the tyre pressure
s.ktR = 530; % [Ibs/in] Rear "
s.ktF = s.ktF*ref.Ibin2Nm; s.ktR = s.ktR*ref.Ibin2Nm; % [N/m] Tyre stiffnesses

s.kcF = inf; % [N/m] (front compliance)^-1 ("front compliance stiffness")
s.kcR = inf; % [N/m] (rear compliance)^-1 ("rear compliance stiffness")


% s.fF = 3.5; % [s^-1] Front ride frequency. https://www.ijera.com/papers/vol9no3/Series-3/K0903036064%20.pdf
% s.fdelta = 0.15; % [% / 100] Difference in front and rear frequencies
% s.fR = s.fF*(1-s.fdelta); % [s^-1] Rear ride frequency

s.ksF = 250; % [Ibs/in] Front spring stiffness
s.ksR = 250; % [Ibs/in] Rear "
s.ksF = s.ksF*ref.Ibin2Nm; s.ksR = s.ksR*ref.Ibin2Nm; % [N/m] Springs stiffnesses

% 0.5 Damping

% d.z.bsF = 0.7; % [-] Bump - Slow - Front damping ratio (zeta). http://www.kaztechnologies.com/wp-content/uploads/2014/03/A-Guide-To-Your-Dampers-Chapter-from-FSAE-Book-by-Jim-Kasprzak.pdf
% d.z.bfF = 0.7;
% d.z.rsF = 0.7; % [-] Rebound - Slow - Front damping ratio (zeta)
% d.z.rfF = 0.7;
% d.z.bsR = 0.7;
% d.z.bfR = 0.7;
% d.z.rsR = 0.7;
% d.z.rfR = 0.7;

d.c.bsF = 4130; % [N/(m/s)] Bump - Slow - Front damping setting
d.c.bfF = 4130;
d.c.rsF = 4130; % [N/(m/s)] Rebound - Slow - Front damping setting
d.c.rfF = 4130;
d.c.bsR = 4130;
d.c.bfR = 4130;
d.c.rsR = 4130;
d.c.rfR = 4130;

