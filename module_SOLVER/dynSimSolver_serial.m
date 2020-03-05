function [DX] = dynSimSolver_serial(T,X,integration_flag,varargin)

global dyn

if isempty(varargin) % do nothing
elseif length(varargin) == 1
    index = varargin{1};
else
    error('Wrong number of input arguments');
end

% Unfold parameters
e = dyn.e; c = dyn.c; g = dyn.g; i = dyn.i; s = dyn.s; d = dyn.d; p = dyn.p; a = dyn.a; n = dyn.n; o = dyn.o;

% Unfold the input
r = X(1:3);
v = X(4:6);


% APPLY DRIVER INPUTS (prescribed driver models)
delta_t = 1; % throttle position [0 1]
delta_b = 0; % brake pedal position [0 1]
theta_s = pi/4; % steering wheel position [-pi/2 pi/2]

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
m = 315;
W = m*e.g;
mu = 1.7; %*

% AERO
SCDf = 2.5;
SCD = 1.0;
V = norm(v);
FXaero = -1/2*e.w.rho*V^2*SCD;
FZaero = 1/2*e.w.rho*V^2*SCDf;

% ENGINE
R = 0.1955;

% rpm
p.rpm = V/R*p.ratios(p.gear)*(60/2/pi);
if p.rpm > 13500
    p.rpm = 13500;
end

% Gear selection
if p.rpm > p.rpm_gearshift_top && p.gear < 6
    p.gear = p.gear+1;
elseif p.rpm < p.rpm_gearshift_bottom && p.gear > 1
    p.gear = p.gear-1;
end

FXengine = engineMap(p,V,delta_t,e.w.rho);

% DYNAMICS
theta_t = steeringMap(theta_s); % front wheels steering position



% EOM
FX = FXaero + FXengine;

FZtyre = (W + FZaero)/4;
% compute slip angle from theta_t and v direction
% FY=f(alpha,FZtyre,IA,p,V,mu?) from tyreMap
FY = mu*(W + FZaero)*theta_t; % just provisional

FZ = 0;

a_loc = [FX FY FZ]/m;


cr = cross([1 0 0],v);
if cr(3) < 0
    angle = -atan2(norm(cr),dot([1 0 0],v)); % THIS DOES NOT WORK OUT OF (-PI, PI)
else
    angle = atan2(norm(cr),dot([1 0 0],v));
end
M = eul2rotm([angle 0 0]);
a_abs = M*a_loc.';
% a_abs = [-sin(angle)*a(2) cos(angle)*a(2) 0];

% Collect the output
DX = zeros(6,1);
DX(1:3) = v;
DX(4:6) = a_abs;


if ~integration_flag % if not integrating (retriving values)
    p.rpm_history(index) = p.rpm;
    p.gear_history(index) = p.gear;
    o.G(index,:) = a_loc/e.g;
end


% Fold parameters
dyn.e = e; dyn.c = c; dyn.g = g; dyn.i = i; dyn.s = s; dyn.d = d; dyn.p = p; dyn.a = a; dyn.n = n; dyn.o = o;

end