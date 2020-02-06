function DX = pointMass(T,X)

% Unfold the input
r = X(1:3);
v = X(4:6);

% CONSTANTS
g = 9.80665;

% APPLY DRIVER INPUTS (prescribed driver models)
delta_t = 1; % throttle position [0 1]
delta_b = 0; % brake pedal position [0 1]
theta_s = pi/6; % steering wheel position [-pi/2 pi/2]
theta_t = steeringMap(theta_s); % front wheels steering position

% GET DATA FROM MAPS (using data from ICs / previous step)

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
W = m*g;
max_steer = pi/2;
mu = 1.7;

% AERO
rho = 1.225;
SCDf = 2.5;
SCD = 1.0;
V = norm(v);
% v = v*10/V;
% V = norm(v);

% ENGINE
Pmax = 55*735.5;

delta_t = 0.5; % 0,1
delta_b = 0; % 0,1
theta_s = pi/6; % -pi,pi -> But pi/2 yields maximum!

FXaero = -1/2*rho*V^2*SCD;
FZaero = 1/2*rho*V^2*SCDf;

% FXengine = engineMap(gear,V,delta_t,rho)
FXengine = Pmax/1000; % *delta_t/V_engine;

FX = 0; % FXaero + FXengine;

FZtyre = (W + FZaero)/4;
% compute slip angle from theta_t and v direction
% FY=f(alpha,FZtyre,IA,p,V,mu?) from tyreMap
FY = mu*(W + FZaero)*1/3; % just provisional

FZ = 0;

a = [FX FY FZ]/m;


c = cross([1 0 0],v);
if c(3) < 0
    angle = -atan2(norm(c),dot([1 0 0],v)); % THIS DOES NOT WORK OUT OF (-PI, PI)
else
    angle = atan2(norm(c),dot([1 0 0],v));
end
% M = eul2rotm([angle 0 0]);
% a_abs = M*a.';
a_abs = [-sin(angle)*a(2) cos(angle)*a(2) 0];

angle2 = atan2(norm(cross(v,a_abs)),dot(v,a_abs));

% Collect the output
DX = zeros(6,1);
DX(1:3) = v;
DX(4:6) = a_abs;

% close all
% figure
% plot([0 0 0],v);
% hold on; plot([0 0 0],a);
% hold on; plot([0 0 0],a_abs);
% legend('v','a','a_abs');
end