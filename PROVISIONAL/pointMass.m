function DX = pointMass(T,X)

% Unfold the input
r = X(1:3);
v = X(4:6);

% GENERAL
m = 315;
W = m*9.81;
max_steer = pi/2;
mu = 1.7;

% AERO
rho = 1.225;
S = 1;
CDf = 2.5;
CD = 1.0;
V = norm(v);
% v = v*10/V;
% V = norm(v);

% ENGINE
Pmax = 55*735.5;

delta_t = 0.5; % 0,1
delta_b = 0; % 0,1
theta_s = pi/6; % -pi,pi -> But pi/2 yields maximum!
% controls = [delta_t delta_b theta_s];

FXaero = -1/2*rho*V^2*S*CD;
FZaero = 1/2*rho*V^2*S*CDf;

% if V < eps
%     V_engine = 1;
% else
%     V_engine = V;
% end

FXengine = Pmax/1000; % *delta_t/V_engine;

FX = 0; % FXaero + FXengine;
FY = 1800; % mu*(W + FZaero)*(theta_s/max_steer);
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