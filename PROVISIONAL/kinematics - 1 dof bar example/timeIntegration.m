function [T, R] = timeIntegration(body, ground)

% 0. Data
L = body.nodalDistance;
m = body.mass;
I = body.inertia(2,2);
g = body.loads{1}.magnitude;

% 1. Kinematics: differentiation. Symbolic.
% theta = sym('theta');
syms theta(t) Rx Rz

r = L/2*[cos(theta(t)) sin(theta(t))];
v = diff(r, t);
a = diff(v, t);

eqs = [m*a(1) - Rx, m*a(2) + m*g - Rz, I*diff(theta(t), t, t) + Rz*r(1) - Rx*r(2)];

theta0 = atan2(norm(cross(body.refFrame(:,1),ground.refFrame(:,1))),dot(body.refFrame(:,1),ground.refFrame(:,1)));
dtheta0 = 0;
X0 = [theta0; dtheta0];
tspan = 0:0.02:20;

options = odeset('OutputFcn',@odeplot);
figure
[T,THETA] = ode45(@(t,X) dynamicsSolver(t,X,eqs),tspan,X0,options);

R = zeros(length(T),2); % preallocate
for i = 1:length(T)
    R(i,:) = double(subs(r,theta,THETA(i,1)));
end

end