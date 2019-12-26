function DX = dynamicsSolver(t,X,eqs)

syms theta(t) Rx Rz

% theta = X(1);
% dtheta = X(2);

% solve nonlinear system to get ddtheta
eqs = subs(eqs,diff(theta(t),t,t),'ddtheta');
eqs = subs(eqs,diff(theta(t),t),X(2));
eqs = subs(eqs,theta(t),X(1));

% F{1} = matlabFunction(eqs(1));
% F{2} = matlabFunction(eqs(2));
% F{3} = matlabFunction(eqs(3));
% 
% X0 = zeros(3,1);
% NLoutput = fsolve(F,X0);
% ddtheta = NLoutput(1);
% Rx = NLoutput(2);
% Rz = NLoutput(3);

sol = solve([eqs(1)==0, eqs(2)==0, eqs(3)==0], ['ddtheta', Rx, Rz]);
ddtheta = double(sol.ddtheta);

DX(1,1) = X(2);
DX(2,1) = ddtheta;

end