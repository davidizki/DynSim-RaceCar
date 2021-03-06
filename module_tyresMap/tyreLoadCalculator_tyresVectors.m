function F_Y = tyreLoadCalculator_tyresVectors(alpha,FZ,F_Z0,Pacejka)

alpha = reshape(alpha,1,[]); % row
FZ = reshape(FZ,[],1); % col

% Unfold inputs
Pacejka = [F_Z0 Pacejka];
F_Z0 = Pacejka(1);
p_DY1 = Pacejka(2); % To change the sign of the curves, just change the sign of the p_DY1 and p_KY1 coefficients
p_KY1 = Pacejka(3); % 
p_KY2 = Pacejka(4);
p_CY1 = Pacejka(5);
p_EY1 = Pacejka(6);
p_HY1 = Pacejka(7);
p_VY1 = Pacejka(8);
p_DY2 = Pacejka(9);
p_DY3 = Pacejka(10);
p_HY3 = Pacejka(11);
p_VY3 = Pacejka(12);
p_VY4 = Pacejka(13);
p_KY3 = Pacejka(14);

% Other parameters -not used in this model-
camber = 0; % camber is taken into account by changing the parameters themselves

lambda_muy = 1; % change in grip is just taken into account linearly, out of the formula -to avoid stability issues-

p_HY2 = 0;
p_VY2 = 0;
p_EY2 = 0;
p_EY3 = 0;
p_EY4 = 0;

lambda_C_y = 1;
lambda_Z0 = 1;
lambda_FZ0 = 1;
lambda_Kya = 1;
lambda_Hy = 1;
lambda_Ey = 1;
lambda_Vy = 1;

% F_Z0 = max([TYRE.FZnom]);
dfz = (FZ - F_Z0)/F_Z0;

D_y = FZ.*(p_DY1 + p_DY2.*dfz)*(1 - p_DY3*camber^2)*lambda_muy;
C_y = p_CY1*lambda_C_y;
B_y = p_KY1*F_Z0*sin(2*atan(FZ/(p_KY2*F_Z0*lambda_Z0)))*(1 - p_KY3*abs(camber))*lambda_FZ0*lambda_Kya/C_y/D_y;
S_Hy = (p_HY1 + p_HY2.*dfz + p_HY3*camber)*lambda_Hy;
alpha_y = alpha + S_Hy;
% E_y = (p_EY1 + p_EY2*dfz)*(1 - (p_EY3 + p_EY4*camber)*sign(alpha_y))*lambda_Ey;
E_y = (p_EY1 + p_EY2.*dfz)*lambda_Ey;
S_Vy = FZ.*(p_VY1 + p_VY2.*dfz + (p_VY3 + p_VY4.*dfz)*camber)*lambda_Vy*lambda_Kya;

D_y_mat = repmat(D_y,1,numel(alpha));
E_y_mat = repmat(E_y,1,numel(alpha));
S_Vy_mat = repmat(S_Vy,1,numel(alpha));

F_Y = D_y_mat.*sin(C_y*atan(B_y*alpha_y - E_y_mat.*(B_y*alpha_y - atan(B_y*alpha_y)))) + S_Vy_mat;

% F_Y matrix: for each FZ (time instant), a row of FY(alpha)

% mainCoeffs = [B_y,C_y,D_y,E_y,S_Hy,S_Vy];

end