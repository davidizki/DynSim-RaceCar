function [DXe,DYe,DZe,DPSI,DTHETA,DPHI,DU,DV,DW,DP,DQ,DR] = EoMsolver(i,e,Xe,Ye,Ze,PSI,THETA,PHI,U,V,W,P,Q,R,VE,FX,FY,FZ,MX,MY,MZ)

vecsize = size(Xe);

% Linear - 2nd Derivatives - PROJECTED IN BODY FRAME; WITH RESPECT TO EARTH FRAME
DU = (FX - i.m.*e.g.*sin(THETA))./i.m - Q.*W + R.*V;

DV = (FY + i.m.*e.g.*cos(THETA).*sin(PHI))./i.m - R.*U + P.*W;

DW = (FZ + i.m.*e.g.*cos(THETA).*cos(PHI))./i.m - P.*V + Q.*U;

% Angular - 2nd Derivatives - PROJECTED IN BODY FRAME; WITH RESPECT TO EARTH FRAME
DP1 = (MX - Q.*R*(i.Izz-i.Iyy) + i.Izx*P.*Q)/i.Ixx;

DQ = (MY - R.*P*(i.Ixx-i.Izz) - i.Izx*(P.^2-R.^2))/i.Iyy;

DR1 = (MZ - P.*Q*(i.Iyy-i.Ixx) - i.Izx*Q.*R)/i.Izz;

% To avoid implicit equations, DP and DR are computed without the implicit term, and then corrected
DP = DP1 + i.Izx*DR1/i.Ixx;
DR = DR1 + i.Izx*DP1/i.Izz;

% Angular - 1st Derivatives - IN BODY FRAME
DPHI = P + (Q.*sin(PHI) + R.*cos(PHI)).*tan(THETA);

DTHETA = Q.*cos(PHI) - R.*sin(PHI);

DPSI = (Q.*sin(PHI) + R.*cos(PHI)).*sec(THETA); % sec=1/cos

% Linear - 1st Derivatives - PROJECTED IN EARTH FRAME; WITH RESPECT TO EARTH FRAME. Velocities in body frame are rotated through Euler angles.
DXe = VE(1).*cos(THETA).*cos(PSI) + VE(2).*(sin(PHI).*sin(THETA).*cos(PSI) - cos(PHI).*sin(PSI)) + VE(3).*(cos(PHI).*sin(THETA).*cos(PSI) + sin(PHI).*sin(PSI));

DYe = VE(1).*cos(THETA).*sin(PSI) + VE(2).*(sin(PHI).*sin(THETA).*sin(PSI) + cos(PHI).*cos(PSI)) + VE(3).*(cos(PHI).*sin(THETA).*sin(PSI) - sin(PHI).*cos(PSI));

DZe = -VE(1).*sin(THETA) + VE(2).*sin(PHI).*cos(THETA) + VE(3).*cos(PHI).*cos(THETA);
% If all angles == 0, should be equivalent to:
% DXe = VE(1);
% DYe = VE(2);
% DZe = VE(3);

DW = zeros(vecsize); % TO FIX IT TO ZERO UNTIL LOADS ARE WELL COMPUTED
DP = zeros(vecsize);
DQ = zeros(vecsize);
DPHI = zeros(vecsize);
DTHETA = zeros(vecsize);


end