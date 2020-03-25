function [DXe,DYe,DZe,DPSI,DTHETA,DPHI,DU,DV,DW,DP,DQ,DR] = EoMsolverV3(i,e,Xe,Ye,Ze,PSI,THETA,PHI,U,V,W,P,Q,R,FX,FY,FZ,MX,MY,MZ)

vecsize = size(Xe);

FZ = zeros(vecsize);
MX = zeros(vecsize);
MY = zeros(vecsize);

FX = FX - i.m.*e.g.*sin(THETA);
FY = FY + i.m.*e.g.*cos(THETA).*sin(PHI);
% FZ = FZ + i.m.*e.g.*cos(THETA).*cos(PHI);
F = [FX FY FZ];

OMEGA = [P Q R];
VB = [U V W];

I = [i.Ixx i.Iyy i.Izz]; % products of inertia not taken into account
M = [MX MY MZ];


% Linear - 2nd Derivatives - PROJECTED IN BODY FRAME; WITH RESPECT TO EARTH FRAME
DVB = F/i.m -cross(OMEGA,VB);

% Angular - 2nd Derivatives - PROJECTED IN BODY FRAME; WITH RESPECT TO EARTH FRAME
DOMEGA = (M - cross(OMEGA,OMEGA.*I))./I; % Assuming that all the products of inertia are negligible

% Angular - 1st Derivatives - IN BODY FRAME
DPHI = P + (Q.*sin(PHI) + R.*cos(PHI)).*tan(THETA);
DTHETA = Q.*cos(PHI) - R.*sin(PHI);
DPSI = (Q.*sin(PHI) + R.*cos(PHI)).*sec(THETA); % sec=1/cos

% Linear - 1st Derivatives - PROJECTED IN EARTH FRAME; WITH RESPECT TO EARTH FRAME. Velocities in body frame are rotated through Euler angles.
if vecsize == 1
%     rotm = eul2rotm(-[PHI THETA PSI],'XYZ'); % "-" and "XYZ" because the rotation goes from body to earth
    rotm = quat2rotm(eul2quat(-[PHI THETA PSI],"XYZ")); % "-" and "XYZ" because the rotation goes from body to earth. Quat to avoid gimbal lock.
    DX = VB*rotm; % eul2rotm documentation states that the rotation matrix should be premultiplied by the vector that is to be rotated
    DXe = DX(1);
    DYe = DX(2);
    DZe = DX(3);
else
    DXe = U.*cos(THETA).*cos(PSI) + V.*(sin(PHI).*sin(THETA).*cos(PSI) - cos(PHI).*sin(PSI)) + W.*(cos(PHI).*sin(THETA).*cos(PSI) + sin(PHI).*sin(PSI));
    DYe = U.*cos(THETA).*sin(PSI) + V.*(sin(PHI).*sin(THETA).*sin(PSI) + cos(PHI).*cos(PSI)) + W.*(cos(PHI).*sin(THETA).*sin(PSI) - sin(PHI).*cos(PSI));
    DZe = -U.*sin(THETA) + V.*sin(PHI).*cos(THETA) + W.*cos(PHI).*cos(THETA);
end


% RENAME OUTPUTS
DU = DVB(:,1);
DV = DVB(:,2);
DW = DVB(:,3);
DP = DOMEGA(:,1);
DQ = DOMEGA(:,2);
DR = DOMEGA(:,3);

%% Alternatives

% % Terms due to the product of inertia ZX
% DP = DP + (i.Izx*P.*Q)/i.Ixx;
% DQ = DQ - (i.Izx*(P.^2-R.^2))/i.Iyy;
% DR = DR - (i.Izx*Q.*R)/i.Izz;

% % Correction due to the coupling DP--DR
% DR = DR - i.Izx*DP/i.Izz;
% DP = DP - i.Izx*DR/i.Ixx;

% % Actual value of DP, DR
% function F = eq46(x0_F)
%     dp_F = x0_F(1);
%     dr_F = x0_F(2);
%     eq4 = i.Ixx*dp_F - i.Izx*dr_F + Q*R*(i.Izz-i.Iyy) - i.Izx*P*Q - MX;
%     eq6 = i.Izz*dr_F - i.Izx*dp_F + P*Q*(i.Iyy-i.Ixx) + i.Izx*Q*R - MZ;
%     F(1) = eq4;
%     F(2) = eq6;
% end
% 
% ansatz = 0;
% output_F = fsolve(@eq46,[ansatz ansatz]);
% DP = output_F(1);
% DR = output_F(2);


% % SIMPLIFIED VERSION
% DU = FX/i.m;
% DV = FY/i.m;
% DW = zeros(vecsize);
% DZe = zeros(vecsize);
% DP = zeros(vecsize);
% DQ = zeros(vecsize);
% DR = MZ/i.Izz;
% DPHI = zeros(vecsize);
% DTHETA = zeros(vecsize);
% DPSI = R;
% DXe = U.*cos(THETA).*cos(PSI) + V.*(sin(PHI).*sin(THETA).*cos(PSI) - cos(PHI).*sin(PSI)) + W.*(cos(PHI).*sin(THETA).*cos(PSI) + sin(PHI).*sin(PSI));
% DYe = U.*cos(THETA).*sin(PSI) + V.*(sin(PHI).*sin(THETA).*sin(PSI) + cos(PHI).*cos(PSI)) + W.*(cos(PHI).*sin(THETA).*sin(PSI) - sin(PHI).*cos(PSI));
% DZe = -U.*sin(THETA) + V.*sin(PHI).*cos(THETA) + W.*cos(PHI).*cos(THETA);

end