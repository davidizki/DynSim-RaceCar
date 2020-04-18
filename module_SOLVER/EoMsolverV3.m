function [DXe,DYe,DZe,DPSI,DTHETA,DPHI,DU,DV,DW,DP,DQ,DR] = EoMsolverV3(i,e,Xe,Ye,Ze,PSI,THETA,PHI,U,V,W,P,Q,R,F,M)
global nn nnzeros integration_flag

% 0. Prepare variables
VB = [U V W];
OMEGA = [P Q R];
I = [i.Ixx i.Iyy i.Izz]; % products of inertia not taken into account in the equations (correction introduced afterwards

% PROJECT ALL COMPUTED FORCES, MOMENTS INTO THE BODY-FIXED FRAME: Forces are defined in a reference frame aligned with the body PSI angle, but not with THETA and PHI rotations
rotm = quat2rotm(eul2quat([nnzeros THETA PHI],"ZYX"));
for ii = 1:nn % n-dimensional multiplication of arrays is not supported by MATLAB, just up to 2D
    F(ii,:) = F(ii,:)*rotm(:,:,ii); % eul2rotm documentation: rotation matrix should be premultiplied by the vector that is to be rotated
end

for ii = 1:nn
    M(ii,:) = M(ii,:)*rotm(:,:,ii);
end


% 1. EQUATIONS OF MOTION - DERIVATIVES
% Linear - 2nd Derivatives - PROJECTED IN BODY FRAME; WITH RESPECT TO EARTH FRAME
DVB = F/i.m - cross(OMEGA,VB);

% Angular - 2nd Derivatives - PROJECTED IN BODY FRAME; WITH RESPECT TO EARTH FRAME
DOMEGA = (M - cross(OMEGA,OMEGA.*I))./I; % Assuming that all the products of inertia are negligible

% Angular - 1st Derivatives - IN BODY FRAME
DPHI = P + (Q.*sin(PHI) + R.*cos(PHI)).*tan(THETA);
DTHETA = Q.*cos(PHI) - R.*sin(PHI);
DPSI = (Q.*sin(PHI) + R.*cos(PHI)).*sec(THETA); % sec=1/cos

% Linear - 1st Derivatives - PROJECTED IN EARTH FRAME; WITH RESPECT TO EARTH FRAME. Velocities in body frame are rotated through Euler angles.
rotm = quat2rotm(eul2quat(-[PHI THETA PSI],"XYZ")); % "-" and "XYZ" because the rotation goes from body to earth. Quat to avoid gimbal lock.
DX = zeros([3,nn]); % row vector - due to rotm requirements
for ii = 1:nn % n-dimensional multiplication of arrays is not supported by MATLAB, just up to 2D
    DX(:,ii) = VB(ii,:)*rotm(:,:,ii); % eul2rotm documentation states that the rotation matrix should be premultiplied by the vector that is to be rotated
end
DXe(:,1) = DX(1,:);
DYe(:,1) = DX(2,:);
DZe(:,1) = DX(3,:);

% RENAME OUTPUTS
DU = DVB(:,1);
DV = DVB(:,2);
DW = DVB(:,3);
DP = DOMEGA(:,1);
DQ = DOMEGA(:,2);
DR = DOMEGA(:,3);

% Corrections to the angular rates
% Terms due to the product of inertia ZX
DP = DP + (i.Izx*P.*Q)/i.Ixx;
DQ = DQ - (i.Izx*(P.^2-R.^2))/i.Iyy;
DR = DR - (i.Izx*Q.*R)/i.Izz;

% Correction due to the coupling DP--DR
DR = DR - i.Izx*DP/i.Izz;
DP = DP - i.Izx*DR/i.Ixx;

%% Alternatives

% % Actual value of DP, DR
% if integration_flag
%     ansatz = 0;
%     output_F = fsolve(@eq46,[ansatz ansatz]);
%     DP = output_F(1);
%     DR = output_F(2);
% end
% 
% function F = eq46(x0_F)
%     dp_F = x0_F(1);
%     dr_F = x0_F(2);
%     eq4 = i.Ixx*dp_F - i.Izx*dr_F + Q*R*(i.Izz-i.Iyy) - i.Izx*P*Q - M(:,1);
%     eq6 = i.Izz*dr_F - i.Izx*dp_F + P*Q*(i.Iyy-i.Ixx) + i.Izx*Q*R - M(:,3);
%     F(1) = eq4;
%     F(2) = eq6;
% end

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