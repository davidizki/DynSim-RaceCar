function [X, Y, Z, Xc, Yc, Zc, normalsX, normalsY, normalsZ] = module_trackMap()

% 0. USER DATA
filename_in = "Track01in.txt";
filename_out = "Track01out.txt";
scalingFactor = 0.1; % if txt data dimensions is not in meters
refLevelLONG = 3; % factor by which total # points increases
refLevelTRANS = 3; % factor by which total # points increases

% 1. CREATE MATRIX OF DATA FORM INPUT
Min = matrixCreator(filename_in);
Mout = matrixCreator(filename_out);
Min = Min*scalingFactor;
Mout = Mout*scalingFactor;

% 2. IMPLEMENT REFINEMENTS
% 2.1 Longitudinal Refinement
[Xin,Yin] = refinerLONG(Min,refLevelLONG);
[Xout,Yout] = refinerLONG(Mout,refLevelLONG);

% 2.2 Transversal Refinement
[X, Y] = refinerTRANS(Xin,Yin,Xout,Yout,refLevelTRANS);


% 3. 3D (HEIGHT)
Z = randn(size(X))/10;
Z(end,:) = Z(1,:);
gradM = zeros(size(Z));
grad = linspace(0,10,length(Z)/2+1);
grad = [grad(1:end-1) fliplr(grad)].';
for k = 1:size(Z,2)
    gradM(:,k) = grad;
end
Z = Z + gradM;

[normalsX, normalsY, normalsZ, Xc, Yc, Zc] = normalsCreator(X,Y,Z);

% NORMALS ARE "WRONG": QUADRILATERALS ARE NOT PLANES! THEY ARE BILINEAR
% The procedure to compute the normals could be to get the plane tangent to the center of each element
% The simpler approach taken here is to perform the cross product of the average long and trans vectors

% 4. PLOT
% figure()
% grid on
% for i = 1:size(X,2)
%     hold on; plot(X(:,i),Y(:,i),'.')
% end
% 
% figure()
% grid on
% surf(X,Y,Z)
% hold on; plot3(Xc,Yc,Zc,'*')
% zlim([-5 200])
% 
% figure()
% grid on
% surf(X,Y,Z)
% hold on; quiver3(Xc,Yc,Zc,normalsX,normalsY,normalsZ,0.025)
% zlim([-2 50])

end

