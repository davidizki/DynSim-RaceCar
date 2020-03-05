function [A1e, A2e, A3e] = B2E(A,VE)
% Rotate a vector / set of vectors from body (velocity) frame to earth frame
% Inputs:
%     -A: vector in body(velocity)-frame to be rotated
%     -VE: velocity vector in earth-frame to determine the rotation
% Outputs:
%     -A1e, A2e, A3e: components of the rotated vector

% "angle": angle measured from vector 1 ([1 0 0]) to vector 2. Positive if vector 2 is in the "CCW side" of vector 1.

n = size(VE,1);

earth = repmat([1 0 0],[n,1]); % repeat the vector as many times as rows of VE

cr = cross(earth,VE,2); % cross product along second dim. Angle between "velocity-attached" frame and earth-fixed frame

angle = -atan2(vecnorm(cr,2,2),dot(earth,VE,2));
angle(cr(:,3)>=0) = -angle(cr(:,3)>=0);

Ae = zeros(n,3);
for ii = 1:n
    M = eul2rotm([angle(ii) 0 0]);
    Ae(ii,:) = (M*A(ii,:).').';
end

A1e = Ae(:,1);
A2e = Ae(:,2);
A3e = Ae(:,3);
end