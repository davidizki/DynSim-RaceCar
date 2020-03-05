function [A1, A2, A3] = E2B(Ae,VE)
% Rotate a vector / set of vectors from earth frame to body (velocity) frame
% Inputs:
%     -Ae: vector in earth-frame to be rotated
%     -VE: velocity vector in earth-frame to determine the rotation
% Outputs:
%     -A1, A2, A3: components of the rotated vector

% "angle": angle measured from vector 1 ([1 0 0]) to vector 2. Positive if vector 2 is in the "CCW side" of vector 1.

n = size(VE,1);

earth = repmat([1 0 0],[n,1]); % repeat the vector as many times as rows of VE

cr = cross(earth,VE,2); % cross product along second dim. Angle between "velocity-attached" frame and earth-fixed frame

angle = -atan2(vecnorm(cr,2,2),dot(earth,VE,2));
angle(cr(:,3)>=0) = -angle(cr(:,3)>=0);

A = zeros(n,3);
for ii = 1:n
    M = eul2rotm([angle(ii) 0 0]);
    A(ii,:) = (inv(M)*Ae(ii,:).').';
end

A1 = A(:,1);
A2 = A(:,2);
A3 = A(:,3);
end