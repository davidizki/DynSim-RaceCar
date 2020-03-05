function a_abs = rotAccVector(a_loc,v)

% "angle": angle measured from vector 1 ([1 0 0]) to vector 2. Positive if vector 2 is in the "CCW side" of vector 1.

cr = cross([1 0 0],v);
if cr(3) < 0
    angle = -atan2(norm(cr),dot([1 0 0],v)); % THIS DOES NOT WORK OUT OF (-PI, PI). "else" correction needed
else
    angle = atan2(norm(cr),dot([1 0 0],v));
end

M = eul2rotm([angle 0 0]);
a_abs = M*a_loc.';

end