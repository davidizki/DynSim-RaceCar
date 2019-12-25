

function DX = eqsMotion(obj)

% X(1:3) -> GC position; X(4:6) -> orientation; X(7:9) -> GC velocity; X(10:12) -> rate of change of orientation
X(1:3) = obj.xGC;
X(4:6) = obj.orientation;



DX(7:9) = obj.torsorGC(1:3)/obj.mass;
DX(10:12) = obj.torsorGC(4:6)./obj.inertIA???; % INERTIA TENSOR

end