function [X, Y] = refinerTRANS(Xin,Yin,Xout,Yout,refLevelTRANS)

m = length(Xin);
n = refLevelTRANS*2;

% Preallocate
X = zeros(m,n);
Y = zeros(m,n);
X(:,1) = Xout;
X(:,end) = Xin;
Y(:,1) = Yout;
Y(:,end) = Yin;

for i = 1:m
    deltaX = (Xin(i) - Xout(i))/(n-1);
    deltaY = (Yin(i) - Yout(i))/(n-1);
    for j = 2:n-1
        X(i,j) = X(i,j-1) + deltaX;
        Y(i,j) = Y(i,j-1) + deltaY;
    end
end

end