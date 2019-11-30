function [normalsX, normalsY, normalsZ, Xc, Yc, Zc] = normalsCreator(X,Y,Z)

m = size(X,1)-1;
n = size(X,2)-1;

normals = cell(m,n);
normalsX = zeros(m,n);
normalsY = zeros(m,n);
normalsZ = zeros(m,n);
Xc = zeros(m,n);
Yc = zeros(m,n);
Zc = zeros(m,n);

for i = 1:m
    for j = 1:n
        % normals: cross product between two sides of each quadrilateral item
        dxlong1 = X(i+1,j)-X(i,j);
        dxtrans1 = X(i,j+1)-X(i,j);
        dylong1 = Y(i+1,j)-Y(i,j);
        dytrans1 = Y(i,j+1)-Y(i,j);
        dzlong1 = Z(i+1,j)-Z(i,j);
        dztrans1 = Z(i,j+1)-Z(i,j);
        dxlong2 = X(i+1,j+1)-X(i,j+1);
        dxtrans2 = X(i+1,j+1)-X(i+1,j);
        dylong2 = Y(i+1,j+1)-Y(i,j+1);
        dytrans2 = Y(i+1,j+1)-Y(i+1,j);
        dzlong2 = Z(i+1,j+1)-Z(i,j+1);
        dztrans2 = Z(i+1,j+1)-Z(i+1,j);
        dlong1 = [dxlong1 dylong1 dzlong1];
        dtrans1 = [dxtrans1 dytrans1 dztrans1];
        dlong2 = [dxlong2 dylong2 dzlong2];
        dtrans2 = [dxtrans2 dytrans2 dztrans2];
        normals{i,j} = cross(mean([dlong1; dlong2]), mean([dtrans1; dtrans2]));
        normals{i,j} = normals{i,j}/norm(normals{i,j});
        
        normalsX(i,j) = normals{i,j}(1);
        normalsY(i,j) = normals{i,j}(2);
        normalsZ(i,j) = normals{i,j}(3);
        
        % centers: average of the side values
        halfxlong1 = mean([X(i+1,j) X(i,j)]);
        halfxtrans1 = mean([X(i,j+1) X(i,j)]);
        halfylong1 = mean([Y(i+1,j) Y(i,j)]);
        halfytrans1 = mean([Y(i,j+1) Y(i,j)]);
        halfzlong1 = mean([Z(i+1,j) Z(i,j)]);
        halfztrans1 = mean([Z(i,j+1) Z(i,j)]);
        halfxlong2 = mean([X(i+1,j+1) X(i,j+1)]);
        halfxtrans2 = mean([X(i+1,j+1) X(i+1,j)]);
        halfylong2 = mean([Y(i+1,j+1) Y(i,j+1)]);
        halfytrans2 = mean([Y(i+1,j+1) Y(i+1,j)]);
        halfzlong2 = mean([Z(i+1,j+1) Z(i,j+1)]);
        halfztrans2 = mean([Z(i+1,j+1) Z(i+1,j)]);
        
        Xc(i,j) = mean([mean([halfxlong1 halfxlong2]) mean([halfxtrans1 halfxtrans2])]);
        Yc(i,j) = mean([mean([halfylong1 halfylong2]) mean([halfytrans1 halfytrans2])]);
        Zc(i,j) = mean([mean([halfzlong1 halfzlong2]) mean([halfztrans1 halfztrans2])]);
        
    end
end


end