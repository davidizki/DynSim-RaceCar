function [xqIN,yqIN] = refinerLONG(M,refLevel)

xIN = M(:,1); yIN = M(:,2); zIN = M(:,3);

% dIN = [0 sqrt(diff(xIN).^2 + diff(yIN).^2).'];
% for i = 2:length(dIN)
%     dIN(i) = dIN(i)+dIN(i-1);
% end


% Apply the refinement in x
xqIN = zeros(length(xIN)*refLevel-(refLevel-1),1);
for i = 1:(length(xIN)-1)
    j = 1+(i-1)*refLevel;
    xqIN(j) = xIN(i);
    delta = (xIN(i+1) - xIN(i))/refLevel;
    for k = 1:(refLevel-1)
        xqIN(j+k) = xqIN(j+k-1) + delta;
    end
end
xqIN(end) = xIN(end);

% Apply the refinement in y
yqIN = zeros(length(yIN)*refLevel-(refLevel-1),1);
for i = 1:(length(yIN)-1)
    j = 1+(i-1)*refLevel;
    yqIN(j) = yIN(i);
    delta = (yIN(i+1) - yIN(i))/refLevel;
    for k = 1:(refLevel-1)
        yqIN(j+k) = yqIN(j+k-1) + delta;
    end
end
yqIN(end) = yIN(end);


% plot(xIN,yIN,'r-','lineWidth',2)
% hold on; plot(xqIN,yqIN,'bo')


end