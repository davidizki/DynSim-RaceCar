function dyn = shifter(dyn)

% Unfold all parameters
e = dyn.e; g = dyn.g; p = dyn.p;


for jj = 1:(length(p.ratios)-1)
    ii = 2;
    p.gear = jj;
    FXengine(ii) = 1; FXengineUP(ii) = 0; FXengine(ii-1) = 1; FXengineUP(ii-1) = 0; % arbitrary values to match conditions below
    while FXengineUP(ii) < FXengine(ii) || FXengineUP(ii-1) < FXengine(ii-1) || isnan(FXengineUP(ii))
        if FXengine(ii) ~= 1
            ii = ii + 1;
        else
            ii = 1;
        end
        speed(ii) = rpm2s(p,p.curves(ii,1),p.gear,g.R);
        p.rpm = s2rpm(p,speed(ii),p.gear,g.R);
        FXengine(ii) = engineMap_forShifter(p,speed(ii),1,e.w.rho);
        p.gear = p.gear+1;
        p.rpm = s2rpm(p,speed(ii),p.gear,g.R);
        FXengineUP(ii) = engineMap_forShifter(p,speed(ii),1,e.w.rho);
        p.gear = p.gear-1;
        if ii == 1
            ii = 2;
            FXengine(ii) = 2;
        end
    end
    x1(1) = speed(ii-2); % speed 1
    y1(1) = FXengine(ii-2);
    
    x1(2) = speed(ii-1); % speed 1
    y1(2) = FXengine(ii-1);
    
    x2(1) = speed(ii-2); % speed 1
    y2(1) = FXengineUP(ii-2);
    
    x2(2) = speed(ii-1); % speed 1
    y2(2) = FXengineUP(ii-1);
    
    
%     % line1
%     x1  = [7.8 8.5];
%     y1  = [0.96 0.94];
%     
%     % line2
%     x2 = [8.25 8.25];
%     y2 = [0 0.99];
    
    % fit linear polynomial
    p1 = polyfit(x1,y1,1);
    p2 = polyfit(x2,y2,1);
    
    % calculate intersection
    shift_speeds(jj) = fzero(@(x) polyval(p1-p2,x),3);
end

p.shift_speeds = [0 shift_speeds];

% Fold modified parameters
dyn.p = p;

end