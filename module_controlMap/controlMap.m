function c = controlMap(c,T,costf)

c.mode = strings(size(T)); c.mode(:) = "cruise";
c.delta_t = zeros(size(T)); % throttle position [0 1]
c.delta_b = zeros(size(T)); % brake position [0 1]
c.delta_c = zeros(size(T)); % clutch position [0 1]
c.theta_s = zeros(size(T)); % throttle position [0 1]


% % % Lateral Ref. Case
% % Idling 1
% iA.Tbeg = 0;
% iA.Tend = 2;
% c = idlingAction(iA,T,c);
% 
% % Standing Start 1
% ssA.Tbeg = 2;
% ssA.Tend = 5;
% ssA.MaxThrottle = 0.4;
% ssA.InputType = "ramp";
% c = standingstartAction(ssA,T,c);
% 
% % Turn 1
% sA.TbegI = 10;
% sA.TendI = 10.25;
% sA.MaxAngle = pi/3;
% sA.TbegII = 16;
% sA.TendII = 20;
% sA.InputType = "ramp";
% c = steeringAction(sA,T,c);
% 
% % Acceleration 2
% aA.Tbeg = 19;
% aA.Tend = 21;
% aA.MaxThrottle = 0.1;
% aA.InputType = "step";
% c = accelerationAction(aA,T,c);
% 
% % Turn 2
% sA.TbegI = 20;
% sA.TendI = 20.25;
% sA.MaxAngle = -pi/3;
% sA.TbegII = 24;
% sA.TendII = 25;
% sA.InputType = "ramp";
% c = steeringAction(sA,T,c);
% 
% % Acceleration 3
% aA.Tbeg = 25;
% aA.Tend = 30;
% aA.MaxThrottle = 0.1;
% aA.InputType = "step";
% c = accelerationAction(aA,T,c);
% 
% % Turn 3
% sA.TbegI = 30;
% sA.TendI = 30.25;
% sA.MaxAngle = pi/3;
% sA.TbegII = 30.5;
% sA.TendII = 31;
% sA.InputType = "ramp";
% c = steeringAction(sA,T,c);
% 
% % Turn 4
% sA.TbegI = 31;
% sA.TendI = 34;
% sA.MaxAngle = -pi/10;
% sA.TbegII = 35;
% sA.TendII = 39;
% sA.InputType = "ramp";
% c = steeringAction(sA,T,c);



% % Longitudinal Ref. Case
% Idling 1
iA.Tbeg = 0;
iA.Tend = 5;
c = idlingAction(iA,T,c);

% Acceleration 1 (while idle)
ssA.Tbeg = 5;
ssA.Tend = 7;
ssA.MaxThrottle = 1;
ssA.InputType = "step";
c = accelerationAction(ssA,T,c);
c.delta_c(T>=ssA.Tbeg & T<ssA.Tend) = 1; % (while idle)

% Idling 2
iA.Tbeg = 7;
iA.Tend = 10;
c = idlingAction(iA,T,c);

% Standing Start 1
ssA.Tbeg = 10;
ssA.Tend = 12;
ssA.MaxThrottle = 1;
ssA.InputType = "ramp";
c = standingstartAction(ssA,T,c);

% Acceleration 2
ssA.Tbeg = 12;
ssA.Tend = 15;
ssA.MaxThrottle = 1;
ssA.InputType = "step";
c = accelerationAction(ssA,T,c);

end

function c = steeringAction(A,T,c)
tspanI = (T>=A.TbegI & T<A.TendII); % indices that lie inside the specified time span
tspanMAX = (T>=A.TendI & T<A.TbegII);
tspanII = (T>=A.TbegII & T<A.TendII);

c.mode(tspanI | tspanMAX | tspanII) = "steering";
switch A.InputType
    case "ramp"
        c.theta_s(tspanI) = A.MaxAngle*(T(tspanI)-A.TbegI)/(A.TendI-A.TbegI);
        c.theta_s(tspanMAX) = A.MaxAngle;
        c.theta_s(tspanII) = A.MaxAngle*(A.TendII-T(tspanII))/(A.TendII-A.TbegII);
end

end

function c = accelerationAction(A,T,c)
tspan = (T>=A.Tbeg & T<A.Tend);

c.mode(tspan) = "acceleration";
switch A.InputType
    case "step"
        c.delta_t(tspan) = A.MaxThrottle;
end

end

function c = standingstartAction(A,T,c)
tspan = (T>=A.Tbeg & T<A.Tend);

c.mode(tspan) = "standingstart";
switch A.InputType
    case "ramp"
        c.delta_t(tspan) = A.MaxThrottle*(T(tspan)-A.Tbeg)/(A.Tend-A.Tbeg);
        c.delta_c(tspan) = 1 - 1*(T(tspan)-A.Tbeg)/(A.Tend-A.Tbeg);
end

end

function c = idlingAction(A,T,c)
tspan = (T>=A.Tbeg & T<A.Tend);

c.mode(tspan) = "idling";
c.delta_t(T>=A.Tbeg & T<A.Tend) = 0;
c.delta_c(T>=A.Tbeg & T<A.Tend) = 1;

end
