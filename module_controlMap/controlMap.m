function c = controlMap(c,T,mode)

c.delta_t = zeros(size(T)); % throttle position [0 1]
c.delta_b = NaN(size(T)); % throttle position [0 1]
c.theta_s = zeros(size(T)); % throttle position [0 1]


% Acceleration 1
aA(1).Tbeg = 0;
aA(1).Tend = 5;
aA(1).MaxThrottle = 0.5;
aA(1).InputType = "step";
c = accelerationAction(aA(1),T,c);

% Turn 1
sA(1).TbegI = 10;
sA(1).TendI = 10.25;
sA(1).MaxAngle = pi/3;
sA(1).TbegII = 16;
sA(1).TendII = 20;
sA(1).InputType = "ramp";
c = steeringAction(sA(1),T,c);

% Acceleration 2
aA(2).Tbeg = 19;
aA(2).Tend = 21;
aA(2).MaxThrottle = 0.1;
aA(2).InputType = "step";
c = accelerationAction(aA(2),T,c);

% Turn 2
sA(2).TbegI = 20;
sA(2).TendI = 20.25;
sA(2).MaxAngle = -pi/3;
sA(2).TbegII = 24;
sA(2).TendII = 25;
sA(2).InputType = "ramp";
c = steeringAction(sA(2),T,c);

% Acceleration 3
aA(3).Tbeg = 25;
aA(3).Tend = 30;
aA(3).MaxThrottle = 0.1;
aA(3).InputType = "step";
c = accelerationAction(aA(3),T,c);

% Turn 3
sA(3).TbegI = 30;
sA(3).TendI = 30.25;
sA(3).MaxAngle = pi/3;
sA(3).TbegII = 30.5;
sA(3).TendII = 31;
sA(3).InputType = "ramp";
c = steeringAction(sA(3),T,c);

% Turn 4
sA(4).TbegI = 31;
sA(4).TendI = 34;
sA(4).MaxAngle = -pi/10;
sA(4).TbegII = 35;
sA(4).TendII = 39;
sA(4).InputType = "ramp";
c = steeringAction(sA(4),T,c);

% % Longitudinal Ref. Case
% Tfullthrottle = 5;
% c.delta_t(T<Tfullthrottle) = 0.5 + 0.5/Tfullthrottle*T(T<Tfullthrottle);
% c.delta_t(T>=Tfullthrottle) = 1;
% c.delta_b(:) = 0;
% c.theta_s(:) = 0;

end

function c = steeringAction(sA,T,c)

switch sA.InputType
    case "ramp"
        c.theta_s(T>=sA.TbegI & T<sA.TendI) = sA.MaxAngle*(T(T>=sA.TbegI & T<sA.TendI)-sA.TbegI)/(sA.TendI-sA.TbegI);
        c.theta_s(T>=sA.TendI) = sA.MaxAngle;
        c.theta_s(T>=sA.TbegII & T<sA.TendII) = sA.MaxAngle*(sA.TendII-T(T>=sA.TbegII & T<sA.TendII))/(sA.TendII-sA.TbegII);
        c.theta_s(T>=sA.TendII) = 0;
end

end

function c = accelerationAction(aA,T,c)

switch aA.InputType
    case "step"
        c.delta_t(T>aA.Tbeg & T<aA.Tend) = aA.MaxThrottle;
end

end

